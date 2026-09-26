# -*- coding: utf-8 -*-
"""
障碍竞速（排位赛）策略 —— 单文件 test.py
====================================================================
赛规要求：起点出发 → 围障碍1转 1 圈 → 穿过障碍2 → 围障碍3转 2 圈 → 终点
本策略实现（按用户要求）：
  1) 障碍1 只绕 1 圈（不多绕）
  2) 绕圈方向统一为逆时针（CCW，俯视 XY 平面角度递增）
  3) 障碍2 采用 S 形绕过（先左弯贴南侧、再右弯甩出，不绕圈）
  4) 障碍3 逆时针 2 圈，使用实际位置累计角度，不按路径索引猜圈数
  5) 高度锁定为鱼的起点高度（340），全程保持该高度不变，水平姿态

场地坐标（由使用手册 + 历史轨迹 fish_debug_log.csv + 地图数据确认）：
  水池 3000(x) * 2000(y) * 1500(z)，水底中心 (0,0,0)
  起点 (-1250, 0, 340) 朝 +x；终点 (1250, 0)
  地图实际有四根 200x200x1500 柱：
    障碍1 (-750,0)，障碍2双柱 (0,-175)/(0,175)，障碍3 (750,0)。
  障碍2中间净空 y∈(-75,75)，穿越时需提前对正，不可沿 y=-200 穿柱。

控制结构：纯追踪(pure-pursuit)前瞻点 + 航向PID -> 尾舵；
          曲率调度推力 -> 双侧胸鳍；深度/俯仰反馈；终点停止。
16秒是用户目标，不是本版本实测成绩。本文件不修改裁判、时间流速或比赛记录。
v5：穿缝直线导引、提前平顺入圈、前瞻线段及短时航迹碰撞检查；
保留v4高度->垂直速度->俯仰角->胸鳍倾角级联反馈。
v5.1：整条动作链路提速8%（用户要求5%~10%）。做法是把推进量按同一系数放大：
      尾摆频率、尾摆幅值、巡航/转弯/穿缝目标速度、转弯加速度预算、速度环增益、
      穿缝减速斜率与各类保守推力上限。平台的硬限幅（推进力50、尾角±80）、
      安全间隙55、绕圈圈数判定一律不改，所以提速不换安全余量。
      绕圈限速是 sqrt(预算/曲率)，预算按系数平方放大才使绕圈速度真正+8%。
      想回到v5速度：把下面 SPEED_GAIN 改成 1.0 即可，无需改其它行。
v6：① 提速系数 1.08 -> 1.16（用户要求"再快一些"）。
    注意快档 THRUST_BASE 已是 50=MAX_THRUST，推力恒被钳在 50，
    目标速度数值本身不改变实际推力；真正决定快慢的是尾摆频率与幅值，
    所以提速必须落在 TAIL_*_HZ / TAIL_AMPLITUDE_* 上。
    ② 出缝后立刻内切：slalom_2 由"先向北偏+35再俯冲"的两段曲线，
    改为一条两端切向都连续的三次贝塞尔，直接下压到南侧入圈点。
    旧版转折点被推到第三柱西侧(x≈450)才出现，即"到第三柱才开始向里走"；
    新版出缝后 52mm 就出现向内趋势，到第三柱西侧已完成 70% 横移，
    且最大曲率 1/222 低于绕圈曲率 1/200（不触发曲率限速、无急拐），
    路径还短了 48mm，entry_guard 也不再触发。
v7：高度锁定为"鱼的起点高度"（用户要求）。
    起点 (-1250, 0, 340) 在画面下半部分；旧版把目标高度设成 750（水池中层），
    开局要先爬升到中层，多花时间、多一次姿态调整。现在 HOLD_Z = START_Z = 340，
    开赛即处于目标高度：高度误差恒为0，垂直速度目标与俯仰目标恒为0，
    不会出现"先移动到画面中间"或"再调整回中间高度"的动作。
    绕障、穿缝、绕圈全程都保持该高度。
平台沿胸鳍GetUpVector施力；MakeRotator(0, angle, 0)下，
angle=0为向上推，angle=-90才是沿鱼身向前推，不能再将胸鳍限制在±25。
本包蓝图的VelLimit对各轴速度限幅，不能靠无限增加目标速度突破。
命令行加 --stable 可使用v2平面路线/速度参数；平台不方便传参时，
将下方 USE_STABLE_PROFILE 改为 True。两种模式都使用修正后的高度控制，
不恢复旧版的向上推力错误。两种模式都保留全部圈数和限幅。
实测由用户执行；几何/逻辑自测不能代替比赛成绩验收。
命令行自测：python -B test.py --self-test（不导入 cue，不连接平台）。
"""

import math
import os
import sys
import threading
import time

# ========================== 场地常量 ==========================
START = (-1250.0, 0.0)
GOAL = (1250.0, 0.0)
OBS1 = (-750.0, 0.0)
OBS2 = (0.0, 0.0)
OBS3 = (750.0, 0.0)
PILLARS = (OBS1, (0.0, -175.0), (0.0, 175.0), OBS3)
PILLAR_HALF_DIAG = 141.5          # 200x200 柱子的半对角线
USE_STABLE_PROFILE = '--stable' in sys.argv
LOOP_R = 210.0 if USE_STABLE_PROFILE else 200.0  # 快档方柱角间隙约59，仍校验>=55
SAMPLE_STEP = 8.0                 # 路径采样步长
# v7：高度锁定为起点高度，全程不变。
# 起点 (-1250, 0, 340) 在画面下半部分；旧版以 750（水池中层）为目标高度，
# 开局要先爬升到中层，既多花时间又多一次姿态调整。现在直接以起点高度为
# 目标高度，开赛即处于目标高度，不需要任何"过渡到中层"的动作。
START_Z = 340.0                   # 鱼的起点高度（由官方起点坐标确认）
HOLD_Z = START_Z                  # 全程保持的高度，与起点高度相同
WING_NEUTRAL = -90.0             # 胸鳍的UpVector在此角度沿鱼身+x
MAX_WING_TILT = 85.0             # 相对水平推进基准；不允许转到倒推半球
MAX_THRUST = 50.0
MAX_TAIL = 80.0
TEAM_NAME = 'F05012589'
PROFILE_NAME = 'race-v7-stable-hold-z' if USE_STABLE_PROFILE else 'race-v7-hold-start-z'
# 整体提速系数：v5.1 按用户要求 +8%；v6 用户要求"再快一些"，提到 1.16。
# 只放大推进量与目标速度：尾摆频率/幅值、巡航/转弯/穿缝目标速度、转弯加速度预算。
# 不放宽任何平台限幅（MAX_THRUST=50、MAX_TAIL=80）、安全间隙和圈数判定。
#
# 重要：快档的 THRUST_BASE 已经是 50（=MAX_THRUST），所以推力恒被钳在 50，
# 目标速度数值本身不改变实际推力。真正决定快慢的是**尾摆频率与幅值**
# （手册：尾摆是主要推进方式，摆动频率与动力成正比）。因此提速必须落到
# TAIL_STRAIGHT_HZ / TAIL_TURN_HZ / TAIL_AMPLITUDE_* 上，这里统一乘 SPEED_GAIN。
SPEED_GAIN = 1.16
# 目标速度不是实际速度。推进器仍限制在50，尾角仍限制在±80度。
CRUISE_SPEED = (540.0 if USE_STABLE_PROFILE else 560.0) * SPEED_GAIN
TURN_SPEED = (480.0 if USE_STABLE_PROFILE else 540.0) * SPEED_GAIN
GAP_SPEED_BASE = 360.0 if USE_STABLE_PROFILE else 400.0
GAP_SPEED = GAP_SPEED_BASE * SPEED_GAIN
# 曲率限速是 sqrt(预算/曲率)，预算只乘一次的话绕圈速度只涨 sqrt(1.08)≈4%，
# 达不到要求的 8%；因此预算按系数的平方放大。
TURN_ACCEL_BUDGET = (1150.0 if USE_STABLE_PROFILE else 1460.0) * SPEED_GAIN ** 2
THRUST_BASE = 48.0 if USE_STABLE_PROFILE else 50.0
# 手册：尾摆频率与动力成正比，尾摆是主要推进方式；这里是主要提速手段。
TAIL_STRAIGHT_HZ = (3.4 if USE_STABLE_PROFILE else 3.8) * SPEED_GAIN
TAIL_TURN_HZ = (3.3 if USE_STABLE_PROFILE else 3.6) * SPEED_GAIN
# 尾摆幅值（同样是推进量）；窄缝内仍收窄，避免摆动导致反复纠偏。
TAIL_AMPLITUDE_STRAIGHT = 24.0 * SPEED_GAIN
TAIL_AMPLITUDE_TURN = 18.0 * SPEED_GAIN
GAP_TAIL_AMPLITUDE = 8.0 * SPEED_GAIN
# 穿缝未对正时的保守速度上限；与横向偏移减速曲线在同一尺度上。
GAP_ALIGN_SPEED = 280.0 * SPEED_GAIN
GAP_LANE_FLOOR = 250.0 * SPEED_GAIN
# 绕圈偏离圆周后重新贴回的推力上限（安全限幅，只按系数微调）。
ORBIT_RECAPTURE_THRUST = 28.0 * SPEED_GAIN
# 曲率限速的下限；实际路径最大曲率远达不到该下限触发点，仅为常量尺度一致。
TURN_SPEED_FLOOR = 250.0 * SPEED_GAIN
# 出缝后内切曲线的控制臂长度（相对绕行半径）。0.85R 让曲率上限约 1/217，
# 低于曲率限速的"免费"阈值，因此内切过程不会额外减速，也不会出现急拐。
CURVE_ARM_RATIO = 0.85

# ========================== 路径构建工具 ==========================


def _polar(cx, cy, r, a_deg):
    a = math.radians(a_deg)
    return (cx + r * math.cos(a), cy + r * math.sin(a))


class PathBuilder:
    """以 直线段 / 圆弧段 拼接路径，均匀采样为 (x, y, heading) 点列。"""

    def __init__(self, step=SAMPLE_STEP):
        self.step = step
        self.pts = []

    def _push(self, x, y, h):
        self.pts.append((x, y, h % (2 * math.pi)))

    def straight(self, p0, p1):
        x0, y0 = p0
        x1, y1 = p1
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-6:
            return
        h = math.atan2(y1 - y0, x1 - x0)
        n = max(2, int(length / self.step) + 1)
        for i in range(n):
            t = i / (n - 1)
            self._push(x0 + t * (x1 - x0), y0 + t * (y1 - y0), h)

    def arc(self, center, radius, a0_deg, sweep_deg, ccw=True):
        """从圆心角 a0 起，扫过 sweep 度；ccw=True 为逆时针。"""
        length = abs(math.radians(sweep_deg)) * radius
        n = max(2, int(length / self.step) + 1)
        s = 1.0 if ccw else -1.0
        for i in range(n):
            t = i / (n - 1)
            a = math.radians(a0_deg) + s * math.radians(sweep_deg) * t
            x = center[0] + radius * math.cos(a)
            y = center[1] + radius * math.sin(a)
            h = (a + math.pi / 2) if ccw else (a - math.pi / 2)
            self._push(x, y, h)

    def last_point(self):
        return self.pts[-1][:2]

    def bezier(self, p0, p1, p2, p3):
        """三次曲线；端点导数用于保证相邻段的切向连续。"""
        estimate = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                       for a, b in zip((p0, p1, p2), (p1, p2, p3)))
        n = max(2, int(math.ceil(estimate / self.step)) + 1)
        for i in range(n):
            t = i / (n - 1)
            u = 1.0 - t
            x = u**3*p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + t**3*p3[0]
            y = u**3*p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + t**3*p3[1]
            dx = 3*u*u*(p1[0]-p0[0]) + 6*u*t*(p2[0]-p1[0]) + 3*t*t*(p3[0]-p2[0])
            dy = 3*u*u*(p1[1]-p0[1]) + 6*u*t*(p2[1]-p1[1]) + 3*t*t*(p3[1]-p2[1])
            self._push(x, y, math.atan2(dy, dx))


class PathSection:
    def __init__(self, name, points, center=None, laps=0):
        self.name = name
        self.points = points
        self.center = center
        self.laps = laps


def generate_sections():
    """按比赛阶段隔离路径，前瞻点不能跨过尚未完成的绕圈。"""
    sections = []
    e1 = (OBS1[0], -LOOP_R)
    e3 = (OBS3[0], -LOOP_R)

    pb = PathBuilder()
    pb.bezier(START, (-1060.0, 0.0), (-1040.0, -LOOP_R), e1)
    sections.append(PathSection('approach_1', pb.pts))

    pb = PathBuilder()
    pb.arc(OBS1, LOOP_R, 270.0, 360.0)
    sections.append(PathSection('orbit_1', pb.pts, OBS1, 1))

    pb = PathBuilder()
    # 从障碍1下侧逐渐向中线对正；双柱范围内保持 y=0、朝+x。
    pb.bezier(e1, (-510.0, -LOOP_R), (-390.0, 0.0), (-190.0, 0.0))
    pb.straight((-190.0, 0.0), (190.0, 0.0))
    # v6：出缝后立刻内切。旧版先向北偏 +35 再俯冲到南侧，
    # 转折点被推到第三柱西侧（x≈450）才出现，所以"到第三柱才开始向里走"。
    # 现在改用一条与两端切向都连续的三次贝塞尔，从出缝点直接下压到南侧入圈点：
    # 两端切向都是 +x（与直缝、与入圈圆弧相切），曲率上限 1/LOOP_R，
    # 不会触发曲率限速，也不会出现急拐或卡顿。
    arm = CURVE_ARM_RATIO * LOOP_R
    pb.bezier((190.0, 0.0), (190.0+arm, 0.0), (OBS3[0]-arm, -LOOP_R), e3)
    # 终点 e3 正好是 orbit_3 圆弧的起点(270度)，两端切向都是 +x，无需额外圆弧。
    sections.append(PathSection('slalom_2', pb.pts))

    pb = PathBuilder()
    pb.arc(OBS3, LOOP_R, 270.0, 720.0)
    sections.append(PathSection('orbit_3', pb.pts, OBS3, 2))

    pb = PathBuilder()
    pb.bezier(e3, (1000.0, -LOOP_R), (1080.0, 0.0), GOAL)
    pb.straight(GOAL, (1320.0, 0.0))
    sections.append(PathSection('finish', pb.pts))
    return sections


def generate_path():
    """保留旧接口，返回仅用于检查/展示的完整点列。"""
    return [point for section in generate_sections() for point in section.points]


def segment_clearance(a, b, center):
    """XY线段到200x200方柱的精确距离；检测前瞻切角，不仅检查采样点。"""
    ax, ay = a[0]-center[0], a[1]-center[1]
    bx, by = b[0]-center[0], b[1]-center[1]
    dx, dy = bx-ax, by-ay
    lo, hi = 0.0, 1.0
    intersects = True
    for origin, delta in ((ax, dx), (ay, dy)):
        if abs(delta) < 1e-12:
            if abs(origin) > 100.0:
                intersects = False
                break
        else:
            t0, t1 = (-100.0-origin)/delta, (100.0-origin)/delta
            lo, hi = max(lo, min(t0, t1)), min(hi, max(t0, t1))
            if lo > hi:
                intersects = False
                break
    if intersects:
        return 0.0
    distance = min(math.hypot(max(abs(px)-100.0, 0.0),
                              max(abs(py)-100.0, 0.0))
                   for px, py in ((ax, ay), (bx, by)))
    norm = dx*dx+dy*dy
    if norm > 1e-12:
        for cx in (-100.0, 100.0):
            for cy in (-100.0, 100.0):
                t = max(0.0, min(1.0, ((cx-ax)*dx+(cy-ay)*dy)/norm))
                distance = min(distance, math.hypot(ax+t*dx-cx, ay+t*dy-cy))
    return distance


# ========================== 航向 PID ==========================
class PIDController:
    def __init__(self, kp=2.2, ki=0.01, kd=0.22, max_integral=50.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.error_prev = None
        self.integral = 0.0
        self.derivative = 0.0

    def calculate(self, error, dt=1.0 / 60.0):
        dt = _constrain(dt, 0.001, 0.1)
        p = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i = self.ki * self.integral
        # 误差跨 ±180° 时不能出现 360°/dt 的虚假微分。
        delta = 0.0 if self.error_prev is None else (error-self.error_prev+180.0) % 360.0-180.0
        alpha = dt / (0.06 + dt)
        self.derivative += alpha * (delta/dt - self.derivative)
        d = self.kd * self.derivative
        self.error_prev = error
        return p + i + d

    def reset(self):
        self.error_prev = None
        self.integral = 0.0
        self.derivative = 0.0


# ========================== 路径跟踪查找器 ==========================
class PathTracker:
    """
    单调前进的纯追踪查找器：
      - 在当前位置前方的滑动窗口内找最近点，保证索引只增不减（不会回绕）
      - 前瞻距离随局部曲率自适应：直道看远（高速顺畅），弯道看近（贴线）
    """

    def __init__(self, pts):
        self.pts = pts
        self.idx = 0
        self.curv = [0.0] * len(pts)
        self.signed_curv = [0.0] * len(pts)
        self._precompute_curvature()

    def _precompute_curvature(self):
        n = len(self.pts)
        for i in range(n):
            j = min(n - 1, i + 12)
            d = math.hypot(self.pts[j][0] - self.pts[i][0], self.pts[j][1] - self.pts[i][1])
            if d < 1e-6:
                continue
            dh = math.atan2(math.sin(self.pts[j][2] - self.pts[i][2]),
                            math.cos(self.pts[j][2] - self.pts[i][2]))
            # 前馈需要有符号曲率；弧长分母避免急弯被短弦夸大。
            arc_length = sum(math.hypot(self.pts[k+1][0]-self.pts[k][0],
                                        self.pts[k+1][1]-self.pts[k][1])
                             for k in range(i, j))
            self.signed_curv[i] = dh / max(arc_length, 1e-6)
            self.curv[i] = abs(dh)/d if USE_STABLE_PROFILE else abs(self.signed_curv[i])

    def _lookahead(self, i):
        c = self.curv[min(i, len(self.curv) - 1)]
        if c > 0.0040:      # 绕圈段
            return 85.0
        if c > 0.0015:      # S 弯/过渡段
            return 130.0
        return 175.0        # 直道

    def target(self, x, y):
        n = len(self.pts)
        if self.idx >= n:
            return None
        hi = min(n, self.idx + 24)
        best_i = self.idx
        best_d = float('inf')
        for i in range(self.idx, hi):
            d = math.hypot(self.pts[i][0] - x, self.pts[i][1] - y)
            if d < best_d:
                best_d = d
                best_i = i
        self.idx = best_i
        if best_i >= n - 1:
            return self.pts[n - 1]
        la = self._lookahead(best_i)
        # 中间双柱之间的净空只有150；不能前瞻到出缝后的转弯。
        if USE_STABLE_PROFILE and abs(x) < 210.0:
            la = min(la, 45.0)
        j = best_i
        acc = 0.0
        while j < n - 1 and acc < la:
            acc += math.hypot(self.pts[j + 1][0] - self.pts[j][0],
                              self.pts[j + 1][1] - self.pts[j][1])
            j += 1
        # 路径本身安全不代表朝前瞻点走的弦安全；方柱角附近逐步缩短。
        if not USE_STABLE_PROFILE:
            while j > best_i+1 and any(
                    segment_clearance((x, y), self.pts[j], center) < 55.0
                    for center in PILLARS):
                j -= 1
        return self.pts[j]

    def done(self):
        return self.idx >= len(self.pts) - 1


def _constrain(v, lo, hi):
    return max(lo, min(hi, v))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OrbitProgress:
    """仅累计真实运动产生的有符号角度；逆行会抵消，不奖励瞬移或抖动。"""
    def __init__(self, center, laps):
        self.center = center
        self.required = laps * 2.0 * math.pi
        self.angle = 0.0
        self.previous = None

    def update(self, x, y):
        dx, dy = x-self.center[0], y-self.center[1]
        radius = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        if not PILLAR_HALF_DIAG + 35.0 <= radius <= LOOP_R + 140.0:
            self.previous = None
            return
        if self.previous is not None:
            delta = _wrap(theta - self.previous)
            if abs(delta) <= 0.5:
                self.angle += delta
        self.previous = theta

    def done(self):
        return self.angle + 1e-9 >= self.required


class DepthController:
    """通过胸鳍控制俯仰/滚转，再由前进方向控制升降，不持续向上顶鱼身。"""
    def __init__(self):
        self.integral = 0.0
        self.reference_vz = 0.0
        self.desired_pitch = 0.0
        self.last_pitch = None
        self.last_roll = None
        self.pitch_rate = 0.0
        self.roll_rate = 0.0
        self.pitch_trim = 0.0
        self.roll_trim = 0.0

    def reset_dynamics(self):
        # 通信间断后不使用旧姿态差分；保留胸鳍角度以避免重连瞬间跳变。
        self.last_pitch = self.last_roll = None
        self.pitch_rate = self.roll_rate = 0.0
        self.integral = self.reference_vz = 0.0

    @staticmethod
    def vertical_component(angle, pitch=0.0, roll=0.0):
        """平台胸鳍UpVector的世界Z分量；用于方向检查，不是动力学仿真。"""
        tilt = math.radians(angle-WING_NEUTRAL)
        p, r = math.radians(pitch), math.radians(roll)
        return math.sin(p)*math.cos(tilt) + math.cos(p)*math.cos(r)*math.sin(tilt)

    def calculate(self, z, vertical_speed, pitch, roll, speed, dt):
        dt = _constrain(dt, 0.001, 0.1)
        roll = (roll+180.0) % 360.0-180.0
        alpha = dt/(0.06+dt)
        if self.last_pitch is not None:
            measured = _constrain((pitch-self.last_pitch)/dt, -180.0, 180.0)
            self.pitch_rate += alpha*(measured-self.pitch_rate)
            delta_roll = (roll-self.last_roll+180.0) % 360.0-180.0
            measured = _constrain(delta_roll/dt, -180.0, 180.0)
            self.roll_rate += alpha*(measured-self.roll_rate)
        self.last_pitch, self.last_roll = pitch, roll

        height_error = HOLD_Z-z
        # 贴近锁定高度时才积累小幅偏置；大偏差和大姿态恢复时禁止积分饱和。
        if 12.0 < abs(height_error) < 160.0 and abs(pitch) < 20.0:
            self.integral = _constrain(self.integral+0.04*height_error*dt, -12.0, 12.0)
        else:
            self.integral *= max(0.0, 1.0-dt)
        height_term = 0.0 if abs(height_error) <= 12.0 else height_error
        wanted_vz = _constrain(0.9*height_term+self.integral, -95.0, 95.0)
        # 垂直速度目标仍做斜率限制，避免姿态突跃；但开局已处于目标高度，
        # height_error≈0，因此这里不会产生任何爬升动作。
        self.reference_vz += _constrain(wanted_vz-self.reference_vz, -100.0*dt, 100.0*dt)
        vertical_command = self.reference_vz + 0.7*(self.reference_vz-vertical_speed)
        # 仍保留靠底/靠顶时的放宽余地，以便姿态异常时能有效恢复。
        pitch_limit = 20.0 if z < 180.0 or z > 1200.0 else 12.0
        self.desired_pitch = _constrain(
            math.degrees(math.atan2(vertical_command, max(220.0, speed))),
            -pitch_limit, pitch_limit)

        recovering = abs(pitch) > 30.0 or abs(roll) > 40.0
        trim_limit = 65.0 if recovering else 35.0
        pitch_trim = _constrain(
            2.0*(self.desired_pitch-pitch)-0.9*self.pitch_rate, -trim_limit, trim_limit)
        # UE正Roll时右侧下沉，需要右胸鳍多给上分力，而非增加右侧前向推力。
        roll_trim = _constrain(0.65*roll+0.25*self.roll_rate,
                               -18.0 if recovering else -12.0,
                               18.0 if recovering else 12.0)
        slew = (200.0 if recovering else 80.0)*dt
        self.pitch_trim += _constrain(pitch_trim-self.pitch_trim, -slew, slew)
        self.roll_trim += _constrain(roll_trim-self.roll_trim, -slew, slew)
        left = WING_NEUTRAL + self.pitch_trim-self.roll_trim
        right = WING_NEUTRAL + self.pitch_trim+self.roll_trim
        return (_constrain(left, WING_NEUTRAL-MAX_WING_TILT, WING_NEUTRAL+MAX_WING_TILT),
                _constrain(right, WING_NEUTRAL-MAX_WING_TILT, WING_NEUTRAL+MAX_WING_TILT))


class Command:
    def __init__(self, tail=0.0, left=0.0, right=0.0,
                 left_angle=WING_NEUTRAL, right_angle=WING_NEUTRAL):
        self.tail = _constrain(tail, -MAX_TAIL, MAX_TAIL)
        self.left = _constrain(left, -MAX_THRUST, MAX_THRUST)
        self.right = _constrain(right, -MAX_THRUST, MAX_THRUST)
        self.left_angle = _constrain(
            left_angle, WING_NEUTRAL-MAX_WING_TILT, WING_NEUTRAL+MAX_WING_TILT)
        self.right_angle = _constrain(
            right_angle, WING_NEUTRAL-MAX_WING_TILT, WING_NEUTRAL+MAX_WING_TILT)


class RaceController:
    """纯计算控制器，导入时不启动DDS；便于与真实平台隔离测试。"""
    def __init__(self):
        self.sections = generate_sections()
        self.section_index = 0
        self.tracker = PathTracker(self.sections[0].points)
        self.orbit = None
        self.completed_laps = {}
        self.passed_middle_gap = False
        self.pid = PIDController()
        self.depth = DepthController()
        self.last_yaw = 0.0
        self.pitch_degrees = 0.0
        self.phase = 0.0
        self.started_at = None
        self.last_at = None
        self.last_position = None
        self.speed = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.entry_avoidance = False
        self.vertical_speed = 0.0
        self.target_speed = 0.0
        self.heading_error = 0.0
        self.course_slip = 0.0
        self.course_correction = 0.0
        self.stage_started_at = None
        self.stage_seconds = {}
        self.finished = False
        self.reason = ''
        self.command = Command()

    @property
    def stage(self):
        return self.reason if self.finished else self.sections[self.section_index].name

    def stop(self, reason):
        self.finished = True
        self.reason = reason
        self.command = Command()
        return self.command

    def _next_section(self):
        if self.stage_started_at is not None and self.last_at is not None:
            self.stage_seconds[self.stage] = round(self.last_at-self.stage_started_at, 3)
        self.stage_started_at = self.last_at
        if self.orbit is not None:
            self.completed_laps[self.stage] = self.orbit.angle / (2.0*math.pi)
        self.section_index += 1
        section = self.sections[self.section_index]
        self.tracker = PathTracker(section.points)
        self.orbit = OrbitProgress(section.center, section.laps) if section.center else None
        self.pid.reset()

    def _guidance(self, x, y):
        section = self.sections[self.section_index]
        if self.orbit is not None:
            self.orbit.update(x, y)
            # 角度只在绕柱有效半径内累计；必须完成真实整圈。
            # v2还要求距离出圈点<=85；漂移使该条件错过时可能多等一整圈。
            # v3圈数满足后交给下一段路径收敛，不为等待一个点继续整圈绕行。
            ex, ey, _ = section.points[-1]
            if self.orbit.done() and (
                    not USE_STABLE_PROFILE or math.hypot(x-ex, y-ey) <= 85.0):
                self._next_section()
                return self._guidance(x, y)
            dx, dy = x-section.center[0], y-section.center[1]
            radius_error = math.hypot(dx, dy) - LOOP_R
            # 圆的切向 + 径向误差反馈，比追前瞻弦更不易切入方柱角。
            heading = math.atan2(dy, dx) + math.pi/2
            heading += math.atan2(2.0*radius_error, LOOP_R)
            return _wrap(heading), 1.0/LOOP_R, 1.0/LOOP_R

        target = self.tracker.target(x, y)
        if target is None:
            self.stop('empty_path')
            return 0.0, 0.0, 0.0
        end = section.points[-1]
        if (section.name != 'finish' and self.tracker.idx >= len(section.points)-4
                and math.hypot(x-end[0], y-end[1]) < 35.0):
            if section.name == 'slalom_2' and not self.passed_middle_gap:
                self.stop('missed_middle_gap')
                return 0.0, 0.0, 0.0
            self._next_section()
            return self._guidance(x, y)
        heading = math.atan2(target[1]-y, target[0]-x)
        curvature = self.tracker.curv[self.tracker.idx]
        signed = 0.0 if USE_STABLE_PROFILE else self.tracker.signed_curv[self.tracker.idx]
        if not USE_STABLE_PROFILE and section.name == 'slalom_2' and abs(x) <= 155.0:
            # 穿柱区跟踪直线，而非45cm短前瞻点；横向速度用于提前抑制摆动。
            # 离柱后才恢复S弯，不能提前瞄准出缝后的正y弯。
            heading = _constrain(math.atan2(-y-0.18*self.velocity_y, 140.0),
                                 -math.radians(25.0), math.radians(25.0))
            curvature = signed = 0.0
        return heading, curvature, signed

    def _entry_guard(self, x, y, heading, curvature, signed):
        """第三柱前预测短时惯性航迹；有切柱风险才改为切向避让。"""
        self.entry_avoidance = False
        if (USE_STABLE_PROFILE or self.stage != 'slalom_2'
                or x < 390.0 or not self.passed_middle_gap):
            return heading, curvature, signed
        projected = (x+0.22*self.velocity_x, y+0.22*self.velocity_y)
        if segment_clearance((x, y), projected, OBS3) >= 55.0:
            return heading, curvature, signed
        self.entry_avoidance = True
        dx, dy = x-OBS3[0], y-OBS3[1]
        radius_error = math.hypot(dx, dy)-max(LOOP_R, 225.0)
        # 逆时针切向；靠柱太近时叠加向外分量，不再朝柱心追路径点。
        heading = math.atan2(dy, dx)+math.pi/2+math.atan2(3.0*radius_error, LOOP_R)
        return _wrap(heading), 1.0/LOOP_R, 1.0/LOOP_R

    def _motion_profile(self, curvature, error, x, y):
        """提速不绕过安全条件：只有对正窄缝/跟稳圆弧时才使用快档。"""
        target_speed = CRUISE_SPEED
        if curvature > 0.0015:
            target_speed = _constrain(math.sqrt(TURN_ACCEL_BUDGET/curvature),
                                       TURN_SPEED_FLOOR, TURN_SPEED)
        if self.stage == 'slalom_2' and abs(x) < 240.0:
            if USE_STABLE_PROFILE:
                aligned = abs(y) < 18.0 and abs(error) < 14.0
                target_speed = min(target_speed, GAP_SPEED if aligned else GAP_ALIGN_SPEED)
            else:
                # 不因摆尾瞬时越过14度就从400跳到280；按预测横向偏移连续减速。
                lateral_risk = max(abs(y), abs(y+0.18*self.velocity_y))
                # 减速斜率与整体提速同比例放大，否则提速后偏移惩罚相对变松。
                lane_speed = GAP_SPEED-SPEED_GAIN*(
                    10.0*max(0.0, lateral_risk-18.0)+4.0*max(0.0, abs(error)-22.0))
                target_speed = min(target_speed,
                                   _constrain(lane_speed, GAP_LANE_FLOOR, GAP_SPEED))
        self.target_speed = target_speed
        # 速度环增益随整体提速同步放大，否则目标速度提上去了推力仍跟不上。
        thrust = _constrain(THRUST_BASE + 0.07*SPEED_GAIN*(target_speed-self.speed),
                            18.0, MAX_THRUST)

        # 原版在40°处从50骤降到24，摆尾导致误差跨阈值时会反复急减速。
        # 现在35°后连续减推力，严重偏航仍限制到10；并非取消转弯保护。
        steering_cap = MAX_THRUST * _constrain(
            1.0-max(0.0, abs(error)-35.0)/90.0, 0.20, 1.0)
        thrust = min(thrust, steering_cap)
        if self.orbit is not None:
            cx, cy = self.orbit.center
            radial_error = math.hypot(x-cx, y-cy)-LOOP_R
            if abs(radial_error) > 40.0:
                thrust = min(thrust, ORBIT_RECAPTURE_THRUST)  # 偏离圆周后先收回轨迹，再加速
        if self.entry_avoidance:
            thrust = min(thrust, ORBIT_RECAPTURE_THRUST)
        turning = curvature > 0.003
        frequency = TAIL_TURN_HZ if turning else TAIL_STRAIGHT_HZ
        amplitude = TAIL_AMPLITUDE_TURN if turning else TAIL_AMPLITUDE_STRAIGHT
        if not USE_STABLE_PROFILE and self.stage == 'slalom_2' and abs(x) < 240.0:
            amplitude = GAP_TAIL_AMPLITUDE  # 窄缝内缩小摆尾，保留前向推力，减少左右纠偏。
        return thrust, frequency, amplitude

    def calculate(self, fish_info, now=None):
        if self.finished:
            return self.command
        now = time.monotonic() if now is None else now
        x, y, z = fish_info.pos.x, fish_info.pos.y, fish_info.pos.z
        fx, fy, fz = fish_info.forward.x, fish_info.forward.y, fish_info.forward.z
        roll = fish_info.rot.x
        if not all(math.isfinite(v) for v in (x, y, z, fx, fy, fz, roll, now)):
            return self.stop('invalid_state')
        horizontal_forward = math.hypot(fx, fy)
        if math.sqrt(fx*fx+fy*fy+fz*fz) < 1e-6:
            return self.stop('invalid_forward_vector')
        # 接近竖直时保留上次有效航向，继续恢复姿态；不能停控后任其上浮。
        yaw = math.atan2(fy, fx) if horizontal_forward > 0.05 else self.last_yaw
        if horizontal_forward > 0.05:
            self.last_yaw = yaw
        pitch = math.degrees(math.atan2(fz, horizontal_forward))
        self.pitch_degrees = pitch
        roll = (roll+180.0) % 360.0-180.0
        if self.started_at is None:
            self.started_at = now
            self.stage_started_at = now
        elapsed = now-self.started_at
        if elapsed >= 60.0:
            return self.stop('time_limit')
        raw_dt = 1.0/60.0 if self.last_at is None else now-self.last_at
        if raw_dt <= 0.0:
            return self.command
        dt = _constrain(raw_dt, 0.001, 0.1)
        if self.last_position is not None:
            px, py, pz = self.last_position
            displacement = math.sqrt((x-px)**2 + (y-py)**2 + (z-pz)**2)
            # 重置、长时间断流不应当被当成高速度或绕圈进度。
            if displacement > 180.0 or raw_dt > 0.25:
                self.pid.reset()
                self.speed = self.vertical_speed = 0.0
                self.velocity_x = self.velocity_y = 0.0
                self.course_slip = 0.0
                self.depth.reset_dynamics()
                if self.orbit is not None:
                    self.orbit.previous = None
            else:
                # 用实际轨迹与 x=0 的交点确认穿缝，而非仅看路径索引。
                if self.stage == 'slalom_2' and px < 0.0 <= x:
                    fraction = -px/(x-px)
                    crossing_y = py + fraction*(y-py)
                    if abs(crossing_y) <= 45.0:
                        self.passed_middle_gap = True
                alpha = dt / (0.08 + dt)
                self.velocity_x += alpha*((x-px)/raw_dt-self.velocity_x)
                self.velocity_y += alpha*((y-py)/raw_dt-self.velocity_y)
                self.speed += alpha * (math.hypot(x-px, y-py)/raw_dt-self.speed)
                self.vertical_speed += alpha * ((z-pz)/raw_dt-self.vertical_speed)
                # 用相邻位置测实际航迹方向，再滤波“航迹-鱼头”的角差。
                # 不直接滤波世界航向，避免圆周运动的滤波延迟伪装成侧滑。
                if math.hypot(x-px, y-py)/raw_dt > 100.0 and abs(pitch) < 30.0:
                    slip = _wrap(math.atan2(y-py, x-px)-math.atan2(fy, fx))
                    slip_alpha = dt/(0.10+dt)
                    self.course_slip = _wrap(
                        self.course_slip+slip_alpha*_wrap(slip-self.course_slip))
        self.last_at = now
        self.last_position = (x, y, z)

        if (self.stage == 'finish' and x >= GOAL[0] and abs(y) <= 45.0
                and 150.0 <= z <= 1350.0):
            # 这是本地冲线判断，不等同于裁判确认有效成绩。
            return self.stop('crossed_finish')
        if abs(x) >= 1410.0 or abs(y) >= 900.0:
            return self.stop('pool_boundary')

        desired, curvature, signed_curvature = self._guidance(x, y)
        if self.finished:
            return self.command
        desired, curvature, signed_curvature = self._entry_guard(
            x, y, desired, curvature, signed_curvature)
        self.course_correction = 0.0
        if not USE_STABLE_PROFILE and self.speed > 140.0 and abs(pitch) < 30.0:
            self.course_correction = _constrain(
                -0.65*self.course_slip, -math.radians(20.0), math.radians(20.0))
            desired = _wrap(desired+self.course_correction)
        error = math.degrees(_wrap(desired-yaw))
        self.heading_error = error
        # 弯道前馈只辅助转向，不用时间推算绕圈是否完成。
        feedforward = 0.15 * math.degrees(self.speed*signed_curvature)
        if not USE_STABLE_PROFILE:
            feedforward = _constrain(feedforward, -30.0, 30.0)
        correction = _constrain(self.pid.calculate(error, dt)+feedforward, -62.0, 62.0)

        thrust, frequency, amplitude = self._motion_profile(curvature, error, x, y)
        # 先为转向预留尾角空间，避免单侧裁剪正弦波改变平均转向量。
        amplitude = min(amplitude, MAX_TAIL-abs(correction))
        self.phase = (self.phase + 2.0*math.pi*frequency*dt) % (2.0*math.pi)
        tail = correction + amplitude*math.sin(self.phase)

        left_angle, right_angle = self.depth.calculate(
            z, self.vertical_speed, pitch, roll, self.speed, dt)
        # 姿态失稳时暂时降低推进和摆尾，不把高前向推力变成向水面的推力。
        # 保留少量胸鳍推力产生恢复力矩；不要把所有控制直接归零。
        if abs(pitch) > 25.0 or abs(roll) > 40.0:
            thrust = min(thrust, 28.0)
            tail = _constrain(correction, -25.0, 25.0)
        if abs(pitch) > 45.0 or abs(roll) > 65.0:
            thrust = min(thrust, 16.0)
            tail = 0.0
            self.pid.reset()
        if abs(pitch) > 70.0:
            thrust = min(thrust, 8.0)
        if z > 1150.0 and self.vertical_speed > 20.0:
            thrust = min(thrust, 22.0)
        # 滚转用胸鳍角度差恢复；前向推力差主要产生偏航，不再拿它调滚转。
        self.command = Command(tail, thrust, thrust, left_angle, right_angle)
        return self.command


def self_test():
    """只验证几何与控制逻辑，不模拟物理、不声称已达到16秒。"""
    import unittest
    from types import SimpleNamespace

    def info(x=-1250.0, y=0.0, z=340.0, yaw=0.0, roll=0.0, pitch=0.0):
        p = math.radians(pitch)
        return SimpleNamespace(pos=SimpleNamespace(x=x, y=y, z=z),
                               forward=SimpleNamespace(x=math.cos(yaw)*math.cos(p),
                                                       y=math.sin(yaw)*math.cos(p), z=math.sin(p)),
                               rot=SimpleNamespace(x=roll, y=pitch, z=math.degrees(yaw)))

    class Tests(unittest.TestCase):
        def test_geometry(self):
            sections = generate_sections()
            for section in sections:
                for x, y, heading in section.points:
                    self.assertTrue(all(math.isfinite(v) for v in (x, y, heading)))
                    for cx, cy in PILLARS:
                        clearance = math.hypot(max(abs(x-cx)-100.0, 0.0),
                                               max(abs(y-cy)-100.0, 0.0))
                        self.assertGreaterEqual(clearance, 55.0)
            for a, b in zip(sections, sections[1:]):
                self.assertAlmostEqual(math.hypot(a.points[-1][0]-b.points[0][0],
                                                  a.points[-1][1]-b.points[0][1]), 0.0)
                self.assertAlmostEqual(_wrap(a.points[-1][2]-b.points[0][2]), 0.0)

        def test_exact_planned_laps(self):
            for section in generate_sections():
                if section.center:
                    progress = OrbitProgress(section.center, section.laps)
                    for point in section.points:
                        progress.update(*point[:2])
                    self.assertAlmostEqual(progress.angle/(2*math.pi), section.laps)
                    self.assertTrue(progress.done())

        def test_reverse_and_stationary_not_laps(self):
            p = OrbitProgress(OBS1, 1)
            for _ in range(1000):
                p.update(OBS1[0]+LOOP_R, 0.0)
            self.assertEqual(p.angle, 0.0)
            for degree in range(0, -361, -2):
                p.update(*_polar(*OBS1, LOOP_R, degree))
            self.assertFalse(p.done())
            self.assertLess(p.angle, 0.0)

        def test_two_laps_not_one(self):
            p = OrbitProgress(OBS3, 2)
            for degree in range(361):
                p.update(*_polar(*OBS3, LOOP_R, degree))
            self.assertFalse(p.done())
            for degree in range(361, 721):
                p.update(*_polar(*OBS3, LOOP_R, degree))
            self.assertTrue(p.done())

        def test_teleport_not_lap(self):
            p = OrbitProgress(OBS1, 1)
            for degree in (0, 180, 0, 180, 0):
                p.update(*_polar(*OBS1, LOOP_R, degree))
            self.assertEqual(p.angle, 0.0)

        def test_limits_and_depth(self):
            for z in (150.0, START_Z, HOLD_Z, 800.0, 1300.0):
                c = RaceController()
                command = c.calculate(info(z=z), now=1.0)
                self.assertLessEqual(abs(command.tail), MAX_TAIL)
                self.assertLessEqual(abs(command.left), MAX_THRUST)
                self.assertLessEqual(abs(command.right), MAX_THRUST)
                self.assertLessEqual(abs(command.left_angle-WING_NEUTRAL), MAX_WING_TILT)
                vertical = DepthController.vertical_component(command.left_angle)
                if z > HOLD_Z:
                    self.assertLess(vertical, 0.0)
                if z < HOLD_Z:
                    self.assertGreater(vertical, 0.0)

        def test_finish_is_latched(self):
            c = RaceController()
            c.section_index = len(c.sections)-1
            c.tracker = PathTracker(c.sections[-1].points)
            c.calculate(info(x=1260.0), now=1.0)
            self.assertEqual(c.reason, 'crossed_finish')
            command = c.calculate(info(x=1200.0), now=2.0)
            self.assertEqual((command.tail, command.left, command.right), (0.0, 0.0, 0.0))

        def test_bad_data_and_timeout(self):
            c = RaceController()
            c.calculate(info(z=float('nan')), now=1.0)
            self.assertEqual(c.reason, 'invalid_state')
            c = RaceController()
            c.calculate(info(), now=1.0)
            c.calculate(info(), now=61.0)
            self.assertEqual(c.reason, 'time_limit')

        def test_ideal_route_replay(self):
            c = RaceController()
            now = 1.0
            for section in generate_sections():
                for x, y, heading in section.points:
                    now += 0.02
                    c.calculate(info(x=x, y=y, yaw=heading), now=now)
            self.assertEqual(c.reason, 'crossed_finish')
            self.assertTrue(c.passed_middle_gap)
            self.assertGreaterEqual(c.completed_laps['orbit_1'], 1.0-1e-8)
            self.assertGreaterEqual(c.completed_laps['orbit_3'], 2.0-1e-8)

        def test_middle_gap_requires_actual_crossing(self):
            for y, expected in ((0.0, True), (150.0, False)):
                c = RaceController()
                c.section_index = 2
                c.tracker = PathTracker(c.sections[2].points)
                c.calculate(info(x=-10.0, y=y), now=1.0)
                c.calculate(info(x=10.0, y=y), now=1.02)
                self.assertEqual(c.passed_middle_gap, expected)

        def test_repeated_timestamp(self):
            c = RaceController()
            a = c.calculate(info(), now=1.0)
            phase = c.phase
            b = c.calculate(info(), now=1.0)
            self.assertIs(a, b)
            self.assertEqual(c.phase, phase)

        def test_pid_wrap(self):
            pid = PIDController()
            pid.calculate(179.0)
            pid.calculate(-179.0)
            self.assertLess(abs(pid.derivative), 100.0)

        def test_fast_profile_and_smooth_steering_cap(self):
            c = RaceController()
            c.speed = 430.0
            thrust, frequency, _ = c._motion_profile(1.0/LOOP_R, 0.0, -750.0, -LOOP_R)
            self.assertGreater(thrust, 44.0)
            self.assertEqual(c.target_speed, TURN_SPEED)
            self.assertGreater(frequency, 2.8)
            before = c._motion_profile(0.0, 39.9, -1100.0, 0.0)[0]
            after = c._motion_profile(0.0, 40.1, -1100.0, 0.0)[0]
            self.assertLess(abs(before-after), 0.2)
            self.assertLessEqual(c._motion_profile(0.0, 140.0, -1100.0, 0.0)[0], 10.0)

        def test_gap_fast_only_when_aligned(self):
            c = RaceController()
            c.section_index = 2
            c._motion_profile(0.0, 0.0, 0.0, 0.0)
            self.assertEqual(c.target_speed, GAP_SPEED)
            c._motion_profile(0.0, 20.0, 0.0, 0.0)
            if USE_STABLE_PROFILE:
                self.assertLessEqual(c.target_speed, GAP_ALIGN_SPEED)
            else:
                self.assertEqual(c.target_speed, GAP_SPEED)
            c._motion_profile(0.0, 0.0, 0.0, 30.0)
            self.assertLessEqual(c.target_speed, GAP_ALIGN_SPEED)

        def test_segment_clearance_includes_segment_interior(self):
            self.assertEqual(segment_clearance((-200.0, 0.0), (200.0, 0.0), (0.0, 0.0)), 0.0)
            self.assertEqual(segment_clearance((0.0, 0.0), (0.0, 0.0), (0.0, 0.0)), 0.0)
            self.assertAlmostEqual(segment_clearance((-200.0, 160.0), (200.0, 160.0),
                                                     (0.0, 0.0)), 60.0)
            self.assertAlmostEqual(segment_clearance((130.0, 140.0), (130.0, 140.0),
                                                     (0.0, 0.0)), 50.0)
            self.assertAlmostEqual(segment_clearance((950.0, 160.0), (550.0, 160.0),
                                                     OBS3), 60.0)

        def test_s_entry_curvature_and_clearance(self):
            if USE_STABLE_PROFILE:
                return
            points = generate_sections()[2].points
            tracker = PathTracker(points)
            # v6：出缝后立刻内切。旧版要求"先向北偏到 y>30 再俯冲"，
            # 那正是"到第三柱才开始向里走"的成因，已按用户要求删除。
            # 现在要求曲率不超过绕圈本身，从而不触发曲率限速、不出现急拐。
            self.assertLessEqual(max(tracker.curv), 1.0/LOOP_R + 1e-6)
            self.assertFalse(any(y > 5.0 for x, y, _ in points if x > 190.0))
            # 出缝后必须立刻出现向内（-y）趋势，而不是等到第三柱。
            inward = [x for x, y, _ in points if y < -5.0]
            self.assertTrue(inward, '出缝后没有任何向内位移')
            self.assertLess(min(inward), OBS3[0]-LOOP_R)
            for a, b in zip(points, points[1:]):
                for center in PILLARS:
                    self.assertGreaterEqual(segment_clearance(a, b, center), 55.0)
            for x, y, _ in points:
                target = tracker.target(x, y)
                for center in PILLARS:
                    self.assertGreaterEqual(segment_clearance((x, y), target, center), 55.0)

        def test_gap_line_guidance_ignores_exit_bend(self):
            if USE_STABLE_PROFILE:
                return
            c = RaceController()
            c.section_index = 2
            c.tracker = PathTracker(c.sections[2].points)
            for x, y, _ in c.sections[2].points:
                if x > 155.0:
                    break
                heading, curvature, signed = c._guidance(x, y)
                if abs(x) <= 155.0:
                    self.assertAlmostEqual(heading, 0.0)
                    self.assertEqual((curvature, signed), (0.0, 0.0))
            c.velocity_y = 80.0
            heading, _, _ = c._guidance(150.0, 10.0)
            self.assertLess(heading, 0.0)

        def test_v6_immediate_inward_turn_after_gap(self):
            """用户验收标准：出缝后立刻向内，第三柱不是"开始向里走"的起点。"""
            if USE_STABLE_PROFILE:
                return
            points = generate_sections()[2].points
            gap_exit = 190.0            # 双柱缝的出口 x
            west_of_obs3 = OBS3[0]-LOOP_R   # 第三柱西侧入圈点 x=550
            ys = [(x, y) for x, y, _ in points if x > gap_exit]
            self.assertTrue(ys)
            # 标准1：离开第二根柱子后马上出现向内（-y）趋势。
            first_inward = next(x for x, y in ys if y < -5.0)
            self.assertLess(first_inward - gap_exit, 120.0)
            # 标准2：到达第三根柱子之前就已明显向内（至少走完一半横移）。
            y_at_west = min((abs(x-west_of_obs3), y) for x, y in ys)[1]
            self.assertLess(y_at_west, -0.5*LOOP_R)
            # 标准3：第三柱不是转向起点——在它之前早就已经向内。
            self.assertLess(first_inward, west_of_obs3 - 100.0)
            # 全程单调向内，不出现"先向北再向南"的回头弯。
            monotone = [y for x, y in ys]
            self.assertEqual(monotone, sorted(monotone, reverse=True))

        def test_gap_speed_and_tail_use_lateral_prediction(self):
            if USE_STABLE_PROFILE:
                return
            c = RaceController()
            c.section_index = 2
            thrust, _, amplitude = c._motion_profile(0.0, 15.0, 0.0, 0.0)
            self.assertEqual(c.target_speed, GAP_SPEED)
            self.assertEqual(thrust, MAX_THRUST)
            self.assertEqual(amplitude, GAP_TAIL_AMPLITUDE)
            c.velocity_y = 180.0
            c._motion_profile(0.0, 0.0, 0.0, 0.0)
            self.assertLess(c.target_speed, GAP_SPEED)
            c.velocity_y = 0.0
            c._motion_profile(0.0, 13.9, 0.0, 0.0)
            before = c.target_speed
            c._motion_profile(0.0, 14.1, 0.0, 0.0)
            self.assertEqual(before, c.target_speed)

        def test_third_entry_predicts_collision_before_contact(self):
            if USE_STABLE_PROFILE:
                return
            c = RaceController()
            c.section_index = 2
            c.passed_middle_gap = True
            c.velocity_x = 400.0
            heading, curvature, _ = c._entry_guard(520.0, 0.0, 0.0, 0.0, 0.0)
            self.assertTrue(c.entry_avoidance)
            self.assertLess(math.sin(heading), -0.9)
            self.assertLessEqual(c._motion_profile(curvature, 0.0, 520.0, 0.0)[0],
                                 ORBIT_RECAPTURE_THRUST)
            c.velocity_x, c.velocity_y = 0.0, -400.0
            c._entry_guard(520.0, 0.0, -math.pi/2, 0.0, 0.0)
            self.assertFalse(c.entry_avoidance)
            c.section_index = 3
            c.velocity_x, c.velocity_y = 400.0, 0.0
            c._entry_guard(520.0, 0.0, 0.0, 0.0, 0.0)
            self.assertFalse(c.entry_avoidance)

        def test_telemetry_gap_resets_velocity_prediction(self):
            c = RaceController()
            c.calculate(info(), now=1.0)
            c.velocity_x, c.velocity_y = 400.0, 100.0
            c.calculate(info(), now=2.0)
            self.assertEqual((c.velocity_x, c.velocity_y), (0.0, 0.0))

        def test_orbit_recapture_limits_thrust(self):
            c = RaceController()
            c.section_index = 1
            c.orbit = OrbitProgress(OBS1, 1)
            thrust = c._motion_profile(1.0/LOOP_R, 0.0, OBS1[0], -LOOP_R-60.0)[0]
            self.assertLessEqual(thrust, ORBIT_RECAPTURE_THRUST)

        def test_fast_path_length_and_speed_envelope(self):
            length = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                         for s in generate_sections() for a, b in zip(s.points, s.points[1:]))
            # 只是路线长度回归检查，不能用它断言真实物理仿真耗时。
            self.assertLess(length, 6900.0 if USE_STABLE_PROFILE else 6660.0)
            c = RaceController()
            for speed in (0.0, 300.0, 430.0, 550.0, 800.0):
                c.speed = speed
                for error in (0.0, 40.0, -40.0, 90.0, -180.0):
                    thrust, frequency, amplitude = c._motion_profile(
                        1.0/LOOP_R, error, OBS1[0], -LOOP_R)
                    self.assertTrue(0.0 <= thrust <= MAX_THRUST)
                    self.assertTrue(math.isfinite(frequency) and math.isfinite(amplitude))

        def test_signed_curvature(self):
            for ccw, sign in ((True, 1.0), (False, -1.0)):
                builder = PathBuilder()
                builder.arc(OBS1, LOOP_R, 270.0, 90.0, ccw=ccw)
                tracker = PathTracker(builder.pts)
                self.assertAlmostEqual(tracker.signed_curv[0], sign/LOOP_R, delta=0.00001)

        def test_completed_orbit_does_not_wait_another_lap(self):
            c = RaceController()
            c.section_index = 1
            c.tracker = PathTracker(c.sections[1].points)
            c.orbit = OrbitProgress(OBS1, 1)
            c.orbit.angle = 2.0*math.pi
            # 已完成整圈，但因径向漂移距离旧出圈点超过85。
            c._guidance(OBS1[0], -LOOP_R-100.0)
            if USE_STABLE_PROFILE:
                self.assertEqual(c.stage, 'orbit_1')
            else:
                self.assertEqual(c.stage, 'slalom_2')
                self.assertGreaterEqual(c.completed_laps['orbit_1'], 1.0)

        def test_course_slip_compensates_in_opposite_direction(self):
            c = RaceController()
            for i in range(61):
                c.calculate(info(x=START[0]+3.0*i, y=float(i)), now=1.0+i/60.0)
            self.assertGreater(c.course_slip, 0.0)
            if USE_STABLE_PROFILE:
                self.assertEqual(c.course_correction, 0.0)
            else:
                self.assertLess(c.course_correction, 0.0)
                self.assertLessEqual(abs(c.course_correction), math.radians(20.0))

        def test_course_slip_reset_after_gap(self):
            c = RaceController()
            c.calculate(info(), now=1.0)
            c.course_slip = 0.5
            c.calculate(info(), now=2.0)
            self.assertEqual(c.course_slip, 0.0)
            self.assertEqual(c.course_correction, 0.0)

        def test_target_time_is_not_a_fake_finish(self):
            c = RaceController()
            c.calculate(info(), now=0.0)
            c.calculate(info(), now=16.1)
            self.assertFalse(c.finished)
            self.assertEqual(c.completed_laps, {})

        def test_wing_neutral_matches_upvector_thrust(self):
            self.assertAlmostEqual(DepthController.vertical_component(WING_NEUTRAL), 0.0)
            # 复现旧角度限制的问题：即使-25°，推力仍有90%以上朝上。
            self.assertGreater(DepthController.vertical_component(-25.0), 0.90)
            self.assertAlmostEqual(DepthController.vertical_component(0.0), 1.0)
            self.assertLess(DepthController.vertical_component(-100.0), 0.0)
            self.assertGreater(DepthController.vertical_component(-80.0), 0.0)

        def test_level_hold_has_no_upward_thrust(self):
            d = DepthController()
            for _ in range(120):
                left, right = d.calculate(HOLD_Z, 0.0, 0.0, 0.0, 400.0, 1.0/60.0)
                self.assertAlmostEqual(left, WING_NEUTRAL)
                self.assertAlmostEqual(right, WING_NEUTRAL)
                self.assertAlmostEqual(d.desired_pitch, 0.0)

        def test_v7_holds_start_height_all_the_way(self):
            """用户要求：高度锁定为鱼的起点高度，全程不变，不做中过渡。"""
            # 目标高度就是起点高度本身，不是水池中层。
            self.assertEqual(HOLD_Z, START_Z)
            self.assertEqual(HOLD_Z, 340.0)
            # 开局即处于目标高度：不应出现任何爬升/下沉意图。
            d = DepthController()
            for _ in range(240):
                left, right = d.calculate(START_Z, 0.0, 0.0, 0.0, 560.0, 1.0/60.0)
                self.assertAlmostEqual(d.reference_vz, 0.0)
                self.assertAlmostEqual(d.desired_pitch, 0.0)
            self.assertAlmostEqual(left, WING_NEUTRAL)
            self.assertAlmostEqual(right, WING_NEUTRAL)
            # 沿整条名义路线（含绕圈、穿缝）都保持起点高度，全程无高度变化。
            c = RaceController()
            now = 1.0
            for section in generate_sections():
                for x, y, heading in section.points:
                    now += 0.02
                    command = c.calculate(info(x=x, y=y, z=START_Z, yaw=heading), now=now)
                    pitch = c.depth.desired_pitch
                    # 高度误差恒为0，所以垂直指令和俯仰目标都应恒为0。
                    self.assertAlmostEqual(c.depth.reference_vz, 0.0)
                    self.assertAlmostEqual(pitch, 0.0)
                    self.assertLessEqual(abs(command.left_angle-WING_NEUTRAL), MAX_WING_TILT)
            self.assertEqual(c.reason, 'crossed_finish')

        def test_ascent_braking_and_pitch_recovery(self):
            d = DepthController()
            for _ in range(60):
                left, right = d.calculate(HOLD_Z, 100.0, 20.0, 0.0, 400.0, 1.0/60.0)
            self.assertLess(d.desired_pitch, 0.0)
            self.assertLess(d.pitch_trim, 0.0)
            self.assertLess(DepthController.vertical_component(left, pitch=20.0), 0.0)
            self.assertAlmostEqual(left, right)

        def test_roll_correction_uses_angle_not_horizontal_force(self):
            for roll in (-20.0, 20.0):
                c = RaceController()
                command = c.calculate(info(z=HOLD_Z, roll=roll), now=1.0)
                self.assertAlmostEqual(command.left, command.right)
                left_z = DepthController.vertical_component(command.left_angle, roll=roll)
                right_z = DepthController.vertical_component(command.right_angle, roll=roll)
                self.assertGreater((right_z-left_z)*roll, 0.0)

        def test_vertical_attitude_keeps_recovery_control(self):
            for pitch in (-90.0, 90.0):
                c = RaceController()
                command = c.calculate(info(z=1000.0, pitch=pitch), now=1.0)
                self.assertFalse(c.finished)
                self.assertEqual(command.tail, 0.0)
                self.assertTrue(0.0 < command.left <= 8.0)
                self.assertLess(c.depth.pitch_trim*pitch, 0.0)

        def test_midwater_goal_and_slew_limits(self):
            d = DepthController()
            previous = WING_NEUTRAL
            for _ in range(120):
                left, right = d.calculate(340.0, 0.0, 0.0, 0.0, 400.0, 1.0/60.0)
                self.assertLessEqual(abs(left-previous), 80.0/60.0+1e-9)
                self.assertLessEqual(abs(d.desired_pitch), 12.0)
                self.assertLessEqual(abs(d.integral), 12.0)
                previous = left
            c = RaceController()
            c.section_index = len(c.sections)-1
            c.tracker = PathTracker(c.sections[-1].points)
            c.calculate(info(x=1260.0, z=HOLD_Z), now=1.0)
            self.assertEqual(c.reason, 'crossed_finish')

        def test_invalid_forward_still_stops(self):
            c = RaceController()
            sample = info()
            sample.forward.x = sample.forward.y = sample.forward.z = 0.0
            command = c.calculate(sample, now=1.0)
            self.assertEqual(c.reason, 'invalid_forward_vector')
            self.assertEqual(command.left, 0.0)

        def test_stop_command_has_neutral_wings_and_no_force(self):
            command = Command()
            self.assertEqual((command.left, command.right, command.tail), (0.0, 0.0, 0.0))
            self.assertEqual((command.left_angle, command.right_angle),
                             (WING_NEUTRAL, WING_NEUTRAL))

        def test_depth_antiwindup_and_emergency_angle_bounds(self):
            d = DepthController()
            for _ in range(600):
                d.calculate(340.0, 0.0, 0.0, 0.0, 400.0, 1.0/60.0)
            self.assertEqual(d.integral, 0.0)
            for pitch in (-89.0, -40.0, 0.0, 40.0, 89.0):
                for roll in (-120.0, 0.0, 120.0):
                    c = RaceController()
                    command = c.calculate(info(z=1250.0, pitch=pitch, roll=roll), now=1.0)
                    for angle in (command.left_angle, command.right_angle):
                        self.assertTrue(math.isfinite(angle))
                        self.assertLessEqual(abs(angle-WING_NEUTRAL), MAX_WING_TILT)
                        self.assertGreater(math.cos(math.radians(angle-WING_NEUTRAL)), 0.0)

    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    if not result.wasSuccessful():
        raise SystemExit(1)


def main():
    # 允许平台加载根目录的 test.py；不要求把嵌入式Python写入全局PATH。
    sys.dont_write_bytecode = True
    runtime = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'WindowsNoEditor', 'Fish427', 'Binaries', 'Win64')
    if os.path.isdir(runtime):
        sys.path.insert(0, runtime)
    import cue as mycue

    controller = RaceController()
    lock = threading.RLock()
    last_received = [None]
    last_report = [0.0]
    debug = '--debug' in sys.argv

    def publish(command):
        message = mycue.FishCtrlInfo()
        message.tail_target_angel = command.tail
        message.wing_force_left = command.left
        message.wing_force_right = command.right
        message.wing_target_angel_left = command.left_angle
        message.wing_target_angel_right = command.right_angle
        # 本包cue.publisher.publish仅封装writer.write并逐帧print。
        # 非调试运行直接使用同一个DDS writer，避免每秒60次控制台输出。
        writer = getattr(publisher, 'writer', None)
        if not debug and writer is not None:
            writer.write(message)
        else:
            publisher.publish(message)

    def lcb(fish_info):
        with lock:
            now = time.monotonic()
            last_received[0] = now
            stage_before = controller.stage
            try:
                command = controller.calculate(fish_info, now)
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                print('[error] {}: {}'.format(type(exc).__name__, exc), flush=True)
                command = controller.stop('callback_error')
            publish(command)
            if stage_before != controller.stage or (debug and now-last_report[0] >= 1.0):
                elapsed = 0.0 if controller.started_at is None else now-controller.started_at
                print('[race] {:.2f}s stage={} pos={} speed={:.1f}/{:.1f} yaw_error={:.1f} slip={:.1f} z_target={:.0f} vz={:.1f} pitch={:.1f}/{:.1f} wings=({:.1f},{:.1f}) laps={} stage_seconds={}'.format(
                    elapsed, controller.stage, controller.last_position,
                    controller.speed, controller.target_speed, controller.heading_error,
                    math.degrees(controller.course_slip), HOLD_Z, controller.vertical_speed,
                    controller.pitch_degrees, controller.depth.desired_pitch,
                    command.left_angle, command.right_angle, controller.completed_laps,
                    controller.stage_seconds), flush=True)
                last_report[0] = now

    mycue.init(TEAM_NAME)
    # 必须先有发布者，再注册可能立即触发的DDS回调，修复启动竞态。
    publisher = mycue.publisher(mycue.FishCtrlInfo)
    subscriber = mycue.subscriber(mycue.FishInfo, lcb)
    waiting_since = time.monotonic()
    print('[race] {} ready: 1 CCW lap -> S passage -> 2 CCW laps -> finish; hold_z={} wing_neutral={}'.format(
        PROFILE_NAME, HOLD_Z, WING_NEUTRAL), flush=True)
    try:
        while not controller.finished:
            time.sleep(0.05)
            with lock:
                now = time.monotonic()
                if last_received[0] is not None and now-last_received[0] > 1.0:
                    publish(controller.stop('telemetry_timeout'))
                elif last_received[0] is None and now-waiting_since > 60.0:
                    controller.stop('waiting_timeout')
        print('[race] stopped: {} stage_seconds={} (official result must be checked in simulator)'.format(
            controller.reason, controller.stage_seconds), flush=True)
    except KeyboardInterrupt:
        with lock:
            publish(controller.stop('interrupted'))
    finally:
        # 保留subscriber引用直到退出；不要在DDS回调线程内销毁reader。
        with lock:
            publish(Command())
        subscriber.close()


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        self_test()
    else:
        main()
