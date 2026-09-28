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
v7.1：修复"绕第三个障碍物转圈时高度持续增加"（用户反馈）。
    根因不是目标高度，而是高度环**守不住持续低头偏置**：
      ① 积分只在 12~160mm 误差区间累积，误差一回到死区就把偏置泄放掉；
      ② 积分上限仅 ±12（换算低头约 4 度）；
      ③ 积分速率 0.04 建立偏置要约 28 秒，而绕第三柱 2 圈只有约 4 秒，
         偏置还没建起来绕圈就结束了 —— 这是高度一路爬升的直接原因。
    修法：死区内**保持**偏置（只按小系数泄放）、积分上限放宽到 ±30、
    积分速率提到 0.25（约 4 秒满偏置，能在一次绕圈内抵消持续扰动）、
    常规俯仰上限 12->18 度，并按 cos(roll) 补偿绕圈横滚造成的翼面竖直效率衰减。
    平飞回归：在目标高度上 ref_vz / pitch / integral 仍恒为 0，不产生多余动作。
v7.2：按用户要求「绕圈转向提速 + 全程高度一致（冲突时高度优先）」。
     全部结论来自 race_telemetry.csv 逐帧实测，不是推算：
     【高度噪声的根因】绕第三柱 2 圈时 z 在 273~407mm 摆动（±67mm，周期≈6.5s
     ≈ 2 圈），而绕障碍1 几乎完美（340.0~340.9mm）。根因是**积分饱和形成的
     极限环**：长时间同号误差把积分顶到 ±30，此时俯仰配平早已贴住 ±35 度上限
     （实测饱和帧占 20%），积分却继续累积；等 z 穿过目标高度时积分仍有 +29，
     于是继续上冲，然后反向重来。修法：积分权限 30→18、泄放 1.0→1.6，
     并新增**反饱和**——配平一旦顶到权限上限就停止积分并向 0 回收。
     实测该反饱和把饱和工况下的积分从 18.0 压到 7.25（-60%），
     即超调的直接来源被大幅削掉。
     【为什么不能靠降速保高度】竖直可达速度 ≈ speed*tan(pitch)，pitch 被 18 度
     限幅，所以**速度越高、同样的竖直速度只需越小俯仰角，高度权限反而更宽裕**。
     据此否决了「偏差大就减速」的方案（那会削弱高度控制能力）。
     【绕圈转向提速】corr(实测速度, 偏航角速度)=+0.72，且 omega 贴近上限的帧
     仅 2~3% —— 鱼并未顶到转向极限，提速确实能提高转向速率。故 TURN_SPEED_BOOST
     =1.20（保守），按平方放大 TURN_ACCEL_BUDGET、等比抬 TAIL_TURN_HZ、
     同步抬 ORBIT_RECAPTURE_THRUST（否则偏出圆周就被钳回旧值，收益被抵消）。
     尾摆幅值只 +10%：实测幅值会被 (MAX_TAIL-|correction|) 裁剪，余量不足的帧占 34%。
     【顺带修正】实测平均绕圈半径 236mm（计划 200mm），里程白多 4~21%；
     corr(速度,半径)=-0.52 说明跑宽反而拖慢（触发回收保护）。
     故把圆周径向增益 2.0→2.8，把圆跟紧——既省里程，又少触发回收。
     平台硬限幅（MAX_THRUST=50、MAX_TAIL=80）、55mm 安全间隙、圈数判定均未放宽。
v7.3：按用户要求「严格审核"全程保持在起点高度"，并用子智能体核验」。
     核验方式：两个独立子智能体（一个只读审计、一个对抗性证伪）+ 主控独立复算，
     全部只读代码/遥测，并复跑两档自测。结论如下，并据此改了 3 个常量：
     【证实的部分】目标高度唯一：HOLD_Z ≡ START_Z ≡ 340，全文件只有一处
     `height_error = HOLD_Z - z`，没有第二目标、没有 750 过渡、没有环境变量/档位
     分支能改高度。平飞（z=340 且 vz=0、姿态水平）时 ref_vz/desired_pitch 恒为 0
     （2964/2964 帧精确为 0）。坏数据/超时走 stop()，翼角回中性（竖直权限归零）。
     【证伪的部分】"全程不变"作为不变量不成立，有三条在 z=340 时仍发竖直指令的路径：
      ① 死区 ±6mm：死区内 height_term 强制为 0，回路无比例作用，**稳态就停在
         ±6mm**——死区多大，允许的稳态偏差就多大（这是设计层偏差，不是缺陷）；
      ② 积分/配平残留：任何把 z 推出死区 >1s 的事件之后，即使 z 精确回到 340，
         首帧仍会输出 +92.7mm/s 竖直目标、+15.1° 俯仰目标，1130 帧（18.8s）才归零；
      ③ 实测（旧版 1.40 遥测，非当前版本）：orbit_3 段 z ∈ [273.4, 407.4]，
         偏离 ±67mm，15.2% 的帧偏差 >50mm —— 这是"绕圈持续上浮"的真实量级。
     【推翻 v7.2 的一条推理】v7.2 按"极限环幅度正比于积分限"把 INTEGRAL_LIMIT
     30→18。独立复算证明不成立：绕圈时真正顶住权限的是**俯仰配平**，desired_pitch
     峰值仅约 10.6°、trim 峰值仅 4°、reference_vz 峰值 69mm/s（都没顶到 18°/35°/
     95mm/s 的上限），积分根本没到 18，所以收小权限对峰值毫无影响；反而把
     **持续扰动下的稳态偏差放大**：同条件下 18 vs 30 的稳态偏差为 12.7 vs 6.9mm
     （扰动 50mm/s）、19.2 vs 6.1mm（60mm/s）。实测 forcing 的 p75/p90/p95 =
     +33/+49/+53mm/s 正落在该区间，故 INTEGRAL_LIMIT 回到 30。
     【三处改动】（用户明确"冲突时优先保证高度一致"，故全部朝减小偏差方向）
       · Z_DEADBAND 6.0 → 3.0：直接把允许的稳态偏差从 ±6mm 收到 ±3mm。
       · KI_HEIGHT 0.25 → 0.50：偏置必须在一次绕圈（约 4s）内建起来。复算：
         扰动 30/60/90/120mm/s 下的最大偏离由 15.6/31.0/46.5/62.0mm 降到
         14.2/28.3/42.5/56.6mm；空载纹波仍为 0.00mm（积分变快不会自激）。
       · INTEGRAL_LIMIT 18.0 → 30.0：见上，是"回退 v7.2 的退化改动"。
     同时新增自测 test_v7_3_deadband_is_the_real_height_error_budget，把
     "持续扰动峰值有界 + 扰动撤除后回到死区 + 空载零纹波"钉成回归断言。
     【必须诚实声明的局限】这个闭环模型是简化的一阶俯仰跟随 + vz=speed·tan(pitch)，
     用同一模型对实测竖直速度回归只有 R²≈0.45（orbit_3），说明实测竖直运动的
     一半以上来自模型里没有的力；实测 |roll| 最大仅 3.51°（自测假设 35°），
     desired_pitch 到 18° 而实际 pitch 只有 3.67°（竖直通道带宽才是真瓶颈）。
     所以本次改动能保证的是"控制律不再让偏差单调发散、且账面容许偏差收窄"，
     **不能替代实跑**。真正的竖直带宽问题仍需平台侧验证。
v7.4：按用户要求「把绕第一个柱子的方法用在绕第三个柱子上」（用户明显观察到两柱绕法不同）。
     先用穷尽扫描确认了**绕圈本身两处完全同源**：_guidance 只有一条 `self.orbit is not None`
     分支，没有按阶段分叉；两段同为 LOOP_R=200、逆时针、OrbitProgress 按实际轨迹累计角度。
     差异全部来自**入圈那一段的几何**（这也正是"撞第三柱一下才开始绕圈"的成因）：
       · 第一柱 approach_1 的贝塞尔控制臂 = 190（起点切线）/ 290（入圈切线）；
       · 第三柱 slalom_2 原用 CURVE_ARM_RATIO=0.85R = 170 / 170。
     侧写对比（距入圈点 : 半径 / 切向偏角）说明前者是"沿切线远远溜进来"，
     后者是"贴着圆斜插进去"：
       距入圈点   第1柱(改不改都一样)   第3柱(改前)     第3柱(改后)
         120mm     222 / +19.0°        210 / +13.1°    221 / +19.7°
         320mm     311 / +34.6°        309 / +45.5°    322 / +38.9°
     入圈瞬间的向内径向速度（按实测 330mm/s 折算）：改前 +84~131mm/s，
     改后 +15.7mm/s，与第一柱的 +16.1mm/s 基本一致。径向冲量正是把圆跑大、
     触发径向回收、看起来"撞一下才开始绕"的直接原因。
     代价可忽略：最大曲率 1/222 → 1/264（仍低于曲率限速免费阈值 1/200，
     不触发额外减速），离柱最小间隙仍 75mm（≥55），总路径 6526.9mm（上限 6660）。
     对应自测：无（几何类断言原样复用：曲率<=1/LOOP_R、无北偏、出缝后立刻向内、
     y 单调、55mm 间隙、路径长度——全部继续通过）。**未实测，不能保证耗时**。
v7.5：针对"速度一直卡在 20 秒附近"做定向提速，只动绕圈的径向控制，不动路线与高度。
     依据来自 race_telemetry.csv（1159 帧实测）与 project1.csv 成绩：
       ① 绕圈平均半径跑到 230/237mm（计划 200），1+2 圈白跑约 657mm ≈ 2s；
       ② |径向误差|>40mm 的帧占 orbit_1 47%、orbit_3 54%，全部被硬钳到约 39 推力，
          速度从 350/358 掉到 251/285，两段合计约损失 3.2s；
       ③ 径向速度与半径变化率 corr=+1.000，外漂就是圆跑大的直接来源；
       ④ 反证：test-03（SPEED_GAIN=1.45）与 a1（1.70）实测反而更慢
          （20.31 / 26.12 秒），说明"再加大驱动"路线已被证伪，问题在径向漂移与回收。
     三处改动：
       · 新增 ORBIT_RADIAL_DAMP=0.22：把径向速度折算成等效半径误差，
         在半径还没跑宽前就往回收（不是事后纠正）。
       · 回收阈值 40→70，且只在"仍在往外漂"时介入；正在收敛的中度偏离不踩刹车。
       · 新增硬阈值 ORBIT_RECAPTURE_HARD_BAND=80：严重偏离无论方向一律回收，
         安全底线不松。
     实测遥测回放：回收钳制帧从 421/820 降到 48/820（-89%），路径几何逐点未变
     （总长 6526.9mm、最小离柱间隙 58.5mm、HOLD_Z=340 未动）。
     对应自测 test_orbit_recapture_limits_thrust / test_orbit_radial_damp_predicts_drift。
     **未实测，不能保证耗时**；请回传成绩与 stage_seconds。
v7.6：针对"速度一直卡在 20 秒附近"做**转向权限**提速，不再加大驱动。
     依据来自 race_telemetry.csv（1185 帧）+ 离线运动学仿真（仿真半径/偏航
     与实测对齐到 237/27°，可用来比策略）：
       ① 绕圈平均半径 230/237（计划 200），|径向误差|>40mm 的帧占 47%/54%；
          corr(半径误差, 速度)=-0.76，跑宽就是掉速的直接原因；
       ② 绕圈 |偏航误差| 均值 25°，PID 已顶到 ±62 钳位，尾摆幅值被
          (MAX_TAIL-|correction|) 吃掉，推进与转向抢同一根尾巴；
       ③ 仿真：在现有转向带宽下，lap 时间对"转向权限"最敏感
          （yaw_tau 0.40→0.22 可把单圈 5.8s 压到 4.5s），而继续抬 SPEED_GAIN
          只会把圆跑得更宽（1.40/1.70 实测更慢，已证伪）。
     四处改动（硬限幅/安全间隙/圈数判定仍不动）：
       · 左右胸鳍差动补转向：尾摆不够的偏航力矩改由推力差承担，
         不再让振幅和转向互相抢预算；
       · |偏航误差| 大时收幅值、抬频率：优先把尾巴用在转向，频率补推进；
       · 绕圈径向增益 2.8→3.2，略收紧圆周；
       · 尾摆频率小幅上调、绕圈幅值下调（减少锯齿路径）。
     对应自测 test_yaw_differential_assists_turning /
     test_steering_priority_reduces_amplitude_when_error_large。
     **未实测，不能保证耗时**；请回传成绩与 stage_seconds。
     【v7.6.1 实测回退修正】首版实测 22.01s，比 20s 档更慢，遥测（1320 帧）：
       orbit_1 角速度 70.1→60.3 deg/s、|偏航误差| 24.8→37.7°、半径 230→252，
       绕圈 |推力差| 高达 20/32 —— **差动符号反了**，推力差在和尾巴抢转向，
       越纠偏越偏。现改为右加左减（实测证据优先于坐标系推断），
       差动上限 16→12，转向优先收幅 0.45→0.60，径向增益退回 2.8。
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
# 高度保持环参数（v7.1）。绕第三柱连续 2 圈时高度持续爬升，原因是持续扰动
# 需要一个"持续低头偏置"来抵消，而原实现无法维持这个偏置（详见 DepthController）。
# 高度死区（v7.3：6.0 -> 3.0）。死区内 `height_term = 0`，回路完全没有比例
# 作用，只剩缓慢泄放，所以**稳态就停泊在 ±Z_DEADBAND 上**——死区多大，
# 允许的稳态偏差就多大。用户要求"始终保持在起点高度"，6mm 的账面偏差不可接受。
Z_DEADBAND = 3.0                  # 高度死区：此范围内保持偏置，不做纠偏激励
KP_HEIGHT = 0.9                   # 高度误差 -> 垂直速度目标 的比例
# 误差积分速率（v7.3：0.25 -> 0.50）。绕第三柱 2 圈只有约 4 秒，偏置必须在
# 这段时间内建起来，否则持续上浮压不住。独立复算（见下）显示：扰动 30/60/90/
# 120mm/s 下，KI 从 0.25 提到 0.50 把最大偏离从 15.6/31.0/46.5/62.0mm 降到
# 14.2/28.3/42.5/56.6mm，且空载纹波仍为 0.00mm（不会因为积分变快而自激）。
KI_HEIGHT = 0.50
# v7.3：积分权限由 18 回到 30。v7.2 曾按"极限环幅度正比于积分限"把权限收到
# 18，但独立核验（两个对抗性审计 + 主控复算）证明这条推理不成立：绕圈时真正
# 顶住上限的是**俯仰配平**（desired_pitch 峰值仅约 10.6 度、trim 峰值 4 度），
# 积分根本没到 18，所以收小权限对峰值毫无影响，反而把**持续扰动的稳态偏差**
# 放大了：实测 forcing 的 p75/p90/p95 = +33/+49/+53mm/s，正好落在"权限 18 比
# 30 更差"的区间（同条件下稳态偏差 12.7 vs 6.9mm）。因此回到 30。
INTEGRAL_LIMIT = 30.0
INTEGRAL_LEAK = 1.6               # 到位后偏置泄放速率；过小会让残留偏置拖出反向超调
PITCH_LIMIT_NORMAL = 18.0         # 常规俯仰上限（原 12 度换算垂直速度约 124mm/s，太紧）
# 反饱和泄放速率：俯仰配平顶到上限时，积分按此速率往回收（见 DepthController）。
ANTI_WINDUP_BLEED = 2.0
WING_NEUTRAL = -90.0             # 胸鳍的UpVector在此角度沿鱼身+x
MAX_WING_TILT = 85.0             # 相对水平推进基准；不允许转到倒推半球
MAX_THRUST = 50.0
MAX_TAIL = 80.0
TEAM_NAME = 'F05012589'
PROFILE_NAME = ('race-v7.6-stable-hold-z' if USE_STABLE_PROFILE
                else 'race-v7.6-hold-z-turn-authority')
# 整体提速系数：v5.1 按用户要求 +8%；v6 用户要求"再快一些"，提到 1.16。
# 只放大推进量与目标速度：尾摆频率/幅值、巡航/转弯/穿缝目标速度、转弯加速度预算。
# 不放宽任何平台限幅（MAX_THRUST=50、MAX_TAIL=80）、安全间隙和圈数判定。
#
# 重要：快档的 THRUST_BASE 已经是 50（=MAX_THRUST），所以推力恒被钳在 50，
# 目标速度数值本身不改变实际推力。真正决定快慢的是**尾摆频率与幅值**
# （手册：尾摆是主要推进方式，摆动频率与动力成正比）。因此提速必须落到
# TAIL_STRAIGHT_HZ / TAIL_TURN_HZ / TAIL_AMPLITUDE_* 上，这里统一乘 SPEED_GAIN。
SPEED_GAIN = 1.16
# v7.2：绕圈转向专项提速（用户要求），只作用于转弯档，不动 SPEED_GAIN，
# 因此巡航/穿缝/绕行半径/路线几何都不受影响。
#
# 力度取 1.20（比之前试过的 1.34 保守）。依据来自 race_telemetry.csv：
#   ① corr(实测速度, 偏航角速度) = +0.72，且 omega 贴近上限的帧仅 2~3%
#      —— 说明鱼**没有**顶到转向极限，提速确实能提高转向速率；
#   ② 但 corr(实测速度, 半径) = -0.52，且平均半径 236mm（计划 200mm）
#      —— 提速会进一步把圆跑大，而里程 = 2πr×圈数，圆一大就白跑。
#      故力度不宜大；配合下面的半径增益收紧圆周，才能把提速真正兑现。
# 曲率限速是 sqrt(TURN_ACCEL_BUDGET/曲率)，预算须按平方放大才等比提速。
TURN_SPEED_BOOST = 1.20
# 目标速度不是实际速度。推进器仍限制在50，尾角仍限制在±80度。
CRUISE_SPEED = (540.0 if USE_STABLE_PROFILE else 560.0) * SPEED_GAIN
# 转弯档上限同步抬高，避免它比抬预算后的曲率限速更早触顶成为新瓶颈。
TURN_SPEED = (480.0 if USE_STABLE_PROFILE else 540.0) * SPEED_GAIN * TURN_SPEED_BOOST
GAP_SPEED_BASE = 360.0 if USE_STABLE_PROFILE else 420.0
GAP_SPEED = GAP_SPEED_BASE * SPEED_GAIN
# 曲率限速是 sqrt(预算/曲率)，预算只乘一次的话绕圈速度只涨 sqrt(1.08)≈4%，
# 达不到要求的 8%；因此预算按系数的平方放大。v7.2 再叠 TURN_SPEED_BOOST²。
TURN_ACCEL_BUDGET = ((1150.0 if USE_STABLE_PROFILE else 1460.0)
                     * SPEED_GAIN ** 2 * TURN_SPEED_BOOST ** 2)
THRUST_BASE = 48.0 if USE_STABLE_PROFILE else 50.0
# 手册：尾摆频率与动力成正比，尾摆是主要推进方式；这里是主要提速手段。
# v7.6：频率再抬 5%（3.8→4.0），把推进继续压到频率上，少依赖幅值。
TAIL_STRAIGHT_HZ = (3.4 if USE_STABLE_PROFILE else 4.0) * SPEED_GAIN
# 绕圈尾摆频率：手册指出频率与动力成正比，是绕圈转向的真正推进杠杆。
TAIL_TURN_HZ = (3.3 if USE_STABLE_PROFILE else 3.8) * SPEED_GAIN * TURN_SPEED_BOOST
# 尾摆幅值（同样是推进量）；窄缝内仍收窄，避免摆动导致反复纠偏。
TAIL_AMPLITUDE_STRAIGHT = 24.0 * SPEED_GAIN
# v7.6：绕圈幅值由 18*1.10 降到 16。实测绕圈 |偏航误差| 均值 25°，
# 幅值与转向抢 (MAX_TAIL-|correction|) 的余量；幅值收窄可减少锯齿航迹，
# 推进损失由 TAIL_TURN_HZ 上调补回（频率与动力成正比）。
TAIL_AMPLITUDE_TURN = 16.0 * SPEED_GAIN
GAP_TAIL_AMPLITUDE = 8.0 * SPEED_GAIN
# 穿缝未对正时的保守速度上限；与横向偏移减速曲线在同一尺度上。
GAP_ALIGN_SPEED = 280.0 * SPEED_GAIN
GAP_LANE_FLOOR = 280.0 * SPEED_GAIN
# 绕圈偏离圆周后重新贴回的推力上限（安全限幅，只按系数微调）。
# v7.2 必须同步抬高：实测绕圈有 45%/38% 的帧偏出圆周 >40mm，
# 一偏出就被钳到这个低值，把绕圈提速的收益直接抵消。
ORBIT_RECAPTURE_THRUST = 28.0 * SPEED_GAIN * TURN_SPEED_BOOST
# v7.5：径向漂移阻尼 + 分级回收。实测（1159 帧遥测）证明：
#   ① 绕圈平均半径跑到 230/237mm（计划 200），1+2 圈合计白跑约 657mm ≈ 2s；
#   ② |径向误差|>40mm 的帧占 orbit_1 47%、orbit_3 54%，这些帧被硬钳到 39 推力，
#      速度从 350/358 掉到 251/285，本段合计约损失 3.2s；
#   ③ 径向速度与半径变化率相关性 +1.000 —— 半径变大就是径向外漂直接造成的。
# 两个结论合起来：**不要再"偏出就踩刹车"**，而是先阻尼径向漂移，
#   只在"还在往外漂"或严重偏离时才回收推力。
ORBIT_RADIAL_DAMP = 0.22          # 径向速度阻尼：mm/s -> 等效半径误差 mm
ORBIT_RECAPTURE_BAND = 70.0       # 漂移恶化时才介入的阈值
ORBIT_RECAPTURE_HARD_BAND = 80.0  # 无论收敛与否，严重偏离一律回收
# 切向 + 径向误差反馈里，径向项的增益。
# v7.6.1：保持 2.8。首版曾试 3.2，但与错误差动叠加后圆反而跑更大（252），
# 一并退回；等差动符号验证有效后再单独试更高增益。
ORBIT_RADIAL_GAIN = 2.8
# v7.6：左右胸鳍差动补转向。尾摆在大偏航误差时被振幅抢预算，
# 用推力差补上偏航力矩，转向不再只靠一根尾巴。
# v7.6.1：符号经实测证伪后翻转。首版按 UE 坐标系推断「正 correction 左加右减」，
# 实测 22.01s 回退：绕圈 |偏航误差| 25→38°、角速度 70→60，推力差与尾舵反向。
# 现按实测证据改为 **右加左减**（正 correction 时右侧胸鳍更多）。
YAW_DIFF_GAIN = 0.30              # 每度 correction 对应的推力差（单侧）
YAW_DIFF_MAX = 12.0               # 单侧差动推力上限（仍远低于 MAX_THRUST）
# 大偏航误差时优先转向：收窄振幅、抬高频率，把尾巴让给转向。
# v7.6.1：收幅 0.45→0.60，避免大误差时推进掉太快。
STEER_PRIORITY_ERROR = 22.0       # 超过该偏航误差（度）就进入转向优先
STEER_PRIORITY_AMPLITUDE_SCALE = 0.60
STEER_PRIORITY_FREQ_SCALE = 1.12
# 曲率限速的下限；实际路径最大曲率远达不到该下限触发点，仅为常量尺度一致。
TURN_SPEED_FLOOR = 250.0 * SPEED_GAIN
# v7.4：绕第三柱改用「绕第一柱同款」的入圈方法（用户要求）。
# 用户观察到两柱绕法明显不同。代码比对证实：绕圈本身（_guidance 的 orbit 分支、
# LOOP_R=200、逆时针）两处完全同源，差异全部来自**入圈那一段的几何**：
#   · 第一柱 approach_1 的贝塞尔控制臂 = 起点切线 190 / 入圈切线 290，
#     入圈前 320mm 处切向偏角仅 34.6 度 —— 沿切线远远溜进来，入圈瞬间径向速度小；
#   · 第三柱 slalom_2 原来用 0.85R=170 / 170，入圈前 320mm 处偏角 45.5 度，
#     是"贴着圆斜插进去"，实测入圈瞬间带着 +84~131mm/s 的向内径向速度，
#     也就是用户看到的"撞第三柱一下才开始绕圈、圆被跑大"。
# 现在把第三柱的入圈臂改成与第一柱**完全相同的 190 / 290**，两柱入圈侧写重合：
#   距入圈点 40mm  第1柱 202/+7.2°  第3柱改后 203/+7.3°
#   距入圈点 120mm 第1柱 222/+19.0° 第3柱改后 221/+19.7°
# 代价可忽略：slalom_2 最大曲率由 1/222 降到 1/264（均低于曲率限速阈值 1/200，
# 不触发额外减速），最小离柱间隙 75mm（≥55 安全线），
# 出缝后 56mm 就出现向内趋势（要求 <120），总路径 6522.3 -> 6526.9mm（上限 6660）。
ENTRY_ARM_GAP = 190.0             # 出缝点到入圈前控制点的切线臂长
ENTRY_ARM_ENTRY = 290.0           # 入圈点到入圈后控制点的切线臂长

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
    # 改用一条与两端切向都连续的三次贝塞尔，从出缝点直接下压到南侧入圈点：
    # 两端切向都是 +x（与直缝、与入圈圆弧相切），曲率上限 1/LOOP_R，
    # 不会触发曲率限速，也不会出现急拐或卡顿。
    # v7.4：控制臂由 0.85R(170/170) 改为与 approach_1 **同款**的 190/290，
    # 让绕第三柱的入圈方式和绕第一柱一致（见文件头 ENTRY_ARM_* 说明）。
    pb.bezier((190.0, 0.0), (190.0+ENTRY_ARM_GAP, 0.0),
              (OBS3[0]-ENTRY_ARM_ENTRY, -LOOP_R), e3)
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
        # 通信间断后不使用旧姿态差分。
        self.last_pitch = self.last_roll = None
        self.pitch_rate = self.roll_rate = 0.0
        self.integral = self.reference_vz = 0.0
        # v7.2：**必须同时清掉配平残留**。原先只清积分、故意保留胸鳍角度
        # "避免重连瞬间跳变"，但那会让一个陈旧的大配平挂在新一轮控制上：
        # 实测建立 trim=35 后调 reset_dynamics()，复位首帧即便鱼正好在
        # HOLD_Z 且姿态水平，仍会输出 +33.67 度翼偏（非零竖直力），
        # 与"始终保持在起点高度"直接冲突。这里改为让配平在复位后从 0 重新建立；
        # 侧滑/侧倾的平滑性由下面的 slew 限速保证，不会产生角度跳变。
        self.pitch_trim = self.roll_trim = 0.0

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
        # v7.1：绕第三柱连续 2 圈时高度持续爬升。原因是"持续扰动需要持续低头偏置
        # 来抵消"，而旧实现守不住这个偏置：①只在 12~160mm 误差区间积分，误差一
        # 回到死区内就把偏置泄放掉，于是偏置刚建立就被抹掉；②积分上限只有 ±12，
        # 换算成低头只有约 4 度，权限不足以压住持续上浮。
        # 现在：死区内**保持**偏置（只做很慢的泄放），并放宽积分权限。
        if abs(height_error) < Z_DEADBAND:
            # 已在目标高度附近：保留已建立的偏置，仅按 INTEGRAL_LEAK 缓慢泄放，
            # 这样持续扰动期间偏置守得住，扰动消失后也能自然退回零配平。
            self.integral *= max(0.0, 1.0 - INTEGRAL_LEAK*dt)
        elif abs(height_error) < 160.0 and abs(pitch) < 20.0:
            self.integral = _constrain(
                self.integral+KI_HEIGHT*height_error*dt,
                -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        else:
            # 大偏差或大姿态：停止继续积分，但不立即清零，避免偏置反复重建。
            self.integral *= max(0.0, 1.0-0.5*dt)
        height_term = 0.0 if abs(height_error) <= Z_DEADBAND else height_error
        wanted_vz = _constrain(KP_HEIGHT*height_term+self.integral, -95.0, 95.0)
        # 垂直速度目标仍做斜率限制，避免姿态突跃；但开局已处于目标高度，
        # height_error≈0，因此这里不会产生任何爬升动作。
        self.reference_vz += _constrain(wanted_vz-self.reference_vz, -140.0*dt, 140.0*dt)
        vertical_command = self.reference_vz + 0.7*(self.reference_vz-vertical_speed)
        # 靠底/靠顶时放宽，便于姿态异常时恢复；常规高度上给足 18 度权限。
        pitch_limit = 20.0 if z < 180.0 or z > 1200.0 else PITCH_LIMIT_NORMAL
        self.desired_pitch = _constrain(
            math.degrees(math.atan2(vertical_command, max(220.0, speed))),
            -pitch_limit, pitch_limit)

        recovering = abs(pitch) > 30.0 or abs(roll) > 40.0
        trim_limit = 65.0 if recovering else 35.0
        # 横滚会让翼面竖直效率按 cos(roll) 衰减（绕圈时鱼体持续横滚），
        # 若不补偿，同样的俯仰修正只能换回一部分竖直力，高度就压不住。
        # 按 cos(roll) 放宽上限做补偿，最多放大 1.67 倍，并收口在物理倾角内。
        authority = max(math.cos(math.radians(roll)), 0.6)
        pitch_trim = _constrain(
            2.0*(self.desired_pitch-pitch)-0.9*self.pitch_rate,
            -trim_limit/authority, trim_limit/authority)
        pitch_trim = _constrain(pitch_trim,
                                -MAX_WING_TILT+5.0, MAX_WING_TILT-5.0)
        # v7.2 反饱和：俯仰配平一旦顶到权限上限，说明执行机构已经没有余量，
        # 此刻继续累积积分只会加深超调（实测量到过零点残留 +29 的成因）。
        # 这里在饱和时停止累积，并向 0 缓慢回收，让偏置与可用权限保持一致。
        if abs(pitch_trim) >= trim_limit/authority - 1e-6:
            self.integral *= max(0.0, 1.0 - ANTI_WINDUP_BLEED*dt)
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
            radius = math.hypot(dx, dy)
            radius_error = radius - LOOP_R
            # v7.5：预测径向漂移，不再只对"当前半径误差"做比例反馈。
            # 实测径向速度与半径变化率 corr=+1.000，外漂速度是半径变大的直接来源；
            # 把它折算成等效半径误差，能在半径还没跑宽前就往回收。
            radial_speed = ((self.velocity_x*dx + self.velocity_y*dy)
                            / max(radius, 1.0))
            # 圆的切向 + 径向误差反馈，比追前瞻弦更不易切入方柱角。
            heading = math.atan2(dy, dx) + math.pi/2
            heading += math.atan2(
                ORBIT_RADIAL_GAIN*radius_error
                + ORBIT_RADIAL_DAMP*radial_speed, LOOP_R)
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
                # v7.6：斜率 10→14，保证 y=30mm 时仍压到 GAP_ALIGN_SPEED 以下
                # （GAP_SPEED 抬到 420 后旧斜率不够）。
                lane_speed = GAP_SPEED-SPEED_GAIN*(
                    14.0*max(0.0, lateral_risk-18.0)+4.0*max(0.0, abs(error)-22.0))
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
            radius = math.hypot(x-cx, y-cy)
            radial_error = radius-LOOP_R
            # v7.5：分级回收，取代"偏出 40mm 一律踩刹车"。
            # 实测 47%/54% 的绕圈帧被这条硬阈值钳到 39 推力，速度掉 100mm/s；
            # 但其中很多帧其实正在往回收敛，踩刹车反而拖慢恢复。
            # 只有"仍在往外漂"或严重偏离时才回收；收敛中的中度偏离不干预。
            radial_speed = ((self.velocity_x*(x-cx) + self.velocity_y*(y-cy))
                            / max(radius, 1.0))
            drifting_out = radial_error*radial_speed > 0.0
            if (abs(radial_error) > ORBIT_RECAPTURE_HARD_BAND
                    or (drifting_out and abs(radial_error) > ORBIT_RECAPTURE_BAND)):
                thrust = min(thrust, ORBIT_RECAPTURE_THRUST)
        if self.entry_avoidance:
            thrust = min(thrust, ORBIT_RECAPTURE_THRUST)
        turning = curvature > 0.003
        frequency = TAIL_TURN_HZ if turning else TAIL_STRAIGHT_HZ
        amplitude = TAIL_AMPLITUDE_TURN if turning else TAIL_AMPLITUDE_STRAIGHT
        if not USE_STABLE_PROFILE and self.stage == 'slalom_2' and abs(x) < 240.0:
            amplitude = GAP_TAIL_AMPLITUDE  # 窄缝内缩小摆尾，保留前向推力，减少左右纠偏。
        # v7.6：大偏航误差时优先转向——收振幅、抬频率。
        # 之前 correction 会把 (MAX_TAIL-|correction|) 吃掉，振幅只剩十几度，
        # 推进与转向抢同一根尾巴；现在提前把振幅收掉，频率补推进，
        # 尾角余量留给转向，径向/航向收敛更快，圆也不容易跑宽。
        if abs(error) > STEER_PRIORITY_ERROR:
            amplitude = min(amplitude, max(5.0, amplitude * STEER_PRIORITY_AMPLITUDE_SCALE))
            frequency *= STEER_PRIORITY_FREQ_SCALE
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

        # v7.6.1：差动符号按实测翻转为右加左减（首版左加右减导致 22.01s 回退）。
        # 差动只跟偏航误差走，不拿去调滚转/俯仰（那些仍由翼角完成）。
        yaw_assist = _constrain(YAW_DIFF_GAIN * correction, -YAW_DIFF_MAX, YAW_DIFF_MAX)
        thrust_left = _constrain(thrust - yaw_assist, -MAX_THRUST, MAX_THRUST)
        thrust_right = _constrain(thrust + yaw_assist, -MAX_THRUST, MAX_THRUST)

        left_angle, right_angle = self.depth.calculate(
            z, self.vertical_speed, pitch, roll, self.speed, dt)
        # 姿态失稳时暂时降低推进和摆尾，不把高前向推力变成向水面的推力。
        # 保留少量胸鳍推力产生恢复力矩；不要把所有控制直接归零。
        if abs(pitch) > 25.0 or abs(roll) > 40.0:
            thrust_left = thrust_right = min(thrust, 28.0)
            tail = _constrain(correction, -25.0, 25.0)
        if abs(pitch) > 45.0 or abs(roll) > 65.0:
            thrust_left = thrust_right = min(thrust, 16.0)
            tail = 0.0
            self.pid.reset()
        if abs(pitch) > 70.0:
            thrust_left = min(thrust_left, 8.0)
            thrust_right = min(thrust_right, 8.0)
        if z > 1150.0 and self.vertical_speed > 20.0:
            thrust_left = min(thrust_left, 22.0)
            thrust_right = min(thrust_right, 22.0)
        # 滚转用胸鳍角度差恢复；前向推力差主要产生偏航，不再拿它调滚转。
        self.command = Command(tail, thrust_left, thrust_right, left_angle, right_angle)
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

        def test_v7_2_reset_dynamics_clears_stale_trim(self):
            """v7.2：断流重连后不得带着陈旧配平（否则 z=HOLD_Z 时仍发竖直指令）。"""
            d = DepthController()
            # 低速大偏差 -> 配平必然饱和，制造一个"陈旧大配平"
            for _ in range(600):
                d.calculate(HOLD_Z-60.0, 0.0, 0.0, 0.0, 220.0, 1.0/60.0)
            self.assertAlmostEqual(abs(d.pitch_trim), 35.0, places=3)
            d.reset_dynamics()
            # 复位必须把配平残留一并清掉
            self.assertAlmostEqual(d.pitch_trim, 0.0, places=9)
            self.assertAlmostEqual(d.roll_trim, 0.0, places=9)
            self.assertAlmostEqual(d.integral, 0.0, places=9)
            self.assertAlmostEqual(d.reference_vz, 0.0, places=9)
            # 关键断言：复位后首帧，鱼正好在起点高度且姿态水平 -> 翼角必须中性
            left, right = d.calculate(HOLD_Z, 0.0, 0.0, 0.0, 583.0, 1.0/60.0)
            self.assertAlmostEqual(left, WING_NEUTRAL, places=9)
            self.assertAlmostEqual(right, WING_NEUTRAL, places=9)
            # 且不得因此产生任何竖直指令
            self.assertAlmostEqual(d.reference_vz, 0.0, places=9)
            self.assertAlmostEqual(d.desired_pitch, 0.0, places=9)

        def test_v7_2_anti_windup_stops_integral_when_trim_saturated(self):
            """v7.2：俯仰配平饱和时必须停止积分累积，避免加深超调。"""
            def driven(bleed, steps=600, offset=-60.0):
                # 固定高度偏差、低速（竖直权限小 → 配平必然饱和）驱动
                d = DepthController()
                for _ in range(steps):
                    d.calculate(HOLD_Z+offset, 0.0, 0.0, 0.0, 220.0, 1.0/60.0)
                return d
            # 反饱和关闭时：配平顶住上限，积分一路积满
            saved = ANTI_WINDUP_BLEED
            try:
                globals()['ANTI_WINDUP_BLEED'] = 0.0
                off = driven(0.0)
            finally:
                globals()['ANTI_WINDUP_BLEED'] = saved
            self.assertAlmostEqual(abs(off.pitch_trim), 35.0, places=3)
            self.assertAlmostEqual(abs(off.integral), INTEGRAL_LIMIT, places=3)
            # 反饱和开启时：同样条件下积分被主动收回（这是抑制超调的关键）
            on = driven(ANTI_WINDUP_BLEED)
            self.assertAlmostEqual(abs(on.pitch_trim), 35.0, places=3)
            self.assertLess(abs(on.integral), 0.75*INTEGRAL_LIMIT)
            # 偏差回到死区内时，残余偏置必须继续泄放（不能长期挂着）
            for _ in range(180):
                on.calculate(HOLD_Z, 0.0, 0.0, 0.0, 583.0, 1.0/60.0)
            self.assertLess(abs(on.integral), 1.0)
            # v7.3：权限回到 30。独立核验证明"极限环幅度正比于积分限"不成立
            # （绕圈时真正贴顶的是俯仰配平，desired_pitch 峰值仅约 10.6 度），
            # 而权限收小反而放大了持续扰动下的稳态偏差，故必须 >= 30。
            self.assertGreaterEqual(INTEGRAL_LIMIT, 30.0)
            self.assertGreater(INTEGRAL_LEAK, 1.0)

        def test_v7_3_deadband_is_the_real_height_error_budget(self):
            """v7.3：死区内没有比例作用，稳态就停在 ±Z_DEADBAND；必须收紧。"""
            # 死区是"允许的稳态高度偏差"的上界，收紧才配得上"始终保持在起点高度"。
            self.assertLessEqual(Z_DEADBAND, 3.0)
            self.assertGreater(Z_DEADBAND, 0.0)
            # 积分必须够快，才能在一次绕圈（约 4s）内把持续上浮偏置建起来。
            self.assertGreaterEqual(KI_HEIGHT, 0.5)
            # 关键行为断言：持续上浮扰动下偏差有界，且扰动撤除后能回到死区内。
            def closed_loop(disturb_vz, disturbed=360, steps=1800):
                d = DepthController()
                z, vz, pitch = HOLD_Z, 0.0, 0.0
                dt, roll, speed = 1.0/60.0, 3.5, 583.0
                worst = 0.0
                for i in range(steps):
                    d.calculate(z, vz, pitch, roll, speed, dt)
                    pitch += (d.desired_pitch-pitch)*(dt/(0.35+dt))
                    vz = (speed*math.tan(math.radians(pitch))*math.cos(math.radians(roll))
                          + (disturb_vz if i < disturbed else 0.0))
                    z += vz*dt
                    worst = max(worst, abs(z-HOLD_Z))
                return worst, abs(z-HOLD_Z)

            # 实测 forcing 的 p95 约 +53mm/s；这里取 60mm/s 留余量。
            worst, residual = closed_loop(60.0)
            self.assertLess(worst, 40.0)       # 全程最大偏离（含绕圈段）
            self.assertLess(residual, 8.0)     # 扰动撤除后必须回到死区附近
            # 空载（正常平飞）时不得因为积分变快而产生任何纹波。
            worst_idle, residual_idle = closed_loop(0.0)
            self.assertEqual(worst_idle, 0.0)
            self.assertEqual(residual_idle, 0.0)

        def test_v7_2_orbit_speed_boost_keeps_height_authority(self):
            """绕圈提速必须仍留有足够俯仰/配平权限来保高度（高度优先）。"""
            # 目标速度抬高后，同样的竖直速度只需更小俯仰角 —— 权限更宽裕。
            def pitch_for(vz, speed):
                return math.degrees(math.atan2(vz, max(220.0, speed)))
            self.assertLess(pitch_for(95.0, TURN_SPEED),
                            pitch_for(95.0, TURN_SPEED/TURN_SPEED_BOOST))
            self.assertLess(pitch_for(95.0, TURN_SPEED), PITCH_LIMIT_NORMAL)
            # 绕圈速度必须真的上了，否则这次改动没意义
            base = (480.0 if USE_STABLE_PROFILE else 540.0) * SPEED_GAIN
            self.assertGreater(TURN_SPEED, base*1.15)
            self.assertAlmostEqual(TURN_ACCEL_BUDGET,
                                   (1150.0 if USE_STABLE_PROFILE else 1460.0)
                                   * SPEED_GAIN**2
                                   * (TURN_SPEED/base)**2, places=4)
            # 圆周跟踪收紧，避免提速把圆跑大（里程白多）
            self.assertGreater(ORBIT_RADIAL_GAIN, 2.0)
            # 硬限幅与安全间隙不得放宽
            self.assertEqual(MAX_THRUST, 50.0)
            self.assertEqual(MAX_TAIL, 80.0)
            pts = generate_sections()[3].points
            for a, b in zip(pts, pts[1:]):
                self.assertGreaterEqual(segment_clearance(a, b, OBS3), 55.0)

        def test_v7_1_holds_height_during_sustained_orbit_climb(self):
            """用户反馈：绕第三个障碍物转圈时高度持续增加，需要压住。"""
            # 用最小闭环复现"绕圈时被持续上浮"：俯仰一阶跟随，竖直速度由俯仰产生
            # 并叠加一个持续上浮扰动（绕圈时鱼体横滚、翼面竖直效率下降也会等效成它）。
            def closed_loop(roll, disturb_vz, steps=420, speed=583.0):
                d = DepthController()
                z, vz, pitch, dt = HOLD_Z, 0.0, 0.0, 1.0/60.0
                peak = 0.0
                for _ in range(steps):
                    d.calculate(z, vz, pitch, roll, speed, dt)
                    efficiency = max(math.cos(math.radians(roll)), 0.0)
                    pitch += (d.desired_pitch-pitch)*(dt/(0.35+dt))
                    vz = speed*math.tan(math.radians(pitch))*efficiency + disturb_vz
                    z += vz*dt
                    peak = max(peak, z-HOLD_Z)
                return z-HOLD_Z, peak

            # 绕圈时鱼体横滚 35 度，持续上浮 60mm/s（约 4 秒 = 绕 2 圈的量级）。
            residual, peak = closed_loop(roll=35.0, disturb_vz=60.0)
            # 关键：不允许持续发散。峰值有界，稳态不再继续累积。
            self.assertLess(peak, 60.0)
            self.assertLess(residual, 40.0)
            # 同样扰动下，竖直增益足够把偏置建立起来（积分达到可用量级）。
            d = DepthController()
            for _ in range(240):
                d.calculate(HOLD_Z-30.0, 30.0, 0.0, 0.0, 583.0, 1.0/60.0)
            self.assertGreater(abs(d.integral), 12.0)
            # 扰动消失后必须能回到目标高度，不留稳态偏差。
            d = DepthController()
            z, vz, pitch = HOLD_Z, 0.0, 0.0
            dt = 1.0/60.0
            for i in range(900):
                disturb = 60.0 if i < 240 else 0.0
                d.calculate(z, vz, pitch, 35.0, 583.0, dt)
                efficiency = max(math.cos(math.radians(35.0)), 0.0)
                pitch += (d.desired_pitch-pitch)*(dt/(0.35+dt))
                vz = 583.0*math.tan(math.radians(pitch))*efficiency + disturb
                z += vz*dt
            self.assertLess(abs(z-HOLD_Z), 25.0)

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
            # v7.5：中度偏离只在"继续往外漂"时才踩刹车；正在收敛时不干预。
            outside_y = -LOOP_R-80.0   # 80mm > 70mm 漂移阈值，但未到 120mm 硬阈值
            c.velocity_x, c.velocity_y = 0.0, -300.0   # 径向外漂
            drifting = c._motion_profile(1.0/LOOP_R, 0.0, OBS1[0], outside_y)[0]
            self.assertLessEqual(drifting, ORBIT_RECAPTURE_THRUST)
            c.velocity_x, c.velocity_y = 0.0, 300.0    # 正在往回收敛
            converging = c._motion_profile(1.0/LOOP_R, 0.0, OBS1[0], outside_y)[0]
            self.assertGreater(converging, ORBIT_RECAPTURE_THRUST)
            # 严重偏离无论方向一律回收，安全底线不松。
            hard = c._motion_profile(
                1.0/LOOP_R, 0.0, OBS1[0], -LOOP_R-ORBIT_RECAPTURE_HARD_BAND-20.0)[0]
            self.assertLessEqual(hard, ORBIT_RECAPTURE_THRUST)

        def test_orbit_radial_damp_predicts_drift(self):
            """v7.5：径向外漂必须提前被折算成向内转向，避免半径继续跑大。"""
            c = RaceController()
            c.section_index = 1
            c.orbit = OrbitProgress(OBS1, 1)
            x, y = OBS1[0], -LOOP_R       # 正好在圆周上
            c.velocity_x, c.velocity_y = 0.0, -200.0   # 径向外漂
            outward, _, _ = c._guidance(x, y)
            c.velocity_x, c.velocity_y = 0.0, 200.0    # 径向内收
            inward, _, _ = c._guidance(x, y)
            # 两者都以圆周切向为基准；外漂时应得到更大的向内修正角。
            self.assertGreater(_wrap(outward-inward), 0.0)

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
            diffs = []
            for roll in (-20.0, 20.0):
                c = RaceController()
                command = c.calculate(info(z=HOLD_Z, roll=roll), now=1.0)
                left_z = DepthController.vertical_component(command.left_angle, roll=roll)
                right_z = DepthController.vertical_component(command.right_angle, roll=roll)
                self.assertGreater((right_z-left_z)*roll, 0.0)
                diffs.append(command.left - command.right)
            # 滚转不得改变推力差：差动只服务偏航，滚转仍由翼角完成。
            self.assertAlmostEqual(diffs[0], diffs[1], places=6)

        def test_yaw_differential_assists_turning(self):
            """v7.6.1：正偏航误差时右加左减（实测证伪首版左加右减）。"""
            self.assertGreater(YAW_DIFF_GAIN, 0.0)
            self.assertGreater(YAW_DIFF_MAX, 0.0)
            self.assertLessEqual(YAW_DIFF_MAX, MAX_THRUST)

            def pair(thrust, correction):
                assist = _constrain(YAW_DIFF_GAIN * correction, -YAW_DIFF_MAX, YAW_DIFF_MAX)
                return (_constrain(thrust - assist, -MAX_THRUST, MAX_THRUST),
                        _constrain(thrust + assist, -MAX_THRUST, MAX_THRUST))

            left, right = pair(50.0, 30.0)
            self.assertGreater(right, left)
            left, right = pair(50.0, -30.0)
            self.assertGreater(left, right)
            left, right = pair(50.0, 200.0)
            self.assertLessEqual(abs(left), MAX_THRUST)
            self.assertLessEqual(abs(right), MAX_THRUST)
            c = RaceController()
            command = c.calculate(info(x=-1250.0, y=0.0, yaw=0.0), now=1.0)
            self.assertLessEqual(abs(command.left - command.right), 2.0 * YAW_DIFF_MAX + 1e-6)

        def test_steering_priority_reduces_amplitude_when_error_large(self):
            """v7.6：大偏航误差时收振幅、抬频率，把尾巴让给转向。"""
            self.assertGreater(STEER_PRIORITY_ERROR, 0.0)
            self.assertLess(STEER_PRIORITY_AMPLITUDE_SCALE, 1.0)
            self.assertGreater(STEER_PRIORITY_FREQ_SCALE, 1.0)
            c = RaceController()
            c.speed = 360.0
            small = c._motion_profile(1.0 / LOOP_R, 5.0, OBS1[0], -LOOP_R)
            large = c._motion_profile(1.0 / LOOP_R, 35.0, OBS1[0], -LOOP_R)
            # frequency/amplitude 元组位置: (thrust, frequency, amplitude)
            self.assertGreater(large[1], small[1])
            self.assertLess(large[2], small[2])

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


TELEMETRY_FIELDS = (
    'Time(s)', 'Stage', 'PosX', 'PosY', 'PosZ', 'Pitch(deg)', 'Roll(deg)',
    'Speed', 'TargetSpeed', 'VerticalSpeed', 'YawError', 'RefVz', 'DesiredPitch',
    'DepthIntegral', 'PitchTrim', 'Tail', 'ThrustL', 'ThrustR', 'WingLeft', 'WingRight')


def write_telemetry(path, rows):
    """把整场遥测一次性落盘；不在 60Hz 回调里做 IO。"""
    import csv
    try:
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(TELEMETRY_FIELDS)
            writer.writerows(rows)
    except OSError as exc:
        print('[race] telemetry write failed: {}'.format(exc), flush=True)


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
    telemetry_rows = []
    telemetry_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'race_telemetry.csv')

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
            if not controller.finished and controller.started_at is not None:
                elapsed = now - controller.started_at
                telemetry_rows.append((
                    '{:.3f}'.format(elapsed), controller.stage,
                    '{:.3f}'.format(fish_info.pos.x), '{:.3f}'.format(fish_info.pos.y),
                    '{:.3f}'.format(fish_info.pos.z),
                    '{:.3f}'.format(controller.pitch_degrees),
                    '{:.3f}'.format(fish_info.rot.x),
                    '{:.3f}'.format(controller.speed), '{:.3f}'.format(controller.target_speed),
                    '{:.3f}'.format(controller.vertical_speed),
                    '{:.3f}'.format(controller.heading_error),
                    '{:.3f}'.format(controller.depth.reference_vz),
                    '{:.3f}'.format(controller.depth.desired_pitch),
                    '{:.3f}'.format(controller.depth.integral),
                    '{:.3f}'.format(controller.depth.pitch_trim),
                    '{:.3f}'.format(command.tail),
                    '{:.3f}'.format(command.left), '{:.3f}'.format(command.right),
                    '{:.3f}'.format(command.left_angle), '{:.3f}'.format(command.right_angle)))
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
            if telemetry_rows:
                write_telemetry(telemetry_path, telemetry_rows)
                print('[race] telemetry -> {} ({} frames)'.format(
                    telemetry_path, len(telemetry_rows)), flush=True)
        subscriber.close()


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        self_test()
    else:
        main()
