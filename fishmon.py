#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fishmon.py —— 水中机器鱼「科目一 障碍竞速」低开销监控与瓶颈分析工具
=====================================================================

设计原则
  1. **绝不修改 test.py**。本工具只从外部观测，并读取平台/策略自己写出的文件
     （project1.csv 成绩、race_telemetry.csv 遥测）。对 test.py 只做只读扫描，
     用于提取策略参数快照。
  2. **自动触发**：监视若干候选路径，检测到 test.py「出现 / 被更新」即自动开始
     记录一轮（trial）。也支持 --now 立即开始。
  3. **低开销**：系统/进程默认 1s 采样；线程枚举、GPU 查询等较重采集用低频次。
     （本机 330+ 进程，全量带 cmdline 枚举约 4s，因此做了分级过滤 + 句柄缓存。）
  4. **可配置**：同目录 fishmon_config.json，命令行参数可覆盖。
     **新版本会自动把缺的新选项补进旧配置文件（只增不改你已有的取值）。**
  5. **输出报告**：Markdown 报告 + 逐帧 CSV（系统、进程、线程、日志、事件）。

用法（**推荐直接双击 fishmon.bat**，它会自动挑一个装了 psutil 的 Python）
    python fishmon.py                  # 等 test.py 出现/更新后自动开始
    python fishmon.py --now            # 立即开始一轮
    python fishmon.py --duration 60    # 采 60 秒后收尾出报告
    python fishmon.py --start-if-present --stop-when-idle   # fishmon.bat 用的组合
    python fishmon.py --list           # 只列出当前相关进程后退出

需要 psutil（仿真器自带的 Python 3.7 没有）。本机可用 D:\Python311\python.exe。

报告内容：
  · 整机 / 相关进程的**运行分钟数**（按进程真实 create_time 计算，非发现时刻）
  · 每个进程、子进程、线程的 CPU 增长与「等效核数」（真正用了几个核）
  · CPU / 内存 / GPU / IO / 网络 / 日志 的逐帧 CSV
  · 迭代速率（控制回调 Hz、监控采样 Hz）
  · 瓶颈定位：分段耗时、速度利用率、绕圈径向误差触发的推力回收
  · 提速建议 + **候选补丁**（含依据与风险；本工具不会自动改 test.py）
"""

import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime

try:
    import psutil
except ImportError:                                    # pragma: no cover
    psutil = None


HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_NAME = 'fishmon_config.json'


# ======================================================================
# 配置
# ======================================================================
DEFAULT_CONFIG = {
    # 监视这些路径；任一文件的「出现」或「内容变化」都算一次上传/更新事件
    "watch_paths": [
        "test.py",
        "WindowsNoEditor/Scripts/test.py",
        "WindowsNoEditor/Scripts/project1/test.py",
        "WindowsNoEditor/Scripts/project1/main.py",
    ],
    # 也监视这些目录下新出现的 .py（平台可能把策略拷到别处）
    "watch_dirs": [
        "WindowsNoEditor/Scripts/project1",
        "WindowsNoEditor/Scripts",
    ],
    # 目标进程匹配：进程名含这些词，或命令行含这些词
    "process_name_patterns": ["fish"],
    "process_cmdline_patterns": ["test.py"],
    # 只有当进程名命中这些「脚本宿主」时才去拉它的 cmdline。
    # 原因：本机 330+ 进程，全量拉 cmdline 约 3.8s，只拉 name 约 1.2s。
    # 分级过滤后既保留 cmdline 匹配能力，又把发现代价压到 ~1/4。
    "process_cmdline_hosts": ["python.exe", "pythonw.exe", "python3.exe",
                              "py.exe", "cmd.exe", "powershell.exe", "pwsh.exe"],
    "process_discover_every": 10,     # 每 N 次采样做一次全量进程发现（较重，须限频）
    "process_rescan_gap_sec": 3.0,    # 还没命中目标时的重扫间隔（静默期）

    "sample_interval_sec": 1.0,       # 系统 + 进程采样周期
    "thread_sample_every": 5,         # 每 N 次采样做一次线程枚举（较重）
    "gpu_sample_every": 3,            # 每 N 次采样查一次 GPU
    "keep_top_threads": 12,           # 每次线程采样记录 CPU 增长最快的前 N 个线程
    "thread_fallback_refresh_sec": 30.0,  # 线程枚举被拒时，PowerShell 兜底的刷新间隔

    "auto_start_on_watch": True,      # 检测到文件事件即自动开始
    "restart_trial_on_update": True,  # 运行中 test.py 被更新 → 开新一轮（分段）
    "trial_idle_timeout_sec": 45.0,   # 无新成绩且文件无变化这么久 → 结束本轮
    "max_trial_sec": 900.0,           # 单轮最长时长，防止忘了关

    # 日志采集：只读 + 只取尾部增量（tail -f 语义）
    "log_paths": [
        "WindowsNoEditor/Fish427/Binaries/Win64/fish_race.log",
    ],
    # 只扫这些「平铺」目录里的 *.log/*.txt。
    # 不要指向 WindowsNoEditor 根目录：那棵树有上万个 .txt（引擎自带 Python 库），
    # 会把日志采集变成一次全盘扫描。log_paths 里的具体文件不受影响。
    "log_dirs": [],
    "log_tail_max_bytes": 262144,     # 每次最多读 256KB 尾部
    "log_keep_lines": 4000,           # 报告里最多保留多少行日志

    "output_dir": "monitor_reports",
    "report_top_n": 8,

    # 绕圈几何与「半径回收」阈值（用于诊断绕圈速度损失，只读分析用）
    # 数值来自 test.py 的 OBS1/OBS3 与 LOOP_R / ORBIT_RECAPTURE_THRUST；
    # 若 test.py 改了这些常量，请同步这里，否则诊断会偏差。
    "orbit_centers": {"orbit_1": [-750.0, 0.0], "orbit_3": [750.0, 0.0]},
    "orbit_recapture_threshold_mm": 40.0,   # |径向误差| 超过它就触发推力回收
    "orbit_recapture_thrust": 39.2,          # 32.0 * 1.40（SPEED_GAIN=1.40）
    "loop_r_mm": 200.0,                      # LOOP_R（快档）；推力回收的基准半径
    "max_thrust": 50.0,                      # 平台硬上限
}


def _deep_merge(base, extra):
    """把 extra 覆盖到 base 的副本上（只做一层 dict 合并，够用）。"""
    out = dict(base)
    for key, value in (extra or {}).items():
        out[key] = value
    return out


def load_config(path=None):
    path = path or os.path.join(HERE, CONFIG_NAME)
    cfg = dict(DEFAULT_CONFIG)
    if os.path.isfile(path):
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                cfg = _deep_merge(cfg, json.load(handle))
        except (IOError, OSError, ValueError) as exc:
            print('[warn] 配置文件读取失败，用默认配置: {}'.format(exc))
    return cfg


def write_default_config(path=None):
    """首次运行时落一份带注释的默认配置，方便用户手改。"""
    path = path or os.path.join(HERE, CONFIG_NAME)
    if os.path.isfile(path):
        return path
    try:
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(DEFAULT_CONFIG, handle, ensure_ascii=False, indent=2)
    except (IOError, OSError) as exc:
        print('[warn] 无法写出默认配置: {}'.format(exc))
    return path


def sync_default_config(path=None):
    """
    把 DEFAULT_CONFIG 里新增、而旧配置文件里没有的键补进去（**只增不改**）。
    原因：配置文件是工具自己的，旧版本没有 log_paths 等键时，
    新功能会静默不生效。这里补齐并回报，用户已有取值一律保留。
    """
    path = path or os.path.join(HERE, CONFIG_NAME)
    if not os.path.isfile(path):
        return [], write_default_config(path)
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            current = json.load(handle)
    except (IOError, OSError, ValueError) as exc:
        print('[warn] 配置文件读取失败，跳过升级: {}'.format(exc))
        return [], path
    if not isinstance(current, dict):
        return [], path
    added = [k for k in DEFAULT_CONFIG if k not in current]
    if not added:
        return [], path
    merged = dict(current)
    for key in added:
        merged[key] = DEFAULT_CONFIG[key]
    try:
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(merged, handle, ensure_ascii=False, indent=2)
    except (IOError, OSError) as exc:
        print('[warn] 无法升级配置文件: {}'.format(exc))
        return [], path
    return added, path


def resolve(path):
    """把配置里的相对路径解析成本文件所在目录下的绝对路径。"""
    return path if os.path.isabs(path) else os.path.join(HERE, path.replace('/', os.sep))


def harden_console():
    """
    双击运行时保证 Windows 控制台里的中文不乱码、不因编码异常中断。

    cmd 默认代码页可能是 936(GBK)，而本脚本全部按 UTF-8 输出，
    中文会变乱码。这里做两件事（都允许失败，不阻断主流程）：
      1) 把控制台输入/输出代码页切到 UTF-8；
      2) 把 stdout/stderr 的编码强制成 UTF-8。
    """
    if os.name != 'nt':
        return
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleOutputCP(65001)
        kernel32.SetConsoleCP(65001)
    except Exception:                                  # noqa: BLE001
        pass
    for name in ('stdout', 'stderr'):
        stream = getattr(sys, name, None)
        reconfigure = getattr(stream, 'reconfigure', None)
        if reconfigure is None:
            continue
        try:
            reconfigure(encoding='utf-8', errors='replace')
        except Exception:                              # noqa: BLE001
            continue


def now_stamp():
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def safe_float(value, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


# ======================================================================
# GPU：走 nvidia-smi，拿显存/利用率/温度/功耗/频率，以及各进程显存占用
# ======================================================================
class GpuSampler(object):
    QUERY = ('index,name,utilization.gpu,utilization.memory,memory.used,memory.total,'
             'temperature.gpu,power.draw,power.limit,clocks.sm')
    PROC_QUERY = 'pid,used_memory'

    def __init__(self):
        self.available = False
        self.error = ''
        try:
            out = self._run(['-L'])
            self.available = bool(out.strip())
            self.list_text = out.strip()
        except Exception as exc:                       # noqa: BLE001
            self.error = str(exc)

    @staticmethod
    def _run(args, timeout=6):
        proc = subprocess.Popen(['nvidia-smi'] + args,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = proc.communicate(timeout=timeout)
        if proc.returncode != 0:
            raise RuntimeError((stderr or b'').decode('utf-8', 'replace').strip())
        return (stdout or b'').decode('utf-8', 'replace')

    def sample(self):
        """返回 list[dict]，每个元素一块 GPU。失败返回空列表。"""
        if not self.available:
            return []
        try:
            text = self._run(['--query-gpu=' + self.QUERY,
                              '--format=csv,noheader,nounits'])
        except Exception as exc:                       # noqa: BLE001
            self.error = str(exc)
            return []
        gpus = []
        for line in text.strip().splitlines():
            cells = [c.strip() for c in line.split(',')]
            if len(cells) < 10:
                continue
            gpus.append({
                'index': cells[0], 'name': cells[1],
                'util_gpu': safe_float(cells[2]),
                'util_mem': safe_float(cells[3]),
                'mem_used_mb': safe_float(cells[4]),
                'mem_total_mb': safe_float(cells[5]),
                'temp_c': safe_float(cells[6]),
                'power_w': safe_float(cells[7]),
                'power_limit_w': safe_float(cells[8]),
                'clock_sm_mhz': safe_float(cells[9]),
            })
        return gpus

    def sample_processes(self):
        """返回 {pid: 显存MB}，用于把 GPU 占用归因到具体进程。"""
        if not self.available:
            return {}
        try:
            text = self._run(['--query-compute-apps=' + self.PROC_QUERY,
                              '--format=csv,noheader,nounits'])
        except Exception:                              # noqa: BLE001
            return {}
        out = {}
        for line in text.strip().splitlines():
            cells = [c.strip() for c in line.split(',')]
            if len(cells) < 2:
                continue
            try:
                out[int(cells[0])] = safe_float(cells[1])
            except ValueError:
                continue
        return out


# ======================================================================
# 进程发现：找出整机里与仿真/策略相关的进程（含子进程）
# ======================================================================
class ProcessFinder(object):
    def __init__(self, cfg):
        self.names = [p.lower() for p in cfg.get('process_name_patterns', [])]
        self.cmdlines = [p.lower() for p in cfg.get('process_cmdline_patterns', [])]
        self.cmdline_hosts = [p.lower() for p in
                              cfg.get('process_cmdline_hosts', [])]
        self._roots = {}
        self._discover_every = max(1, int(cfg.get('process_discover_every', 10)))
        self._tick = 0
        self._last_scan = 0.0
        # 静默期（还没命中任何目标）的重扫间隔，避免空转时反复全量枚举
        self._force_scan_gap = max(1.0, float(
            cfg.get('process_rescan_gap_sec', 3.0)))
        self._pending_rescan = False
        # 进程生命周期：用于「这个进程已经跑了多少分钟」的统计
        self.lifetime = {}
        self._alive_pids = set()

    def _matches(self, proc):
        try:
            name = (proc.info.get('name') or '').lower()
            cmd = ' '.join(proc.info.get('cmdline') or []).lower()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return False
        if any(pat in name for pat in self.names):
            return True
        if cmd and any(pat in cmd for pat in self.cmdlines):
            return True
        return False

    def _update_lifetime(self, roots):
        """记录进程第一次被看到的时刻，用于统计「跑了多少分钟」。

        create_time() 是进程真实启动时刻（不是本轮发现时刻），
        所以「已运行分钟」= now - create_time，能反映整轮测试期间进程的真实寿命。
        """
        now = time.time()
        for pid, proc in roots.items():
            if pid in self.lifetime:
                continue
            created = now
            name = '?'
            try:
                created = proc.create_time()
            except Exception:                          # noqa: BLE001
                pass
            try:
                name = proc.name()
            except Exception:                          # noqa: BLE001
                pass
            self.lifetime[pid] = {'pid': pid, 'name': name,
                                  'created': created, 'first_seen': now}
        self._alive_pids = set(roots)

    def find(self):
        """返回 (roots, all_targets)：roots 是直接命中的进程，all_targets 含其子树。"""
        if psutil is None:
            return [], []
        # 全量枚举进程较慢（本机 330+ 进程，带 cmdline 约 4s），因此限频 +
        # 缓存进程句柄。句柄失效时才触发重新发现。
        self._tick += 1
        due = time.monotonic() - self._last_scan >= self._force_scan_gap
        need_scan = ((not self._roots) and due) or \
                    (self._tick % self._discover_every == 0)
        self._pending_rescan = False
        if need_scan:
            self._discover()
        else:
            self._drop_dead()
        # 缓存里的进程全部退出了 → 立刻补一次，避免一直空转等不到新进程
        if self._pending_rescan and not self._roots:
            self._discover()
        self._update_lifetime(self._roots)

        seen = {}
        for pid, proc in self._roots.items():
            try:
                for item in [proc] + proc.children(recursive=True):
                    seen[item.pid] = item
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return list(self._roots.keys()), list(seen.values())

    def _drop_dead(self):
        alive = {}
        for pid, proc in self._roots.items():
            try:
                if proc.is_running() and proc.status() != psutil.STATUS_ZOMBIE:
                    alive[pid] = proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        if self._roots and not alive:
            self._pending_rescan = True
        self._roots = alive

    def _discover(self):
        """分级过滤：先只取 name（便宜），只对少数候选再拉 cmdline（贵）。"""
        try:
            snapshot = list(psutil.process_iter(['pid', 'name', 'ppid']))
        except Exception:                              # noqa: BLE001
            return
        roots = {}
        for proc in snapshot:
            try:
                name = (proc.info.get('name') or '').lower()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            if any(pat in name for pat in self.names):
                roots[proc.pid] = proc
                continue
            # cmdline 匹配有代价，只对「可能承载脚本」的宿主进程做
            if self.cmdlines and name in self.cmdline_hosts:
                try:
                    cmd = ' '.join(proc.cmdline() or []).lower()
                except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                    continue
                if cmd and any(pat in cmd for pat in self.cmdlines):
                    roots[proc.pid] = proc
        self._roots = roots
        self._last_scan = time.monotonic()


# ======================================================================
# 文件监视：test.py 出现/更新即触发
# ======================================================================
class FileWatcher(object):
    def __init__(self, cfg):
        self.paths = [resolve(p) for p in cfg.get('watch_paths', [])]
        self.dirs = [resolve(d) for d in cfg.get('watch_dirs', [])]
        self.state = {}
        self._prime()

    @staticmethod
    def _fingerprint(path):
        try:
            stat = os.stat(path)
        except (IOError, OSError):
            return None
        digest = ''
        try:
            with open(path, 'rb') as handle:
                digest = hashlib.sha1(handle.read()).hexdigest()[:12]
        except (IOError, OSError):
            pass
        return (stat.st_mtime_ns, stat.st_size, digest)

    def _scan(self):
        found = {}
        for path in self.paths:
            fp = self._fingerprint(path)
            if fp:
                found[path] = fp
        for folder in self.dirs:
            if not os.path.isdir(folder):
                continue
            try:
                entries = os.listdir(folder)
            except (IOError, OSError):
                continue
            for entry in entries:
                if not entry.lower().endswith('.py'):
                    continue
                full = os.path.join(folder, entry)
                fp = self._fingerprint(full)
                if fp:
                    found[full] = fp
        return found

    def _prime(self):
        self.state = self._scan()

    def poll(self):
        """返回事件列表 [{'path','kind'}]，kind ∈ {appeared, updated}。"""
        current = self._scan()
        events = []
        for path, fp in current.items():
            if path not in self.state:
                events.append({'path': path, 'kind': 'appeared'})
            elif self.state[path] != fp:
                events.append({'path': path, 'kind': 'updated'})
        self.state = current
        return events


# ======================================================================
# 采集器：系统 + 进程 +（低频）线程/GPU
# ======================================================================
class LogTailer(object):
    """
    日志采集：只从文件尾部增量读（tail -f 语义），不重读全文件。
    平台/策略写出的日志一律只读，绝不改动。
    """

    def __init__(self, cfg):
        self.paths = [resolve(p) for p in cfg.get('log_paths', [])]
        self.dirs = [resolve(d) for d in cfg.get('log_dirs', [])]
        self.max_bytes = int(cfg.get('log_tail_max_bytes', 262144))
        self.lines = []
        self.max_lines = int(cfg.get('log_keep_lines', 4000))
        self._offsets = {}
        self._prime()

    def _candidates(self):
        out = []
        for path in self.paths:
            if os.path.isfile(path):
                out.append(path)
        for folder in self.dirs:
            if not os.path.isdir(folder):
                continue
            try:
                entries = os.listdir(folder)
            except (IOError, OSError):
                continue
            for entry in entries:
                if entry.lower().endswith(('.log', '.txt')):
                    out.append(os.path.join(folder, entry))
        return out

    def _prime(self):
        """记录当前文件大小，只采后续新增内容（避免把历史日志全灌进报告）。"""
        for path in self._candidates():
            try:
                self._offsets[path] = os.path.getsize(path)
            except (IOError, OSError):
                continue

    def poll(self):
        collected = []
        for path in self._candidates():
            try:
                size = os.path.getsize(path)
            except (IOError, OSError):
                continue
            offset = self._offsets.get(path, 0)
            if size < offset:            # 文件被截断/重建 → 从头读
                offset = 0
            if size == offset:
                continue
            start = max(offset, size - self.max_bytes)
            # start 是否被 max_bytes 截断？只有被截断时才可能落在半行中间。
            truncated = start > offset
            try:
                with open(path, 'rb') as handle:
                    handle.seek(start)
                    chunk = handle.read()
            except (IOError, OSError):
                continue
            self._offsets[path] = size
            # 只有在半行中间起步时才需要丢掉第一段残行。
            # 若 start 正好等于上次记录的位置，那一处本来就是行边界，
            # 再丢一次会把**第一条新增日志**误删（曾出现过的 bug）。
            if truncated:
                _, _, chunk = chunk.partition(b'\n')
            for raw in chunk.decode('utf-8', 'replace').splitlines():
                text = raw.strip()
                if not text:
                    continue
                row = {'t': now_stamp(), 'file': os.path.basename(path),
                       'line': text[:400]}
                self.lines.append(row)
                collected.append(row)
        if len(self.lines) > self.max_lines:
            self.lines = self.lines[-self.max_lines:]
        return collected


class Sampler(object):
    def __init__(self, cfg, gpu, finder):
        self.cfg = cfg
        self.gpu = gpu
        self.finder = finder
        self.samples = []
        self.thread_rows = []
        self.events = []
        self.trial_index = 0
        self.trial_label = ''
        self._window_baseline = None
        self._tick = 0
        self._last_thread_cpu = {}
        self._last_proc_cpu = {}
        self._last_thread_wall = None
        self._thread_fallback_cache = {}
        self._fallback_refresh_sec = float(
            cfg.get('thread_fallback_refresh_sec', 30.0))
        self._last_net = None
        self._last_disk = None
        self._last_wall = None
        self._cpu_count = psutil.cpu_count(logical=True) if psutil else 1

    # ---------- 轮次管理 ----------
    def start_trial(self, label, reason):
        self.trial_index += 1
        self.trial_label = label
        self._t0 = time.monotonic()
        self.events.append({'t': now_stamp(), 'kind': 'trial_start',
                            'detail': '{} (触发: {})'.format(label, reason)})
        # 重置窗口基线，避免跨轮次算差分
        self._reset_counters()

    def add_event(self, kind, detail):
        self.events.append({'t': now_stamp(), 'kind': kind, 'detail': detail})

    def _reset_counters(self):
        try:
            self._last_thread_cpu = {}
            self._last_proc_cpu = {}
            self._last_thread_wall = None
            self._thread_fallback_cache = {}
            self._last_net = psutil.net_io_counters()
            self._last_disk = psutil.disk_io_counters()
        except Exception:                              # noqa: BLE001
            pass
        self._last_wall = time.monotonic()

    # ---------- 单次采样 ----------
    def sample(self):
        wall = time.monotonic()
        row = {'t': now_stamp(), 'elapsed': round(wall - self._t0, 3)
               if hasattr(self, '_t0') else 0.0}
        self._tick += 1

        # 系统
        try:
            row['cpu_total'] = psutil.cpu_percent(interval=None)
            freq = psutil.cpu_freq()
            row['cpu_freq_mhz'] = round(freq.current, 1) if freq else 0.0
            vm = psutil.virtual_memory()
            row['mem_used_mb'] = round(vm.used / 1048576.0, 1)
            row['mem_percent'] = vm.percent
            row['boot_uptime_min'] = round((time.time() - psutil.boot_time()) / 60.0, 1)
        except Exception:                              # noqa: BLE001
            pass
        self._sample_io(row, wall)

        # 目标进程
        root_pids, procs = self.finder.find()
        row['target_count'] = len(procs)
        row['root_pids'] = '|'.join(str(p) for p in root_pids)
        per_proc = []
        total_cpu = 0.0
        total_rss = 0.0
        for proc in procs:
            info = self._proc_info(proc)
            if info is None:
                continue
            per_proc.append(info)
            total_cpu += info['cpu_percent']
            total_rss += info['rss_mb']
        row['target_cpu_percent'] = round(total_cpu, 2)
        row['target_rss_mb'] = round(total_rss, 1)
        # 目标进程「已经跑了多少分钟」——用进程真实启动时间，不是本轮发现时间
        life = getattr(self.finder, 'lifetime', {}) or {}
        now_epoch = time.time()
        row['proc_uptime_min'] = round(
            max((now_epoch - life[p]['created']) / 60.0
                for p in root_pids if p in life), 1) if any(
                    p in life for p in root_pids) else 0.0
        row['proc_lifetimes'] = [
            {'pid': p, 'name': life[p]['name'],
             'started': datetime.fromtimestamp(life[p]['created']).strftime(
                 '%Y-%m-%d %H:%M:%S'),
             'uptime_min': round((now_epoch - life[p]['created']) / 60.0, 1)}
            for p in root_pids if p in life]
        row['proc_lifetime'] = '; '.join(
            '{}#{} 已运行{:.1f}min'.format(life[p]['name'], p,
                                            (now_epoch - life[p]['created']) / 60.0)
            for p in root_pids if p in life)
        row['proc_detail'] = '; '.join(
            '{}#{} cpu={:.1f}% rss={:.0f}MB thr={}'.format(
                p['name'], p['pid'], p['cpu_percent'], p['rss_mb'], p['threads'])
            for p in per_proc)

        # GPU（低频）
        if self._tick % max(1, int(self.cfg.get('gpu_sample_every', 3))) == 0:
            gpus = self.gpu.sample()
            if gpus:
                g = gpus[0]
                row['gpu_util'] = g['util_gpu']
                row['gpu_mem_used_mb'] = g['mem_used_mb']
                row['gpu_temp_c'] = g['temp_c']
                row['gpu_power_w'] = g['power_w']
                row['gpu_clock_mhz'] = g['clock_sm_mhz']
                row['gpu_all'] = '; '.join(
                    '#{} {} util={:.0f}% mem={:.0f}/{:.0f}MB {:.0f}C {:.0f}W'.format(
                        g['index'], g['name'], g['util_gpu'], g['mem_used_mb'],
                        g['mem_total_mb'], g['temp_c'], g['power_w']) for g in gpus)
                gpu_proc = self.gpu.sample_processes()
                row['gpu_proc_mem'] = '; '.join(
                    '{}:{}MB'.format(pid, mb) for pid, mb in sorted(gpu_proc.items()))
        # GPU 是低频采样：本帧没采到（不是采样帧，或 nvidia-smi 失败）时，
        # 沿用上一次的值做**前向填充**，否则 CSV/曲线会出现空洞。
        # 注意：这段必须和上面的 if 平级，不能被 gpu.sample() 的返回值挡住。
        if 'gpu_util' not in row and self.samples:
            for key in ('gpu_util', 'gpu_mem_used_mb', 'gpu_temp_c',
                        'gpu_power_w', 'gpu_clock_mhz', 'gpu_all', 'gpu_proc_mem'):
                if self.samples[-1].get(key) is not None:
                    row[key] = self.samples[-1][key]

        self.samples.append(row)

        # 线程（更低频、较重）
        if self._tick % max(1, int(self.cfg.get('thread_sample_every', 5))) == 0:
            self._sample_threads(wall, procs)
        return row

    def _sample_io(self, row, wall):
        try:
            net = psutil.net_io_counters()
            # 首个样本没有基线：此时只能记 0。否则会拿「开机以来的累计字节数」
            # 去除以一个近乎 0 的 dt，产生几百 MB/s 的假峰值。
            dt = 0.0 if self._last_wall is None else wall - self._last_wall
            have_dt = dt >= 1e-3
            if have_dt and self._last_net:
                row['net_sent_kbs'] = round(
                    (net.bytes_sent - self._last_net.bytes_sent) / dt / 1024.0, 1)
                row['net_recv_kbs'] = round(
                    (net.bytes_recv - self._last_net.bytes_recv) / dt / 1024.0, 1)
            else:
                row['net_sent_kbs'] = 0.0
                row['net_recv_kbs'] = 0.0
            self._last_net = net
            disk = psutil.disk_io_counters()
            if disk and self._last_disk and have_dt:
                row['disk_read_kbs'] = round(
                    (disk.read_bytes - self._last_disk.read_bytes) / dt / 1024.0, 1)
                row['disk_write_kbs'] = round(
                    (disk.write_bytes - self._last_disk.write_bytes) / dt / 1024.0, 1)
            else:
                row['disk_read_kbs'] = 0.0
                row['disk_write_kbs'] = 0.0
            if disk:
                self._last_disk = disk
            self._last_wall = wall
        except Exception:                              # noqa: BLE001
            pass

    @staticmethod
    def _proc_info(proc):
        try:
            with proc.oneshot():
                cpu = proc.cpu_percent(interval=None)
                mem = proc.memory_info()
                return {
                    'pid': proc.pid,
                    'name': proc.name(),
                    'cpu_percent': round(cpu, 2),
                    'rss_mb': round(mem.rss / 1048576.0, 1),
                    'threads': proc.num_threads(),
                    'status': proc.status(),
                }
        except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
            return None

    def _sample_threads(self, wall, procs):
        top = int(self.cfg.get('keep_top_threads', 12))
        for proc in procs:
            try:
                threads = proc.threads()
                cpu_times = proc.cpu_times()
            except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                continue
            # 进程总体 CPU 时间：用于算「有效并行度」（真的用了几个核）
            prev_total = self._last_proc_cpu.get(proc.pid)
            now_total = cpu_times.user + cpu_times.system
            delta_total = 0.0 if prev_total is None else max(0.0, now_total - prev_total)
            self._last_proc_cpu[proc.pid] = now_total
            dt = max(1e-6, wall - self._last_thread_wall) if self._last_thread_wall else 0.0
            # 第一次采样没有基线（dt=0）→ 记为 None（未知），专门区别于「测出来是 0」，
            # 否则 0 会被算进平均值，把并行度拉低。
            effective_cores = round(delta_total / dt, 2) if dt >= 1e-3 else None
            self.thread_rows.append({
                't': now_stamp(), 'pid': proc.pid, 'tid': -1,
                'name': proc.name() + ' [process]', 'user': round(cpu_times.user, 3),
                'system': round(cpu_times.system, 3),
                'cpu_delta': round(delta_total, 4),
                'state': 'total', 'context_switches': '',
                'effective_cores': '' if effective_cores is None else effective_cores,
            })
            # 单线程明细：psutil 能枚举时用 psutil；UE 进程常被拒绝，
            # 此时退化为 PowerShell 枚举线程 ID（只拿到数量与状态，拿不到每线程 CPU）。
            deltas = []
            for thread in threads:
                key = (proc.pid, thread.id)
                prev = self._last_thread_cpu.get(key)
                now_cpu = getattr(thread, 'user_time', 0.0) + getattr(thread, 'system_time', 0.0)
                delta = 0.0 if prev is None else max(0.0, now_cpu - prev)
                self._last_thread_cpu[key] = now_cpu
                deltas.append((delta, thread, now_cpu))
            if deltas:
                deltas.sort(key=lambda item: item[0], reverse=True)
                for delta, thread, now_cpu in deltas[:top]:
                    self.thread_rows.append({
                        't': now_stamp(), 'pid': proc.pid, 'tid': thread.id,
                        'name': proc.name(), 'user': round(now_cpu, 3), 'system': '',
                        'cpu_delta': round(delta, 4),
                        'state': '', 'context_switches': '',
                        'effective_cores': '',
                    })
            else:
                # 兜底：只记线程数量与状态分布（Windows 上 UE 进程常见）
                # 注意：PowerShell 调用约 1~1.5s，属于重操作，故按进程缓存并限频。
                states, cached_at = self._thread_fallback_cache.get(proc.pid, ({}, 0.0))
                if not states or wall - cached_at >= self._fallback_refresh_sec:
                    states = self._thread_states_fallback(proc.pid)
                    self._thread_fallback_cache[proc.pid] = (states, wall)
                if states:
                    self.events.append({
                        't': now_stamp(), 'kind': 'thread_enum_fallback',
                        'detail': 'PID {} 线程枚举被系统拒绝；改用 PowerShell 兜底，'
                                  '线程数={}，状态分布={}'.format(
                                      proc.pid, sum(states.values()), states)})
        self._last_thread_wall = wall

    @staticmethod
    def _thread_states_fallback(pid):
        """PowerShell 兜底：拿线程数与状态分布（拿不到每线程 CPU 时间）。"""
        # 注意：不要用 str.format()，PowerShell 的花括号会被当成占位符。
        script = ('try { (Get-Process -Id ' + str(int(pid))
                  + ' -ErrorAction Stop).Threads | Group-Object ThreadState | '
                    'ForEach-Object { "$($_.Name)=$($_.Count)" } } catch { "" }')
        try:
            proc = subprocess.Popen(
                ['powershell', '-NoProfile', '-NonInteractive', '-Command',
                 script],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, _ = proc.communicate(timeout=8)
        except Exception:                              # noqa: BLE001
            return {}
        out = {}
        for token in (stdout or b'').decode('utf-8', 'replace').splitlines():
            token = token.strip()
            if '=' in token:
                key, _, value = token.partition('=')
                try:
                    out[key.strip()] = int(value.strip())
                except ValueError:
                    continue
        return out


# ======================================================================
# 分析：速率 / 瓶颈 / 耗时
# ======================================================================
def _stats(values):
    vals = sorted(v for v in values if v is not None)
    if not vals:
        return {'n': 0, 'min': 0.0, 'max': 0.0, 'avg': 0.0, 'p50': 0.0, 'p95': 0.0}
    n = len(vals)
    def pick(frac):
        return vals[min(n - 1, int(frac * (n - 1)))]
    return {'n': n, 'min': round(vals[0], 2), 'max': round(vals[-1], 2),
            'avg': round(sum(vals) / n, 2), 'p50': round(pick(0.50), 2),
            'p95': round(pick(0.95), 2)}


def read_scores(path):
    """读 project1.csv：文件名,成绩,日期,时间。返回 [(时间戳, 秒数或None, 原文)]。"""
    out = []
    if not os.path.isfile(path):
        return out
    for encoding in ('utf-8-sig', 'gbk', 'utf-8'):
        try:
            with open(path, 'r', encoding=encoding) as handle:
                rows = list(csv.reader(handle))
            break
        except (UnicodeDecodeError, IOError, OSError):
            continue
    else:
        return out
    for cells in rows:
        cells = [c.strip() for c in cells if c is not None and c.strip() != '']
        if len(cells) < 4:
            continue
        stamp = '{} {}'.format(cells[-2], cells[-1])
        m = re.match(r'^(\d+):(\d+(?:\.\d+)?)$', cells[1])
        seconds = (int(m.group(1)) * 60 + float(m.group(2))) if m else None
        out.append((stamp, seconds, ','.join(cells)))
    return out


def read_telemetry(path):
    """读 race_telemetry.csv（v7.2 起由 test.py 自动写）。返回 list[dict]。"""
    if not os.path.isfile(path):
        return []
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            return list(csv.DictReader(handle))
    except (IOError, OSError, UnicodeDecodeError, ValueError):
        return []


def analyse_telemetry(rows):
    """从逐帧遥测里提取速率、耗时、路径过冲等关键量。"""
    if not rows:
        return {'available': False}
    def col(name):
        out = []
        for r in rows:
            if name in r:
                try:
                    out.append(float(r[name]))
                except (TypeError, ValueError):
                    pass
        return out
    usec = col('Time(s)')
    out = {'available': True, 'frames': len(rows)}
    if len(usec) >= 2:
        span = usec[-1] - usec[0]
        out['trial_seconds'] = round(span, 2)
        # 帧率：用采样点间隔估计（策略侧回调频率，通常 ~60Hz）
        gaps = [b - a for a, b in zip(usec, usec[1:]) if b > a]
        if gaps:
            out['frame_hz'] = round(1.0 / (sum(gaps) / len(gaps)), 1)
    # 分段耗时
    per_stage = {}
    if 'Stage' in rows[0]:
        order = []
        for r in rows:
            stage = r.get('Stage', '')
            if not order or order[-1] != stage:
                order.append(stage)
        for stage in order:
            ts = [float(r['Time(s)']) for r in rows
                  if r.get('Stage') == stage and r.get('Time(s)')]
            if ts:
                per_stage[stage] = round(max(ts) - min(ts), 2)
        out['stage_seconds'] = per_stage
    # 位置/速度：路径长度与过冲
    xs, ys, zs = col('PosX'), col('PosY'), col('PosZ')
    sp = col('Speed')
    if len(xs) == len(ys) and len(xs) > 2:
        length = 0.0
        for a in range(len(xs) - 1):
            d = math.hypot(xs[a + 1] - xs[a], ys[a + 1] - ys[a])
            if d < 200.0:                              # 排除采样跳变
                length += d
        out['path_length_mm'] = round(length)
    if sp:
        out['speed'] = _stats(sp)
    if zs:
        zstat = _stats(zs)
        out['z'] = {'min': zstat['min'], 'max': zstat['max'],
                    'avg': zstat['avg'], 'drift': round(zstat['max'] - zstat['min'], 1)}
    # 时间占比：速度落在低速/高速区间的比例 → 找"卡在哪"
    if sp:
        top = out.get('speed', {}).get('p95', 0) or 1.0
        out['slow_fraction'] = round(
            sum(1 for v in sp if v < 0.5 * top) / float(len(sp)), 3)
    # 分段明细：每段「耗时 / 里程 / 平均速度 / 速度利用率」。
    # 光看耗时只能知道哪段久，看「速度利用率」才能知道那段**是不是被限速拖慢的**：
    #   util = 该段平均速度 / 全程 p95 速度。util 低 = 那段没跑起来。
    if 'Stage' in rows[0] and len(xs) == len(rows) and sp:
        p95 = (out.get('speed') or {}).get('p95', 0.0) or 1.0
        by_stage = {}
        order = []
        for idx, r in enumerate(rows):
            stage = r.get('Stage', '')
            if stage not in by_stage:
                by_stage[stage] = {'t': [], 'v': [], 'dist': 0.0}
                order.append(stage)
            entry = by_stage[stage]
            entry['t'].append(idx)
            if idx < len(sp):
                entry['v'].append(sp[idx])
        # 里程按「相邻两点同段」计入，跨段的那一步不计，避免把段间跳变算进某一段
        for idx in range(len(xs) - 1):
            if rows[idx].get('Stage') == rows[idx + 1].get('Stage'):
                d = math.hypot(xs[idx + 1] - xs[idx], ys[idx + 1] - ys[idx])
                if d < 200.0:
                    by_stage[rows[idx].get('Stage')]['dist'] += d
        detail = {}
        for stage in order:
            entry = by_stage[stage]
            if not entry['t']:
                continue
            idxs = entry['t']
            span = (float(rows[idxs[-1]]['Time(s)']) - float(rows[idxs[0]]['Time(s)'])
                    ) if rows[idxs[-1]].get('Time(s)') and rows[idxs[0]].get('Time(s)') else 0.0
            avg_v = sum(entry['v']) / len(entry['v']) if entry['v'] else 0.0
            detail[stage] = {
                'seconds': round(span, 2),
                'frames': len(idxs),
                'distance_mm': round(entry['dist']),
                'avg_speed': round(avg_v, 1),
                'speed_util': round(avg_v / p95, 3),
            }
        if detail:
            out['stage_detail'] = detail
    return out


def analyse_orbit_deviation(rows, cfg):
    """
    绕圈段的「径向误差」分析 —— 这是绕圈跑不快的**真正原因**。

    原理：`test.py` 有一段保护——
        if abs(radial_error) > 40:  thrust = min(thrust, ORBIT_RECAPTURE_THRUST)
    而 fast 档 `THRUST_BASE` 已经是 50（=MAX_THRUST），所以只要鱼偏离
    目标圆周超过 40mm，推力就被从 50 砍到 39.2（约 -22%）。
    偏离时间占比越高，绕圈段就越慢 —— 而且这时**加曲率限速预算是无效的**，
    因为瓶颈根本不是「目标速度」，而是这条回收保护在反复踩刹车。

    本函数只做诊断，不改 test.py。
    """
    centers = cfg.get('orbit_centers') or {}
    threshold = float(cfg.get('orbit_recapture_threshold_mm', 40.0))
    recapture = float(cfg.get('orbit_recapture_thrust', 39.2))
    max_thrust = float(cfg.get('max_thrust', 50.0))
    # 计划绕行半径。**推力回收的分母就是这个值**（test.py 里减的是 LOOP_R），
    # 所以判定不能用实测中位数半径，否则会低估被判超限的帧数。
    loop_r = float(cfg.get('loop_r_mm', 200.0))
    out = {}
    if not centers:
        return {'available': False}
    for stage, center in centers.items():
        try:
            cx, cy = float(center[0]), float(center[1])
        except (TypeError, ValueError, IndexError):
            continue
        errs = []
        speeds = []
        for row in rows:
            if row.get('Stage') != stage:
                continue
            try:
                x = float(row['PosX'])
                y = float(row['PosY'])
            except (TypeError, ValueError, KeyError):
                continue
            errs.append(math.hypot(x - cx, y - cy))
            try:
                speeds.append(float(row['Speed']))
            except (TypeError, ValueError):
                pass
        if len(errs) < 5:
            continue
        # 相对**计划半径**的径向误差 —— 与 test.py 的判据一致
        dev = [r - loop_r for r in errs]
        over = [d for d in dev if abs(d) > threshold]
        actual_r = sorted(errs)[len(errs) // 2]
        out[stage] = {
            'frames': len(dev),
            'planned_radius_mm': round(loop_r, 1),
            'actual_radius_mm': round(actual_r, 1),
            'radius_overshoot_mm': round(actual_r - loop_r, 1),
            'dev_min': round(min(dev), 1),
            'dev_max': round(max(dev), 1),
            'dev_avg': round(sum(dev) / len(dev), 1),
            'over_fraction': round(len(over) / float(len(dev)), 3),
            'over_count': len(over),
            'avg_speed': round(sum(speeds) / len(speeds), 1) if speeds else 0.0,
            # 回收期推力相对满值的保留比例（0.784 = 只剩 78.4%）
            'thrust_retained': round(recapture / max_thrust, 3),
        }
    if not out:
        return {'available': False}
    out['available'] = True
    out['threshold_mm'] = threshold
    out['recapture_thrust'] = recapture
    # 顶层也放一份，供报告/结论直接引用（否则会取到默认值 0.0）
    out['thrust_retained'] = round(recapture / max_thrust, 3)
    return out


def analyse_system(samples):
    out = {'samples': len(samples)}
    for key in ('cpu_total', 'mem_percent', 'mem_used_mb', 'target_cpu_percent',
                'target_rss_mb', 'gpu_util', 'gpu_mem_used_mb', 'gpu_power_w',
                'net_sent_kbs', 'net_recv_kbs', 'disk_read_kbs', 'disk_write_kbs',
                'proc_uptime_min', 'boot_uptime_min'):
        vals = [safe_float(s.get(key), None) for s in samples if s.get(key) is not None]
        if vals:
            out[key] = _stats(vals)
    # 每个相关进程的「已运行分钟」（取最后一次采样的值，最接近报告时刻）
    for sample in reversed(samples):
        if sample.get('proc_lifetimes'):
            out['lifetimes'] = sample['proc_lifetimes']
            break
    peaks = [s for s in samples if s.get('target_cpu_percent') is not None]
    if peaks:
        worst = max(peaks, key=lambda s: safe_float(s.get('target_cpu_percent')))
        out['busiest_proc_sample'] = worst.get('proc_detail', '')
        out['proc_uptime_time'] = worst.get('proc_uptime_min', 0.0)
        out['proc_uptime_last'] = peaks[-1].get('proc_uptime_min', 0.0)
        out['proc_lifetime_last'] = peaks[-1].get('proc_lifetime', '')
        if len(peaks) > 1:
            span = safe_float(peaks[-1].get('elapsed')) - safe_float(peaks[0].get('elapsed'))
            out['monitored_span_sec'] = round(span, 1)
            start_up = safe_float(peaks[0].get('proc_uptime_min'))
            end_up = safe_float(peaks[-1].get('proc_uptime_min'))
            # 进程运行分钟数的实际增量，应当 ≈ 监视时长（误差说明进程中途重启过）
            out['proc_uptime_delta_min'] = round(end_up - start_up, 2)
    gpu_line = next((s.get('gpu_all') for s in reversed(samples) if s.get('gpu_all')), '')
    out['gpu_last'] = gpu_line
    return out


def detect_bottlenecks(sysstat, tel, scores, cfg):
    """给出一组"嫌疑瓶颈"，每条含依据。全部基于实测量，不猜。"""
    findings = []
    cpu = sysstat.get('target_cpu_percent', {})
    gpu = sysstat.get('gpu_util', {})
    cpu_total = sysstat.get('cpu_total', {})
    single_core = 100.0

    if cpu.get('avg', 0) >= 0.85 * single_core:
        findings.append((
            'medium', '策略进程吃满单核',
            'test.py 进程平均 CPU {:.0f}%（单核饱和线 100%），峰值 {:.0f}%。'
            'Python 主循环逐帧算路径+PID，是典型单核瓶颈。'.format(cpu['avg'], cpu['max'])))
    elif cpu.get('avg', 0) >= 0.55 * single_core:
        findings.append((
            'low', '策略进程 CPU 偏高',
            'test.py 进程平均 {:.0f}%，尚未打满单核，但已是主要占用方。'.format(cpu['avg'])))
    if cpu_total.get('avg', 0) >= 75.0:
        findings.append((
            'medium', '整机 CPU 高负载',
            '整机 CPU 平均 {:.0f}%、峰值 {:.0f}%；仿真器与策略在同机竞争。'.format(
                cpu_total['avg'], cpu_total['max'])))
    if gpu.get('avg', 0) >= 60.0:
        findings.append((
            'low', 'GPU 负载不低',
            'GPU 利用率平均 {:.0f}%、峰值 {:.0f}%。若策略没用到 GPU，'
            '这部分是仿真渲染占用，属正常。'.format(gpu['avg'], gpu['max'])))
    elif gpu and gpu.get('avg', 0) < 15.0:
        findings.append((
            'info', 'GPU 基本空闲',
            'GPU 利用率平均 {:.0f}%，说明瓶颈**不是**渲染/算力，'
            '而在 CPU 侧的逐帧计算与控制回路上。'.format(gpu['avg'])))

    if tel.get('available'):
        sp = tel.get('speed', {})
        if sp:
            findings.append((
                'info', '实际速度与命令速度的差距',
                '遥测实测速度 p50={} p95={} max={} mm/s，速度低于峰值一半的时间占 {:.0%}。'
                '（命令速度是目标值，不一定达得到。）'.format(
                    sp.get('p50'), sp.get('p95'), sp.get('max'),
                    tel.get('slow_fraction', 0.0))))
        stages = tel.get('stage_seconds') or {}
        if stages:
            worst = max(stages.items(), key=lambda kv: kv[1])
            findings.append((
                'info', '最耗时的阶段',
                '分段耗时 {}，其中 `{}` 最久（{:.2f}s）。优化优先动这一段。'.format(
                    json.dumps(stages, ensure_ascii=False), worst[0], worst[1])))
        detail = tel.get('stage_detail') or {}
        p95 = (tel.get('speed') or {}).get('p95', 0.0)
        if detail and p95:
            # 「没跑起来」的段：速度利用率低说明这一段主要输在速度，而不是距离
            slow = [(name, info) for name, info in detail.items()
                    if info.get('speed_util', 1.0) < 0.80 and info.get('distance_mm')]
            if slow:
                lines = ['`{}`：{}mm 用了 {:.2f}s，平均 {}mm/s，'
                         '速度利用率仅 {:.0%}'.format(
                             name, info['distance_mm'], info['seconds'],
                             info['avg_speed'], info['speed_util'])
                         for name, info in sorted(slow, key=lambda kv: -kv[1]['seconds'])]
                findings.append((
                    'high', '这几段「没跑起来」（速度利用率低）',
                    '{}。速度利用率 = 该段平均速度 / 全程 P95（{}mm/s）。'
                    '说明这些段的时间不是花在「路远」，而是花在「跑不快」——'
                    '优先查入弯减速、曲率限速偏保守、以及绕圈时的航向振荡。'.format(
                        '；'.join(lines), p95)))
            loss = []
            for name, info in detail.items():
                if info.get('distance_mm') and info.get('avg_speed'):
                    ideal = info['distance_mm'] / p95
                    if info['seconds'] - ideal > 0.3:
                        loss.append((name, info['seconds'] - ideal))
            if loss:
                loss.sort(key=lambda kv: -kv[1])
                total_loss = sum(v for _, v in loss)
                findings.append((
                    'high', '理论可压缩时间合计 {:.2f}s'.format(total_loss),
                    '若每段都达到全程 P95 速度，可省：{}。'
                    '注意这是**上界估算**（弯道必然比直道慢），'
                    '实际能拿到一部分就已经很可观。'.format(
                        '；'.join('`{}` -{:.2f}s'.format(n, v) for n, v in loss))))

    # 绕圈径向误差 → 推力回收保护（绕圈跑不快的真正原因）
    orbit = tel.get('orbit') or {}
    if orbit.get('available'):
        thr = orbit.get('threshold_mm', 40.0)
        recapture = orbit.get('recapture_thrust', 39.2)
        bad = [(n, i) for n, i in orbit.items()
               if isinstance(i, dict) and i.get('over_fraction', 0) > 0.25]
        if bad:
            bad.sort(key=lambda kv: -kv[1]['over_fraction'])
            body = '；'.join(
                '`{}` 有 {:.0%} 的帧偏出圆周 >{}mm'.format(
                    n, i['over_fraction'], thr) for n, i in bad)
            findings.append((
                'high', '绕圈推力被「半径回收保护」反复踩刹车',
                '{}。`test.py` 里 `abs(radial_error) > {}` 时把推力从 50 钳到 '
                '{}（**推力只有满值的 {:.0%}**）。所以绕圈段慢，不是目标速度不够，'
                '而是这条保护在持续降推力。实测绕圈半径比计划大：{}。'
                '**结论：单纯提高曲率限速预算对这两段无效**，'
                '要么把圆周跟得更准（减小径向误差），要么放宽回收阈值/回收推力。'.format(
                    body, thr, recapture, orbit.get('thrust_retained', 0.0),
                    '；'.join('`{}` 实测 {:.0f}mm vs 计划 {:.0f}mm'.format(
                        n, i['actual_radius_mm'], i['planned_radius_mm'])
                        for n, i in bad))))
        z = tel.get('z') or {}
        if z.get('drift', 0) > 80.0:
            findings.append((
                'medium', '高度漂移偏大',
                '全程 z 在 {:.0f}~{:.0f}mm 之间波动（漂移 {:.0f}mm）。'
                '若已锁定高度，说明高度环仍有稳态偏差。'.format(
                    z.get('min', 0), z.get('max', 0), z['drift'])))
    else:
        findings.append((
            'warn', '没有拿到逐帧遥测',
            '未找到 race_telemetry.csv。它是 v7.2 起 test.py 自动写的；'
            '若你的 test.py 更旧，跑一次最新版即可生成，能大幅提高诊断精度。'))

    if scores:
        recent = [s for _, s, _ in scores if s is not None][-8:]
        if recent:
            findings.append((
                'info', '最近成绩',
                '最近 {} 次有效成绩 {} s，最好 {:.2f}s，平均 {:.2f}s。'.format(
                    len(recent), [round(v, 2) for v in recent],
                    min(recent), sum(recent) / len(recent))))
    return findings


def read_strategy_snapshot(test_py_path):
    """
    只读扫描 test.py，抓出关键常量，便于报告里记录"这一轮跑的是哪套参数"。
    **本函数绝不写回 test.py。**
    """
    if not os.path.isfile(test_py_path):
        return {}
    wanted = ['SPEED_GAIN', 'LOOP_R', 'HOLD_Z', 'START_Z', 'MAX_THRUST', 'MAX_TAIL',
              'TAIL_STRAIGHT_HZ', 'TAIL_TURN_HZ', 'TAIL_AMPLITUDE_STRAIGHT',
              'TAIL_AMPLITUDE_TURN', 'GAP_TAIL_AMPLITUDE', 'CRUISE_SPEED',
              'TURN_SPEED', 'GAP_SPEED', 'CURVE_ARM_RATIO', 'KI_HEIGHT',
              'INTEGRAL_LIMIT', 'PITCH_LIMIT_NORMAL', 'PROFILE_NAME']
    out = {}
    try:
        with open(test_py_path, 'r', encoding='utf-8') as handle:
            text = handle.read()
    except (IOError, OSError, UnicodeDecodeError):
        return out
    for name in wanted:
        m = re.search(r'^{}\s*=\s*([^#\n]+)'.format(re.escape(name)), text, re.M)
        if m:
            out[name] = m.group(1).strip()
    out['_lines'] = str(text.count('\n') + 1)
    return out


# ======================================================================
# 提速 / 算法改进建议（基于观测结论，逐条给依据）
# ======================================================================
def _patch_candidates(tel, sysstat):
    """
    生成「候选补丁」清单：每条包含改动点、依据（实测数字）、风险。

    只产出**文本建议**，不生成也不应用 patch。用户点名某条后，
    再由主代理去精确改 test.py 并跑自测。
    """
    out = []
    detail = (tel.get('stage_detail') or {}) if tel.get('available') else {}
    p95 = (tel.get('speed') or {}).get('p95', 0.0)
    orbit = (tel.get('orbit') or {}) if tel.get('available') else {}

    # 优先级最高的候选：绕圈径向误差触发的推力回收
    if orbit.get('available'):
        hit = [(n, i) for n, i in orbit.items()
               if isinstance(i, dict) and i.get('over_fraction', 0) > 0.25]
        if hit:
            hit.sort(key=lambda kv: -kv[1]['over_fraction'])
            out.append((
                'B4-0（最高优先）：处理绕圈「半径回收保护」踩刹车',
                '实测 {}。`test.py` 里 `if abs(radial_error) > {}: '
                'thrust = min(thrust, ORBIT_RECAPTURE_THRUST)`，'
                '而 fast 档 `THRUST_BASE` 已经是 50（=MAX_THRUST），'
                '所以一偏出圆周推力就被砍到 {}（**只剩满值的 {:.0%}**）。'
                '实测绕圈半径比计划大（{}），说明鱼确实长期跑在外侧。'.format(
                    '；'.join('`{}` 偏出 {:.0%}'.format(n, i['over_fraction'])
                              for n, i in hit),
                    orbit.get('threshold_mm', 40.0),
                    orbit.get('recapture_thrust', 39.2),
                    orbit.get('thrust_retained', 0.0),
                    '、'.join('`{}` {:.0f}mm vs 计划 {:.0f}mm'.format(
                        n, i['actual_radius_mm'], i['planned_radius_mm'])
                        for n, i in hit)),
                '三种改法各有代价，需实跑对比：'
                '① 把回收阈值从 40 放宽到 60~70（改动最小，风险是离心更远、'
                '可能更贴柱，必须跑 `test_geometry` 的 55mm 间隙自测）；'
                '② 提高 `ORBIT_RECAPTURE_THRUST`（现 39.2）到 45 左右，'
                '让回收期也能维持较高推力；'
                '③ 从控制上减小径向误差（`_guidance` 里的 `3.0*radius_error` 权重可加），'
                '这是最「正确」但改动最大的一条。'))

    if detail and p95:
        # 找速度利用率最低、且耗时最长的那一段 —— 最值得动的一处
        cand = [(n, i) for n, i in detail.items()
                if i.get('distance_mm') and i.get('speed_util', 1.0) < 0.80]
        if cand:
            name, info = max(cand, key=lambda kv: kv[1]['seconds'])
            out.append((
                'B4-1（次优先）：提高 `{}` 段的目标速度 / 曲率限速预算'.format(name),
                '该段实测 {}mm / {:.2f}s，平均 {}mm/s，速度利用率仅 {:.0%}'
                '（全程 P95={}mm/s）。`test.py` 里 `TURN_ACCEL_BUDGET` 决定曲率限速 '
                '`sqrt(budget / curvature)`，理论上提高 10~15% 可抬升绕圈目标速度。'
                '**但注意**：若 B4-0 成立（绕圈长期偏出圆周），'
                '目标速度根本不是限制项 —— 这条的收益会很小，'
                '必须先验证 B4-0 的结论或与它一起改。'.format(
                    info['distance_mm'], info['seconds'], info['avg_speed'],
                    info['speed_util'], p95),
                '提高预算会加大绕圈时的离心外冲 → 航迹变长、离柱更近，'
                '**可能让 B4-0 的推力回收更频繁**（目标速度更高→更偏离圆周）。'
                '`test_geometry` 要求所有采样点离柱 ≥55mm，必须跑自测确认不破线。'))
        # 入段加速：第一段的利用率通常最低（刚从静止起步）
        first = detail.get('approach_1')
        if first and first.get('speed_util', 1.0) < 0.70:
            out.append((
                'B4-2：缩短起步段 `approach_1`（{:.2f}s，利用率仅 {:.0%}）'.format(
                    first['seconds'], first['speed_util']),
                '起步段 {}mm 却用了 {:.2f}s，平均只有 {}mm/s；'
                '这是从静止加速的过程。可考虑：起点到入圈点之间用更短的直线'
                '（减小入圈前的绕行弧），或让入圈点更靠近柱体切点。'.format(
                    first['distance_mm'], first['seconds'], first['avg_speed']),
                '入圈点前移会改变入圈切向，可能影响绕圈初期的姿态稳定。需自测 + 实跑。'))

    z = (tel.get('z') or {}) if tel.get('available') else {}
    if z.get('drift', 0) > 80.0:
        out.append((
            'C1：收紧高度环（当前漂移 {:.0f}mm）'.format(z['drift']),
            '实测 z 在 {:.0f}~{:.0f}mm 波动。既然已锁定 `HOLD_Z`，'
            '漂移说明高度环在持续低头偏置下守不住。可加大 `KI_HEIGHT`'
            '（现 0.25）或 `INTEGRAL_LIMIT`（现 30）。'.format(
                z.get('min', 0), z.get('max', 0)),
            '过强的积分会与俯仰控制耦合产生振荡；且高度修正本身消耗推进力，'
            '**可能反而变慢**。建议先小步试（KI 0.25→0.35）。'))

    cpu = sysstat.get('target_cpu_percent', {}) if sysstat else {}
    if cpu.get('avg', 0) >= 60.0:
        out.append((
            'A1：把逐帧重算搬到「只在分段切换时做」',
            '策略进程平均 CPU {:.0f}%、峰值 {:.0f}%，已接近单核饱和。'
            'Python 每帧都在算前瞻点与 PID，属于纯 CPU 开销。'.format(
                cpu['avg'], cpu['max']),
            '注意：本机实测回调只有 ~59Hz（目标 60Hz），'
            '**目前看 CPU 并没有拖慢控制频率**，所以这条对成绩的收益可能很小，'
            '优先级应低于 B4。'))
    return out


def build_recommendations(sysstat, tel, findings):
    tips = []
    kinds = {f[1] for f in findings}
    cpu = sysstat.get('target_cpu_percent', {})
    gpu = sysstat.get('gpu_util', {})
    gpu_busy = bool(gpu) and gpu.get('avg', 0) >= 40.0

    # ---- A. 控制回路 ----
    tips.append(('A1', 'P0', '把逐帧计算从主循环里搬出去（只在直道做重算）',
                 'Python 主循环每帧都算前瞻点 + PID + 深度级联。'
                 '建议：路径曲率、终点判定、目标点这些"不随帧变化"的量只在**分段切换**时重算一次；'
                 '直道段直接把航向误差喂给 PID，省掉每帧的采样求距离。'
                 '预计省 20~35% 的单核占用（需实测确认）。'))
    tips.append(('A2', 'P1', 'PID 用增量式 / 预计算增益，避免每帧重复乘法',
                 '`error` 计算里每帧调用 `math.degrees/radians` 与 `_wrap`。'
                 '把角度系数预乘成常量、把 `degrees()` 合并进增益，可省掉每帧若干三角函数。'))
    tips.append(('A3', 'P1', '延迟与抖动：把回调内的日志/格式化移到低频',
                 '回调里每次 stage 变化都会拼长字符串并 print。'
                 '已改成只在阶段切换时打印；若 `--debug` 打开则是**每秒**打印，'
                 '确认比赛时**不要**开 `--debug`（它会明显增加抖动）。'))

    # ---- B. 推进/速度 ----
    if 'info' in kinds:
        sp = tel.get('speed', {}) or {}
        if sp.get('max'):
            tips.append(('B1', 'P0', '实际速度上不去：先确认是否撞到平台 VelLimit',
                         '遥测峰值 {} mm/s。把尾摆频率/幅值继续加大后若峰值**不再上升**，'
                         '说明撞上了平台蓝图的轴速度上限，加尾摆无效；'
                         '此时唯一有效手段是缩短路径（见 B3/B4）。'.format(sp.get('max'))))
    tips.append(('B2', 'P1', '尾摆频率不要超过安全区',
                 '尾摆是唯一推进杠杆（手册：频率与动力成正比）。'
                 '但频率过高会让单周期摆幅变小、推进反而下降。'
                 '建议以 0.2Hz 为步长做二分扫描，用遥测峰值速度作反馈，找到拐点。'))

    # ---- C. 路径（最可能的大头） ----
    if tel.get('available') and tel.get('path_length_mm'):
        tips.append(('B3', 'P0', '路径过冲：实测航迹明显长于规划路径',
                     '实测航迹 {} mm。规划路径约 6522mm。差值主要来自绕圈时的离心外冲。'
                     '手段：把绕行半径做到"贴着安全间隙 55mm"（当前 LOOP_R=200 的间隙约 59mm，'
                     '已很近），或改用**圆弧+切向**入圈（减少进圈的甩尾）。'
                     '每省 500mm ≈ 省 1.0~1.3s。'.format(tel.get('path_length_mm'))))
    # B4 用实测的分段数据说话：指出「哪段最久 + 那段速度利用率多少」
    detail = (tel.get('stage_detail') or {}) if tel.get('available') else {}
    if detail:
        worst = max(detail.items(), key=lambda kv: kv[1]['seconds'])
        name, info = worst
        total = sum(v['seconds'] for v in detail.values()) or 1.0
        p95 = (tel.get('speed') or {}).get('p95', 0.0)
        ideal = (info['distance_mm'] / p95) if (p95 and info.get('distance_mm')) else 0.0
        # 把绕圈段的实测径向误差写进结论里；这是「为什么慢」的直接答案
        orbit = tel.get('orbit') or {}
        orbit_note = '无绕圈径向数据。'
        if orbit.get('available'):
            hit = [(n, i) for n, i in orbit.items()
                   if isinstance(i, dict) and i.get('over_fraction', 0) > 0.25]
            if hit:
                hit.sort(key=lambda kv: -kv[1]['over_fraction'])
                orbit_note = (
                    '**真正的瓶颈在这里** —— '
                    '{}。`test.py` 在 `abs(radial_error) > {}mm` 时'
                    '把推力从 50 钳到 {}（-{:.0%}）。所以这两段慢'
                    '**不是目标速度不够**，而是半径回收保护在持续降推力。'
                    '方向应是「把圆周跟得更准」或「放宽回收阈值/回收推力」，'
                    '而不是继续加大 `TURN_ACCEL_BUDGET`。'.format(
                        '；'.join('`{}` 有 {:.0%} 的帧偏出圆周 >{}mm'.format(
                            n, i['over_fraction'], orbit.get('threshold_mm', 40.0))
                            for n, i in hit),
                        orbit.get('threshold_mm', 40.0),
                        orbit.get('recapture_thrust', 39.2),
                        orbit.get('thrust_retained', 0.0)))
            else:
                orbit_note = '绕圈径向误差不大（偏出帧 <25%），可优先动速度参数。'
        tips.append((
            'B4', 'P0',
            '最耗时的是 `{}`（{:.2f}s，占全程 {:.0%}），优先压它'.format(
                name, info['seconds'], info['seconds'] / total),
            '实测：该段 {}mm、耗时 {:.2f}s、平均 {}mm/s，**速度利用率 {:.0%}**'
            '（全程 P95 为 {}mm/s）。若这一段能按 P95 跑完只要 {:.2f}s。'
            '在**不减小半径**（否则贴柱）的前提下，按见效顺序：'
            '① 提高绕圈的目标速度（曲率限速里的 `TURN_ACCEL_BUDGET`）；'
            '② 入圈前把速度提起来，用切向入圈而不是先减速再转；'
            '③ 减少绕圈过程中的航向振荡（圆周上尾摆幅值应当更小）。'
            '④ **（实测指向的真正瓶颈）** {}'.format(
                info['distance_mm'], info['seconds'], info['avg_speed'],
                info['speed_util'], p95, ideal, orbit_note)))
    else:
        tips.append(('B4', 'P0', '绕圈段是最大时间块，优先压它',
                     '绕位置 3 要转 2 圈，通常占全程约 40%。'
                     '在**不减小半径**（否则贴柱）的前提下，可行手段有：'
                     '① 提高绕圈时的目标速度（曲率限速里的 `TURN_ACCEL_BUDGET`）；'
                     '② 入圈前把速度提起来，用切向入圈而不是先减速再转；'
                     '③ 减少绕圈过程中的航向振荡（尾摆幅值在圆周上应更小）。'))

    # ---- D. 系统层面 ----
    if not gpu_busy:
        tips.append(('D1', 'P2', '瓶颈在 CPU 而非 GPU，别往 GPU 方向优化',
                     'GPU 利用率平均 {:.0f}%，基本空闲。'
                     '可以放心地把优化集中在控制回路与路径上。'.format(gpu.get('avg', 0))))
    tips.append(('D2', 'P2', '进程亲和性与优先级',
                 '仿真器（Fish427 + Shipping，共 2 个进程）与策略在同机。'
                 '可用**高优先级 + 绑定不同核心**减少互相抢占（本工具可扩展采集，'
                 '但改优先级属于系统级操作，需你手动决定）。'))
    return tips


# ======================================================================
# 报告输出
# ======================================================================
def write_report(outdir, run_id, cfg, sysstat, tel, scores, findings, tips,
                 threads, events, snapshot, root_pids, logs=None):
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, 'report_{}.md'.format(run_id))
    lines = []
    add = lines.append

    add('# 机器鱼科目一 · 监控与瓶颈分析报告')
    add('')
    add('- 生成时间：{}'.format(now_stamp()))
    add('- 采样轮次：{}'.format(run_id))
    add('- 主机 Python：{}'.format(sys.version.split()[0]))
    add('- 相关进程 PID：{}'.format(root_pids or '（本轮未发现）'))
    if snapshot:
        add('- 本轮 test.py 行数：{}'.format(snapshot.get('_lines', '?')))
    add('')

    add('## 1. 关键结论（先看这里）')
    add('')
    if findings:
        for level, title, detail in findings:
            add('- **[{0}] {1}** —— {2}'.format(level.upper(), title, detail))
    else:
        add('- 本轮没有发现明显异常。')
    add('')

    add('## 2. 资源占用（实测）')
    add('')
    add('| 指标 | 平均 | P50 | P95 | 峰值 |')
    add('| --- | --- | --- | --- | --- |')
    label = [('cpu_total', '整机 CPU %'), ('target_cpu_percent', '策略进程 CPU %'),
             ('mem_percent', '内存占用 %'), ('target_rss_mb', '策略进程 RSS MB'),
             ('gpu_util', 'GPU 利用率 %'), ('gpu_mem_used_mb', 'GPU 显存 MB'),
             ('gpu_power_w', 'GPU 功耗 W'), ('net_recv_kbs', '网络接收 KB/s')]
    for key, name in label:
        st = sysstat.get(key)
        if st and st.get('n'):
            add('| {} | {} | {} | {} | {} |'.format(name, st['avg'], st['p50'],
                                                    st['p95'], st['max']))
    add('')
    if sysstat.get('gpu_last'):
        add('GPU：`{}`'.format(sysstat['gpu_last']))
        add('')
    if sysstat.get('busiest_proc_sample'):
        add('最忙时刻的进程明细：`{}`'.format(sysstat['busiest_proc_sample']))
        add('')

    add('## 2.1 运行时长统计（分钟）')
    add('')
    boot = sysstat.get('boot_uptime_min', {})
    if boot:
        add('- **整机已运行**：{:.1f} 分钟（≈{:.2f} 小时）'.format(
            boot.get('max', 0.0), boot.get('max', 0.0) / 60.0))
    span = sysstat.get('monitored_span_sec')
    if span:
        add('- **本轮实际监视时长**：{:.1f} 秒（≈{:.2f} 分钟）'.format(
            span, span / 60.0))
    up = sysstat.get('proc_uptime_min', {})
    if up:
        add('- **监控进程已运行**（本轮结束时）：{:.1f} 分钟（≈{:.2f} 小时）；'
            '本轮监视期内增加 {:.2f} 分钟'.format(
                sysstat.get('proc_uptime_last', up.get('max', 0.0)),
                sysstat.get('proc_uptime_last', 0.0) / 60.0,
                sysstat.get('proc_uptime_delta_min', 0.0)))
    if sysstat.get('proc_lifetime_last'):
        add('')
        add('| PID | 进程 | 启动时间 | 已运行(分钟) |')
        add('| --- | --- | --- | --- |')
        for item in (sysstat.get('lifetimes') or []):
            add('| {} | {} | {} | {:.1f} |'.format(
                item['pid'], item['name'], item['started'], item['uptime_min']))
        add('')
        add('> 进程启动时间取自系统 `create_time()`，是本轮测试期间进程的真实寿命，'
            '不是「被本工具发现的时刻」。')
        add('')
    add('')

    add('## 3. 进程 / 线程运行情况')
    add('')
    add('采样数：{}（系统级，间隔 {:.1f}s）'.format(
        sysstat.get('samples', 0), cfg.get('sample_interval_sec', 1.0)))
    add('')
    # 进程级：CPU 时间增长 = 这段时间真正用掉的 CPU 秒数
    proc_rows = {}
    for row in threads:
        if row.get('tid') != -1:
            continue
        key = row.get('pid')
        cpu = safe_float(row.get('cpu_delta'))
        entry = proc_rows.setdefault(key, {'name': row.get('name', ''), 'cpu': 0.0,
                                           'cores': [], 'n': 0})
        entry['cpu'] += cpu
        entry['n'] += 1
        if row.get('effective_cores') != '':
            entry['cores'].append(safe_float(row.get('effective_cores')))
    if proc_rows:
        add('进程级并行度（按进程累计 CPU 秒 / 平均等效核数）：')
        add('')
        add('| PID | 进程 | 累计 CPU 秒 | 平均等效核数 | 峰值等效核数 |')
        add('| --- | --- | --- | --- | --- |')
        for pid, entry in sorted(proc_rows.items(), key=lambda kv: -kv[1]['cpu']):
            cores = entry['cores']
            if cores:
                avg_core = '{:.2f}'.format(sum(cores) / len(cores))
                max_core = '{:.2f}'.format(max(cores))
            else:
                avg_core = max_core = '需 ≥2 次线程采样'
            add('| {} | {} | {:.2f} | {} | {} |'.format(
                pid, entry['name'], entry['cpu'], avg_core, max_core))
        add('')
        add('> **等效核数 = 该进程在这段时间里平均占用的 CPU 核数**。'
            '接近 1.0 说明串行、吃满一个核；远小于 1.0 说明大部分时间在等。')
        add('')
    if threads:
        agg = {}
        for row in threads:
            if row.get('cpu_delta') == '' or row.get('tid') == -1:
                continue
            key = (row['pid'], row['tid'])
            agg[key] = agg.get(key, 0.0) + safe_float(row.get('cpu_delta'))
        top = sorted(agg.items(), key=lambda kv: kv[1], reverse=True)[:int(cfg.get('report_top_n', 8))]
        if top:
            add('CPU 时间增长最快的线程（{:.0f}s 窗口内累计 CPU 秒）：'.format(
                cfg.get('trial_idle_timeout_sec', 45.0)))
            add('')
            add('| PID | TID | 累计 CPU 秒 |')
            add('| --- | --- | --- |')
            for (pid, tid), value in top:
                add('| {} | {} | {:.3f} |'.format(pid, tid, value))
            add('')
            add('> 线程多且集中在少数 TID 上，说明计算是串行的；'
                '若某 TID 长期占满一个核，即为控制回路（A1/A2 针对它）。')
            add('')
    else:
        add('（本轮未采集到线程明细；把 `thread_sample_every` 调小可提高密度）')
        add('')

    add('## 4. 迭代 / 处理速率')
    add('')
    if sysstat.get('samples'):
        # 监控自身也提供一路「迭代速率」：采集循环实际达到的频率
        span = safe_float(sysstat.get('monitored_span_sec'), 0.0)
        if span > 0:
            add('- 监控采样：{} 次 / {:.1f}s → 实际 {:.2f} Hz'
                '（配置目标 {:.2f} Hz，即间隔 {}s）'.format(
                    sysstat['samples'], span, sysstat['samples'] / span,
                    1.0 / max(1e-6, float(cfg.get('sample_interval_sec', 1.0))),
                    cfg.get('sample_interval_sec', 1.0)))
            add('')
    if tel.get('available'):
        add('- 遥测帧数：{}'.format(tel.get('frames')))
        if tel.get('trial_seconds'):
            add('- 这轮策略运行时长：{} s'.format(tel['trial_seconds']))
        if tel.get('frame_hz'):
            add('- 控制回调频率（估）：{} Hz'.format(tel['frame_hz']))
        if tel.get('path_length_mm'):
            add('- 实际航迹长度：{} mm'.format(tel['path_length_mm']))
        sp = tel.get('speed') or {}
        if sp:
            add('- 实际速度 mm/s：p50={} p95={} max={}'.format(
                sp.get('p50'), sp.get('p95'), sp.get('max')))
        if tel.get('stage_seconds'):
            add('- 分段耗时：')
            add('')
            add('| 阶段 | 秒 |')
            add('| --- | --- |')
            for stage, sec in tel['stage_seconds'].items():
                add('| `{}` | {} |'.format(stage, sec))
            add('')
        detail = tel.get('stage_detail') or {}
        if detail:
            total = sum(v['seconds'] for v in detail.values()) or 1.0
            add('- 分段明细（**速度利用率 = 该段平均速度 ÷ 全程 P95 速度**，'
                '越低说明这段越没跑起来）：')
            add('')
            add('| 阶段 | 秒 | 占比 | 里程 mm | 平均速度 mm/s | 速度利用率 |')
            add('| --- | --- | --- | --- | --- | --- |')
            for stage, info in sorted(detail.items(), key=lambda kv: -kv[1]['seconds']):
                add('| `{}` | {} | {:.0%} | {} | {} | {:.2f} |'.format(
                    stage, info['seconds'], info['seconds'] / total,
                    info['distance_mm'], info['avg_speed'], info['speed_util']))
            add('')
            # 把「最久的那一段」换算成「如果按全程峰值速度跑要多久」
            worst = max(detail.items(), key=lambda kv: kv[1]['seconds'])
            name, info = worst
            p95 = (tel.get('speed') or {}).get('p95', 0.0)
            if p95 and info['distance_mm']:
                best = info['distance_mm'] / p95
                add('> **最久的一段是 `{}`：{:.2f}s / {}mm（平均 {}mm/s）。'
                    '若这一段能按全程 P95（{}mm/s）跑，只需 {:.2f}s，'
                    '可省约 {:.2f}s。**这是当前最大的单点提速空间。'.format(
                        name, info['seconds'], info['distance_mm'], info['avg_speed'],
                        p95, best, info['seconds'] - best))
                add('')
        orbit = tel.get('orbit') or {}
        if orbit.get('available'):
            add('- 绕圈径向误差（**绕圈跑不快的真正原因**）：')
            add('')
            add('| 阶段 | 计划半径 | 实测半径 | 平均径向误差 | 偏出 >{}mm 的帧 | 该段平均速度 |'.format(
                orbit.get('threshold_mm', 40.0)))
            add('| --- | --- | --- | --- | --- | --- |')
            for stage, info in orbit.items():
                if not isinstance(info, dict):
                    continue
                add('| `{}` | {:.0f}mm | {:.0f}mm | {:+.1f}mm | {:.0%} | {}mm/s |'.format(
                    stage, info['planned_radius_mm'], info['actual_radius_mm'],
                    info['dev_avg'], info['over_fraction'], info['avg_speed']))
            add('')
            add('> `test.py` 的 `if abs(radial_error) > {}: thrust = min(thrust, {})` '
                '会在鱼偏离圆周时把推力从 50 砍到 {}（**只剩满值的 {:.0%}**）。'
                '偏出的帧占比越高，这一段就越慢——**这与目标速度无关**，'
                '所以只调曲率限速预算是治不好的。'.format(
                    orbit.get('threshold_mm', 40.0),
                    orbit.get('recapture_thrust', 39.2),
                    orbit.get('recapture_thrust', 39.2),
                    orbit.get('thrust_retained', 0.0)))
            add('')
    else:
        add('未找到 `race_telemetry.csv`，无法测算迭代速率。'
            '用 v7.2 及以后的 test.py 跑一次即可自动生成。')
        add('')

    add('## 5. 成绩记录（project1.csv）')
    add('')
    if scores:
        valid = [s for _, s, _ in scores if s is not None]
        add('- 记录总数：{}，其中有效 {} 条'.format(len(scores), len(valid)))
        if valid:
            add('- 最好 {:.2f}s，最差 {:.2f}s，平均 {:.2f}s'.format(
                min(valid), max(valid), sum(valid) / len(valid)))
        add('')
        add('| 时间 | 成绩 |')
        add('| --- | --- |')
        for stamp, sec, _ in scores[-int(cfg.get('report_top_n', 8)):]:
            add('| {} | {} |'.format(stamp, sec if sec is not None else '无效'))
        add('')
    else:
        add('未读到成绩文件。')
        add('')

    add('## 6. 提速方案与算法改进建议')
    add('')
    add('优先级：P0 立刻做 / P1 值得做 / P2 有余力再做。')
    add('')
    for code, prio, title, detail in tips:
        add('### {} [{}] {}'.format(code, prio, title))
        add('')
        add(detail)
        add('')

    add('## 7. 事件时间线')
    add('')
    if events:
        for ev in events:
            add('- `{}` **{}** {}'.format(ev['t'], ev['kind'], ev['detail']))
    else:
        add('- （无）')
    add('')

    add('## 7.1 采集到的日志（尾部增量，只读）')
    add('')
    if logs:
        by_file = {}
        for row in logs:
            by_file.setdefault(row.get('file', '?'), []).append(row)
        for name, rows in by_file.items():
            add('### {} （{} 行）'.format(name, len(rows)))
            add('')
            add('```text')
            for row in rows[-40:]:
                add('{} {}'.format(row.get('t', ''), row.get('line', '')))
            add('```')
            add('')
    else:
        add('（本轮没有采集到新增日志行；日志采集只取运行期间的新增内容，'
            '不重读历史文件。可在 `fishmon_config.json` 里改 `log_paths` / `log_dirs`。）')
        add('')

    if snapshot:
        add('## 8. 本轮策略参数快照（只读提取，未修改 test.py）')
        add('')
        add('| 常量 | 值 |')
        add('| --- | --- |')
        for key, value in snapshot.items():
            if key.startswith('_'):
                continue
            add('| `{}` | {} |'.format(key, value))
        add('')

    add('---')
    add('')
    add('### 重要说明')
    add('')
    add('本报告只做**观测与分析**：全程没有修改 `test.py`，'
        '也没有操作仿真界面。所有数值都是实测采样，'
        '但"预计能省多少"属于**推断**，必须由实跑验证。')
    add('')

    add('### 可选的补丁片段（需要你同意才应用，本工具不会自动改 test.py）')
    add('')
    add('下面给出**候选改动**，以及每一处的依据与风险。'
        '要应用时请明确说"应用 B4-1"，我再生成精确 patch 并跑自测。')
    add('')
    for pid_, why, risk in _patch_candidates(tel, sysstat):
        add('#### {}'.format(pid_))
        add('')
        add('- 依据：{}'.format(why))
        add('- 风险：{}'.format(risk))
        add('')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    return path


def write_csv(outdir, run_id, name, rows, header=None):
    if not rows:
        return None
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, '{}_{}.csv'.format(name, run_id))
    if header is None:
        # 表头取所有行的键的并集（保持首次出现顺序）。
        # 若只看 rows[0]，首样本缺失的列（例如 disk_*、gpu_*）会整列丢失。
        seen = {}
        for row in rows:
            for key in row:
                seen[key] = None
        header = list(seen.keys())

    def flat(value):
        """把 list/dict 压成紧凑文本，标量原样返回（CSV 只认标量）。"""
        if isinstance(value, (list, tuple, dict)):
            return json.dumps(value, ensure_ascii=False, default=str)
        return value

    with open(path, 'w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=header, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({key: flat(row.get(key, '')) for key in header})
    return path


# ======================================================================
# 主流程
# ======================================================================
def list_targets(finder):
    roots, procs = finder.find()
    print('发现 {} 个相关进程（含子进程共 {} 个）：'.format(len(roots), len(procs)))
    for proc in procs:
        try:
            with proc.oneshot():
                print('  PID {:>7}  {:<28} threads={:<4} rss={:>8.1f}MB  {}'.format(
                    proc.pid, proc.name(), proc.num_threads(),
                    proc.memory_info().rss / 1048576.0,
                    ' '.join(proc.cmdline())[:90]))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return roots, procs


def run(args, cfg):
    if psutil is None:
        print('ERROR: 需要 psutil。请用带 psutil 的解释器运行，例如：')
        print(r'  D:\Python311\python.exe fishmon.py')
        return 2

    gpu = GpuSampler()
    finder = ProcessFinder(cfg)
    watcher = FileWatcher(cfg)
    logtail = LogTailer(cfg)
    sampler = Sampler(cfg, gpu, finder)
    sampler._t0 = time.monotonic()
    interval = max(0.2, float(cfg.get('sample_interval_sec', 1.0)))

    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    outdir = resolve(cfg.get('output_dir', 'monitor_reports'))

    print('=' * 68)
    print('机器鱼科目一 · 监控工具 fishmon.py   （只观测，不修改 test.py）')
    print('=' * 68)
    print('工作目录   : {}'.format(HERE))
    print('报告目录   : {}'.format(outdir))
    print('GPU 采集   : {}'.format('可用' if gpu.available else
                                   '不可用（{}）'.format(gpu.error or '未找到 nvidia-smi')))
    print('采样间隔   : {:.1f}s   线程每 {} 次 / GPU 每 {} 次'.format(
        interval, cfg.get('thread_sample_every'), cfg.get('gpu_sample_every')))
    print('监视文件   :')
    for path in watcher.paths:
        mark = '存在' if os.path.isfile(path) else '缺失'
        print('   [{}] {}'.format(mark, path))

    roots, procs = list_targets(finder)
    print('')

    # 决定何时开始
    auto = bool(cfg.get('auto_start_on_watch', True))
    started = False
    if args.now:
        sampler.start_trial('manual', '用户指定 --now')
        started = True
        print('[monitor] 立即开始记录。按 Ctrl+C 结束并出报告。')
    else:
        print('[monitor] 等待 test.py 出现或被更新 ...')
        print('[monitor] （若想立刻开始，请用 --now；想手动停止按 Ctrl+C）')
        # 如果文件已存在且策略进程已在跑，也允许用户用 --start-now-file 立刻开始
        if args.start_if_present and any(os.path.isfile(p) for p in watcher.paths):
            sampler.start_trial('present', '文件已存在且指定 --start-if-present')
            started = True
            print('[monitor] 文件已存在，按 --start-if-present 立即开始记录。')

    idle_limit = float(cfg.get('trial_idle_timeout_sec', 45.0))
    max_trial = float(cfg.get('max_trial_sec', 900.0))
    last_activity = time.monotonic()
    started_at = time.monotonic() if started else None
    activity = {'count': 0}       # 开始记录后出现过多少次「真实活动」
    last_proc_count = len(procs)

    def collect_and_report(reason):
        print('[monitor] 结束记录（{}），正在生成报告 ...'.format(reason))
        scores = read_scores(resolve('WindowsNoEditor/Scripts/project1.csv'))
        tel_rows = read_telemetry(os.path.join(HERE, 'race_telemetry.csv'))
        tel = analyse_telemetry(tel_rows)
        tel['orbit'] = analyse_orbit_deviation(tel_rows, cfg)
        sysstat = analyse_system(sampler.samples)
        findings = detect_bottlenecks(sysstat, tel, scores, cfg)
        tips = build_recommendations(sysstat, tel, findings)
        snapshot = read_strategy_snapshot(os.path.join(HERE, 'test.py'))
        report = write_report(outdir, run_id, cfg, sysstat, tel, scores, findings, tips,
                              sampler.thread_rows, sampler.events, snapshot,
                              sampler.samples[-1].get('root_pids', '') if sampler.samples else '',
                              logs=logtail.lines)
        sysfile = write_csv(outdir, run_id, 'system', sampler.samples)
        thrfile = write_csv(outdir, run_id, 'threads', sampler.thread_rows)
        logfile = write_csv(outdir, run_id, 'logs', logtail.lines)
        print('[monitor] 报告 : {}'.format(report))
        if sysfile:
            print('[monitor] 系统 : {}'.format(sysfile))
        if thrfile:
            print('[monitor] 线程 : {}'.format(thrfile))
        if logfile:
            print('[monitor] 日志 : {} （{} 行）'.format(logfile, len(logtail.lines)))
        return report

    try:
        while True:
            loop_start = time.monotonic()

            # 文件事件
            for event in watcher.poll():
                name = os.path.basename(event['path'])
                detail = '{} {}'.format(name, '出现' if event['kind'] == 'appeared' else '被更新')
                sampler.add_event('file_' + event['kind'], detail)
                print('[monitor] 文件事件: {}'.format(detail))
                last_activity = time.monotonic()
                activity['count'] += 1
                if auto and not started:
                    sampler.start_trial('auto:' + name, detail)
                    started = True
                    started_at = time.monotonic()
                    last_activity = time.monotonic()
                    print('[monitor] 自动开始记录这一轮。')
                elif started and cfg.get('restart_trial_on_update', True) \
                        and event['kind'] == 'updated':
                    # 运行中 test.py 被更新 → 从这一刻起算新的一轮（分段统计）
                    if name == 'test.py':
                        sampler.start_trial('update:' + name, detail)
                        started_at = time.monotonic()
                        print('[monitor] 检测到策略更新，从此刻起算新一轮。')

            if started:
                row = sampler.sample()
                for entry in logtail.poll():
                    sampler.add_event('log:' + entry['file'], entry['line'][:160])

                # 成绩文件出现新行 → 记录该轮成绩
                scores = read_scores(resolve('WindowsNoEditor/Scripts/project1.csv'))
                if scores and scores[-1][1] is not None:
                    marker = scores[-1][2]
                    if marker not in getattr(sampler, '_seen_scores', set()):
                        if not hasattr(sampler, '_seen_scores'):
                            sampler._seen_scores = set()
                        sampler._seen_scores.add(marker)
                        sampler.add_event('score', '成绩 {} ({})'.format(
                            scores[-1][1], scores[-1][0]))
                        print('[monitor] 新成绩: {} s'.format(scores[-1][1]))
                        last_activity = time.monotonic()
                        activity['count'] += 1

                # 进程数变化也算活动
                if row.get('target_count') != last_proc_count:
                    sampler.add_event('proc_change',
                                      '目标进程数 {} -> {}'.format(
                                          last_proc_count, row.get('target_count')))
                    last_proc_count = row.get('target_count')
                    last_activity = time.monotonic()
                    activity['count'] += 1

                elapsed = time.monotonic() - started_at
                idle = time.monotonic() - last_activity
                if args.duration and elapsed >= args.duration:
                    collect_and_report('达到指定时长 {:.0f}s'.format(args.duration))
                    return 0
                if elapsed >= max_trial:
                    collect_and_report('达到单轮上限 {:.0f}s'.format(max_trial))
                    return 0
                # 只有「确实运行过」才允许按静默收尾，避免空转 45s 就自己退出
                if idle >= idle_limit and args.stop_when_idle and activity['count']:
                    collect_and_report('静默 {:.0f}s'.format(idle))
                    return 0

            time.sleep(max(0.05, interval - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        if started:
            collect_and_report('用户中断 Ctrl+C')
        else:
            print('\n[monitor] 未开始记录就退出了。')
        return 0


def main(argv=None):
    harden_console()
    parser = argparse.ArgumentParser(
        description='机器鱼科目一：低开销监控与瓶颈分析（不修改 test.py）')
    parser.add_argument('--now', action='store_true', help='立即开始记录')
    parser.add_argument('--start-if-present', action='store_true',
                        help='若 test.py 已存在则立即开始（否则等它被更新）')
    parser.add_argument('--duration', type=float, default=0.0, help='采这么多秒后出报告')
    parser.add_argument('--stop-when-idle', action='store_true',
                        help='长时间无活动就自动收尾出报告')
    parser.add_argument('--interval', type=float, default=None, help='采样间隔秒，覆盖配置')
    parser.add_argument('--output-dir', default=None, help='报告输出目录，覆盖配置')
    parser.add_argument('--config', default=None, help='指定配置文件路径')
    parser.add_argument('--list', action='store_true', help='只列出相关进程后退出')
    parser.add_argument('--write-config', action='store_true',
                        help='写出默认配置文件后退出')
    args = parser.parse_args(argv)

    if args.write_config:
        path = write_default_config(args.config)
        print('默认配置已写出: {}'.format(path))
        return 0

    added, cfg_path = sync_default_config(args.config)
    if added:
        print('[monitor] 配置文件已补齐新选项 {}: {}'.format(added, cfg_path))
    if not os.path.isfile(args.config or os.path.join(HERE, CONFIG_NAME)):
        write_default_config(args.config)

    cfg = load_config(args.config)
    if args.interval:
        cfg['sample_interval_sec'] = args.interval
    if args.output_dir:
        cfg['output_dir'] = args.output_dir

    if args.list:
        if psutil is None:
            print('需要 psutil')
            return 2
        list_targets(ProcessFinder(cfg))
        return 0
    return run(args, cfg)


if __name__ == '__main__':
    sys.exit(main())
