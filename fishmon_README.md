# fishmon —— 机器鱼监控与瓶颈分析工具

**用途**：在你手动跑仿真时，从外部观测整机与 `test.py` 相关进程的资源占用、
运行时长、迭代速率，并输出一份带**实测证据**的瓶颈分析与提速建议报告。

**它绝不会修改 `test.py`**。策略代码只被只读扫描（用于记录本轮参数快照），
所有分析都基于平台/策略自己写出的文件（`project1.csv`、`race_telemetry.csv`）与系统采样。

---

## 怎么用（只要双击）

1. 鼠标双击 **`fishmon.bat`**。
2. 它会自动找一个装了 `psutil` 的 Python 解释器，然后开始监控。
   - `test.py` 已存在 → 立即开始记录；
   - 之后每次 `test.py` 被更新 → 从那一刻起算新一轮；
   - 长时间没有新成绩/文件变化 → 自动收尾并生成报告（约 45 秒静默）。
3. 也可以随时在那个黑色窗口里按 **Ctrl+C** 提前结束并出报告。

报告输出在 `monitor_reports\` 目录：

| 文件 | 内容 |
| --- | --- |
| `report_<时间>.md` | **主报告**：结论、资源占用、运行时长、迭代速率、成绩、建议、候选补丁 |
| `system_<时间>.csv` | 逐帧系统指标（CPU/内存/GPU/IO/网络/进程/高度…）可直接画图 |
| `threads_<时间>.csv` | 线程/进程级 CPU 增长明细 |
| `logs_<时间>.csv` | 运行期间采集到的日志增量行 |

> 报告与 CSV 已加入 `.gitignore`（属运行时产物）。想留档就单独复制出来。

---

## 依赖

- **Python 3.8+ 且装了 `psutil`**。本机用 `D:\Python311\python.exe`（3.11，已有 psutil）。
- ⚠️ 仿真器自带的 `WindowsNoEditor\...\python.exe` 是 **3.7 且没有 psutil**，不能跑本工具。
- GPU 指标走 `nvidia-smi`（NVIDIA 显卡）；没有也不影响其余功能。

装 psutil：

```powershell
D:\Python311\python.exe -m pip install psutil
```

---

## 命令行（不想用 .bat 时）

```powershell
# 立即开始，采 60 秒后出报告
D:\Python311\python.exe -B fishmon.py --now --duration 60

# 等 test.py 出现/更新再开始；空转不会自己退出
D:\Python311\python.exe -B fishmon.py

# 只看看当前找到了哪些相关进程
D:\Python311\python.exe -B fishmon.py --list

# 把默认配置写到 fishmon_config.json 后退出
D:\Python311\python.exe -B fishmon.py --write-config
```

| 开关 | 作用 |
| --- | --- |
| `--now` | 立即开始记录 |
| `--start-if-present` | 文件已存在就立即开始（`.bat` 用这个） |
| `--duration N` | 采 N 秒后收尾出报告 |
| `--stop-when-idle` | 长时间无活动自动收尾（`.bat` 用这个） |
| `--interval N` | 采样间隔秒，覆盖配置 |
| `--output-dir DIR` | 报告输出目录 |
| `--config PATH` | 指定配置文件 |
| `--list` | 只列出相关进程 |

---

## 配置（`fishmon_config.json`）

首次运行会自动生成；**升级工具时新选项会自动补进去，你已有的取值不会被覆盖**。常用项：

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `sample_interval_sec` | 1.0 | 系统+进程采样周期 |
| `thread_sample_every` | 5 | 每 N 次采样做一次线程枚举（较重） |
| `gpu_sample_every` | 3 | 每 N 次采样查一次 GPU |
| `process_name_patterns` | `["fish"]` | 目标进程名匹配 |
| `process_discover_every` | 10 | 每 N 次采样做一次全量进程发现 |
| `trial_idle_timeout_sec` | 45 | 静默多久自动收尾 |
| `max_trial_sec` | 900 | 单轮最长时长 |
| `log_paths` / `log_dirs` | 见文件 | 日志采集位置（只读、只取尾部增量） |
| `orbit_*` / `loop_r_mm` | 见文件 | 绕圈诊断用的几何常量，**改了 `test.py` 的这些常量要同步这里** |

> `log_dirs` 默认是空的：不要指向 `WindowsNoEditor` 根目录，那棵树里有上万个 `.txt`。

---

## 报告怎么读

### 运行时长（分钟）

- **整机已运行** = 开机时长。
- **相关进程已运行** = 该进程的真实寿命（取自系统 `create_time()`，
  不是「本工具发现它的时刻」）。这样中途重启过仿真器也能看出来。

### 等效核数

`等效核数 = 进程 CPU 秒增量 ÷ 时间`。接近 `1.0` 表示串行吃满一个核；
远小于 `1.0` 表示大部分时间在等（而不是在算）。

### 速度利用率（关键指标）

`速度利用率 = 该段平均速度 ÷ 全程 P95 速度`。
**利用率低说明那一段输在「跑不快」，而不是「路太长」** ——
优化方向完全不同：前者要查限速/姿态，后者要缩路径。

### 绕圈径向误差（当前最大的瓶颈）

`test.py` 有一段保护：鱼偏离目标圆周超过阈值时把推力从 50 钳到更小的值。
报告会算出**每段有多少比例的帧踩到了这条保护**。占比高就说明绕圈慢
不是目标速度不够，而是这条保护在持续降推力。

---

## 常见问题

**双击一闪而过？**
`fishmon.bat` 结尾有 `pause`，不会闪退。若是启动器找不到解释器，它会用英文提示
（`.bat` 保持纯 ASCII 是为了避免 cmd 读 UTF-8 中文时解析错乱；中文输出由 `fishmon.py` 负责）。

**窗口里中文是乱码？**
`fishmon.py` 启动时会把控制台代码页切到 UTF-8。若仍异常，先执行 `chcp 65001`。

**`fish_debug_log.csv` 是空的/旧的？**
那是早期版本遗留的。现在以 `race_telemetry.csv` 为准（v7.2 起 `test.py` 自动写）。

**它会影响比赛成绩吗？**
设计目标是低开销：默认 1 秒采一次，重的采集（线程枚举、GPU）低频次。
但**任何外部监控都可能有轻微影响**，正式比赛建议只在你需要诊断时开着。

**报告里的"预计能省多少"可信吗？**
那是**上界估算**，不是保证。实测数字（耗时、里程、速度、偏出比例）是真实的，
但「如果按 P95 跑就能省 X 秒」忽略了弯道的物理限制。必须实跑验证。
