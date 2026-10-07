# Full teleop 软件集成报告

**最新 normal LitchiBot 设计：** 保留独立 NormalHandAuthority/NormalHandRuntime，Start/Stop 仅逐侧 forwarding。用户状态 READY/ACTIVE/OFFLINE；heartbeat 只诊断，单个坏 packet 只丢当前帧，source/control 恢复回 READY，无普通 runtime 永久 latch。以下是历史验收记录，其 normal lease/PAUSED/warmup/pattern/validation 描述不适用于当前 normal。当前九项审计见 [normal forwarding 最终审计](normal_forwarding_audit.md)。

本轮只完成代码、模拟设备测试和独立 DDS domain 199 的软件验证。没有启动用户的 FR3、Sharpa、真实手套或生产 Docker stack，没有点击现场 GUI 的 Start hand。本文不代表首次实体验收已通过。历史 `left pinky_MCP_FE`（index 18）`1.5708 → 0 rad` 风险仍然保留。

## 1. 验收条件与日常安全门分开

| 条件 | supervised validation | normal full teleop |
|---|---|---|
| 首次验收 permit、实名/SOP/急停/工作区/已知风险确认 | 必须 | 不作为每次 Start hand 的前提 |
| permit 最长 15 分钟，运动窗口 60 秒 | 保留 | 不使用 |
| 实体验收位移窗口 0.15 rad | 保留 | 不使用验收窗口；保留物理限位和有效采集，LitchiBot normal 移除自加 shared slew |
| hold-to-run | 独立验收原面板保留；full 验收按住 F12，松开/失焦锁停 | 不要求持续按键；GUI Start hand 授权该侧 |
| 左右实体序列号映射、实际 SDK 侧别核验 | 必须 | 必须；部署配置一次填写，启动时核验 |
| profile | permit 与 SDK profile 匹配 | 使用部署 profile；不要求重做本人校准或生成验收 permit |
| 初始 hand permit | 双侧关闭 | 双侧关闭 |
| RawContinuityGate / 22D / joint order / NaN、Inf / limits / timestamp / freshness / watchdog / timeout / discontinuity / FAULT | 始终执行 | 始终执行 |
| GUI enable lease | full 模式必须 | 必须；50 ms 心跳，0.20 s 超时撤权并锁停 |

区分点在 `validation_safety.py:MotionLatch.validation_only`，以及 `full_control.py:load_full_config`。normal 路径不会调用 `load_permit()`；不依赖 `validation_permit.example.json`，不读取成人/SOP声明，也不模拟一个已签署的验收 permit。内部运行策略只是共享 latch 需要的会话 token、绑定序列号和 profile。

运行阈值保持：输入/command/GUI lease 0.20 s；supervised 原始 source 或 target 单帧跳变 ≥0.10 rad 锁停；LitchiBot normal/diagnostic 单帧 moderate/快速动作 WARNING + 合法 target 原值转发，持续异常模式才逐侧 SOFT_HOLD；Start hand 授权保持，30 帧稳定后已授权侧自动 ACTIVE；基础危险错误/极端跳变为全局 HARD_FAULT（详见 [分级与恢复](normal_discontinuity.md)）；0.5 rad/s 只保留在原 supervised/Manus 等独立路径，LitchiBot full normal/fake 不使用；双侧各至少 30 个有效采集帧。停止并重新 Start 时验证反馈与采集有效性；LitchiBot normal 不再将目标按 shared speed clamp 改写；RawContinuityGate 历史不会被清空，不会借重新启用掩盖原始跳变。

## 2. 原 GUI 为什么没有控制 Terminal 2

`apps/operator_gui/operator_gui.py:OperatorWindow._build_ui` 的左右 hand 按钮调用 `_send('engage_hand' / 'disengage_hand', {'side': ...})`。原 `_send` 发往 arm operator 的 JSON-TCP server。

`adapters/pico/src/pico_bimanual_franka_teleop/control_server.py:OperatorControlServer.dispatch` 调用 `OperatorConsole.set_active(target='hand')`，只更新本后端的 `hand_active`。GUI 的 `_apply_status` 用这个内存值刷新按钮。T1 又明确使用 `--hand-source none`，没有 HandWorker 消费这些手部授权；独立 T2 的 ROS/Sharpa 进程没有订阅此 JSON-TCP 状态。

原 Open hand 发送 `open_hand`，server 转成旧键盘的一次性 open_hands/open_left_hand/open_right_hand 请求，供旧 HandWorker/pipeline.request_open 消费。它没有定义本次 Sharpa 的固定 22D open pose。

## 3. 新启动图与运行环境

```text
Host: ops/run/run_gello_full_teleop.sh
  └─ teleop_runtime.full_launcher.FullLauncher
       ├─ 原 run_gello_arms_only.sh
       │    ├─ 原 Docker Compose 五个 arm 服务
       │    │    franka-control / teleop-control / moveit-ik / preset-ik / gello-bridge
       │    └─ 原 run_operator.sh
       │         ├─ Host arm operator backend (--hand-source none)
       │         └─ Host Operator GUI（仍使用原 base Python / PySide6）
       └─ 原 run_sharpa_hands_cyclonedds.sh --full-config <私有会话文件>
            └─ 原 Host pixi ROS 环境 / 原 teleop.launch.py
                 ├─ Manus driver + 原 calibration + 原 V4 worker
                 │     或 LitchiBot Python 3.10 worker + V3.5 + 原厂 mapper
                 └─ 一个 guarded SharpaDriverNode
                      ├─ FullHandRuntime：source 路由、GUI transport、状态接口
                      ├─ 原 RawContinuityGate / TargetGate / MotionLatch
                      ├─ 原 SharpaDriverNode callback / 原 SDK
                      └─ 原 joint_states / tactile timers

独立 Terminal B：原 recorder + cameras，仍由用户另开终端启动
```

Sharpa/LitchiBot/Manus 都留在 Host。arm Docker 和 Host hands 继续使用原 CycloneDDS、部署 `TELEOP_ROS_DOMAIN_ID`、localhost 设置、housekeeping CPU affinity。没有新建第二套 driver，没有把 SDK 放入 GUI 或 arm Docker。LitchiBot worker 继续使用独立 Python 3.10，ROS bridge/guard 留在现有 pixi ROS Python。

## 4. wrapper 与共享下游

新增 wrapper 只转发给 `teleop_runtime/full_launcher.py`。它实际以子进程执行上述两个旧 wrapper，没有复制 compose up、SDK 初始化或 tactile 实现。

T1、run_operator、T3 脚本逐字未改。T2 只新增显式 `--full-config` 分支；没有该参数时，原 Manus 默认 argv/DDS/标定路径、LitchiBot dry-run、独立 supervised validation 都按原逻辑运行。普通 hands-only LitchiBot 实体启动仍被原 blocker 拒绝。`--dry-run` 与任何 full hardware 会话冲突都会被拒绝，不能把 dry-run 解释为实体模式。

原 `teleop.launch.py` 中普通 Sharpa node 与 guarded host 条件互斥。full 模式也使用现有 `validation_driver.py` 的唯一 guarded host，保留 `SharpaDriverNode`、SDK sender 和 tactile 的原代码。FullHandRuntime 仅提供路由/GUI transport，不是第二套 safety latch 或第二个 hardware process。

## 5. Manus/LitchiBot 汇合

Manus V4 节点通过 launch remap 将 `/sharpa/{side}/command` 输出改到 `/teleop/sharpa/{side}/target`；原 V4 算法不修改。full 之外仍为原 topic。

LitchiBot worker 在 full 模式使用有界 FIFO，每一帧都发到 `/teleop/sharpa/{side}/source`（`std_msgs/String`，含 canonical 20D、valid masks、fallback/clip metadata、原始 22D）；同时发布统一 `/teleop/sharpa/{side}/target`（`sensor_msgs/JointState`）。安全门读取含完整诊断的 source topic，避免在 latest-frame 合并或限速之后漏掉一次跳变。坏 JSON/packet 或 FIFO overflow 明确发布 `/teleop/sharpa/source_fault`，不能静默跳过后继续实发。

Manus 路径对 V4 target 的同一 header timestamp 配对 `/manus/{side}/keypoints`，要求 25 个有限 keypoint，再执行 raw target 连续性检查。keypoint 无效、配对队列溢出或 0.20 s 内不能配对都会 FAULT。原 PoseArray 没有 LitchiBot 20D validity metadata，不能假称两种源有同一传感器有效性字段。

共享发送链：

```text
source → RawContinuityGate（先检查原始不连续/有效性）
       → TargetGate（实测反馈、结构、物理限位、时效检查；LitchiBot normal/full fake 原值转发）
       → MotionLatch.command（许可、22D/限位/时序/timeout 等）
       → GuardedSharpaDriverNode.forward_full
       → 原 SharpaDriverNode._on_command
       → 原 hand.send_action / Sharpa SDK
       → /sharpa/{side}/command（记录已接受命令）
```

full host 会销毁基类的公共 command subscriptions，只在安全门通过后直接调用同一个原 callback。这样外部发布 `/sharpa/.../command` 不能绕过 gate 驱动物理手。公开 command topic 作为记录接口保留；唯一 command publisher 是 `sharpa_driver`。

DISABLED/FAULT 时继续发布带 `supervisor_no_send:<side>:<state>` frame_id 的**实测位姿快照**，用于 recorder 的原 command stream 连通性，不把 glove target 冒充已执行 action；这些快照不进入 SDK。ACTIVE 发布接受的目标。topic 类型、关节顺序不变。

## 6. GUI 调用链和实际状态

```text
Left/Right Start hand / Stop hand
 → OperatorWindow._send
 → HandClient.request（127.0.0.1 JSON-TCP，私有 session token）
 → HandControlServer 队列
 → guarded host 的 10 ms timer / FullHandRuntime.dispatch
 → FullHandRuntime.request
 → 同一个 MotionLatch.request_hand
 → 后续 input 通过所有 gate 才可能 forwarding

返回状态
 → FullHandRuntime.status
 → HandClient.status signal
 → OperatorWindow._apply_hand_status
 → Start/Stop [DISABLED/READY/ACTIVE/SOFT_HOLD/HARD_FAULT（supervised 原 FAULT）]，fault reason 进入原 Event log
```

normal Start 后下一个合法心跳更新授权，正常运行不要求额外 hold。GUI 不执行数字安全检查，旧 arm `hand_active` 不能覆盖真实 hand 状态。

状态：后台 warmup 且未授权时显示 DISABLED；有效样本不足时 Start 被拒绝并记录原因。Start 接受后显示 READY；GUI lease + 采集及 runtime checks 满足后 ACTIVE。Stop 显示 DISABLED，调用原 `hand.stop()` 撤销该侧 SDK enable，继续读取 source/state/tactile。normal 双侧 Stop 不制造验收 deadman FAULT，可以重新 Start。LitchiBot normal/diagnostic 的持续软异常只暂停该侧并自动恢复，保持 permit；真正 HARD_FAULT 才清除双侧 permit，不能由 GUI Start、重连或服务调用清除，必须重启新会话并处理故障原因。

原急停、timeout 和 `hand.stop()` 保留；SDK connect 原本会 auto-enable，本轮未重写它。full 初始化完成即 stop 双手，且没有授权就没有 joint target 发送。normal 后续通过所有 gate 才调用 SDK 原 `_ensure_enabled()` 恢复该侧。这保留了此前验收记录中的 connect auto-enable 风险，不声称修改了 SDK 的电气上电行为。

## 7. GUI transport 与 ROS 接口

原 GUI 使用 ROS-free base Python，arm controls 也是 JSON-TCP。因此沿用 JSON-TCP 到同一个 guarded host，不往 GUI 加 ROS/CycloneDDS 或 safety logic。

同时提供 ROS 接口：

- `/teleop/sharpa/supervisor/state`：`std_msgs/String`，包含双侧真实状态/permit、mode、reason、send count。
- `/teleop/sharpa/{side}/enable`：`std_srvs/SetBool`。False 撤权；True 不能新建 GUI operator permit，必须该侧已由认证 GUI Start 授权。ROS 请求不刷新 GUI lease。
- `/teleop/sharpa/disengage_all`：`std_srvs/Trigger`，仅撤销双手许可。

这个限制防止“GUI 在线但未点击 Start”时，ROS enable 请求代替 operator 授权。JSON-TCP 仅允许一个 GUI owner；另一条认证连接只允许撤销 all，便于 launcher 退出时先撤权。关闭连接/重连不恢复授权。

## 8. DISENGAGE ALL、arm 独立性

GUI `_send('disengage_all')` 先向 hand supervisor 发 revoke all，再保留原 arm TCP 请求。原 `OperatorControlServer._disengage_all → disable_all + abort_action` 完整保留。手部关闭不杀 source/retarget/driver；Start arm / Hold arm 沿用旧接口，不操作 hand permit。arm TCP 断线状态不会覆盖 hand authority。

## 9. 失联、异常和停止

GUI 每 50 ms 发带递增 sequence 和 monotonic timestamp 的认证 heartbeat；过期/重放心跳锁停。GUI 连接消失，或最后合法 heartbeat 超过 0.20 s 时，即使按钮以前 ACTIVE，也由 MotionLatch 撤权并锁停双手。guard 的 10 ms timer 独立于 GUI，原 driver 的 command timeout 仍强制 0.20 s。

source/worker 超时、NaN/Inf、限位、mask、fallback、wrong side/order、历史 raw jump 等同样拒帧并 stop 双手。FAULT 保留第一条原因，后来的 shutdown 不覆盖原始原因。SDK Stop 失败会立即尝试双侧 stop、保持 no-forward 并报告设备 SOP/物理急停；软件不能承诺 SDK/总线故障时存在物理停机上界。

## 10. 唯一 driver 与生命周期

full launcher 有独立会话 file lock；启动前检查控制端口和相关 compose 服务，拒绝接管旧系统，不能顺手关闭用户已有 stack。guard 继续使用原共享 hardware lock 和 SDK 连接前 DDS graph preflight。启动后检查 driver node、joint_states publisher、command publisher 和 source publisher 唯一性；非预期 publisher 或 source 消失锁停。允许 recorder 的被动订阅，不把 recorder 当 hardware driver。

Ctrl+C/子系统退出：先认证 hand revoke all，并向原 arm JSON-TCP server 发送原 disengage_all，立即撤销 arm 授权；向 hands launch group 发 SIGINT，让原 worker/driver 清理并 stop/disconnect；再停止原 arm wrapper，让它回收 Docker 与原 operator/GUI。旧 wrapper 内部 Docker/GUI 清理会并发发生，未改原清理顺序。Linux subreaper + PID start-time identity 回收 nested setsid GUI/SDK worker；必要时 TERM/KILL 自己拥有的子进程，不按进程名字杀用户进程。guard/source/Manus/retarget launch process 退出触发 hand launch shutdown；full owner 再停止自己的 arm 子系统。

如果在 compose up 中途退出、旧 arm wrapper 尚未设 stack_started，full owner 会检查并停止本次拥有的剩余服务。recorder/cameras 不归 full owner 回收。SIGKILL/主机掉电属于现有设备 watchdog/SOP 范围，本文没有保证用户空间进程在被 SIGKILL 后仍能执行 cleanup。

## 11. Open hand

full 模式只向同一个 supervisor 请求。当前明确拒绝 `Open hand has no validated Sharpa policy; no command sent`，并显示 Event log。没有生成/发布猜测的 22D pose。旧 standalone GUI 的 Open hand 协议保持原状。

## 12. 回归与 Terminal 3

57 个 protected 文件的 SHA256 全部匹配：包含 T1/T3、原 Manus/V4、Sharpa node/sender、校准和 tactile。另一个 Manus 历史 manifest 中仅 T2 wrapper 不同，这是新增接入参数，其他 62 个匹配；full 开工前 4 个脚本 baseline 也仅 T2 wrapper 因 `--full-config` 新增分支变化。旧 Manus wrapper 的实际 exec argv/环境与保存的原入口逐项回归一致。

Terminal 3 的 command、joint_states、20 个 tactile raw/deformation 图像 topics、arms/cameras/QoS 配置未修改。DDS 软件验证确认原 topic/type 与唯一 driver；禁用时 command 是带 no-send 标记的实测快照，fake tactile 只是模拟数据。真实 tactile 质量、设备连接、机械跟随、Docker 栈联合运行仍需现场后续验收，本轮没有宣称通过。

## 13. 软件测试结果

见 `full_test_results.json`，含环境、完整命令和 passed/skipped/failed。最终结果：Host/Manus 回归 58 passed、24 skipped；ROS/V4/Sharpa 回归 92 passed、0 skipped；GUI 12 passed；recorder 契约 25 passed；各组 failed 均为 0。各组存在重叠，不作为独立用例总数相加。Host 的 24 个 skip 为 ROS 专属用例，ROS 环境已执行这些用例。ROS 使用 domain 199，glove worker 被 synthetic stdout 程序替代，Sharpa 使用 MockSharpaHand；SDK loader spy 禁止加载物理 SDK。launcher 测试只启动临时 stub wrapper/嵌套 worker，不运行真实 compose。

覆盖 full Manus/LitchiBot launch 选择、V4 remap、唯一 guarded host、dry-run/hardware 矛盾拒绝、normal 无验收 permit/hold/60 秒窗口、独立侧启停/重新采集、ROS enable 不能建 GUI permit、DISENGAGE ALL、GUI 失联、worker 超时、历史 raw jump/FIFO 不丢帧、FAULT 优先、Qt 真实 TCP/arm 后端、旧 GUI 回归、实际 CycloneDDS command/state/tactile、真实 bridge 与 synthetic worker 的 guarded parameter proof、FAULT 后 source 继续读、Ctrl+C/wrapper crash/orphan 清理、原 Manus calibration/V4 和 recorder 契约。

没有使用“启动真正生产 Docker + SDK 但不发动作”作为软件测试：本轮明确禁止替用户启动实体硬件。

## 14. 操作命令（本轮未执行实体启动）

先看软件链路，Sharpa 为 fake，真实 source 按选择读取：

```bash
cd /home/user/gello_upper_body_teleop
./ops/run/run_gello_full_teleop.sh --hand-source litchibot --dry-run
# 或
./ops/run/run_gello_full_teleop.sh --hand-source manus --dry-run
```

不加 mode 参数也默认 fake；可以用 `--hand-source manus --dry-run --fake-input` 让 Manus input 也模拟。LitchiBot 没有另外伪装真实 SDK 的 fake-input 分支。

日常实体 full（只在完成现场实体验收以后使用）：一次将 `config/litchibot/hand_devices.example.json` 复制为 `hand_devices.json`，填实际左右手不同 serial；文件不是 permit，也不存 SOP  attestations。运行时仍核验真实 side/serial，不自动根据截图猜写配置。

```bash
./ops/run/run_gello_full_teleop.sh --hand-source litchibot --hardware
# 或
./ops/run/run_gello_full_teleop.sh --hand-source manus --hardware
```

双臂和双手启动后仍未授权。GUI Start hand 授权该侧；Stop hand / DISENGAGE ALL 撤权；normal 不要求验收 permit/F12。当前已知左小指风险未被这些命令宣布解除。

首次工程验收仍使用已完成的独立入口，permit 和原 hold-to-run 面板保留：

```bash
./ops/run/run_sharpa_hands_cyclonedds.sh --hand-source litchibot \
  --supervised-hardware-validation --validation-permit /path/to/private-permit.json
```

如要在 full 编排里验收，显式选验收模式，GUI 的 F12 是 hold-to-run；仍满足同一个 permit 条件：

```bash
./ops/run/run_gello_full_teleop.sh --hand-source litchibot \
  --supervised-hardware-validation --validation-permit /path/to/private-permit.json
```

## 15. 调试、回退与 recorder

原独立入口继续存在；不要与 full 同时启动：

```bash
./ops/run/run_gello_arms_only.sh
./ops/run/run_sharpa_hands_cyclonedds.sh
# 独立 LitchiBot no-send
./ops/run/run_sharpa_hands_cyclonedds.sh --hand-source litchibot --dry-run
```

录制仍在独立终端，命令原样：

```bash
./ops/run/start_sharpa_arm_recording_with_tactile.sh --with-cameras
```

full launcher 不启动或关闭 recorder/cameras。不改变 ARM→FR3、Terminal 3、V4、厂家 retarget、SDK driver 或 tactile 实现。

## 16. 交付文件与部署

新增 full wrapper、`teleop_runtime/full_launcher.py`、`adapters/litchibot/full_control.py`、`full_runtime.py`、`apps/operator_gui/hand_client.py`、一次性 serial 配置 example、软件测试与本文。修改 GUI 的 full-only 路由/真实状态显示；在现有 validation_safety/driver 中扩展同一个 latch 的 GUI permit、保留独立验证默认行为；source bridge 增加 full FIFO/source metadata；T2 wrapper/原 launch 增加显式 full 条件。

`portable_deps` 被主仓库忽略，因此 launch 的可复现补丁同步保存在 `docs/litchibot/terminal2_launch.patch`，相对于已保存原始 launch。当前本机 install 为 source symlink，修改可直接被旧 pixi teleop 入口使用；其他部署若不使用 symlink，需要照原构建流程更新 workspace，不能只复制 wrapper。

## 17. 验收边界

软件模式的认证授权、runtime gate、共享 sender、recording contract 与生命周期已进行软件验证。首次实体验收及左小指根因/设备行为仍不由软件测试代替。默认 fake 和实体初始 DISABLED 不改变“正常运行以后 Start hand 就是 operator permit”的语义，也不删除首次验收模式。
