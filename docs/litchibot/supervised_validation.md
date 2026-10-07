# LitchiBot：受监管 Sharpa 实体验收入口

2026-10-06。本轮只完成代码和软件测试，未启动实体 Sharpa。本模式不是历史问题的根因修复，也不表示设备已经通过实体验收。

## 默认行为与独立入口

- 原无参数 / 显式 Manus 的执行 argv、节点参数、calibration、V4、SDK sender、tactile 不变。
- LitchiBot `--dry-run` 仍强制使用唯一原 fake driver，实体发送为 0。
- 普通 `--hand-source litchibot` 仍被拒绝；不能靠删除 `--dry-run` 解锁。
- 独立选择项为 `--supervised-hardware-validation --validation-permit <现场已核验的许可文件>`，仅适用于 LitchiBot，不能与 dry-run 混用。
- 许可模板 `config/litchibot/validation_permit.example.json` 所有授权字段默认 false、serial/token/期限未设置，**不能启动实体模式**。本轮没有生成有效许可。

正式入口仍是 `ops/run/run_sharpa_hands_cyclonedds.sh`。没有新增平行硬件入口。

```text
原 Terminal 2 wrapper / CycloneDDS / pixi / launch
  ├─ Manus：原 ManusDriverNode + V4 + 原 SharpaDriverNode
  ├─ LitchiBot dry-run：原 worker + bridge + 原 fake SharpaDriverNode
  └─ LitchiBot supervised validation：
       原 worker（自身始终 no-device-send）
         → RawContinuityGate（每个 worker packet，coalescing / 限速之前）
         → 原 TargetGate（加强验收参数）
         → 原 /sharpa/{left,right}/command
       validation_driver.py：唯一硬件进程
         → 原 SharpaDriverNode 的安全包装子类
         → MotionLatch + 原 SDK sender / 原 state + tactile
       validation_supervisor.py：只提供本地按住运行信号，不连接设备、不发布关节
```

包装子类继承原 driver 的连接、sender、state、tactile 实现；附加 `_on_command` gate、独立 watchdog、停机锁定和双手清理。原 driver 源文件、SDK driver、tactile 和 V4 文件未修改。验收 launch 禁用普通 Sharpa Node，只启动该包装宿主，仍只有一个 `sharpa_driver` 节点和两只手的同一个 SDK 进程。

## 进入条件

1. 现场具有机器人操作资格的成年人按实际设备 SOP 核验工作区、设备急停、设备左右身份、校准和手套 profile。
2. 许可文件由该操作账号持有，权限不向组/其他用户开放；记录操作人、实际 SOP 引用、成人/资格/SOP/急停/工作区声明、已知小指风险确认及 SDK connect 自动使能确认。
3. 明确且不同的左右 Sharpa serial、匹配的 glove profile、全新 32-byte hex session token；许可期限在未来 15 分钟以内。软件核验声明完整性，**不能代替现场核验资格、急停或 SOP**。
4. wrapper、launch、bridge、硬件宿主各自检查独立模式。硬件宿主在 SDK 连接前再次检查实际 ROS 参数，拒绝降低 watchdog、错误 serial、未知 topic/remap/参数和过高 coefficient。速度 coefficient 不高于 0.3，电流 coefficient 不高于 0.6。
5. 连接前检查当前 ROS 图没有已有 Sharpa driver/command sink/state publisher；验收模式有同一账号的硬件文件锁。SDK 连接后检查实际发现设备明确报告的左右身份及 serial。
6. source/target 所有关节均有效，完整 solved 状态，禁止 hold、fallback 和外层 schema clipping；两侧先积累至少 30 个有效 command preview。单侧 partial（包括历史 18/20）不会被忽略或降级放行。
7. 预览位置必须在 measured pose 的 0.15 rad 内；第一条实际动作还受 0.5 rad/s 的 acquisition 检查，30 Hz 下约 0.0167 rad。未按住运行时 preview 不调用原 sender。
8. 现场操作人打开本地 supervisor 面板，并按住运行按钮。面板只在 guard 状态新鲜、READY/ARMED 时允许操作；按住时每 50 ms 发带 token、递增序号和 monotonic timestamp 的 heartbeat。松开、失焦、关窗、进程退出或消息超时均停止，不提供自动恢复。

许可文件不是单独的“解锁开关”：没有持续按住运行、完整输入、feedback、graph、watchdog 和小范围边界，不会调用 sender。

**设备连接本身会由原 SDK 自动使能。** 原 SDK 未修改；现场人员必须在选择此模式、发生连接前完成 SOP/急停核验。no-send 仅指禁止程序位置命令，不等于设备已断电。

## 安全 gate 位置

| 文件 / 函数 | 检查或职责 |
|---|---|
| `ops/run/run_sharpa_hands_cyclonedds.sh` | 独立模式组合检查；默认与普通 LitchiBot 硬阻断保留 |
| 原 `teleop.launch.py::validate_source` | 许可、profile、guard/bridge 路径、双手绑定、coefficients；条件选择唯一 driver |
| `validation_safety.py::load_permit` | 许可所有权/权限、声明、期限、profile、左右 serial、session |
| `validation_safety.py::RawContinuityGate.observe` | target/source names/order、shape、NaN/Inf、rad 限位、validity、序号/会话、source gap；原始 target 与 canonical 单帧 abs(delta) ≥0.10 rad 即拒绝 |
| `terminal2_bridge.py::WorkerInbox.feed/main/state` | 在 latest-frame coalescing 前逐帧检查；确认真实 guard 参数和 session；side-prefixed feedback 路由；原 TargetGate 新鲜度/限位/限速；fault 向本地 stop 通道发送停机并退出 |
| `validation_driver.py::validate_driver_arguments/ensure_clear_graph` | SDK 连接前参数与已有硬件图检查 |
| `validation_safety.py::MotionLatch.command/watch/heartbeat` | ROS stamp、names/order、22D、finite、limits、side/session/sequence、single-frame delta、实际动作 acquisition/slew、位移范围、deadman、command/许可超时及运行上限 |
| `validation_driver.py::GuardedSharpaDriverNode._on_command/service_guard/stop_validation` | 硬件进程内再次检查，在 gate 放行后调用原 `_on_command`；独立 timer 即使 bridge/supervisor 丢失也停机 |

暂定软件边界：source、ROS command、feedback 使用最长 0.20 s；原 driver 的 `stop_on_command_timeout_s` 强制 0.20 s；guard timer 10 ms；raw/canonical 和 ROS 单帧 ≥0.10 rad 停止；速度 0.5 rad/s；相对进入 ARMED 时实测关节位置最大 0.15 rad；单次 ARMED 最长 60 s。30 s 内未完成 acquisition 也停止。边界不是设备安全认证，不在 CLI 或许可中提供放宽选项。

## 异常时如何停止

任一侧发生故障，整个双手会话进入 **FAULT**：拒绝异常帧、禁止所有后续 sender 调用、调用两侧原 `hand.stop()`。原 SDK 的 stop 路径关闭 enable；退出时再停机并 disconnect。单侧 stop 调用失败时继续尝试，并明确打印现场物理急停提示；双手清理会分别尝试，避免一侧异常跳过另一侧。

source 突跳、invalid、stale、NaN/Inf、越界、乱序/会话改变、feedback 路由错误、许可到期、按住运行释放/超时、command 超时、异常 publisher/重复 driver 均按上述逻辑处理。bridge 异常/退出发本地 stop datagram；guard 故障状态导致 bridge 退出并触发原 launch shutdown。guard 自身退出时也关闭会话。SIGINT/SIGTERM 在连接期间记录取消请求，连接完成后立即清理，避免中断连接后留下未清理的已使能对象。

FAULT 不接受重新按住按钮恢复；必须退出、现场检查、使用新 session 再启动并重新 acquisition。原设备物理急停/SOP 停机机制继续保留。

10 ms timer / 0.20 s watchdog 是软件调度目标，未进行设备停机延迟测试，不能宣称硬实时或物理立即停止；SDK 调用阻塞及停机失败仍需设备原急停。现有 driver 的 state timestamp 表示读取/发布时间，不是可核验的设备采样时钟；本模式没有修改它。

## 唯一硬件进程确认

- 静态 launch 测试：supervised 分支选择两个 ExecuteProcess（一个 guard 宿主、一个 source bridge），普通 Sharpa Node、Manus Node、V4 Node均不启动。
- 宿主连接前检查当前 DDS 图，另以文件锁阻止本账号第二个 supervised 宿主。
- 运行时检查当前 DDS 图 `sharpa_driver` 数量=1、每侧 command 的 driver subscription 数量=1、joint_states publisher 数量=1；command publisher 只能是唯一 `litchibot_command_bridge`。
- 这些检查覆盖当前 DDS domain 和本账号的验收入口；现场 SOP 仍必须保证没有其他 domain、非 ROS 程序或其他账号独立占用设备，软件不能证明整个现场不存在这样的连接。
- Terminal 3 的 command/state/tactile topic 和类型继续沿用原接口；未改 recorder、T1、SDK driver、tactile 或 V4。

## 测试与历史风险

历史 `left pinky_MCP_FE: 1.5708 → 0 rad` 仍记录在原诊断报告。测试把该输入交给 hardware RawContinuityGate，验证它在限速和 coalescing 前被拒绝并锁定，不能被平滑后继续发送。

ROS 环境：专项 + 既有 Terminal 2 / Sharpa / Manus retarget 回归 **55 passed**。原 Python 3.10 backend/hand pipeline/架构及专项 **67 passed，10 skipped**；ROS-only 测试另跑。测试覆盖默认 Manus、dry-run、launch 唯一宿主、许可默认禁止、直接宿主参数不可降级、NaN/Inf、order/shape、invalid、stale、raw 历史跳变、deadman release/watchdog、首次 acquisition、range/slew、实际包装回调拒绝后不再调用 sender，以及原 stop 调用。

硬件宿主测试用 MockSharpaHand 替代设备，并令真实 SDK loader 一旦调用就失败；验证 loader 未被调用。所有物理 SDK 初始化及实体发送为 0。尚未做真实 SDK 连接、supervisor GUI 操作/失焦、物理急停或停机延迟验收。

源码改动：wrapper、原 launch、source bridge（模式/逐帧 gate/故障停机）、worker 注释；新增 safety module、guard 宿主、supervisor、许可模板与测试。`portable_deps` 被主仓库忽略，更新后的原 launch 仍导出在 `docs/litchibot/terminal2_launch.patch`。57 个受保护文件 hash 保持一致。
