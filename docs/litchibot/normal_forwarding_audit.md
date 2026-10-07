## 实体现场 SIGKILL 日志后的 normal shutdown 修复

现场明确记录 validation_driver.py 在 SIGINT 5s + SIGTERM 10s 后仍未退出，被 ROS launch SIGKILL。此前 mock 证明正常退出会 stop，并未证明 SDK feedback 阻塞时可进入清理；不能把进程消失当成设备 disable 成功。

normal/fake 专属 NormalShutdown 生命周期工作线程监听 private session hand-shutdown.sock，独立于 ROS executor/source/heartbeat。FullLauncher 在向外层 ROS launch 发 SIGINT 前，先认证请求这个 endpoint 并等待 SDK 清理报告（18s沿用 launcher既有清理等待量）。退出开始撤销 forwarding，两侧分别后台执行原 hand.stop，读取原 get_enable_state 确认 False（只在退出、复用原 enable settle 时间）；两侧 stop 尝试完毕才分别执行原 disconnect。一个 side 的 disconnect 阻塞不阻止另一侧 disable。原 SDK/driver代码、运行期 watchdog、Start/Stop 语义不变；没有新增运行期 motion/heartbeat gate。

normal wrapper 的 finally 等待同一 teardown 协调器，不重复执行 native stop/disconnect。COMPLETE 必须双侧 disabled_verified/disconnected_verified 均为 True。失败/超时 launcher 报错，不把进程退出报告为清理成功。实时日志记录每侧 stop_started/stop_returned/disabled_verified/disconnect_started/disconnected_verified；持久证据在 /tmp/litchibot-validation-<uid>/<session-prefix>.shutdown.json，独立于随后被删除的 full session 临时目录。

回归包含真实原 MockSharpaHand 的 main ROS feedback 阻塞，由私有 shutdown 请求触发原 stop/disconnect并解除阻塞；双侧 stop 先于任何 disconnect；disable readback 仍 True 必须 FAILED；一侧 disconnect 阻塞另一侧已卸力；launcher等待真正 native 清理报告。测试为软件模拟，未验证实体 SDK 跨线程清理或机械卸力；需要现场实际 disabled_verified=True 和 disconnected_verified=True 结果确认。

## 关闭终端：SIGHUP 退出清理

此前 FullLauncher 与 adapter driver wrapper 只注册 SIGINT/SIGTERM。terminal hangup 的默认 SIGHUP 终止会绕过 finally；独立 setsid 的 Sharpa/ROS 子进程不自动随所属终端退出。

两处新增 SIGHUP 到已有 shutdown-request handler，正常主循环进入原清理：FullLauncher 先结束 hand wrapper，再 arm；driver wrapper 撤销 permit，原 driver stop/disable/disconnect。未恢复 command watchdog、未新增 heartbeat/motion gate，原 Sharpa SDK/driver 无修改。

验证：真实控制伪终端（setsid + TIOCSCTTY）关闭 master 触发 kernel SIGHUP，launcher exit 0；另测显式 SIGHUP/SIGTERM，双手 stub 优先正常退出、独立 worker 回收、endpoint删除。原 MockSharpaHand + wrapper 真正收到 SIGHUP，双方 stop/disconnect各一次，SDK加载禁止。ROS 304 passed /4 deselected；host 222 passed /96 skipped。无实体设备启动。

## GUI 关闭结束 full session

GUI closeEvent 通过 launcher 专属 Unix datagram endpoint 发送携带 session token 的 gui_closed 通知。endpoint 只由当前 FullLauncher 创建（0600，位于 session 临时目录），由 env 传给本 session GUI；不是 heartbeat，也不依赖 hand supervisor TCP 是否已连接或能够回复。

FullLauncher 主循环收到匹配 token 的明确关闭事件立即进入原 close，先 SIGINT 双手子进程，走原 driver stop/disable/disconnect，再结束 arm wrapper。正常 Start/Stop 仍只切换 forwarding，normal command timeout=0.0，runtime warning/TCP transient 不触发该事件。Standalone GUI 没有 launcher endpoint，不新增 shutdown 行为。launcher 退出清理 endpoint。

回归覆盖：真实 Qt window.close 在 hand supervisor 无连接时发送退出事件；真实 FullLauncher + stub 子进程拒绝错误 token、正确退出事件在1s内让手子进程完成清理、随后才开始模拟1.5s arm清理；原 mock driver lifecycle 测试继续验证 shutdown双方 stop/disconnect。所有验证无实体SDK、无实体动作。

## 当前生效：normal 恢复 standalone Terminal2 session 生命周期

LitchiBot full normal 与其 fake/no-send 的 `stop_on_command_timeout_s=0.0`；full Manus / supervised validation 仍使用原来的 0.20s，standalone Manus 默认仍为 0.0。仅改变 launch 配置及 adapter 对配置的核验；原 Sharpa driver、SDK、原退出清理和 native hardware protections 未改。

launcher 由原 driver connect/enable；Start/Stop 只开关 forwarding，Stop 不 stop/disable/disconnect。normal 不创建 NativeReenable worker、不添加 needs_enable、不在 callback 调用 _ensure_enabled，不发送旧 target。原 driver shutdown 仍 stop/disconnect。桥接诊断参数确认同步按模式检查实际 timeout。

回归 `test_native_reenable.py::test_normal_stop_idle_restart_and_shutdown` 使用原 MockSharpaHand、SDK 加载禁止 spy：左右连接/enable、Start、左 Stop 保持 source 30Hz 等待0.35s、右持续 ACTIVE、左无 send/stop/disconnect、无 timeout_stopped/needs_enable、左再 Start 立即发送最新绝对 target、_ensure_enabled零调用、shutdown双方各 stop/disconnect一次。launch selector 回归覆盖 normal/fake LitchiBot 与 supervised/Manus隔离。

验证结果：ROS 299 passed / 4 deselected；host 218 passed / 95 skipped；Qt GUI 15 passed；最终 lifecycle + 10s DDS/TCP 压力测试 7 passed。左右 source/forwarding 均30Hz，command interval max 33.74ms，callback max 0.336ms，heartbeat RTT max 10.14ms，timeout/disconnect/实体SDK加载均0。指标见 normal_session_lifecycle_metrics.json。

以下章节为历史审计记录；其中关于 normal 0.20s command watchdog 与 async re-enable 的描述已被本节取代。0.20s source freshness 不变。

# normal LitchiBot：Terminal2 + GUI forwarding switch 最终审计

2026-10-07。保留 NormalHandRuntime / NormalHandAuthority，替代此前 PAUSED、heartbeat lease、source-loss latch 设计。只调整 LitchiBot normal full 与匹配的 full fake/no-send；不启动实体设备。supervised/Manus 原策略不变。

## 1. 调用链

```text
glove acquisition / V3.5
→ backend.LitchiBotHandPipeline.tick
→ 原 SharpaRetargeter.map / vendor VisualAngleRetargeter
→ terminal2_worker（仅输出 target）
→ terminal2_bridge WorkerInbox(normal_full=True)
→ /teleop/sharpa/{side}/source
→ NormalHandRuntime.litchibot
→ BasicNormalTarget.accept（packet 完整性、物理范围、时效）
→ NormalHandAuthority 对应侧 forwarding permit / availability
→ NormalHandRuntime.process_normal
→ validation_driver.forward_full（共享单 SDK 宿主）
→ 原 SharpaDriverNode._on_command
→ 原 SharpaHandBase.send_action / SharpaSdkHand._write_joint_position
→ 原 SDK set_joint_position / interpolation / native protections
```

NormalHandAuthority 不继承 validation MotionLatch；BasicNormalTarget 不继承 RawContinuityGate/TargetGate。不新增数字阈值、pattern、all-valid、acquisition/slew、validation proof、F12 或恢复 warmup。

## 2. 剩余 command 修改

原 vendor clamp_inputs/input/output mapping/gain/offset；原 retarget 缺失依赖 hold/fallback 与物理 JOINT_LIMITS clip；原 driver 顺序/schema 处理、SDK interpolation 和 native 限制。均未修改。normal 已接受的有限合法 22D q 原值发送，序列化不增加限速/平滑/fallback。partial-valid 使用原 vendor q。

## 3. packet drop

NaN/Inf、坏 JSON/envelope/22D shape、wrong order/side、物理越界、stale/future、缺失/replay/out-of-order metadata：只丢当前 packet、限频 WARNING，permit 不变，不发送设备命令。下一合法帧可继续 ACTIVE。bridge 自身 malformed worker line/序列化失败同样只 drop，不通过 source_fault 变成 latch。FIFO 满丢最旧帧。

## 4. forwarding 停止与恢复

正常 Start/Stop 只修改对应 requested，不调用 hand.stop/disconnect，不停止 glove/V3.5/retarget/driver，不重置 original driver 的 watchdog 标记。

某侧超过既有 fresh-target 界限（0.20s）没有可用帧，或 source/driver ROS graph 暂不可用：该侧 OFFLINE，forwarding OFF；不新增 source-loss permanent latch，完全断流 1s/19s 也不 hard。另一侧独立继续。合法 source 恢复→READY，再 Start→ACTIVE。

真实认证 TCP control connection 消失：双手 forwarding OFF，OFFLINE；认证连接回来→READY。普通 heartbeat stale/replay/jitter/delay/未收到 heartbeat 不修改 permit/READY/ACTIVE，不调用 native stop。client 的 normal 回复延迟只打印限频诊断，不主动 abort；normal server 等待排队回复，不以原 validation 回复 deadline 人为断开连接。

## 5. 结构性阻止运行

仅真实不可恢复 device/SDK sender failure、已确认真实 disconnect、duplicate/unexpected ownership、真实绑定 side/serial 不可能成立等结构性问题，保留内部 FAULT/原 native stop。用户视角显示 OFFLINE、原因可见，不能靠 Start/heartbeat 清除真实底层故障。

单个坏 target/source packet 不在此类。normal source_fault 即使收到旧式 HARD_FAULT 标签，也只视为 packet/status warning，不触发设备 stop；设备/ownership 故障由原单一宿主检查。

## 6. WARNING 与统计

partial-valid、held/fallback 有限 q 接受不升级；快速动作、moderate jump 无 custom gate。坏 packet、旧 heartbeat、临时 acquisition 重启及 telemetry 问题限频记录。per-joint delta 继续只作统计；source gap/新 session 首帧重建 baseline，下一帧 adjacent comparison；非有限 canonical 诊断角度不发给 SDK，统计跳过，不构造 fault。

## 7. GUI 状态

```text
READY --Start side--> ACTIVE --Stop side--> READY
ACTIVE --真实 control/source unavailable--> OFFLINE
OFFLINE --connection/source recover--> READY --Start--> ACTIVE
```

用户视角仅 READY / ACTIVE / OFFLINE。单个坏帧及 heartbeat warning 不改变状态。glove/V3.5/retarget/driver/connection 常驻；左右独立；Start/Hold/Release arm 不动 hand permit；DISENGAGE ALL 可以关闭全部 forwarding。

## 8. heartbeat 与 stop/latch 搜索

heartbeat metadata 只诊断；sequence/transport generation 更新只用于日志与接入记录，不是 lease 或 hold-to-run。认证 TCP 的实际 disconnect 才撤销 forwarding。

NormalHandRuntime.pause_native/reconcile_pauses 是无设备动作的兼容空钩子；request/disconnected/source_fault/坏 packet 不调用 hand.stop。normal 的 stop_validation/trip 只剩真实结构性设备/ownership 故障及相应内部 FAULT 后续处理。原 native watchdog 的 hand.stop 保持原样；normal 不清除 _has_received_command/_timeout_stopped，也不把原 timeout stop 升级成 permanent latch。下次 Start 后发送采用原 enable API。正常进程退出走原 stop/disconnect 清理，不记录假 HARD_FAULT。

原 Sharpa driver/SDK/vendor mapping/native limits 与 Manus/V4/arm/tactile/recorder/camera 原实现未改。

## 9. 回归

结果见 normal_discontinuity_test_results.json 的 normal_terminal2_switch_simplification。覆盖 heartbeat stale/replay/delay ACTIVE、实际 TCP 断连 READY 恢复、19s source gap READY 恢复、各类坏 packet 后合法帧继续、partial fallback、原 native watchdog 实际停手后重新 Start/send、结构性 SDK/duplicate 故障仍阻止发送、Qt 实际延迟 0.35s 回复不重连、arm HOLD 独立。

仅 mock/fake/native-mock、隔离 ROS domain199 与私有测试锁；未加载实体 SDK、未启动实体设备、未清除现场 fault 或重启现场进程。测试组重叠不能加成独立总数。ROS 排除三个需 host pinocchio 的 CLI 用例（host 覆盖）及一个共享 production lock 的直接 supervised guarded-driver 用例，保留隔离 mock-driver 集成与 strict 单元测试。

最终结果：ROS 283 passed / 4 deselected；Qt 14 passed；host 209 passed / 88 skipped；最终 normal 定向 32 passed；均 0 failed。Python compileall 与 git diff --check 通过。现有进程需一次正常重启加载新代码；之后普通断流/坏帧/重连不要求 launcher restart。

## source backlog 修复

现场 bridge 约30Hz/age30–56ms/rejected0，但 driver约20Hz反复拒绝 stale source。normal source 订阅原 depth1000 FIFO 会积压旧 target，使真实 source 活着却长期 OFFLINE。normal/full fake 改为 KEEP_LAST depth1，只执行最新观测；supervised/Manus 原订阅不改，0.20s freshness 不放宽。坏 packet 日志 dt 明确记录其 age。DDS 回归100帧/侧 burst（99 stale +最新 fresh）：旧 depth1000消费200条，修复后仅最新2条，最新sequence与data_ready恢复。完整ROS283 passed /4 deselected，host209 passed /88 skipped。现场进程未重启、permit未改变；实体效果需加载新代码后验证。

## idle feedback 继承路径修复

现场 python_144756_1791351333731.log：SDK connect 后约0.26s 出现 Invalid idle hand feedback，先于 heartbeat warning；bridge 同时持续约30Hz/age32–61ms/rejected0。normal 仍继承 FullHandRuntime.idle_snapshot 的 validation 反馈检查，单个不合检查的 snapshot 直接 stop_validation 并锁存双手，是软件遗漏，无法从旧日志区分 NaN/shape/具体越界 joint。

NormalHandRuntime 现在独立覆盖 idle_snapshot：坏shape/NaN/Inf/临时读取失败只跳过 telemetry snapshot 并限频 WARNING；有限 measured 超出 command range 只诊断，保留真实测量，不 clip、不作为执行target。ACTIVE 路径直接使用已经 BasicNormalTarget 检查的合法 q，经原 sender/SDK 发送，不额外读取实测反馈当 blocker。最终 target 的NaN/Inf/shape/order/side/physical joint limits/freshness 检查保持，真实 disconnect、SDK sender failure 和 ownership/binding 失败仍阻止发送。supervised/Manus 的原 idle 检查不变。

新增 NaN/Inf/shape/越界measurement/临时SDK读异常/坏snapshot后合法target继续回归；原 telemetry topic 与 recorder 保留。真实结构性 fault 的 GUI reason 优先展示首因，不再被 waiting-source 掩盖。未重启现场进程、未操作实体。

## normal hot-path 阻塞隔离

本次只修实时 forwarding 中断。首次 Start 直接转发当前绝对 target 是预期行为；没有 startup blend、offset、alignment、slew/speed clamp、首次delta门槛或 target-current blocker。mapping 不改。

NormalHandRuntime 不再重复同步 SDK read_joint_state：额外 idle/日志实测值使用原 driver 发布的 joint_states 缓存。source 仍 KEEP_LAST depth1，不追赶 FIFO。每个 command callback 仅做 JSON decode、最终完整性/freshness、轻量 per-joint delta 统计、permit、原 sender，以及原实际 command topic 发布。

WARNING 在 owner thread 先限频，再将捕获上下文交给 DeferredTelemetry；JSON编码/日志输出在后台，不读取 SDK、不操作 authority。delta 统计快照复制后由后台序列化/写文件；100Hz 状态文件写入按 key 合并，只存最新待写状态；supervisor ROS 状态消息也后台发布。诊断待处理队列有界32项，同key覆盖；worker锁只取/放任务，不持锁做I/O。原 command 发送与 recorder 的实际 command topic 发布保持在原执行链，未移入可丢弃的诊断队列。

HandClient 在首次回复前就从 session source/mode 得知 normal 路径；初次回复超过0.20s也只诊断，不先 abort TCP 再清 forwarding。原 supervised/Manus 客户端策略不变；normal TCP实际连接消失仍OFFLINE。

可重复 no-send 压力测试为 test_normal_realtime.py：真实ROS DDS、NormalHandRuntime/NormalHandAuthority、原mock Sharpa sender、真实认证TCP heartbeat、原0.20s native watchdog。SDK加载spy禁止实体SDK。注入150ms logger、300ms统计写盘、250ms后台状态磁盘工作，检查连续ACTIVE/每侧接近30Hz/无timeout/无disconnect/无source积压。GUI额外覆盖0.35s首个回复延迟。60s数据存放 normal_realtime_metrics.json；mock数据不能替代实体SDK实时性实测。本次不启动实体动作、不重启现场进程。

60s压力测试：左右 source/forwarding 30.00Hz；原 native command timeout=0，supervisor disconnect=0，normal额外SDK feedback读=0，诊断pending=3（上限32），physical SDK load=0。原厂watchdog0.20s保持。以下数据单位ms，合并双侧callback/age；source是合成30Hz，不包含实体V3.5/USB/SDK延迟。

| 指标 | mean | P95 | max |
|---|---:|---:|---:|
| Left command interval | 33.333 | 33.423 | 35.138 |
| Right command interval | 33.333 | 33.474 | 34.999 |
| callback processing | 0.157 | 0.275 | 0.518 |
| heartbeat RTT | 9.916 | 10.107 | 12.532 |
| target packet age | 0.283 | 0.549 | 1.258 |

后台队列溢出优先保留首次真正HARD_FAULT root日志，诊断可合并/丢弃；不影响已执行command或permit。回归包括拥塞时只保留最新状态以及慢首个GUI回复不主动断开。

最终回归：完整ROS292 passed /4 deselected、Qt15 passed、host217 passed /89 skipped、最后诊断失败隔离/队列定向41 passed、独立60s压力1 passed；全部0 failed。测试组重叠，具体排除与mock边界见结果JSON。compileall/git diff --check通过。

## per-side 异步 native re-enable

保留现有 NormalHandRuntime / NormalHandAuthority，只将 native-enable 等待从 source callback 移到 NativeReenable。每侧一个持久worker，只有一个在执行的operation。SDK _ensure_enabled 原实现、原timeout/保护、glove/V3.5/retarget/mapping、heartbeat/partial/freshness策略均未改。

source→最终检查/更新 latest_target（raw.previous 每侧仅一项）→permit→forward_full：needs_enable且IDLE时提交本侧task并立即return False；RUNNING/FAILED同样不发SDK command，不发布假的已发送command。worker只调用native enable，不存target，不发送target，不修改permit。READY后的当前/下一合法source callback才发送当前q；不回放触发enable的旧q。

后台enable成功不改变permit，期间Stop后的新帧仍不发送。enable失败仅本侧status OFFLINE/ERROR，可用原Stop/Start在同一worker显式重试，普通source帧不自动重试、不创建新线程、不全局FAULT。GUI仅在这个retryable enable错误下保留Stop/Start按钮可用，没有新按钮。原driver watchdog标记由原实际发送成功重置，没有手工绕过。

真实task退出前不disconnect：正常shutdown先等待正在执行的native call结束，再执行原stop/disconnect，防止后台迟到enable在disconnect后运行。这只涉及进程退出，不在realtime callback中等待。

mock验证：原mock driver的真实watchdog停左手后，故意让左enable等待约0.5s，左右各接收约30Hz新帧，右手持续发送，callback均<20ms；worker并发数1；成功只发送新的.33 target，不发送触发旧.08或等待时.24；Stop竞态后无自动发送；失败20个新帧不再次enable、右手继续ACTIVE、显式重试复用原线程。真实DDS+TCP的1s-enable压力数据存于 normal_async_reenable_metrics.json；仅native-enable该侧有预期不发送窗口，右手与heartbeat无等待。实体SDK线程行为未实测，未启动实体动作。

异步enable结果：注入左侧native enable1.000s，左右source29.98Hz，右forward29.98Hz；左全程平均24.82Hz包含故意1s等待（成功后恢复）。callback mean/P95/max0.153/0.219/0.290ms，heartbeat RTT9.928/10.067/10.154ms；timeout/disconnect均0，只启动一次enable。完整ROS297 passed /4 deselected、Qt15 passed、host218 passed /93 skipped；最后native专项4 passed（含真正hard-stop与迟到enable竞态）。首次真实hard-stop后后台enable不得重新激活设备，worker原调用的finally按既有FAULT重新调用原stop，普通Stop/enable失败不会触发该动作。所有0 failed，测试组重叠。
