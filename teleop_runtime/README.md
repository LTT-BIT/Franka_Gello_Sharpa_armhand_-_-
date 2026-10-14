# Teleop runtime

`cli.py` 是统一 Operator 后端入口。它组合机械臂输入、手部输入、Operator
控制协议和硬件协调器；默认仍是 GELLO 双臂与 MANUS/O30i 双手。

Wuji 和 powderweighing 都是显式可选项。前者通过 `--hand-source wuji` 加载，
后者只有传入 `--right-hand-strategy-config` 时才包装右手策略，因此不会改变
默认 O30i 行为。Wuji 模式默认向本机 UDP 5602 发送只读 telemetry；可用
`--disable-hand-telemetry` 关闭。采集进程是独立的，见
`data_collection/README.md`。

机械臂跟随调试日志 `ee_jitter.jsonl` 的文件写入、刷新和关闭由后台线程执行。
控制循环只生成不可变的 JSON 快照，最多排队 256 行；队列满时丢弃新调试行，
不等待磁盘，也不改动训练 bag。后台每 5 秒及退出时更新同目录的
`ee_jitter.writer_status.json`，记录已接收、已写入、待写入、丢弃行数及错误。
丢弃提示由后台限频打印，原有 `follow-debug.v6` 数据行格式不变。
退出时先释放硬件，再最多等待后台 0.2 秒；磁盘持续阻塞或强制退出时，尚未
写完的调试行可能丢失。该队列只存调试日志，不参与指令发送或录包。

临时断档诊断：现有 `--debug-log .../ee_jitter.jsonl` 启动方式会同时启用
`.../ee_jitter.loop_timing.jsonl`，无需增加启动参数。正常循环只在内存中计时；
循环或指令发送间隔超过 100 ms 时，保存当前及前一循环的分段耗时、线程 CPU
耗时、命令序号、会话标识和墙钟/单调时钟对应关系，供对照 episode 时间戳。
`loop_wait` 包含正常 sleep 及其超时；低 CPU 耗时只说明等待或未获调度，不能
单凭它判定具体阻塞原因。所有诊断文件写入及终端提示均在后台线程中完成，
队列最多 64 项，满时丢弃诊断并累计计数，不等待写盘；终端异常提示至多每
5 秒一次。启动和正常退出也有记录，方便确认诊断是否运行。
临时诊断不改变指令或安全阈值。该诊断用于定位本次断档及验证后台写入修复，
根因修复并验证后删除。采集后提供异常 episode 编号即可，不必手工整理日志。

## 默认统一入口与录制告警

后续默认使用统一的臂和 Manus/Sharpa 使能界面：

```bash
./ops/run/run_gello_full_teleop.sh --hand-source manus --hardware
```

在该 GUI 中使能双臂和双手，并确认正常跟随后，在独立终端启动触觉采集器：

```bash
./ops/run/start_sharpa_arm_recording_with_tactile.sh --with-cameras
```

统一入口已启动手部程序，不要再同时启动旧的 `run_sharpa_hands_cyclonedds.sh`。
统一 GUI 使用同一采集控制服务（默认 `127.0.0.1:5592`），窄栏明确显示实时缺帧
监控是否开启。L 踏板开始／停止 episode，Space 打点；普通预警只在窄栏显示，
严重故障由独立采集器自动收尾并归为“报错”，停止后才弹窗。该故障只结束当前
episode，不关闭统一遥操或改变臂手使能。下一次 L 才开始新段。
缺 3 帧按单个相邻帧间隔判断，整段累计数仅作统计。

默认推荐触觉采集器；普通采集配置仍未启用实时缺帧监控，GUI 会明确提示关闭。
