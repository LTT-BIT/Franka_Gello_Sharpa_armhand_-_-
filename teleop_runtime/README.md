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
episode，不关闭统一遥操。停止写盘后自动 Home 双臂并保持 Hold，手不归位。
缺 3 帧按单个相邻帧间隔判断，整段累计数仅作统计。

统一 GUI 开始 episode 时登记本段任务的双臂 Home 要求。rosbag 进程退出后，
采集端 `after_home` 从 armed 转为 pending，统一 launcher 的后台协调线程调用
现有臂服务 `episode_home`；臂服务只复用原 Home 工作线程及实际到位检查。
Home 与 bag 校验并行，质量窗口可以同时操作。`can_start` 及 start RPC 同时检查
Home 成功和分类已保存；严重连续性失败自动“报错”后也必须等 Home 完成。

`after_home` 包含 episode、任务、任务编号、重试次数、状态及原因，完成校验后连同
quality 保存在 `collection_state.json`。带任务编号的回执防止重连／丢回复造成重复 Home；
失败或无法确认结果锁住开录，必须点 Home both arms 或踩 B 重试。
手动 Home、手部 Open hand 保留；自动 Home 不改变手部姿态。协调线程仅处理本次
统一入口会话登记的任务，退出 launcher 停止协调，不接管采集器的进程生命周期。
本次会话重开后若仍留有未完成 Home，需操作员明确按 B／Home 按钮接续重试。


默认推荐触觉采集器；普通采集配置仍未启用实时缺帧监控，GUI 会明确提示关闭。

## DDS 图像接收缓冲（2026-10-15 修复）

统一入口和触觉采集器使用 `config/cyclonedds.xml`，显式要求 8 MiB 的 UDP
接收缓冲。此前系统约 208 KiB 的上限会使图像分片接收丢包并触发重传，出现
源时间戳连续、接收时间却长时间停顿后补齐的现象；该配置不替代源缺帧监控。

每台部署电脑首次运行一次：

```bash
sudo bash ops/setup/install_dds_buffers.sh
python3 teleop_runtime/dds_buffers.py
```

安装器即时提高 `net.core.rmem_max` 至至少 8388608，并写入
`/etc/sysctl.d/90-gello-dds-buffers.conf` 供重启后加载；已有更高上限会保留。
它不修改默认 UDP 缓冲、不启动设备。统一入口及触觉采集器会在启动时检查
系统上限，过低则退出并提示安装命令；DDS 也拒绝静默退回较小缓冲。

应用后重新启动统一遥操和独立采集终端，使 ROS 进程重建接收 socket。
Linux `SO_RCVBUF` 的实际记账值可能显示 16777216，为 8 MiB 请求的双倍。
普通预警、连续缺帧严重退出和臂手安全检查保持现有规则。

本机已完成 8 MiB 设置与实际 DDS socket 检查；上线后还需进行双臂、双手、
触觉和三相机满负载长录制验证。缓冲不能修复相机源端自身缺帧。
回退本次系统安装时，删除上述独立 sysctl 文件、还原原系统上限（本机安装前为
212992），并同时回退 XML 的 8 MiB 要求及对应启动检查；不要仅降低系统上限。

## 相机重连与 Viewer 使用（2026-10-16）

采集器持续检查已启用的相机图像流，开始录制及相机重连后都要求接收与源时间戳
连续稳定 **3 秒**。未就绪时统一 GUI 的采集窄栏显示等待原因，禁用开始；后台
也会在真正创建 episode 前重新检查。禁用相机时不等待相机，普通版维持原有
连续性策略。录制中的 1 秒断流和严重源缺帧退出规则保持不变。

OrbbecViewer 访问顶部 cam0 可能重建设备流，不能与采集终端同时使用。Viewer
启动脚本现在检查普通及触觉采集容器，采集脚本也检查已运行的 Viewer。即使没有
正在录制 episode，只要采集终端仍运行，也会阻止从脚本打开 Viewer。
需要使用 Viewer 时，先结束当前 episode 并关闭独立采集终端，再启动 Viewer；
关闭 Viewer 后重新启动采集终端，等待窄栏就绪再踩 L。不必为此关闭臂手遥操。
不要绕过脚本直接打开 Viewer 访问正在采集的相机。

代码修改需要重新启动采集终端和统一入口 GUI 后加载；不热重启正在工作的设备。
