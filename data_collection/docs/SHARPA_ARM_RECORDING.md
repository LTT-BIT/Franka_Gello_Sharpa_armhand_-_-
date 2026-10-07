# GELLO 双臂 + MANUS/Sharpa 双手 + 三相机录制

需要把整套三终端工作流迁移到另一台电脑时，参见
[`工程迁移与一键安装说明.md`](../../工程迁移与一键安装说明.md)。该文档包含原电脑
打包、新电脑安装、端口修改和首次硬件检查清单。

这条硬件验证链路把双臂 action/state、双手 command/state、三路 RGB 和头部深度
录入同一个 ROS 2 bag。录制器只订阅 ROS topic，不向机器人发命令，也不建立新的
libfranka/FCI 连接。

## 为什么不能直接使用 `manus_sharpa_record/record.py`

该程序会调用 `pylibfranka.Robot(franka_ip)` 直接连接一台 FR3。正在运行的
`franka_ros2_control_node` 已经占用 FCI；再启动一个 FCI 客户端会发生抢占或
`Connection to FCI refused`。它也只有一个 `--franka-ip`，不是双臂录制器。

不要在双臂遥操期间运行这个直接连接 FR3 的录制路径。

## 启动和录制

终端 1：保持原来的双臂启动流程。

```bash
cd /home/user/gello_upper_body_teleop
./ops/run/run_gello_arms_only.sh
```

终端 2：用隔离后的 CycloneDDS 包装器启动原有 MANUS/Sharpa 程序。

```bash
cd /home/user/gello_upper_body_teleop
./ops/run/run_sharpa_hands_cyclonedds.sh
```

终端 3：确认双臂、双手都正常跟随之后启动常驻采集器。

```bash
cd /home/user/gello_upper_body_teleop
./ops/run/start_sharpa_arm_recording.sh --with-cameras
```

该模式从 `data_collection/config/cameras.yaml` 启动三台 Orbbec。相机和采集控制
服务在整场任务中保持运行，不会在每个 episode 之间重新初始化。调机位时使用的
OrbbecViewer 必须先关闭。如只需要复验臂和手、不访问相机，仍可省略
`--with-cameras`。

### 单独的触觉录制模式

上面的原终端 3 命令保持不变，仍然不录触觉。如需在同一个 bag 中额外录入双手
五指的 F6 wrench 和 deformation，共 20 个触觉 topic，用下面的新命令替代终端 3：

```bash
cd /home/user/gello_upper_body_teleop
./ops/run/start_sharpa_arm_recording_with_tactile.sh --with-cameras
```

原终端 3 和触觉版终端 3 是两种互斥方案，不能同时启动。触觉版仍使用 Operator GUI
和采集踏板开始/停止 episode，并保留原有的 rosbag 收尾、完整性校验和质量评价流程。
它单独输出到：
```text
$TELEOP_DATA_ROOT/bags/gello_sharpa_tactile/episodeN/
```
新增```/home/user/gello_upper_body_teleop/data_collection/config/rosbag_qos_sharpa_tactile.yaml```，覆盖全部 36 个录制 topic：
- 相机、FR3、Sharpa command：reliable
- Sharpa joint state、20 路 tactile：best_effort

[/home/user/gello_upper_body_teleop/data_collection/config/record_gello_sharpa_tactile.yaml(line 29)] 现在只显式传入这一份 QoS 文件，不再生成全局 best_effort 临时文件。

触觉版要求终端 2 已发布左右手各五指的 `wrench` 与 `deformation`，不支持
`--without-hands`。它不录 tactile raw 图或 `CONTACT_POINT`。每路 topic 的类型、
最低频率 18 Hz、源时间戳单调性和最大 500 ms 间隔都会在停止后检查。500 ms 是根据
103 秒实测包中 286.8–490.5 ms 的最差间隔设定的首版基线，并非 SDK 标称 60 Hz。
触觉版使用独立的逐 topic QoS 文件
`data_collection/config/rosbag_qos_sharpa_tactile.yaml`：相机、FR3 和 Sharpa
command 使用 reliable；Sharpa joint state、wrench 和 deformation 使用
best-effort。QoS 按 topic 身份配置，不能按消息类型判断，因为相机图像和 tactile
deformation 都是 `sensor_msgs/msg/Image`。原录制配置没有改变。

手套或 Sharpa 不在场时，不启动终端 2，并在终端 3 显式跳过手部 topic：

```bash
cd /home/user/gello_upper_body_teleop
./ops/run/start_sharpa_arm_recording.sh --without-hands --with-cameras
```

这类测试 bag 单独写入 `bags/gello_arms_camera`，不能当作完整的臂手训练数据。
若连相机也不启动，则使用 `--without-hands`，输出到 `bags/gello_arms`。

终端显示 `READY` 后，在 Operator GUI 点击“开始录制”。完成一段演示后点击“停止并
校验”。录制器会向当前 rosbag 发送一次
`SIGINT`，等待 `metadata.yaml` 写完，再执行完整性校验。不要用 `Ctrl-C` 停止单个
episode。

校验完成后 GUI 自动弹出“数据质量评价”。选择“优等 / 一般 / 报错 / 放弃”后，采集器
回到 `READY`，可以直接开始下一段。所有 episode 完成且当前没有正在录制的数据时，
再按 `Ctrl-C` 退出常驻采集器和相机。

## USB 踏板

两块踏板都是普通的 USB 键盘模拟设备，已经使用 ElfKey 写入不同键值。Operator GUI
直接按键值区分动作，不依赖 USB 物理端口，两块踏板可以交换 USB 接口。

机械臂 Hold 踏板：

| 按键 | 动作 | 说明 |
| --- | --- | --- |
| `A` | 左臂 Hold 开/关 | 锁定或解除左臂原地悬停 |
| `C` | 右臂 Hold 开/关 | 锁定或解除右臂原地悬停 |

Hold 只会暂停机械臂跟随遥操输入；控制链仍持续发布机械臂实测关节角。它不是机械臂
使能开关。真正的跟随启用/停用仍需点击 Operator GUI 的 `Start arm` 按钮，目前没有
绑定踏板。

数据采集踏板：

| 按键 | 动作 | 说明 |
| --- | --- | --- |
| `L` | 开始/停止录制 | `READY` 时开始 episode，录制中停止并进入校验 |
| `Space` | 添加里程碑 | 仅在录制过程中打点，不改变录制状态 |

采集踏板由 Operator GUI 通过 `127.0.0.1:5592` 向采集器发送命令。

快捷键禁用自动重复，但 Operator GUI 窗口必须获得键盘焦点，踏板才会响应。手部控制
已在独立的 Sharpa 终端中运行，没有手部踏板绑定。

默认输出到：

```text
$TELEOP_DATA_ROOT/bags/gello_sharpa/episodeN/
```

录制器先核对下列 8 个臂/手 topic：

- `/teleop/validated_arm_commands`
- `/teleop/arm_command_status`
- `/left/franka/joint_states`
- `/right/franka/joint_states`
- `/sharpa/left/command`
- `/sharpa/right/command`
- `/sharpa/left/joint_states`
- `/sharpa/right/joint_states`

相机启动后还必须发现并录到下列 4 个图像 topic：

- `/cam0/color/image_raw`
- `/cam0/depth/image_raw`
- `/cam1/color/image_raw`
- `/cam2/color/image_raw`

发现 topic 名称还不算就绪。录制器会等待四路图像真实到帧，并要求 `cam0` 彩色和
深度至少 18 Hz、两路腕部 RGB 至少 25 Hz；全部数据流连续 5 秒且接收帧间隔不超过
150 ms 后采集器才会继续进入 `READY`。默认最长等待 90 秒，超时会停止相机且不会
留下空包。需要调整硬件诊断等待时间时可传 `--camera-ready-timeout SECONDS`。

停止录制后，脚本会按图像自身的 `header.stamp` 再扫描整包。源时间戳频率不足、
不单调或内部间隔超过 150 ms 会使验收失败；DDS/rosbag 接收时间抖动会作为
`TRANSPORT WARNING` 单独显示，避免把完整但延迟送达的图像误判成相机丢帧。

常驻采集器始终在 `127.0.0.1:5592` 提供控制和质量服务。终端 1 的 Operator GUI
负责开始/停止 episode，并在停止校验后弹出评价窗口。分类写入
`$TELEOP_DATA_ROOT/数据分类/<录制组>/优等.txt` 等文件；不同录制组相互隔离。完成
评价前 GUI 不允许开始下一段，避免上一段失去质量标签。

四路 `camera_info` 也会写入 bag，但不作为非零消息强制验收项。

这是方案 A 的安全实现：只有一个 rosbag 录制器和一条系统时间轴，不需要事后合并。
Sharpa 启动器还会把 MANUS、V4 optimizer 和 Sharpa SDK 固定在 `HOUSEKEEPING_CPUSET`，
避开 FR3 控制器的实时 CPU。

## 当前边界

- 当前只生成原始 ROS bag，不直接写 LeRobot。应先保留原始时间戳数据，验证频率和
  完整性后再离线转换。
- 若右臂日志出现 `Connection to FCI refused`，先在 Panda Desk 为右臂重新启用 FCI；
  这不是 DDS 或 Sharpa topic 冲突。
