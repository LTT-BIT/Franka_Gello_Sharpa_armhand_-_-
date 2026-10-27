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
episode，不关闭统一遥操；随后自动 Home 双臂并保持 Hold，手不归位。满足下面的条件后，下一次 L 才开始新段。
缺 3 帧按单个相邻帧间隔判断，整段累计数仅作统计。

统一入口还会在结束 episode 后自动执行**双臂 Home**：先确认 rosbag 写盘进程已经退出，
然后让双臂 Home 与 bag 校验并行。校验完成即可选择质量，无需等待 Home；选择质量时
本段已经停止写盘。只有**质量分类保存成功、双臂实际到位**两项都完成，采集端才允许
下一次 L 或点击开始录制。Home 使用本段开始时选定任务的已保存臂部 Home，手不归位，
Open hand 保持原功能。手动 Home both arms 和中间踏板 B 仍可使用。

### 录制前 RGB 亮度检查

普通 Sharpa 和触觉 Sharpa 配置均开启此项。每次开录前，对本次启用的必需 RGB
相机 `/cam0/1/2/color/image_raw` 计算有效像素的三通道平均值，忽略行尾填充。
每路均需连续 **3 秒**达到 **25/255**（等于 25 通过），并满足原有相机帧率、
时间戳与新鲜度要求，才允许开始录制。低亮度、无图像、无效图像或帧间断会重置
该路亮度就绪计时。GUI 采集状态栏显示未就绪的相机、当前均值或具体原因；
开始按钮暂不可用，恢复正常后自动解除。深度和触觉图像不参与亮度检查。

参数：`rgb_preflight_enabled: true`、`rgb_brightness_threshold: 25.0`、
`rgb_preflight_stable_sec: 3.0`。没有启用相机时不增加相机亮度等待。
录制期间暂停亮度计算，不因暗图报错或停录；录后 bag 校验也不检查亮度，
不增加亮度导致的 A/B 评价限制。下一段开始前重新检查亮度。原有丢帧监控、
连续性校验、自动停止与评价规则保持原样；不会更改相机曝光、已有 bag 或状态文件。

严重连续性错误自动停止时也执行同样的双臂 Home；自动保存“报错”代替人工评价，
仍需等 Home 成功，且相机恢复连续稳定 3 秒后才能重新开录。Home 失败／中止／无法确认
结果时，状态栏保留原因并锁住开录；处理原因后点击 Home both arms 或踩中间踏板 B
重试。自动 Home 进行中重复 B 不会排入第二次归位任务；不会自动开始下一段。
自动 Home 由统一入口协调，经现有臂控制服务和安全网关执行，采集器不发送关节命令。
旧版分离启动入口维持原行为。


默认推荐触觉采集器；普通采集配置仍未启用实时缺帧监控，GUI 会明确提示关闭。
下面的原三终端说明保留用于分离启动或特殊诊断。

## GUI 右侧保存设置与 A–D 归档

统一界面右侧的“本地保存设置”可输入或浏览选择本机**保存父目录**，界面会记住选择。
未手动选择时默认使用当前录制组的 bag 目录。父目录可以位于当前用户主目录、
`TELEOP_DATA_ROOT`、`/media` 或 `/mnt`；更换映射配置后需重启采集终端。

| 质量选择 | 最终原始 bag 目录 |
|---|---|
| 优等 | `<父目录>/A/episodeN` |
| 一般 | `<父目录>/B/episodeN` |
| 报错 | `<父目录>/C/episodeN` |
| 放弃 | `<父目录>/D/episodeN`，自动删除关闭时保留 |

开始本段时固定保存父目录，录制中修改右栏只影响下一段。原始 bag 先写入
`<父目录>/pending/episodeN`，停止并完成 bag 校验后，选择质量将完整 episode 目录
原子移动到对应分类；同一文件系统内仅改目录名，不复制大 bag，也不转换为 LeRobot。
原始消息、metadata.yaml、collection_state.json 和诊断文件一起保留。右栏显示
当前暂存／归档状态和准确路径，“打开本段文件夹”可直接查看。

严重连续性故障自动归类“报错”时直接移入 C；仍保留 incomplete 和故障原因。
改评质量会将同一目录移到新类别。现有中文质量编号清单继续更新，episode 编号
通过原录制组中的持久计数器保持递增，归档、更换父目录或重启不会重用之前的编号。
已有数据不会由升级操作批量移动。

目录不可写、目标同名或分类文件保存失败时不允许开始下一段，原始 bag 保留在当前
位置，界面显示原因；处理后重新选择质量重试，不会覆盖同名数据。下一段需要
**分类归档成功、双臂 Home 到位、原有相机就绪检查通过**。异常退出未分类的数据
保留在可见的 `pending` 内，可直接通过文件管理器查看、移动或删除；请仅清理确认已停止录制的数据。
升级前遗留的 `.pending` 不会自动搬动或删除，可按 Ctrl+H 显示隐藏文件后手动处理。

## 右栏自动上传与自动删除

可先运行 `./ops/run/run_collection_transfer_settings.sh`，在不启动遥操／采集的情况下
选择本机监控父目录、已挂载的远端父目录，以及“自动上传”“同时上传 D 类”“上传后读回校验”“自动删除”开关。统一
GUI 右栏提供相同设置；首次使用自动上传／上传 D／自动删除关闭，上传后读回校验开启，之后恢复上次设置。远端示例为 `/home/descfly/company-remote-2`，
需要按原 SSHFS 命令自行挂载。未挂载不会把数据误传到本机空目录。

独立后台随采集脚本启动，第一次只建立目录基线，绝不传输／删除已有数据；更换
本机父目录也重新建立基线。之后新归档且质量／Home／本段文件操作全部完成的 A/B
自动进入上传队列，C 不上传；D 是否上传由“同时上传 D 类”决定。新 episode 不等待旧上传，网络操作不持有录制锁。

自动删除关闭：所有本地数据保留。自动删除开启：A/B 和选择上传的 D 在上传成功后删除；开启“上传后读回校验”时还须校验通过，C 和未选择上传的 D 在文件收尾后直接删除。本地删除后的原始诊断 JSON 和操作回执
保留在 `~/.local/state/gello-teleop/collection-transfer/`，不能再通过质量按钮改评。
开关组合允许“只上传保留本地”及“只删除新 C 和未选择上传的新 D，不动未上传 A/B”。

录制终端手动 Ctrl-C 后，后台继续独立收尾队列，再自行退出；遥操 GUI 的启停不
影响它。录制中网络失败会退避重试；关闭采集后网络中断／超时任务保留本地与远端
暂存、记录中断，不无限阻止后台退出。单次操作无进展 90 秒会中断文件子进程。
下次启动不会自动处理这些历史中断数据。上传／删除进度及结果在右栏独立显示。

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

### 触觉版：实时缺帧预警与自动结束 episode

触觉版已默认启用实时连续性监控；普通版保持原来的录制行为。终端 1 的
Operator GUI 在录制中只在数据采集状态窄栏显示黄色警告，不弹窗。
严重故障自动结束本 episode 并归为报错，收尾完成后才弹出红色非模态错误窗口。

- cam0 RGB/depth 按 20 Hz、cam1/cam2 RGB 按 30 Hz 检查各自的源时间戳。
  相邻两帧之间连续缺 1～2 帧警告并继续，单个间隔缺 3 帧及以上自动结束本段。
  整段累计缺帧数只作统计；多处分散缺帧不会按累计 3 帧触发退出。估算采用
  `max(0, floor((gap_ms - 5) / period_ms + 0.5) - 1)`，留出 5 ms 抖动余量。
- 左右臂状态、arm status、Sharpa command/state 按 30 Hz 新鲜度判断；左右臂
  action 分别检查，仅该侧 engaged 时要求更新，HOLD 不要求该侧持续发布 action。
- 20 路触觉按本机基线检查：source gap 超过 250 ms 黄色预警，超过 500 ms
  自动结束。不将 tactile deformation 当作相机图像，不套用 SDK 60 Hz 的缺帧数。
- source 时间戳缺失、重复或倒退判失败。接收间隔大但 source 连续只警告；
  应更新的数据流超过 1 秒没有收到消息，自动结束并提示“接收中断，源或传输原因待确认”。
  Sharpa 时间戳反映 ROS 发布连续性，不等同于 SDK 原始采样时间。

自动结束会先向当前 rosbag 发送 SIGINT 并等待收尾，再校验实际落盘数据，保留
原始 bag。`collection_state.json` 保存 `live_continuity`、逐流计数／最大间隔、
`auto_stop_reason` 和 bag 校验报告。本段保持 `incomplete`、`finalized=false`，
自动加入本录制组的“报错.txt”；再次校验或人工评价不能使自动失败的数据变为合格。
如分类文件保存失败，GUI 会要求重试“报错”或“放弃”，完成后才允许重录。

终端 3 常驻程序、相机和臂手遥操不会退出。收尾与自动分类完成后，GUI 显示
“已归类为报错”；统一入口还需等待双臂 Home 成功。下一次踩 L 或点击开始录制才创建新的 episode；
不用额外确认质量评价，不会自动续录。收尾期间踏板不能开始新段。
如故障仍存在，新段仍会自动停止。需要设备冷却时，由操作员安排暂停相机及遥操。

停止后仍按现有最低平均频率验收；录制首尾覆盖不足与内部缺口分别判断，首尾裁剪
不会隐藏内部缺帧。仅接收抖动不会被称为“过热”或“相机源缺帧”。

当前实现已通过离线模拟测试，设备上线后还需验证满负载开销与触觉预警频率。
本机采集器使用源码链接安装，重启 GUI 和终端 3 即可加载此次修改，无需重建工作区。

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
完成分类；统一入口还需等双臂 Home 到位，才允许开始下一段。所有 episode 完成且当前没有正在录制的数据时，
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
深度至少 18 Hz、两路腕部 RGB 至少 25 Hz；全部数据流连续 3 秒且接收帧间隔不超过
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

### 2026-10-14：启动附近的时间戳异常与修复

当天 episode0 的原始 bag 为北京时间 10:51:15–10:53:03。cam1 在录制约
6.95 秒时，源时间戳回退约 27.25 秒，后续还多次发生跳变；其他三路图像源时间戳
未发生同类异常。cam1 当时仅有一个发布者，使用 `time_domain: global`。
相机在录制前已经运行约 40 秒，不能把这种反复跳时当作正常预热。
可能与设备时间到 SDK 全局时间的转换有关，SDK／固件具体原因仍需硬件诊断。

已修复 ROS Humble 下预警升级为错误时的日志调用异常：不同级别使用独立调用行，
并在诊断日志之前调度自动收尾。日志异常不会再阻止自动停止。时间戳错误现在分别
显示缺失、重复或倒退量，报告保存错误前后的源时间戳。

触觉版的启动就绪检查现在同时要求图像接收和源时间戳连续稳定 3 秒。
时间戳无效、重复、倒退或大间隔会重新计时；这不是录制后忽略前 3 秒。
正常就绪后若相机继续跳时，仍会停止并报错。普通版启动检查保持原样。
不使用 `system` 接收时间替代采样时间来隐藏故障，已有错误 bag 不修改。

加载修复：确认没有正在录制或校验后，Ctrl-C 退出终端 3，再按原触觉命令启动。
本次无需重启终端 1／2。若仍报告 cam1 时间戳倒退，请保留新报告，暂停采集，
再检查该相机的连接、SDK 时间同步和固件；不要放宽缺帧阈值继续收集训练数据。

### 2026-10-14：cam1 全局时钟配置修复

cam1 使用独立的 `camera_cam1_global_params.yaml`：保留采样时间的
`time_domain: global`，将 `enable_sync_host_time` 改为 `false`，符合
[Orbbec 官方时间同步参数说明](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/4_application_guide/launch_parameters.html)。
cam0 与 cam2 保留原参数文件，分辨率、频率、序列号和左右语义不变。
部署配置 `data_collection/config/cameras.yaml` 与示例均已为 cam1 选择该独立配置。
相机参数包已重新安装，后续原触觉录制命令自动使用修复配置。

cam1 的轻量 CSV 诊断默认启用，保存 SDK 帧序号、设备时间、全局时间、
SDK 接收时间与发布时间，不保存额外图像副本。每次启动使用唯一文件名，位于
`$TELEOP_DATA_ROOT/diagnostics/camera_timestamps/`，并保存本次有效参数 YAML，
不会覆盖旧诊断。容器目录为 `/collection_data/diagnostics/camera_timestamps`；
可在相机部署配置的 `camera_bringup.timestamp_diagnostics_dir` 中指定其他目录。

这是针对异常时间映射的配置修复，不放宽缺帧阈值，也不使用主机接收时间伪装采样时间。
SDK 和相机固件未升级，已有错误 bag 未修改。实机短时验证不能替代后续满负载录制验证。

实机验证补充：关闭主机同步、保留 SDK 2.8.6 后，cam1 在三分钟测试内仍发生
设备／传感器时间回退；临时使用官方 SDK 2.10.6 也复现。帧序号与主机接收时间
连续。新版 SDK 未部署到正式流程，固件未修改，因此目前不能标记为故障已修复。
/sys 与相机日志确认：cam1（CV2756100018）协商为 USB 2.1 / 480 Mbps，
cam2（CV2L761000F8）为 USB 3.2 / 5000 Mbps。下一步先更换 cam1 的 USB 3 数据线
并直连 USB 3 端口，再重复验证，判断异常是否跟随线缆、端口或设备本体。
这是一项待验证的硬件差异，并非已证明的根因。
两轮 CSV、参数与诊断汇总保存在 `$TELEOP_DATA_ROOT/diagnostics/camera_timestamps/`。
复验工具：`ops/diagnostics/validate_camera_source_clocks.py`，只订阅相机，
不启动相机、不发送机器人命令；需先启动相机并通过就绪检查。

### USB 3 复测与已验证的 cam1 修复方案

cam1 已直连 USB 3，/sys 协商速度为 5000 Mbps。但保持自动选择的 MJPG 时仍复现
时间戳异常；官方 SDK 2.10.6 的 `ob_timestamp_tracker` 直接采集（绕过 ROS 驱动）
也报告 cam1 时间戳异常，设备重启后 MJPG 仍失败，因此 USB 3、SDK 升级或单次
重启并不足以修复。

已将 cam1 独立配置的 `color_format` 从 `ANY`（实机选择 MJPG）改为显式 `YUYV`，
保持 640×480、30 Hz、`time_domain: global` 和 `enable_sync_host_time: false`。
要求保留本次 USB 3 连接。这是绕开该相机已复现的 MJPG 采集／元数据异常路径；
未证明是纯硬件损坏，也不伪造采样时间。ROS 图像 topic 和录制接口不变，cam2
继续使用原自动格式配置。

实机结果：cam1 的 YUYV 流连续观察 180 秒，约 5400 帧，零时间戳错误与严重缺口；
随后四路相机实际 rosbag 写盘 90 秒，全部通过 source 连续性与内容校验。cam1
录到 2700 帧、30.02 Hz，最大相邻源时间间隔 33.36 ms，重复／倒退为零。
第一次 180 秒观察中 cam0 出现过独立的长暂停；随后写盘复验 cam0 正常。
这不能证明所有设备已完成长期满负载验收，后续仍需观察 cam0 和臂手同时运行情况。

相机参数包通过源码链接安装，下次原录制命令自动加载 YUYV，无需额外启动参数。
正式 SDK 仍为 2.8.6，固件未刷写，缺帧阈值未放宽。原始测试记录和写盘校验报告
保存于 `$TELEOP_DATA_ROOT/diagnostics/camera_timestamps/`，不计入训练 episode。

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

### 上传 D 类与设置默认值

右栏增加“同时上传 D 类（放弃）”，默认关闭。与“自动上传”同时开启时，
新归档的 D 类进入上传队列并保存到远端 D；C 类仍不上传。启用 D 上传后，
自动删除 D 必须等待上传成功；开启“上传后读回校验”时还须校验通过；总上传开关关闭或上传失败则保留本地。
不启用 D 上传时，自动删除仍可直接删除新归档的 D。不会补传或删除启动前已有的数据。

右栏本地父目录、远端父目录、自动上传、上传 D 类、上传后读回校验和自动删除均自动保存为新默认值，
下次主 GUI 或独立设置窗口启动时直接恢复，不需要手动点击保存。
设置文件为 `~/.config/gello-teleop/collection-transfer.json`。
新录制暂存目录为可见的 `<父目录>/pending/episodeN`；质量保存后移入 A/B/C/D。

### 上传与读回校验的并行及开关

右栏“上传后读回校验”默认开启、自动保存。开启时，上一段读回校验可以与下一段
上传并行，全局各最多一个；全部校验成功后才允许自动删除本地 bag。关闭后，
文件复制和大小检查成功即可完成远端归档，不再读回整段内容；按当前用户选择，
若同时开启自动删除，上传成功仍删除本地 A/B 及选择上传的 D。传输失败保留本地。
每次开始上传时采用当时的开关设置，当前任务按原设置收尾。


## 臂不可用时保留界面与任务管理

统一入口 `./ops/run/run_gello_full_teleop.sh --hand-source manus --hardware`
在收不到新鲜双臂状态时保持 GUI 与控制终端运行，显示 `ARMS UNAVAILABLE`。
臂使能、Hold、Home、Ready、姿态记录和预设动作不可用，原运动请求清除；
独立 Manus／Sharpa 手服务、采集器与上传后台生命周期保持独立。
该状态不能单独区分断网、Prog 模式和未开启 FCI，请在机器人侧确认原因。
修复连接、切换到 Exec 并开启 FCI 后，若臂硬件服务已退出，可从 GUI 的
`Connection → 重新连接机械臂` 显式重试。该操作只启动已退出的 `franka-control`，
不会停止运行中的控制器或重启手服务。收到新鲜状态后仍需手动使能，不自动续动。
端口冲突、部署配置错误或其他进程崩溃仍保留原退出处理。

Home 左侧任务下拉框末尾新增“＋ 新增任务…”和“－ 删除当前任务…”。
任务列表保存在 `<TELEOP_DATA_ROOT>/operator_gui/tasks.json`，重启后保留。
每个新增任务获得独立 ID，不沿用其他任务的 Home；须先分别记录左右臂 Home。
姿态沿用 `<TELEOP_DATA_ROOT>/operator_gui/poses.yaml` 中按任务分别存储的结构，
绝对轨迹仍按任务 ID 保存，Shared Ready 仍共用。
新任务服务在约 1 秒内注册；若立即操作提示正在初始化，请稍后重试。
录制、Home 未完成或双臂仍有使能／Hold 时不能增删任务；至少保留一个任务。
删除需确认：同时永久移除任务列表项、该任务左右臂 Home 和对应的
`arm_ui/absolute_trajectories/<任务 ID>.yaml`，不保留用于恢复的副本。
其他任务、共用 Ready 和已录制 episode 不受影响。重新新增同名任务会获得新 ID，
需要重新记录 Home 和轨迹。

任务永久删除使用共享文件锁；记录永久删除的任务 ID，禁止旧 Home 缓存或延迟保存的
轨迹重新写回已删除内容。若清理失败，任务先停用，界面提示失败，可再次删除重试。
容器创建的数据可能由 root 持有，GUI 会在需要时调用现有 tools 容器执行元数据清理，
该工具只编辑任务文件，不初始化 ROS 节点、相机或机器人。

任务文件操作在后台执行，保持 GUI 状态刷新和手服务心跳；处理完成前暂时禁止新录制和臂运动操作。
升级后需重启统一入口，使 Home 服务和界面同时加载永久删除及防回写逻辑。

## cam0 固定 IP 直连与固件（2026-10-23）

本机 cam0（CP4N651006F）已升级至官方固件 1.3.19，SDK 保持 2.8.6。
`data_collection/config/cameras.yaml` 的 cam0 部署项配置为：

```yaml
net_device_ip: 192.168.1.123
net_device_port: 8090
```

有上述地址时，启动器关闭网络设备自动枚举，直接连接该 IP/端口；不配置地址时
仍保留原有 cam0 自动枚举行为。相机和电脑的 IP 地址本身没有被修改。
cam1/cam2 继续按原来的 USB 配置启动。

该设置需重新启动独立采集终端后加载。固件更新已重启 cam0，但不会自动启动新
的训练 episode。固定 IP 模式下，驱动日志的设备 UID 应为 `192.168.1.123:8090`。
更换相机或改变网络地址后，应同步更新部署项并核对实际设备序列号。

升级使用的官方 OTA 文件 SHA-256 为
`b937427ee2487dbafe2e5525944ae94268b3e9566f0ef91d3d546d9c6ba4bb23`。
原 1.3.02 固件会被 SDK 提示不支持 CCP（要求至少 1.3.12），因此之前仅改 `ca`
参数不能作为控制访问机制已生效的证明。新固件与直连的短时验证不能替代正常
满负载长录制；1 秒断流及严重源时间异常仍按原规则停止本段。
