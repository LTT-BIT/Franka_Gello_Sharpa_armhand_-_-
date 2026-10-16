# GELLO Upper Body Teleop

本项目用于双 GELLO 增量遥操双 Franka FR3，并通过 MANUS 手套控制双侧
灵巧手。默认且保留的手部方案是双 O30i；Wuji、手动网页 UI 和 MoveIt
均为独立的可选模式，不会替换原有 O30i 控制链。

默认数据链路：

```text
双 GELLO -> 增量关节映射 -> UDP -> ROS 安全网关 -> 双 FR3
双 MANUS -> 手部重定向 -> LinkerHand 安全桥接 ------> 双 O30i
```

默认模式不启动相机或数据记录。

## 代码结构

```text
gello_upper_body_teleop/
├── teleop_runtime/                 统一 Operator 后端
│   └── cli.py                      组合机械臂、手、任务策略和控制 GUI 协议
├── adapters/                       输入设备与灵巧手适配层
│   ├── gello/                      GELLO 稳定入口
│   ├── pico/                       PICO 输入、共享遥操核心、仿真与测试
│   ├── vive/                       VIVE Tracker 输入、标定与测试
│   ├── manus/                      MANUS bridge、手部重定向、标定和工具
│   └── wuji/                       Wuji 重定向及 Wuji/Wuji Hand 2 后端
├── apps/                           面向操作员的交互应用
│   ├── operator_gui/               桌面 Operator GUI
│   └── hand_ui/                    O30i/G20 手动网页 UI
├── tasks/                          特殊任务，不改变默认遥操行为
│   └── powderweighing/             称粉手势策略、姿态配置和测试
├── config/                         仓库级配置
│   ├── workcell/                   FR3 地址与工作站拓扑
│   ├── modes/                      GELLO/PICO/VIVE/安全网关运行参数
│   └── calibration/                操作员和硬件标定结果
├── data_collection/                Harvest bag/LeRobot 采集契约与配置
│   └── docs/DATA_FORMAT.md         54/108 维顺序与 OpenPI 差异
├── ros_ws/src/                     ROS 2 workspace 源码
│   ├── teleop_interfaces/          ArmCommand 等 ROS 消息契约
│   ├── teleop_core/                FR3 安全网关与统一 topic/joint contract
│   ├── pico_teleop_bridge/         UDP 输入到 ArmCommand 的 ROS bridge
│   ├── franka_fr3_arm_controllers/ 双 FR3 控制器和 Home 服务
│   ├── linker_hand_bridge/         手部命令校验、watchdog 与 slew limiter
│   ├── linker_hand_ros2_sdk/       O30i/G20 ROS 硬件节点
│   ├── lychee_fr3_description/     双 FR3 描述、网格和 RViz 配置
│   ├── lychee_fr3_moveit_config/   双 FR3 MoveIt 配置
│   ├── teleop_data/                数据记录、回放与 LeRobot 导出
│   ├── teleop_data_collector/      Harvest ROS bag 录制与手动 LeRobot 转换
│   ├── teleop_hand_telemetry/      只读 Wuji UDP→ROS telemetry
│   └── teleop_camera_bringup/      三相机 SN bringup（EULA 门禁）
├── ops/                            日常运维入口
│   ├── run/                        启动遥操、Wuji、UI、MoveIt 和 preflight
│   ├── setup/                      环境、Docker/ROS、GELLO 驱动安装
│   └── diagnostics/                GELLO、O30i 和数据诊断工具
├── vendor/                         跨模块复用的第三方 SDK
│   ├── manus_sdk/
│   └── xrobotoolkit_sdk/
├── assets/                         LinkerHand 模型及重定向资源
├── docker/                         镜像、Compose 服务和本机环境模板
└── docs/                           部署、硬件、MoveIt 和交接文档
```

主要连接关系：

```text
ops/run ──> teleop_runtime + apps/operator_gui
                  │
                  ├──> adapters/gello|pico|vive ──UDP──> pico_teleop_bridge
                  ├──> adapters/manus ────────────────> linker_hand_bridge
                  ├──> adapters/wuji（仅显式选择 Wuji 时加载）
                  └──> tasks/powderweighing（仅传入任务配置时加载）

pico_teleop_bridge ──ArmCommand──> teleop_core 安全网关 ──> 双 FR3
linker_hand_bridge ──校验/限速后的命令──────────────────> O30i/G20
Wuji Operator ──只读 UDP 5602──> teleop_hand_telemetry ──> Harvest recorder
```

`apps` 只保存 UI，`tasks` 只保存特殊任务，硬件差异统一封装在 `adapters`。
所有机械臂来源最终都使用 `teleop_interfaces/ArmCommand`；只有
`teleop_core` 安全网关可以写入 FR3 命令总线。`ros_ws/build`、
`ros_ws/install`、`ros_ws/log` 以及各 Python `__pycache__` 都是生成物，不属于
源码结构，也不应提交到 Git。

## 功能与启动入口

| 模式 | 机械臂 | 灵巧手 | 启动命令 |
| --- | --- | --- | --- |
| 标准遥操（默认） | 双 GELLO -> 双 FR3 | MANUS -> 双 O30i | `./ops/run/start_teleop.sh` |
| 仅机械臂 | 双 GELLO -> 双 FR3 | 不启动 | `./ops/run/run_gello_arms_only.sh` |
| 手动网页 UI | 不启动 | 网页 UI -> O30i/G20 安全桥接 | `./ops/run/run_hand_ui.sh` |
| Wuji 遥操 | 双 GELLO -> 双 FR3 | MANUS -> Wuji | `./ops/run/start_wuji_teleop.sh ...` |
| MoveIt 假硬件 | MoveIt -> 双虚拟 FR3 | 不启动 | `./ops/run/run_moveit.sh --fake` |
| MoveIt 真机 | MoveIt -> 双 FR3 | 不启动 | `./ops/run/run_moveit.sh --real` |

## 安全约束

- GELLO/Operator 遥操模式启动后应保持双侧未 Engage，先测试一侧的小幅运动，
  再测试双侧。
- MoveIt 真机模式应先确认关节状态和规划结果，只执行小范围测试轨迹。
- GELLO 遥操与 MoveIt 不能同时控制 FR3；启动脚本会检查并拒绝冲突服务。
- Wuji 模式不会启动 `hand-control`，且会拒绝与 O30i/G20 服务同时运行。
- 手动网页 UI 通过 LinkerHand bridge 的范围校验、命令新鲜度 watchdog 和
  slew limiter，不直接写入厂商命令总线。
- 手动网页 UI 与正常 Operator 后端不能同时作为手部命令源；启动脚本会检查
  Operator 控制端口。
- 真机启动前确保工作区无人、急停可触及、Franka FCI 已解锁。

## 首次安装

```bash
cd /home/descfly/llx/gello_upper_body_teleop
cp docker/.env.example docker/.env
./ops/setup/build.sh
./ops/setup/setup_teleop_env.sh
adapters/manus/scripts/build.sh
GELLO_SOFTWARE_ROOT=/home/descfly/llx/gello_software \
  ./ops/setup/setup_gello_driver.sh
```

当前机器使用 Conda 环境 `gello-upper-body-teleop`。GELLO 使用稳定的 USB
序列号识别左右侧；更换硬件后必须更新身份并重新验证左右侧和关节方向。

如需使用 Wuji，再安装其可选依赖：

```bash
./ops/setup/setup_wuji_env.sh
```

新工控机完成安装后，运行不会连接或 enable 硬件的一键环境验收：

```bash
./ops/diagnostics/check_wuji_gello_environment.sh
```

迁移顺序、硬件存在性和左右 Wuji 网络检查参数见
[docs/NEW_COMPUTER_SETUP.md](docs/NEW_COMPUTER_SETUP.md)。

### Git 克隆范围与本机依赖

GitHub 保存源码、配置模板、文档和 Git 提交历史，但以下 `.gitignore` 排除的
本机内容不会随 `git clone` 下载：

- `portable_deps/gello_software/`：GELLO 驱动及 DynamixelSDK；
- `portable_deps/litchi_hardware/`：Sharpa/MANUS 源码及供应商运行库；
- `docker/.env`：本机路径、端口和许可确认等环境配置；
- `data_collection/config/cameras.yaml`：本机三相机序列号与物理位置配置；
- `ros_ws/build/`、`ros_ws/install/`、`ros_ws/log/`：ROS 构建产物；
- Python/测试缓存、egg-info 和其他可重新生成的编译产物。

因此，在另一台电脑上仅克隆 GitHub 仓库并不等于获得完整的可运行迁移包。
`install_environment.sh` 会检查两个 `portable_deps` 子目录；缺少任一目录时会停止，
不会在运行项目时自动下载。`gello_software` 可以从公开上游重新获取，但
`litchi_hardware` 中包含受供应商许可证约束的 Sharpa/MANUS 文件，应通过 U 盘、
局域网或有权限的私有存储从已授权电脑单独复制，不要发布到公开仓库。

`portable_deps/litchi_hardware` 中还有超过 GitHub 普通 Git 单文件 100 MB 限制的
MANUS SDK 动态库。Git LFS 可以解决文件大小限制，但不能替代供应商授权；未确认
许可证允许前，不应上传。完整工程迁移方式见
[工程迁移与一键安装说明](工程迁移与一键安装说明.md)。

## 启动前检查

标准模式和 Wuji 模式会自动运行完整 preflight（含 GELLO 端口和 MANUS 标定）。
MoveIt 真机模式只跑机械臂基础设施检查，不要求 GELLO 或 MANUS。也可以单独执行：

```bash
./ops/run/preflight.sh
```

仅检查机械臂环境（仍包含 GELLO 端口，跳过 MANUS）：

```bash
./ops/run/preflight.sh --arms-only
```

MoveIt 真机等价检查：

```bash
./ops/run/preflight.sh --arms-only --skip-gello
```

完整 preflight 检查 Docker、Compose、ROS workspace、Conda、GUI、GELLO 端口以及
MANUS 标定文件；检查过程不会发送机器人、灵巧手或 GELLO 电机命令。

## 标准遥操：GELLO + O30i

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./ops/run/start_teleop.sh
```

这是推荐且默认的入口。脚本依次启动：

```text
franka-control + teleop-control + moveit-ik + preset-ik + gello-bridge + hand-control
                                     |
                                     +-> Operator 后端和 GUI
```

当前 `hand-control` 的左右手型号均为 `o30i`。关闭 GUI、Operator 后端退出或
按 `Ctrl-C` 后，脚本会停止它启动的机器人侧服务。

### 两个终端启动

需要持续观察 ROS 和 Franka 日志时，可分开启动。

终端 1：

```bash
./ops/run/run_robot_stack.sh
```

终端 2：

```bash
./ops/run/run_operator.sh
```

确认 `franka-control` 已接受碰撞阈值、`teleop-control` 显示 contact torque
gating active，且 GUI 状态正常后再 Engage。

Operator GUI 支持5个键盘式脚踏输入，机械臂和手独立启停：

| 按键 | 功能 | 按键 | 功能 |
|---|---|---|---|
| `L` | 左臂启动/停止 | `A` | 右臂启动/停止 |
| `R` | 左手启动/停止 | `B` | 右手启动/停止 |
| `Space` | 双臂同时 Home |  |  |

Home 是单次操作，执行前会解除控制权；`DISENGAGE ALL` 仍可同时停止所有臂/手
跟随。Home 下拉框提供 `粉末称量`、`装配`、`夹豆` 三个任务，Space 和界面按钮
都使用当前选择的任务。脚踏模拟普通键盘，使用时 Operator GUI 窗口需要获得
键盘焦点。

普通 MANUS 统一入口现在采用与独立手部入口一致的转发策略：原始驱动命令超时为 `0`，
不再叠加 150 ms 求解输出时限、200 ms 双级命令锁停、跳变锁停、逐时间戳关键点匹配或 30 s 待机获取超时。
原来的 250 ms 输入检查与 HAL 的关节限位/非法指令拒绝仍保留，目标订阅只保留最新一条。
`Start hand` 仍是每侧显式许可；短暂缺帧/无效目标不清除许可，真实 GUI 断开、Stop 或关闭会撤销许可。
源数据恢复后只有仍持有许可的手继续跟随；真实断连后重新连接不会自动恢复动作。
这项策略只用于普通 MANUS（normal/fake），独立入口、LitchiBot 与监督验证沿用各自路径。

统一 Manus 入口中的 `Open hand` / `Open both hands` 是显式的一次性动作：
先关闭所选手的遥操使能，再从实测姿态平滑张开至 Sharpa 默认的 22 关节全零角度。
张开期间 Start hand 暂不可用；完成后保持停止跟随，必须再次点击 Start hand 才恢复。
单手操作不改变另一只手的使能，双手操作关闭两侧使能。GUI 断线、故障锁存、无效反馈
或动作超时会停止动作；不绕过已有安全检查。未点击 Open 时不执行这条动作路径。

左右臂面板中的 `Record arm as Home` 只把同侧机械臂当前实测的 7 个关节角
保存为当前下拉任务的 HomePose。记录前必须停止对应机械臂；手部状态不影响记录，
记录本身不会驱动硬件。Home 只移动机械臂，不记录、读取或回放手部 Home。
首次创建持久化配置时只有 `粉末称量` 的机械臂继承旧 Home；另两个任务必须实际记录。

`Record both arms as Ready` 一次记录共享 Ready，`Move both arms to Ready` 将双臂
平滑移动到该点。Arm UI 的“双臂绝对 Ready → Home 轨迹”面板可按任务同步录制
左右各 7 个关节角。Operator GUI 的 `Ready to Home` 回放当前任务文件；开始前会
检查真实双臂和轨迹首点都在 Ready 的 0.05 rad 内，并检查轨迹末点匹配当前任务
Home，否则拒绝执行。Ready-to-Home 只回放双臂轨迹。
机械臂位姿持久化在 `${TELEOP_DATA_ROOT}/operator_gui/poses.yaml`。

Operator GUI 还提供三个相对末端预设动作槽：`Q`、`W`、`E`。当前 `Q` 配置为
Arm UI 录制的左臂动作 `kuai1`，以 65% 速度执行；`W`、`E` 保留待配置。执行前
系统以当前真实末端姿态作为新原点，并通过常驻的 `preset-ik` 服务走与 Arm UI
相同的 MoveIt KDL `/compute_ik` 链路检查完整轨迹；任一帧不可达、碰撞或发生
IK 跳变都不会执行。该服务绑在非 Franka 实时核上，避免每次按 Q 再起一次
容器。动作完成、点击 `STOP PRESET` 或 GELLO 移动超过 0.08 rad 后会重新锚定增量
映射。正常完成前还会确认末点误差进入 0.03 rad 并稳定 0.25 秒；末点 5 秒未
收敛则保持停止。单键 `Q` 与退出菜单的 `Ctrl+Q` 不冲突。

## 仅启动 GELLO 双臂

```bash
./ops/run/run_gello_arms_only.sh
```

该入口仅启动 `franka-control`、`teleop-control`、`moveit-ik`、`preset-ik` 和
`gello-bridge`，并以
`--hand-source none` 运行 Operator。它会先停止可能残留的 `hand-control`，
避免旧会话继续控制 O30i/G20。

## O30i/G20 手动网页 UI

```bash
./ops/run/run_hand_ui.sh
```

浏览器打开 <http://127.0.0.1:8080>。Compose 当前默认连接双 O30i；UI
也保留 G20 映射支持。该模式只启动 `hand-control` 和 `hand-ui`，不启动 FR3。

详细说明见 [apps/hand_ui/README.md](apps/hand_ui/README.md)。

## Wuji 手遥操

O30i 仍是标准入口的默认方案。使用 Wuji 时需走独立启动脚本，并显式提供
Wuji Hand 2 的左右地址，防止网络发现选错手：

```bash
./ops/run/start_wuji_teleop.sh \
  --wuji-left-address 192.168.1.111:50001 \
  --wuji-right-address 192.168.1.112:50001
```

Wuji 模式继续使用双 GELLO 控制 FR3，使用现有 MANUS bridge 和 Operator GUI
控制 Wuji 手。退出时会停止机械臂侧服务并尝试 disable、断开 Wuji 设备。
Wuji 模式下 Operator 默认向本机 UDP 5602 导出只读手 telemetry；未启动采集时
不影响控制。Harvest 格式采集是独立入口，见
[data_collection/README.md](data_collection/README.md)。

如果只测试 MANUS 到 Wuji 右手，不启动 Docker、GELLO 或 FR3，使用：

```bash
./ops/run/start_wuji_hand_only.sh \
  --wuji-sides right \
  --wuji-right-model wuji_hand_2 \
  --wuji-right-address 192.168.2.111:7447 \
  --wuji-kp 1.0 \
  --wuji-kd 0.1 \
  --wuji-current-limit 0.5
```

该入口默认右手且以 `DISENGAGED` 开始。按 `R` 或空格开始/暂停跟随，按 `X`
停止发送，按 `Q` 或 `Ctrl-C` 退出并 disable、断开手。它不会检查 GELLO 串口。

没有连接 FR3、但需要检查完整的 GELLO + Operator + MANUS + Wuji 集成链时，
在整体入口增加 `--fake-franka`。该模式以虚拟双 FR3 替代真机，其余路径不变：

```bash
./ops/run/start_wuji_teleop.sh --fake-franka \
  --wuji-sides right \
  --wuji-right-model wuji_hand_2 \
  --wuji-right-address 192.168.2.111:7447 \
  --wuji-kp 1.0 \
  --wuji-kd 0.1 \
  --wuji-current-limit 0.5
```

无需连接硬件即可测试正确的 `hand2_beta` 模型。右手默认重放
`r_pinch_2.pkl`，左手默认重放 `l_pinch_2.pkl`：

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right
```

原版 USB Wuji Hand、单侧运行和电流/控制增益参数见
[adapters/wuji/README.md](adapters/wuji/README.md)。首次真机验证应采用低电流、
单手、单侧 Engage。

## GELLO 双臂 + MANUS/Sharpa 双手录制（新增）

原有遥操入口和默认模式保持不变。需要把双 FR3、双 Sharpa 手以及三台 Orbbec
相机录入同一个 ROS 2 bag 时，使用下面的独立三终端流程。录制器只订阅 ROS topic，
不会向机器人发送命令，也不会建立第二条 libfranka/FCI 连接。

终端 1 启动原有 GELLO 双臂流程：

```bash
cd /home/descfly/ZH/gello_upper_body_teleop
./ops/run/run_gello_arms_only.sh
```

终端 2 使用隔离的 CycloneDDS 配置启动 MANUS/Sharpa 双手：

```bash
cd /home/descfly/ZH/gello_upper_body_teleop
./ops/run/run_sharpa_hands_cyclonedds.sh
```

终端 3 在确认双臂和双手正常跟随后启动普通录制：

```bash
cd /home/descfly/ZH/gello_upper_body_teleop
./ops/run/start_sharpa_arm_recording.sh --with-cameras
```

普通录制包含双臂 action/state、双手 command/state、三路 RGB 和头部深度，默认写入
`/home/descfly/franka_teleop_data/bags/gello_sharpa/episodeN/`。如需额外录入双手
五指的 F6 wrench 和 deformation（共 20 个触觉 topic），终端 3 改用：

```bash
./ops/run/start_sharpa_arm_recording_with_tactile.sh --with-cameras
```

普通版和触觉版互斥，不能同时启动。触觉版输出到
`/home/descfly/franka_teleop_data/bags/gello_sharpa_tactile/episodeN/`，并要求终端 2
已发布全部触觉 topic，不支持 `--without-hands`。原普通录制配置和命令没有改变。

只复验机械臂和相机、现场没有手套或 Sharpa 时，不启动终端 2，并使用：

```bash
./ops/run/start_sharpa_arm_recording.sh --without-hands --with-cameras
```

采集器显示 `READY` 后，通过 Operator GUI 或数据采集踏板的 `L` 键开始/停止
episode，录制中按 `Space` 添加里程碑。停止后等待 rosbag 完成收尾、完整性校验和
数据质量评价；不要用 `Ctrl-C` 停止单个 episode。结束全部录制且当前没有活动
episode 时，再按 `Ctrl-C` 退出常驻采集器和相机。

不要在双臂遥操期间运行 `manus_sharpa_record/record.py`。该程序会直接创建新的
`pylibfranka.Robot` 连接，与已经占用 FCI 的 `franka_ros2_control_node` 冲突。
完整 topic、相机就绪条件、QoS、触觉验收阈值和 USB 踏板说明见
[Sharpa 双臂双手与三相机录制](data_collection/docs/SHARPA_ARM_RECORDING.md)。

## 双 FR3 MoveIt

先用假硬件验证规划和 RViz：

```bash
./ops/run/run_moveit.sh --fake
```

确认模型、规划组和控制器均正常后，才使用真机模式：

```bash
./ops/run/run_moveit.sh --real
```

两种模式都会同时启动 FR3 点位示教与轨迹编排 UI，浏览器打开
<http://127.0.0.1:8081>。假硬件可验证点位编辑、任务 YAML 和轨迹执行；真机
模式还可在单臂轨迹控制器与零力矩拖动控制器之间切换。进入拖动模式前必须托住
机械臂，并确认负载、质心和硬件停止手段。

当前 Compose 中的真机地址与 `config/workcell/current.yaml` 一致：左侧
`172.16.0.3`、右侧 `172.16.0.2`。修改工作站地址时需要同步更新两处。MoveIt
与正常 GELLO 安全网关是两套独立的机械臂控制模式，不能同时运行。

详细说明见 [docs/MOVEIT.md](docs/MOVEIT.md)。

## 特殊任务

特殊任务放在 `tasks/` 下，不属于通用遥操控制链。当前的称粉任务位于
`tasks/powderweighing`，只在显式传入任务配置时包装右手 MANUS/O30i
重定向策略：

```bash
./ops/run/run_operator.sh \
  --right-hand-strategy-config tasks/powderweighing/config/poses.json
```

不传该参数时，默认双 O30i/Sharpa 行为不变。标定、独立调试和触发参数见
[tasks/powderweighing/README.md](tasks/powderweighing/README.md)。

## 停止与故障日志

正常停止顺序：

1. 在 GUI 中执行 `DISENGAGE ALL`。
2. 关闭 GUI 或在启动终端按 `Ctrl-C`。
3. 两终端模式下，两个终端都按 `Ctrl-C`。
4. 确认 Compose 服务停止后关闭 Franka FCI。

发生 reflex、碰撞保护或其他异常时，在停止 Compose 前保存日志：

```bash
cd docker
docker compose logs franka-control teleop-control \
  > /tmp/franka_teleop_fault.log
```

查看当前服务：

```bash
cd docker
docker compose ps
```

## 关键配置

- `config/modes/gello.yaml`：左右 GELLO 身份、方向、分侧逐关节增量范围、
  每端 `1%` 的绝对软限位余量和 `0.8 rad/s` GELLO 映射限速；安全网关仍以
  `0.7 rad/s` 作为最终输出上限。
- `config/workcell/current.yaml`：双 FR3 地址、arm ID 和命名空间。
- `config/modes/teleop_control.yaml`：ROS 安全网关、限速和接触力矩 gating。
- `config/modes/pico.yaml`：共享 UDP 与控制周期配置；tracker 兼容入口仍保留。
- `docker/compose.yaml`：O30i、MoveIt、ROS 服务及硬件容器配置。

## 更多文档

- [硬件部署与故障处理](docs/HARDWARE_DEPLOY.md)
- [GELLO 控制语义](docs/GELLO_TELEOP.md)
- [MoveIt 模式](docs/MOVEIT.md)
- [FR3 相对末端动作示教与遥操接入计划](docs/RELATIVE_ACTION_TEACHING.md)
- [手动灵巧手 UI](apps/hand_ui/README.md)
- [Wuji 手集成](adapters/wuji/README.md)
- [Harvest 数据采集](data_collection/README.md)
- [Sharpa 双臂双手与三相机录制](data_collection/docs/SHARPA_ARM_RECORDING.md)
- [Powder weighing 特殊任务](tasks/powderweighing/README.md)
- [第三方代码与许可证](THIRD_PARTY_NOTICES.md)
