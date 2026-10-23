# GELLO + Sharpa 双臂双手遥操作

本工程用于双 GELLO 增量遥操双 Franka FR3，并通过 MANUS 或 LitchiBot 手套控制
双侧 Sharpa 灵巧手。当前分支 `common_entrance` 使用统一 Operator GUI，支持
臂手独立启停、任务 Home / Ready、动作回放，以及独立的三相机与触觉数据采集。

**日常入口：`./ops/run/run_gello_full_teleop.sh --hand-source manus --hardware`。**
LitchiBot 使用同一个入口切换手套来源。启动遥操不会自动启动相机或录制器。

```text
双 GELLO ──> 增量关节映射 ──UDP──> ArmCommand ──> ROS 安全网关 ──> 双 FR3
MANUS / LitchiBot ──> 手部解算与重定向 ──> Sharpa 手部控制链 ──> 双 Sharpa
                                      ↑
                              统一 Operator GUI

FR3 action/state + Sharpa command/state/tactile + 三台 Orbbec
                         └──> 独立 ROS 2 bag 采集器
```

完整操作步骤见 [完整遥操作启动指南](docs/litchibot/完整遥操作启动指南.md)，
采集细节见 [Sharpa 双臂双手与三相机录制](data_collection/docs/SHARPA_ARM_RECORDING.md)。

## 首次部署

新电脑请复制包含 `portable_deps/` 的完整工程，再按
[工程迁移与一键安装说明](工程迁移与一键安装说明.md) 安装：

```bash
cd /path/to/gello_upper_body_teleop
./一键安装环境.sh
```

安装器负责系统依赖、Docker、Conda/Pixi 环境和 ROS workspace 构建；安装过程不启动
机器人、手或相机。已有正常运行的电脑无需重新执行安装器。

Git 只保存源码、配置模板和文档，以下本机内容不会随 `git clone` 下载：

- `portable_deps/gello_software/`：GELLO 驱动及 DynamixelSDK；
- `portable_deps/litchi_hardware/`：Sharpa/MANUS 源码及供应商运行库；
- `docker/.env`：本机路径、端口、CPU 布局及许可配置；
- `data_collection/config/cameras.yaml`：本机相机序列号及物理位置；
- ROS `build/install/log`、Python 缓存及其他编译产物。

安装器缺少两个 `portable_deps` 子目录中的任一个都会停止。供应商依赖应从已授权
电脑单独复制，不应提交公开仓库。历史录制数据位于 `TELEOP_DATA_ROOT`，需另行迁移。

统一入口和触觉采集要求至少 8 MiB 的 DDS UDP 接收缓冲。每台部署电脑首次配置：

```bash
sudo bash ops/setup/install_dds_buffers.sh
python3 teleop_runtime/dds_buffers.py
```

硬件启动前核对 FR3 地址与 FCI、实时 CPU/IRQ 布局、左右 GELLO 身份、手套标定及
Sharpa 左右设备绑定；具体配置见下文。MANUS 路径还需要运行匹配的 MANUS Core。

## 日常启动：统一臂手入口

以下命令均在工程根目录执行。先正常退出旧遥操会话，避免重复控制源或驱动。

### 一次性 Sharpa 设备绑定

首次使用 normal hardware 前创建部署配置；已有文件时不要覆盖：

```bash
cp -n config/litchibot/hand_devices.example.json config/litchibot/hand_devices.json
nano config/litchibot/hand_devices.json
```

将 `serials.left` 和 `serials.right` 填为实际左右 Sharpa 设备序列号。两者必须不同，
运行时会核验 SDK 的 serial 和 hand side。该文件供统一入口使用，MANUS 和
LitchiBot 的 normal hardware 都需要这项绑定。

### MANUS → Sharpa（常用）

终端 A：

```bash
./ops/run/run_gello_full_teleop.sh --hand-source manus --hardware
```

统一入口管理双臂服务、MANUS/Sharpa 手部服务和 Operator GUI，无需另开 hands 终端。
启动后双臂、双手的运动授权均关闭；在 GUI 中分别点击对应侧的 `Start arm` /
`Start hand`，先验证单侧小幅运动，再进行双侧操作。

普通 MANUS 使用独立 forwarding 策略。短暂缺帧或无效目标不会清除已有手部许可；
输入恢复后仍持有许可的手可以继续跟随。`Stop hand`、真实 GUI 断连或退出会撤销
许可，重新连接后需要再次手动启用。输入新鲜度检查、关节限位和非法指令拒绝仍保留。

MANUS 模式的 `Open hand` / `Open both hands` 会先停止所选手跟随，再从实测姿态
平滑张开至 Sharpa 默认的 22 关节全零角度。完成后保持停止，需再次 `Start hand`。

### LitchiBot → Sharpa

关闭原厂手套上位机以释放 USB 接收器，开启左右手套采集盒，使用已标定 profile：

```bash
./ops/run/run_gello_full_teleop.sh \
  --hand-source litchibot \
  --profile LYG226360006 \
  --hardware
```

normal hardware 由 GUI 分侧授权，不要求 supervised validation permit 或按住 F12。
运行时仍执行该路径的输入、关节、连续性及故障检查。临时问题可能显示 `PAUSED`，
保留许可并在恢复后继续；`HARD_FAULT` 需先处理原因，不能通过 Start 或重连覆盖。
LitchiBot full 模式当前不支持 `Open hand`。

首次实体验收的 supervised 模式及其 permit / F12 要求见
[完整遥操作启动指南](docs/litchibot/完整遥操作启动指南.md)。

### 软件调试与 no-send

不指定运行模式时，统一入口默认 fake Sharpa / no-send；连接实体手需显式传入
`--hardware`。可用以下命令调试：

```bash
# 真实 MANUS 输入，fake Sharpa
./ops/run/run_gello_full_teleop.sh --hand-source manus --dry-run

# 模拟 MANUS 输入，fake Sharpa
./ops/run/run_gello_full_teleop.sh --hand-source manus --dry-run --fake-input

# 真实 LitchiBot 输入，fake Sharpa
./ops/run/run_gello_full_teleop.sh \
  --hand-source litchibot --profile LYG226360006 --dry-run
```

**`--dry-run` 只关闭实体 Sharpa 发送，双臂仍走原生产启动流程。**
`--fake-input` 也不会把 FR3 切换为仿真。仅验证手部时保持机械臂未授权。

## Operator GUI

臂、手及采集分别控制。停止手部跟随不会关闭后台手套读取、设备连接或 state/tactile
发布；采集器与上传后台也独立于臂手使能和生命周期。

| 操作 | 行为 |
| --- | --- |
| `Start arm` / 停止对应臂 | 分侧启用或停止 GELLO 跟随 |
| `Start hand` / `Stop hand` | 分侧授权或停止 Sharpa 跟随 |
| `DISENGAGE ALL` | 同时撤销双臂、双手授权 |
| `A` / `C` | 左臂 / 右臂 Hold 切换 |
| `B` | 双臂 Home |
| `L` | 开始 / 停止当前录制 episode |
| `Space` | 录制中添加里程碑 |
| `Q` / `W` / `E` | 已配置的相对末端动作槽 |

快捷键和键盘式踏板需要 GUI 窗口获得焦点。Hold 只暂停机械臂跟随，不等于撤销
机械臂使能。Home、Ready 及预设动作会驱动机械臂，执行前确认目标和工作区。

任务下拉框支持新增、删除任务；各任务独立保存左右臂 Home 和绝对轨迹，Ready 共用。
`Record arm as Home` 只记录对应臂的实测关节角，不移动硬件；记录前先停止该臂。
`Ready to Home` 执行前会核对双臂起点、轨迹首尾和当前任务 Home。
任务与姿态保存在 `${TELEOP_DATA_ROOT}/operator_gui/`。

若显示 `ARMS UNAVAILABLE`，GUI、独立手服务和采集/上传后台仍可保留，臂运动操作
不可用。确认机器人已切换 Exec 并开启 FCI 后，可通过
`Connection → 重新连接机械臂` 重试已退出的臂硬件服务；状态恢复后仍需手动使能。

## 独立录制：双臂、双手、触觉与三相机

终端 A 的遥操稳定后，在终端 B 启动采集器。统一遥操入口不负责启动或关闭
recorder/cameras，两种录制入口只能选择一个运行。

```bash
# 双臂 + 双手 + 三路 RGB / 头部深度 + 双手触觉
./ops/run/start_sharpa_arm_recording_with_tactile.sh --with-cameras

# 不录触觉
./ops/run/start_sharpa_arm_recording.sh --with-cameras
```

触觉版额外录制双手五指的 F6 wrench 和 deformation，共 20 路触觉 topic，并要求
全部触觉流已发布。省略 `--with-cameras` 可只录臂手；只有普通版支持
`--without-hands`，用于现场没有手套或 Sharpa 的臂/相机复验。

1. 等待相机与数据流就绪，采集器显示 `READY`。
2. 在 GUI 点击开始录制或踩 `L`，录制过程中用 `Space` 添加里程碑。
3. 再次点击停止或踩 `L`，等待 rosbag 收尾、完整性校验和质量评价。
4. 完成质量评价及统一入口要求的双臂 Home 后，再开始下一段。

不要用 `Ctrl-C` 停止单个 episode。整场采集结束且无活动 episode 后，再用
`Ctrl-C` 退出采集器和相机。使用 OrbbecViewer 前须先退出独立采集终端，避免争用相机。

默认录制组为 `gello_sharpa` / `gello_sharpa_tactile`；GUI 可配置本地与远端父目录，
新数据暂存于 `<父目录>/pending/episodeN`，质量保存后归档到 A/B/C/D。
自动上传、读回校验与自动删除由 GUI 采集设置控制，设置保存在
`~/.config/gello-teleop/collection-transfer.json`。

采集器只订阅 ROS topic，不创建额外 FCI 连接。**双臂遥操期间不要运行
`manus_sharpa_record/record.py`**，它会创建新的 `pylibfranka.Robot` 连接，与现有
FR3 控制器冲突。当前采集输出原始 ROS bag；LeRobot 转换是后续独立步骤。

完整 topic、QoS、频率验收、相机配置和数据归档规则见
[Sharpa 录制说明](data_collection/docs/SHARPA_ARM_RECORDING.md)。

## 独立调试与兼容入口

这些入口与 full launcher 不要同时运行。旧入口保留各自行为，并不自动获得统一
GUI 的 Sharpa 手部许可管理。

| 用途 | 命令 / 文档 |
| --- | --- |
| 仅 GELLO 双臂 | `./ops/run/run_gello_arms_only.sh` |
| 独立 MANUS → Sharpa | `./ops/run/run_sharpa_hands_cyclonedds.sh` |
| 独立 LitchiBot 手部 no-send | `./ops/run/run_sharpa_hands_cyclonedds.sh --hand-source litchibot --dry-run` |
| MoveIt 假硬件 / 真机 | `./ops/run/run_moveit.sh --fake` / `--real`；[说明](docs/MOVEIT.md) |
| 旧 MANUS → O30i | `./ops/run/start_teleop.sh`，会启动旧 `hand-control` |
| O30i/G20 手动网页 UI | `./ops/run/run_hand_ui.sh`；[说明](apps/hand_ui/README.md) |
| Wuji 可选模式 | [Wuji 集成](adapters/wuji/README.md) |

独立 MANUS/Sharpa 命令保留历史生产行为，会连接实体设备，不能当作 no-send。
`start_teleop.sh` 属于旧 LinkerHand 路径；本分支 Sharpa 日常流程使用前文的 full launcher。

## 安全与停止

- 所有机械臂输入最终发布 `teleop_interfaces/ArmCommand`；只有 `teleop_core`
  安全网关可以写入 FR3 命令总线，topic 和关节名以 `teleop_core.contract` 为准。
- 真机运动前确认工作区、急停、左右设备映射和 FCI 状态；不绕过新鲜度、关节限位、
  限速或故障检查。
- GELLO 遥操与 MoveIt 真机模式不能同时控制 FR3；不要重复启动 Sharpa driver
  或同时运行新旧手部控制源。
- 正常结束先停止并完成当前 episode 收尾，再在 GUI 执行 `DISENGAGE ALL`，关闭
  GUI 或在终端 A 按 `Ctrl-C`，等待 launcher 清理其启动的臂手服务。
- 采集终端独立退出。确认控制服务已停止后，再关闭 Franka FCI。

发生 reflex、碰撞保护或其他异常时，在停止服务前保存日志：

```bash
docker compose -f docker/compose.yaml logs franka-control teleop-control \
  > /tmp/franka_teleop_fault.log
docker compose -f docker/compose.yaml ps
```

## 代码与配置导航

| 目录 / 文件 | 用途 |
| --- | --- |
| `teleop_runtime/` | full launcher、Operator 后端、采集协调与生命周期管理 |
| `apps/operator_gui/` | 统一桌面 GUI |
| `adapters/gello/` | GELLO 输入入口 |
| `adapters/manus/` | MANUS bridge、标定、重定向与普通 Sharpa forwarding |
| `adapters/litchibot/` | LitchiBot 输入、Sharpa 集成与验收控制 |
| `adapters/pico/`、`adapters/vive/`、`adapters/wuji/` | 可选设备适配 |
| `ros_ws/src/` | 消息契约、安全网关、FR3 控制器、相机及数据工具 |
| `data_collection/` | 录制配置、QoS 与数据格式文档 |
| `ops/run/`、`ops/setup/`、`ops/diagnostics/` | 启动、安装与诊断脚本 |
| `config/modes/gello.yaml` | GELLO 左右身份、方向、映射与输入限速 |
| `config/modes/teleop_control.yaml` | 机械臂安全网关与接触力矩 gating |
| `config/workcell/current.yaml` | FR3 地址、arm ID、命名空间及控制 CPU |
| `config/litchibot/` | Sharpa 设备绑定、LitchiBot profile 与验收配置 |
| `docker/.env`、`docker/compose.yaml` | 部署路径、服务、端口、DDS domain 与 CPU 隔离 |
| `data_collection/config/cameras.yaml` | 三相机序列号、角色与参数文件 |
| `tasks/` | 显式启用的任务策略 |
| `portable_deps/`、`vendor/`、`assets/` | 本机迁移依赖、第三方 SDK 与模型资源 |

本机常用 Conda 环境为 `gello-upper-body-teleop`；Sharpa/MANUS ROS 链路使用
`LITCHI_REPO` 下的 Pixi 环境。`HOUSEKEEPING_CPUSET` 将手部 SDK 和解算放在
FR3 实时核之外。更换电脑时应重新核对 CPU/IRQ 布局，不直接照搬旧机配置。

## 更多文档

- [完整遥操作启动指南](docs/litchibot/完整遥操作启动指南.md)
- [统一入口集成报告](docs/litchibot/full_teleop.md)
- [普通 MANUS forwarding 说明](docs/diagnostics/manus_normal_forwarding_20261015/修改说明.md)
- [LitchiBot normal 连续性与恢复](docs/litchibot/normal_discontinuity.md)
- [Sharpa 双臂双手与三相机录制](data_collection/docs/SHARPA_ARM_RECORDING.md)
- [工程迁移与一键安装](工程迁移与一键安装说明.md)
- [硬件部署与故障处理](docs/HARDWARE_DEPLOY.md)
- [GELLO 控制语义](docs/GELLO_TELEOP.md)
- [FR3 相对末端动作示教](docs/RELATIVE_ACTION_TEACHING.md)
- [通用 Harvest 数据采集](data_collection/README.md)
- [第三方代码与许可证](THIRD_PARTY_NOTICES.md)
