# 遥操核心架构

本文解释 gello-retarget 的机械臂与灵巧手遥操控制链：谁产生命令、谁校验、谁允许写硬件，以及各层在故障时如何失败关闭。数据采集、相机 bringup、MoveIt 示教是独立子系统，不进入本控制链。

仓库级硬约束写在 `CLAUDE.md`：

- `teleop_core.contract` 是机器人 topic 与关节名的唯一来源。
- 每个机械臂遥操源最终发布 `teleop_interfaces/ArmCommand`；只有安全网关可以写入 FR3 命令总线。
- 硬件输出默认关闭。不得为了让测试通过而放松新鲜度、采集、限位或限速检查。
- 源适配器、安全/控制、数据工具彼此独立。

默认真机链路：

```text
双 GELLO ──增量关节映射──UDP──ROS 安全网关──双 FR3
双 MANUS ──手部重定向────────LinkerHand 安全桥接──双 O30i
```

启动后双侧均为未 Engage。操作员通过 Operator GUI 分侧打开机械臂或手跟随。

---

## 1. 分层与进程边界

控制链分成三层，物理上跑在两类进程里。

| 层 | 位置 | 职责 | 禁止事项 |
| --- | --- | --- | --- |
| 源适配 | 宿主机 Conda 进程 | 读设备、映射目标、决定 Engage | 不加载 ROS Python，不写 FR3 / 厂商手命令总线 |
| 安全与控制 | Docker ROS 2 | 仲裁来源、校验、限速、写控制器 | 不解析 GELLO / MANUS / PICO 原始输入 |
| 数据工具 | 独立进程 | Harvest bag / LeRobot / 只读 telemetry | 不向命令总线回写 |

宿主机 Operator 进程启动时会调用 `ensure_ros_free_process()`。若当前 shell 污染了 `/opt/ros/` 的 `PYTHONPATH` / `LD_LIBRARY_PATH`，进程会清洗环境后重新 exec。原因是 `LD_LIBRARY_PATH` 只在进程启动时生效，事后改环境变量无法避免 Conda Pinocchio 与发行版库冲突。

标准遥操的进程拓扑：

```text
ops/run/start_teleop.sh
        │
        ├── Docker（network_mode: host，ROS 仅本机）
        │     franka-control     双 FR3 阻抗控制器 + FCI
        │     teleop-control     安全网关 + 关节分流器
        │     gello-bridge       UDP ↔ ArmCommand（source_id=gello）
        │     hand-control       linker_hand_bridge + O30i 驱动
        │     moveit-ik          只读 MoveIt，供预设动作预检
        │     preset-ik          常驻 TCP IK 服务 :5591
        │
        └── 宿主机（HOUSEKEEPING_CPUSET，避开 Franka 实时核）
              teleop_runtime.cli     Operator 后端，100 Hz
              apps/operator_gui      PySide6，遥操 JSON-TCP :5590
                    └─────────────── 独立采集控制 JSON-TCP 127.0.0.1:5592
```

`apps` 只保存 UI，`tasks` 只保存可选任务策略，硬件差异封装在 `adapters`。`teleop_runtime/cli.py` 是唯一的 Operator 后端入口：按 CLI 选择机械臂源与手源，再交给共享协调器 `DualFr3HardwareTeleop`。

MoveIt 真机/假硬件是另一套机械臂控制模式。启动脚本会拒绝与 `moveit-fake` / `moveit-real` / `arm-ui` 同时运行，因为两套控制器不能同时占用 FR3。

---

## 2. 契约：名字、话题、消息

`ros_ws/src/teleop_core/teleop_core/contract.py` 定义命令侧与数据侧两套名字。命令侧带 `fr3v2`，对应当前 URDF；数据侧归一化为 `left_fr3_joint*` / `right_fr3_joint*`，供 Harvest 数据集使用，从不出现在硬件命令 publisher 上。

### 2.1 机械臂命令关节

每侧 7 个关节，左右拼接为 14 维。规范顺序：

```text
left_fr3v2_joint1 … left_fr3v2_joint7
right_fr3v2_joint1 … right_fr3v2_joint7
```

控制器侧单臂名字是无前缀的 `fr3_joint1…7`。分流器负责把网关输出拆成左右控制器话题。

### 2.2 机械臂 ROS 话题

| 常量 | 话题 | 方向 | 含义 |
| --- | --- | --- | --- |
| `SOURCE_COMMAND_TOPIC` | `/teleop/arm_commands` | 源 → 网关 | `ArmCommand`，所有遥操源的唯一入口 |
| `COMMAND_STATUS_TOPIC` | `/teleop/arm_command_status` | 网关 → 源 | 接受了哪些侧、拒绝原因 |
| `VALIDATED_COMMAND_TOPIC` | `/teleop/validated_arm_commands` | 网关旁路 | 已校验命令，供观测 |
| `ARM_COMMAND_TOPIC` | `/target_robot/joint_commands` | **仅网关可写** | FR3 命令总线 |
| `ARM_STATE_TOPIC` | `/{side}/franka/joint_states` | 控制器 → 网关/桥 | 实测关节 |
| `EXTERNAL_TORQUES_TOPIC` | `/{side}/franka_robot_state_broadcaster/external_joint_torques` | 控制器 → 网关 | 接触力矩 gating |
| `CONTROLLER_COMMAND_TOPIC` | `/{side}/gello/joint_states` | 分流器 → 控制器 | 单臂 7 关节目标 |
| `RESET_ACTIVE_TOPIC` | `/reset_to_initial_pose/active` | Home 服务 → 网关 | Home 期间独占命令通道 |

`ArmCommand` 字段：`source`、`session_id`、`sequence`、`active_sides`、`joint_names`、`positions`。`joint_names` 必须与 `active_sides` 对应的规范名字完全一致，顺序也不能错。

### 2.3 手部契约（与机械臂解耦）

LinkerHand 走独立 UDP 与独立 ROS 话题，故障不牵连机械臂网关。Wuji 不经过 LinkerHand 桥，由 Operator 进程直连 SDK；只读 telemetry 使用 `WUJI_*` 名字与 `/teleop/wuji/{side}/command|joint_states`。

---

## 3. 端到端数据流

### 3.1 默认机械臂：GELLO 增量关节

```text
OpenRB-150 USB（by-id 序列号）
        │ Dynamixel 1–7，57600 baud；电机 8 永不打开
        ▼
DualGelloJointInput          每侧独立读线程
        │ 标定符号 × 方向校正；跳变/过期则该侧 fail-closed
        ▼
RelativeJointMapper          Engage 边沿锚定
        │ 100 Hz DualFr3HardwareTeleop
        ▼
UdpRobotBackend              JSON UDP :5560  command
        │                    JSON UDP :5561  state + 网关 faults
        ▼
pico_teleop_bridge           source_id=gello
        │ 发布 ArmCommand
        ▼
SafetyGateway                仲裁 / 新鲜度 / 限位 / 0.7 rad/s / 接触力矩
        │ JointState → /target_robot/joint_commands
        ▼
JointSplitter
        │ /left/gello/joint_states  /right/gello/joint_states
        ▼
franka_fr3_arm_controllers   阻抗跟踪
```

状态回流与命令同构：桥把左右实测 14 关节打成 `kind=state` 的 UDP 包，并把最近一次网关 `ArmCommandStatus` 附在 `command_sequence` / `faults` 上。Operator 看到单侧 `"{side} …"` 故障只停该侧；其它故障停双侧。

### 3.2 姿态输入变体：PICO / VIVE

`--arm-source` 不是 `gello` 时，适配器输出末端位姿而不是 7 关节。协调器改用 `RelativePoseMapper` + `BimanualPinkIK`（Pink / Pinocchio QP），再把 14 关节目标送入同一条 UDP。Docker 侧仍是同一个 `pico_teleop_bridge` 节点，只是 `source_id` 改为 `pico` 或 `vive`。同一时刻只允许一个 UDP 桥占用 `:5560`。

PICO 控制器占用手，因此 `--hand-source pico` 不能与 `--arm-source controllers` 组合。

### 3.3 默认手：MANUS → O30i

```text
MANUS 手套 + Calibration_{left,right}.mcal
        │ ManusHandPipeline（默认 o30i + sharpa）
        ▼
HandWorker                   独立线程，不挡 100 Hz 臂环
        │ HandCommandSender  ≤30 Hz UDP :5570
        ▼
linker_hand_bridge           范围、watchdog 0.25 s、slew
        │ /cb_{side}_hand_control_cmd
        ▼
linker_hand_ros2_sdk         O30i / G20 厂商驱动
```

手跟随与臂跟随在 `OperatorConsole` 里是两套开关。臂故障会停手跟随，但手输入丢失不会让安全网关丢掉 FR3 命令。

### 3.4 可选手：Wuji

`--hand-source wuji` 时不启动 `hand-control`。MANUS 骨架经 `adapters/wuji` 重定向后由 `WujiHand2Backend` 写 SDK。Operator 默认同机 UDP `:5602` 导出只读 telemetry；采集进程订阅，不参与控制。Wuji 启动可能阻塞数秒，因此 CLI 会先接通手，再打开带心跳的 GELLO 读线程，避免总线在等待期间被判死。

---

## 4. Operator 后端：100 Hz 协调器

实现：`adapters/pico/src/pico_bimanual_franka_teleop/hardware.py` 中的 `DualFr3HardwareTeleop`。GELLO 入口包 `adapters/gello` 只是稳定 re-export，共享控制代码仍在 PICO 时代的 Python 包里。

每个 tick 的顺序：

1. 收尾 Home / 录 Home / 预设 IK 后台线程。
2. `UdpRobotBackend.receive_state()`。状态缺失则双侧 Disengage、映射器复位、发送 `active_sides=()`；持续超过 `robot_state_wait_timeout`（默认 10 s）则退出。
3. 消化网关 faults。
4. `arm_source.sample()` 与 `operator.take_requests()`。处理张开手、Home、Ready、Q/W/E、停止预设。
5. Home / 录制 / 预设进行中时强制对应侧不跟随，避免与独占轨迹抢总线。
6. 未 Engage 或映射器未激活的关节，用实测值重播种 `hold_q`，防止下一次 Engage 从过时保持值跳变。
7. **关节源（GELLO）**：每侧 `RelativeJointMapper.update()`，直接写 `hold_q`。
   **位姿源（PICO/VIVE）**：映射相对末端目标，再 `BimanualPinkIK.step()`。
8. 若有预设轨迹，覆盖该侧 `hold_q`（0.5 s 从当前实测切入首点）。
9. `send_command(hold_q, active_sides)`。未激活侧不出现在包里，网关保持该侧上次输出并清除其限速状态。
10. 把手激活表交给 `HandWorker`；可选写 debug JSONL 或左侧数据集。
11. sleep 补齐 10 ms。

`hold_q` 是协调器内部的 14 维命令种子。首次收到状态时用实测初始化，并把 IK 零空间姿态吸引子锚在该姿态（通常是硬件 Home）。

后台 ROS 调用（Home、录 Home、Ready、预设 IK）都走 worker 线程。Operator 本身不 `import rclpy`；Home 通过 `docker compose run tools ros2 service call`。

---

## 5. GELLO 输入

配置：`config/modes/gello.yaml`。左右身份是 OpenRB-150 的 USB 序列号，路径必须是 `/dev/serial/by-id/…` 且包含 `expected_serial`。禁止用 `ttyACM0/1` 推断左右。

### 5.1 读总线

`DynamixelJointReader` 直接使用 `gello.dynamixel.driver.DynamixelDriver`，但关掉上游驱动的三类不安全行为：杀占用进程、sudo 改权限、假设备回退。启动前用 `lsof` 确认端口空闲。

驱动初始化时写一次 torque-disable，之后只读位置/速度。GELLO 是被动 leader，不出力。

读线程给每次成功的 `txRxPacket` 打心跳。超过 `stale_timeout`（0.25 s）没有成功总线事务，该侧进入**终端故障**：停读线程，后续必须重启进程，不能静默用缓存角。

### 5.2 标定与连续化

原始角乘以 `standard_signs * direction_correction`。当前左臂校正为 `[1, -1, 1, 1, 1, -1, 1]`（左 J2/J6 反相），右臂为 `[-1, 1, 1, 1, 1, 1, -1]`（右 J1/J7 反相），`standard_signs` 为 `[1, -1, 1, -1, 1, 1, 1]`。

相邻样本做 \(2\pi\) 展开。单关节跳变超过 `max_joint_jump`（0.35 rad）时该侧本拍无效；连续 10 个稳定样本后才 rebase。重新跟随必须由操作员再 Engage，映射器在 Engage 边沿用实测机器人姿态锚定。

`sample()` 把「GUI 请求跟随」和「该侧样本新鲜」做成 `activations`。过期时 `deny` 该臂，不影响另一臂、也不影响手。

---

## 6. 增量映射

`RelativeJointMapper` 在每侧 Engage 上升沿锁定：

```text
leader_anchor = 当前标定 GELLO 角
robot_anchor  = 当前实测 FR3 角
首条命令      = robot_anchor
```

因此不要求 GELLO 与 FR3 绝对姿态重合。之后：

```text
raw_delta      = gello_now - leader_anchor
scaled_delta   = clip(raw_delta * joint_sensitivity, ±max_relative_delta)
desired        = clip(robot_anchor + scaled_delta, 有效软限位)
target         = 上一目标 + clip(desired - 上一目标, ±0.8 rad/s · dt)
```

要点：

- `joint_sensitivity` 限制在 \([0.1, 2.2]\)。`2.0` 表示 GELLO 转 1°，FR3 目标走 2°。方向只由符号向量决定。
- 左臂相对行程帽每关节 1.5 rad；右臂帽取 FR3 物理行程，实际上由绝对软限位约束。
- 软限位 = 物理限位两端各留 1% 行程。若 Engage 时已在软区外，静止 GELLO 保持实测姿态，只允许往安全区退，不会自动往回拉。
- 映射器 0.8 rad/s 是内层；网关 0.7 rad/s 是最终输出上限。

Disengage 或任何 fail-closed 都 `mapper.reset()`。再 Engage 一定重新锚定，首命令等于当时实测。

位姿源使用 `RelativePoseMapper`：同样在 Engage 锁定输入末端与机器人末端，把位移/旋转增量缩放到机器人坐标系，再交给 Pink IK。IK 诊断分为 `ok` / `joint-limit` / `speed-clamp` / `workspace`，按秒汇总到 GUI。

---

## 7. UDP 关节协议与 ROS 桥

`teleop_core.protocol` 版本 2。数据包是有界 JSON（≤16384 字节），字段集合必须恰好匹配，否则整包丢弃。

命令包：`kind=command`，携带 `stream_id`（进程级 UUID）、单调 `sequence`、`active_sides`、规范关节名与位置。命令包不得带网关反馈字段。

状态包：`kind=state`，始终 14 个规范名字。可选附上针对该 `stream_id` 的网关 `faults`。

`pico_teleop_bridge`（Compose 服务名 `gello-bridge`）每 10 ms：

1. 若左右实测都新鲜，向 `:5561` 发状态。
2. 排空 `:5560`，只取最新合法命令。
3. 填 `ArmCommand.source = source_id` 后发布到 `/teleop/arm_commands`。

`source_id` 必须落在网关 `allowed_sources` 里：`gello`、`pico`、`vive`、`replay`。桥不校验限位或速度，那是网关的工作。

---

## 8. 安全网关

节点：`teleop_core.safety_gateway`。这是仓库里**唯一**允许 publish `/target_robot/joint_commands` 的代码路径。

参数见 `config/modes/teleop_control.yaml`。

### 8.1 来源仲裁 `SourceArbiter`

同一时刻一个 `(source, session_id)` 拥有通道。命令超时 0.25 s 后才允许新 session。序列必须严格递增。新 session 会重置限速器，避免跨会话的速度记忆把机器人甩出去。

### 8.2 命令门 `CommandSafetyGate`

对每个出现在 `active_sides` 的手臂独立处理。一侧失败只丢该侧，另一侧继续（历史上整包丢弃会把完好的左臂一起冻住）。

| 检查 | 行为 |
| --- | --- |
| 双侧实测 14 关节新鲜（0.25 s） | 否则拒绝并重置仲裁/门 |
| Home 占用中 | 直接拒绝，轨迹独占总线 |
| 名字与 `active_sides` 一致 | 否则该侧故障 |
| 硬限位 `LOWER_LIMITS` / `UPPER_LIMITS` | 超限丢该侧 |
| 首次目标距实测 ≤ 0.05 rad | 否则丢该侧；增量映射的设计就是让首目标等于实测 |
| 限速 0.7 rad/s | 相对**上次网关输出**裁剪，不是相对实测 |
| 接触力矩 gating | 见下 |

接触力矩来自 libfranka 的外力矩估计。某关节 \(|\tau|\) 超过阈值（近端 6 Nm，腕部 3 Nm）且命令步进与 \(\tau\) 同号时，该关节本拍保持上次输出（同号会把接触压得更紧）。卸载方向始终放行。力矩话题缺失或过期时该侧 gating **fail-open** 并告警：反射阈值仍在，但诊断话题丢失不得把整臂冻死。

网关同时发布 `ArmCommandStatus`。`accepted_sides` 与 `faults` 经 UDP 回到 Operator。

### 8.3 分流到控制器

`joint_splitter` 订阅 `/target_robot/joint_commands`。消息里出现完整左臂 7 名则发 `/left/gello/joint_states`，右臂同理。控制器内部名字重写为 `fr3_joint1…7`。缺少一侧时该侧本拍不发，阻抗控制器保持上次目标。

---

## 9. 手部控制链

### 9.1 发送器与 worker

`HandCommandSender` 是所有手管道共用的 UDP 发送层：每侧独立序号与截止时间，发送失败记入 `HandStatus`，绝不抛进臂环。默认 30 Hz，上限 60 Hz；厂商驱动在约 100 Hz 以上会丢包。

`HandWorker` 在独立线程里跑管道。臂环只做 `set_active` / `request_open` / `request_pose`。张开手或手部 Home 会先停该侧跟随，再流式发送目标。

### 9.2 LinkerHand 桥（O30i / G20）

`linker_hand_bridge` 是写厂商命令总线的唯一节点。网页手动 UI 也走 `/linker_hand_bridge/{side}/manual_command`，不直连驱动。

每个合法 qpos 包映射到 20 槽厂商命令，经 `CommandLimiter` 按设备剖面限幅、限速。watchdog 0.25 s 无新鲜包则停止发布；**超时不会自动回 Home**。硬件输出始终开启（已无 dry-run）。

O30i 与 G20 的投影表、极性、量化与 URDF 限位绑在一起；改模型必须改测试。

### 9.3 MANUS 重定向

默认双手 `o30i` + `sharpa`。未标定的手套只报告 waiting，另一侧继续。称粉等特殊任务通过 `--right-hand-strategy-config` 包装右手策略，不改默认管线。

---

## 10. 操作员协议、Home 与预设

GUI 连 `127.0.0.1:5590`，行分隔 JSON：`{"id","command","arguments"}` → `{"id","ok","result"|"error"}`。服务线程只改 `OperatorConsole` 的激活位和一次性请求；100 Hz 环按自己的节拍读取，与当年键盘轮询相同。

数据采集按钮另连 `127.0.0.1:5592`，只发送 `status/start/stop/mark_milestone/discard`。该接口由
独立 `teleop_data_collector` 进程提供，不经过遥操 backend，也不能发布机器人命令。

| 操作 | 作用对象 |
| --- | --- |
| A / C | 左/右臂 Hold |
| L | 开始录制；录制中再次按下则结束并校验 |
| Space | 录制中保存中间完成标记，继续录制 |
| DISENGAGE ALL | 停所有跟随 |
| W / E | 相对末端预设快捷键 |

两块 PCsensor 踏板已分别写入 `A/C` 和 `L/Space`，不依赖固定 USB 端口。
踏板模拟普通键盘，Operator GUI 窗口必须获得键盘焦点。
左右 `Start arm` 只保留 GUI 按钮，不再绑定键盘快捷键。
左右 `Start hand` 同样只保留 GUI 按钮，取消原来的 `R` / `B` 跟随快捷键。
采集终端仍可使用 `L` 起停、`Space` 保存中间标记。
“丢弃最近一次”保留 GUI 按钮；采集终端备用丢弃键仍为 `D`。

Home 只移动所选 FR3 并等待到位稳定，不读取或回放手部 Home。录 Home 必须先停该侧臂，只保存实测臂关节，不读取手反馈或驱动硬件。

预设 Q：临时松开该侧 GELLO，经常驻 `preset-ik`（`:5591`，非实时核）把示教的工具相对轨迹贴到当前 `link8`，用 MoveIt KDL `/compute_ik` 逐帧做碰撞与连续性检查。全部成功后轨迹仍走现有 100 Hz UDP，不换控制器。完成、STOP、DISENGAGE、丢 GUI、或 GELLO 相对锚点 ≥ 0.08 rad 都会交还所有权并重置映射器。正常结束要求末点误差 ≤ 0.03 rad 并保持 0.25 s；5 s 未收敛则停臂，不自动恢复跟随。

---

## 11. 失败语义（默认安全）

任何一层检测到问题，原则都是**停跟随、保持或限速、要求显式再 Engage**，而不是猜测或回放缓存。

| 事件 | 机械臂 | 手 |
| --- | --- | --- |
| 启动 | 双侧 Disengage | 双侧 Disengage |
| GELLO 过期 / 跳变 / 总线心跳丢失 | 该臂 deny，映射器复位 | 不变 |
| 双侧机器人状态过期 | 全停，发空 `active_sides` | 停跟随 |
| 网关拒绝单侧 | 该臂 deny | 不变 |
| Home / Ready / 预设占用 | 相关臂强制停跟 | 解除跟随，不执行手部 Home |
| GUI 掉线 | `disable_all` | `disable_all` |
| 手包过期 | 不变 | 桥停止发布，不回 Home |
| 力矩话题丢失 | 该侧接触 gating 关闭（反射仍在） | 不变 |

不要用放宽 `stale_timeout`、`max_joint_speed`、`max_initial_delta` 或关掉接触 gating 的方式让测试变绿。

---

## 12. 关键文件索引

| 路径 | 角色 |
| --- | --- |
| `teleop_runtime/cli.py` | Operator 后端入口：选源、组协调器、GUI 端口 |
| `adapters/pico/.../hardware.py` | 100 Hz 臂/手协调器 |
| `adapters/pico/.../joint_mapping.py` | GELLO 增量映射 |
| `adapters/pico/.../gello_input.py` | 双 GELLO 读线程与 fail-closed |
| `adapters/pico/.../robot_udp.py` | 宿主机 UDP 后端 |
| `adapters/pico/.../control_server.py` | OperatorConsole + JSON-TCP |
| `adapters/pico/.../hand_worker.py` | 手线程与臂环隔离 |
| `ros_ws/src/teleop_core/teleop_core/contract.py` | 名字与话题 |
| `ros_ws/src/teleop_core/teleop_core/protocol.py` | UDP JSON 协议 v2 |
| `ros_ws/src/teleop_core/teleop_core/safety_gateway.py` | 安全网关节点 |
| `ros_ws/src/teleop_core/teleop_core/safety.py` | 限位、限速、接触 gating |
| `ros_ws/src/teleop_core/teleop_core/arbitration.py` | 单源单 session |
| `ros_ws/src/pico_teleop_bridge/` | UDP ↔ `ArmCommand` |
| `ros_ws/src/linker_hand_bridge/` | 手命令校验与厂商映射 |
| `config/modes/gello.yaml` | GELLO 身份、灵敏度、相对帽、映射限速 |
| `config/modes/pico.yaml` | UDP 端口与 100 Hz 主机参数 |
| `config/modes/teleop_control.yaml` | 网关限速与力矩阈值 |
| `config/workcell/current.yaml` | 双 FR3 地址与命名空间 |
| `ops/run/start_teleop.sh` | 默认一键启动 |

相关专题文档：`docs/GELLO_TELEOP.md`（GELLO 部署与验证）、`docs/HARDWARE_DEPLOY.md`、`adapters/wuji/README.md`、`data_collection/README.md`。
