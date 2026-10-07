# 生产 Terminal 2：Manus / LitchiBot 接入报告

> 后续新增的受监管实体验收模式见 [受控验收入口](supervised_validation.md)。本文 dry-run / Manus 结论仍有效；普通 LitchiBot 实体启动仍禁止，只有该独立模式通过许可与运行时 gate 后可进入受控验收。历史跳变风险没有被宣告消除。

2026-10-06。本轮完成软件接入及真实手套 → ROS → 原 fake Sharpa driver 的实测。没有启动实体 Sharpa SDK、FR3 控制或 Terminal 3。之前的 LitchiBot backend、V3.5、原厂 retarget 和真实手套 dry-run 全部保留。

## 1. 最终启动图与入口

Manus 默认模式：

```text
ops/run/run_sharpa_hands_cyclonedds.sh（无参数，或 --hand-source manus）
  → 原 CPU affinity / CycloneDDS / domain / localhost 配置
  → portable_deps/litchi_hardware/pixi.toml：ros2 teleop task
  → 原 manus_sharpa_teleop/launch/teleop.launch.py
      ├─ ManusDriverNode：native SDK + 原 metaglove calibration
      │    → /manus/{left,right}/keypoints
      ├─ RetargetNode / SharpaV4Retargeter
      │    → 原 Python 3.10 V4 worker / Unix socket / optimizer / filter
      │    → /sharpa/{left,right}/command
      └─ 唯一 SharpaDriverNode（默认参数和原来一致）
           → 原 Sharpa SDK sender → 左右 Sharpa
           → /sharpa/{left,right}/joint_states
           → /sharpa/{left,right}/tactile/{finger}/{raw,deformation,wrench}
```

LitchiBot 本轮允许的模式：

```text
ops/run/run_sharpa_hands_cyclonedds.sh --hand-source litchibot --dry-run
  → 同一原 CycloneDDS 环境 / pixi task / teleop.launch.py
      ├─ terminal2_bridge.py（ROS Python 3.12）
      │    └─ terminal2_worker.py（独立 SDK Python 3.10）
      │         → 真实手套 / LYG226360006 profile
      │         → 已有 LitchiBotHandPipeline / V3.5
      │         → 已有原厂 CMC 转换 / 原厂 Sharpa retarget
      │         → 22D target → stdout JSON → bridge 安全检查
      │    → /sharpa/{left,right}/command
      └─ 唯一的原 SharpaDriverNode，强制 use_fake_hardware=true
           ├─ 两个原 MockSharpaHand（没有实体 SDK 连接）
           ├─ 原 joint_states 发布逻辑
           └─ 原 tactile 发布逻辑（本轮发布合成数据）
```

LitchiBot 模式不启动 ManusDriverNode / RetargetNode / V4。没有新增 Sharpa 硬件进程，也没有重写 driver。

正式 dry-run 命令，在工程根目录执行：

```bash
./ops/run/run_sharpa_hands_cyclonedds.sh --hand-source litchibot --dry-run
```

该入口没有独立手套 GUI。bridge 每秒打印 `LitchiBot DRY-RUN` 状态，包括左右 solver 状态、有效通道数、数据年龄、累计 command 帧数、近期 Hz、fake 确认及实体发送数。`waiting for glove data` 表示还没收到该侧输入；command 为 0 Hz 时应检查 source age 和 rejected。Operator GUI 由 Terminal 1 以 `--hand-source none` 启动，其手部区域不表示这条 ROS 链路状态。修改状态打印后，已运行的 Terminal 2 需 Ctrl+C 后重启同一命令才能使用新代码。

30 秒定时结束：

```bash
./ops/run/run_sharpa_hands_cyclonedds.sh \
  --hand-source litchibot --dry-run litchibot_duration:=30
```

默认 profile 为 `LYG226360006`，支持 `litchibot_profile:=...`、`litchibot_python:=...`、`litchibot_sdk_root:=...`、`litchibot_data_root:=...`、`litchibot_receiver_id:=...`、`litchibot_log_dir:=...`。日志目录默认新建，指定目录时不要复用已有日志文件。worker 结束后原 launch 自动退出。Ctrl+C 结束 bridge 时也会关闭 worker。

LitchiBot 缺少 `--dry-run` 会在 wrapper 和 launch 两层被拒绝，发生在 hardware 节点启动之前。即使传入 `use_fake_hardware:=false`，dry-run 也会强制 fake。bridge 发布前必须从 `/sharpa_driver/get_parameters` 确认 fake 参数，并确认 command topic 的原 Sharpa driver 订阅端唯一。

## 2. 本轮修改文件与作用

| 文件（工程根目录相对路径） | 作用 |
|---|---|
| `ops/run/run_sharpa_hands_cyclonedds.sh` | 原 Terminal 2 增加源选择与 dry-run；默认 / 显式 Manus 的原执行 argv 保持一致；原校准、DDS、CPU 配置保留 |
| `portable_deps/litchi_hardware/ros2/src/manus_sharpa_teleop/manus_sharpa_teleop/launch/teleop.launch.py` | 条件启动原 Manus/V4 或新 bridge；SharpaDriverNode 始终只有一个；dry-run 强制 fake |
| `adapters/litchibot/terminal2_worker.py` | SDK Python 3.10；复用已有 backend/retarget；输出带 validity、时间和左右手身份的 target；不创建 sender |
| `adapters/litchibot/terminal2_bridge.py` | ROS Python 3.12；stdout IPC；验证 fake sink、source/feedback freshness、名称、顺序、限位、validity、序号；发布 JointState |
| `adapters/litchibot/transport.py` | 增加显式 `allow_dry_run`，默认关闭；仅本轮已确认 fake 的 bridge 开启。沿用已有 TargetGate 和 2 rad/s 限速 |
| `adapters/litchibot/inspect_terminal2.py` | 只读监测实际 ROS command/state/tactile、消息契约、频率、跳变与 driver 数量 |
| `adapters/litchibot/test_terminal2.py` | 默认 Manus、launch graph、实际 V4 publish 对照、worker EOF/错误、fake SDK 初始化阻断、T3 topic 与保护文件测试 |
| `adapters/litchibot/test_backend.py` | 旧 baseline 中授权修改的 T2 wrapper 改由专项回归验证；其余原文件校验保留 |
| `ops/run/run_litchibot_sharpa_output.sh` | 废弃并立即拒绝启动，含 `--enable-output` 也拒绝 |
| `adapters/litchibot/sharpa_output.launch.py` | 原平行硬件 launch 禁用，防止绕过脚本再启动一个 driver |
| `ops/run/run_sharpa_hands.sh` | LitchiBot 分支拒绝作为正式入口，提示使用原 CycloneDDS 入口；旧 Manus 分支保留 |
| `docs/litchibot/baselines/*` | 保存本轮原 wrapper / launch 与 57 个受保护文件 hash |
| `docs/litchibot/terminal2_launch.patch` | 原生产 launch 修改的可携带补丁 |
| `docs/litchibot/terminal2_ros_result.json` | worker、bridge、实际 ROS 监测与测试结果的汇总 |
| `docs/litchibot/接入报告.md` | 首轮历史报告增加最终报告指引；保留原证据 |

原 backend、V3.5、厂家映射实现、Manus V4、calibration、Sharpa sender、tactile 实现没有重写。`teleop_runtime/cli.py` 是首轮已有修改，本轮未再修改。Terminal 1 / Terminal 3 未修改；用户已有的其他工作区修改保留。

`portable_deps` 被主仓库忽略，故实际修改的 launch 同时导出成上述补丁。当前机器安装的 launch 指向源码，已实际使用此修改；迁移机器若非 symlink install，需要应用补丁并重新构建 ROS 包。

## 3. ROS command 契约

topic：`/sharpa/left/command` 和 `/sharpa/right/command`。类型均为 `sensor_msgs/msg/JointState`，`position` 长度 22，单位 rad，约 30 Hz。命令 joint names 不加左右前缀，和原 V4 最终 publish 一致：

```text
 1 thumb_CMC_FE
 2 thumb_CMC_AA
 3 thumb_MCP_FE
 4 thumb_MCP_AA
 5 thumb_IP
 6 index_MCP_FE
 7 index_MCP_AA
 8 index_PIP
 9 index_DIP
10 middle_MCP_FE
11 middle_MCP_AA
12 middle_PIP
13 middle_DIP
14 ring_MCP_FE
15 ring_MCP_AA
16 ring_PIP
17 ring_DIP
18 pinky_CMC
19 pinky_MCP_FE
20 pinky_MCP_AA
21 pinky_PIP
22 pinky_DIP
```

限位逐项和生产 `SHARPA_WAVE_JOINT_SCHEMA` 对照；详表保留在 `joint_table.json`。测试直接调用原 `RetargetNode._on_keypoints` 的 publish 路径，以相同 22D 数据比较名称、顺序、position 和时间戳，没有只比较 shape。

左右 topic 由 packet side 决定；frame_id 含 `litchibot:side:session:sequence`。`header.stamp` 使用当前 ROS 时钟减去 source monotonic age，表示源接收时间，而非把 monotonic 值当作 ROS epoch。设备原 timestamp 保留在 worker 日志。

source / feedback 超过 250 ms、未来时间、重复或乱序序号、名称/shape 错误、NaN/Inf、超限 target 均跳过；原 backend 的限位裁剪保留。无效 target channels 保持上一命令，首次采集保持对应 fake feedback，不让初始化 fallback 驱动该关节。其他有效关节继续发布。worker SDK 错误被已有 backend 容纳；worker 进程 EOF/异常时 bridge 收尾退出，launch 关闭共享 fake driver。

## 4. 真实手套 / 生产 Terminal 2 的 30 秒 dry-run

实测命令：

```bash
./ops/run/run_sharpa_hands_cyclonedds.sh \
  --hand-source litchibot --dry-run litchibot_duration:=30 \
  litchibot_log_dir:=/home/user/litchibot_glove/validation/logs/terminal2_ros_30s
```

| 指标 | 左手 | 右手 |
|---|---:|---:|
| 原始 22D target 帧数 | 899 | 899 |
| 原始 target Hz | 30.0041 | 30.0041 |
| 实际 ROS command 帧数 | 899 | 899 |
| 实际 ROS command Hz | 30.0051 | 30.0051 |
| position 长度 | 22 | 22 |
| 本轮原始 target 最大相邻变化 rad | 0.011215 | 0.010362 |
| ROS command 最大相邻变化 rad | 0.077168 | 0.077186 |
| ROS command 间隔 P95 ms | 38.498 | 38.518 |
| ROS command 最大间隔 ms | 43.724 | 43.813 |
| joint_states 帧数 | 914 | 914 |
| joint_states Hz | 30.0000 | 29.9999 |

bridge rejected=0、malformed worker lines=0、worker exit=0；所有 ROS 帧的 names/order/shape/finite/限位/side/持续时间戳检查通过，监测 errors={}。图中 SharpaDriverNode 最大数量=1。backend sender count=0，实体 hardware send count=0。原 fake driver 实际接收了 ROS commands，但没有初始化实体 SDK；专项测试用实体 SDK loader 的失败 spy 验证 fake 路径不会调用它。

ROS 最大变化高于原始 target，来自 fake feedback 从初始零位按原 TargetGate 限速靠近初始目标，不是原始解算 target 的变化；这不代表历史跳变已经解决。

左侧 899 帧均为 partial：`thumb_mcp_flex` / `thumb_pip_flex` 无效，但其余通道正常发布；右侧 899 帧 solved。SDK 记录 2 次 `Malformed UWB packet: Invalid/duplicate sensor ID`，链路持续运行。没有把错误隐瞒为完整有效姿态。

原 driver 额外发布 30 个 tactile topic（2 手 × 5 指 × raw/deformation/wrench），每个 1829 帧，约 60 Hz。本轮由 MockSharpaHand 生成：2×2 mono8 合成图像和零 wrench；joint_states 也是 fake 模型状态。只有手套输入是真实传感器数据，实体手状态和实体触觉没有采集。

原始日志：

- `/home/user/litchibot_glove/validation/logs/terminal2_ros_30s/worker_targets.jsonl`
- `/home/user/litchibot_glove/validation/logs/terminal2_ros_30s/bridge.jsonl`
- `/home/user/litchibot_glove/validation/logs/terminal2_ros_topics.json`
- `/home/user/litchibot_glove/validation/logs/terminal2_ros_30s_launch.log`

汇总：`terminal2_ros_result.json`。

## 5. Manus regression 与 Terminal 3

默认 Manus 没有启动实体做运动测试；完成的是静态和自动化回归：

- 无参数、`--hand-source manus`、`hand_source:=manus` 与本轮之前 wrapper 的 exec argv / DDS 环境完全一致。
- 原 launch 所有旧参数默认值、原三个节点的参数字典对照一致；另抽查 fake 大写 True、禁用左侧/触觉和 serial override。
- 原 Manus native/calibration、V4 worker/socket/optimizer/filter、原 Sharpa hardware sender、tactile 和 T1/T3 共 57 个受保护文件 hash 未变化。
- ROS Python 3.12：本轮专项与原 Sharpa / Manus retarget 测试共 **36 passed**。
- 原 Python 3.10 backend / hand pipeline / 架构回归：**47 passed，8 skipped**；跳过项是已在 ROS 环境执行的 ROS 专项测试。
- worker malformed stdout、异常退出 code=3 和正常 EOF 均有测试，零 command，清理后退出；invalid joint / stale / disconnect 的已有 backend 回归通过。

Terminal 3 仍使用原 `start_sharpa_arm_recording_with_tactile.sh --with-cameras`。原 yaml 内 24 个 Sharpa topic 的名称和类型不变：

```text
/sharpa/{left,right}/command                      JointState：2
/sharpa/{left,right}/joint_states                 JointState：2
/sharpa/{left,right}/tactile/{finger}/wrench       WrenchStamped：10
/sharpa/{left,right}/tactile/{finger}/deformation  Image：10
finger = thumb, index, middle, ring, pinky
```

recorder 无需识别 hand_source 来订阅；本轮未开启录制，未重测 arms/cameras。原 yaml 的静态 `calibration_ids.hands` 标签仍指 Manus，遵守本轮不修改 Terminal 3 的要求；它不影响 topic 兼容，但尚未自动记录 LitchiBot profile，需与 worker 日志结合追溯。

## 6. 当前实体运动阻断

**BLOCKER FOR REAL HARDWARE MOTION：历史左侧 1.5708 rad discontinuity。**

首轮真实 30 秒 dry-run 左侧最大跳变 1.5708 rad，右侧约 0.0105 rad，证据继续保留。本轮没有再次出现同样大跳变，不足以撤销阻断；左拇指无效、本人校准/动作方向对照仍需要单独核对。后续沿既有 retarget 排查 source invalid、hold、初始化、小指/拇指映射、CMC projection、限位裁剪与 discontinuity。本轮完成软件集成，不解锁实体跟随。
