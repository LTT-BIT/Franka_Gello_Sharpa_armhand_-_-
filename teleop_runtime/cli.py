#!/usr/bin/env python3
import argparse
import math
import os
import signal
import sys
from pathlib import Path

from pico_bimanual_franka_teleop.env_guard import ensure_ros_free_process

ensure_ros_free_process()

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import subprocess
from pico_bimanual_franka_teleop.config import load_config
from pico_bimanual_franka_teleop.hand_worker import HandWorker
from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop
from pico_bimanual_franka_teleop.preset_ik_client import invoke_preset_ik
from pico_bimanual_franka_teleop.relative_action import load_preset_actions
from pico_bimanual_franka_teleop.xr_input import PicoSession, create_pico_input
from operator_tasks import DEFAULT_OPERATOR_TASK, require_operator_task
from operator_hand_poses import HandHomeStore


def _invoke_trigger_service(service: str, timeout: float = 120.0) -> tuple[bool, str]:
    completed = subprocess.run(
        [
            "docker", "compose", "run", "--rm", "tools",
            "ros2", "service", "call", service, "std_srvs/srv/Trigger", "{}",
        ],
        cwd=REPO_ROOT / "docker",
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    output = (completed.stdout + completed.stderr).strip()
    succeeded = completed.returncode == 0 and "success=True" in completed.stdout
    return succeeded, output[-400:]


def invoke_reset(
    side: str | None = None, task: str = DEFAULT_OPERATOR_TASK
) -> tuple[bool, str]:
    """Call /reset_to_initial_pose (optionally one side) via the container.

    The operator process is deliberately ROS-free (env_guard), so the reset goes
    through the same `docker compose run` path the runbook documents. The
    trajectory itself can take ~20 s for large displacements, plus container
    startup; the timeout is generous because killing the call does not stop the
    controller-side trajectory anyway.
    """
    selected = require_operator_task(task)
    service = f"/reset_to_home/{selected}" + (f"/{side}" if side else "")
    return _invoke_trigger_service(service)


def invoke_capture_home(
    side: str, task: str = DEFAULT_OPERATOR_TASK
) -> tuple[bool, str]:
    """Persist one arm's current measured joints through its ROS service."""
    if side not in ("left", "right"):
        raise ValueError(f"invalid Home capture side: {side}")
    selected = require_operator_task(task)
    return _invoke_trigger_service(
        f"/capture_home/{selected}/{side}", timeout=30.0
    )


def invoke_ready(action: str) -> tuple[bool, str]:
    if action not in {"capture", "move"}:
        raise ValueError(f"invalid Ready action: {action}")
    service = "/capture_ready" if action == "capture" else "/reset_to_ready"
    return _invoke_trigger_service(service, timeout=30.0 if action == "capture" else 120.0)


def invoke_ready_to_home(task: str) -> tuple[bool, str]:
    return _invoke_trigger_service(
        f"/ready_to_home/{require_operator_task(task)}", timeout=180.0
    )


def main() -> None:
    # Bash starts asynchronous commands with SIGINT ignored when job control
    # is disabled. run_operator.sh deliberately backgrounds this process, so
    # explicitly restore an interrupt handler here; an ignored disposition is
    # inherited across exec and otherwise makes the supervisor wait for its
    # timeout before killing an otherwise healthy backend. Route SIGTERM
    # through the same KeyboardInterrupt/finally cleanup as Ctrl-C.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    parser = argparse.ArgumentParser(
        description=(
            "Unified FR3 and LinkerHand teleoperation. GELLO supplies incremental "
            "arm joints by default; legacy pose sources remain available. PICO "
            "optical tracking or MANUS may supply hand poses independently."
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--arm-source",
        required=True,
        choices=(
            "gello",
            "controllers",
            "motion-trackers",
            "hand-roots",
            "vive-trackers",
        ),
    )
    parser.add_argument(
        "--gello-config",
        default=str(REPO_ROOT / "config" / "modes" / "gello.yaml"),
        help="dual GELLO identities, directions, and incremental-control limits",
    )
    parser.add_argument(
        "--preset-config",
        default=str(REPO_ROOT / "config" / "preset_actions.yaml"),
        help="Q/W/E preset action slot configuration",
    )
    parser.add_argument(
        "--preset-data-root",
        default=os.environ.get("TELEOP_DATA_ROOT", str(REPO_ROOT / "data")),
        help="host TELEOP_DATA_ROOT containing arm_ui/actions",
    )
    parser.add_argument(
        "--vive-config",
        default=None,
        help="VIVE Tracker configuration; required with --arm-source vive-trackers",
    )
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument(
        "--control-port",
        type=int,
        default=5590,
        help="JSON-TCP operator control port; the PySide6 GUI "
        "(apps/operator_gui) connects here (default: 5590)",
    )
    # Hand options are CLI arguments rather than YAML, matching how --arm-source is
    # handled: what is being driven is an explicit choice per run, and this keeps
    # existing configuration files valid.
    parser.add_argument(
        "--debug-log",
        default=None,
        help="write one JSONL row per control tick, capturing tracker pose, "
        "mapped target, commanded and measured state, for offline analysis of "
        "following quality",
    )
    parser.add_argument(
        "--hand-source",
        default="none",
        choices=("none", "pico", "manus", "wuji", "litchibot"),
        help="hand source integrated into this operator process (default: none)",
    )
    parser.add_argument(
        "--hand-debug-log",
        default=None,
        help="write live canonical landmarks, emitted hand joints, and thumb "
        "fidelity metrics to JSONL (requires a hand source)",
    )
    parser.add_argument(
        "--record-left-dataset",
        type=Path,
        default=None,
        metavar="EPISODE.npz",
        help=(
            "record measured left FR3 joints, left Wuji Hand 2 joints, and "
            "left link8 pose into one synchronized NPZ episode"
        ),
    )
    parser.add_argument("--hand-host", default="127.0.0.1")
    parser.add_argument(
        "--hand-port",
        type=int,
        default=5570,
        help="where linker_hand_bridge listens (default: 5570)",
    )
    parser.add_argument(
        "--hand-rate",
        type=float,
        default=30.0,
        help="hand commands per second per side; the vendor driver drops "
        "commands above about 100 Hz (default: 30)",
    )
    parser.add_argument(
        "--hand-telemetry-host",
        default="127.0.0.1",
        help="one-way read-only Wuji telemetry destination (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--hand-telemetry-port",
        type=int,
        default=5602,
        help="one-way read-only Wuji telemetry UDP port (default: 5602)",
    )
    parser.add_argument(
        "--disable-hand-telemetry",
        action="store_true",
        help="disable the read-only UDP telemetry exporter",
    )
    parser.add_argument(
        "--left-hand-model",
        default=None,
        help=(
            "legacy combined hand/method name; defaults to g20 for PICO and "
            "o30i+sharpa for MANUS. For MANUS use o30i_casadi (or omit) for "
            "dual O30i; plain o30i selects the right-only landmark path"
        ),
    )
    parser.add_argument(
        "--right-hand-model",
        default=None,
        help=(
            "legacy combined hand/method name; defaults to g20 for PICO and "
            "o30i+sharpa for MANUS"
        ),
    )
    parser.add_argument(
        "--right-hand-strategy-config",
        type=Path,
        default=None,
        help=(
            "enable the powderweighing right-hand MANUS strategy using this "
            "pose/trigger JSON; the left hand remains on its configured method"
        ),
    )
    parser.add_argument(
        "--wuji-hand-strategy-config",
        type=Path,
        default=None,
        help=(
            "enable an optional MANUS-to-saved-pose Wuji strategy using this "
            "task policy JSON"
        ),
    )
    parser.add_argument(
        "--wuji-sides",
        choices=("left", "right", "both"),
        default="both",
        help="Wuji hardware sides to drive (only with --hand-source wuji)",
    )
    for side in ("left", "right"):
        parser.add_argument(
            f"--wuji-{side}-model",
            choices=("wuji_hand", "wuji_hand_2"),
            default="wuji_hand_2",
            help=f"physical {side} Wuji hand model",
        )
        parser.add_argument(
            f"--wuji-{side}-address",
            default="",
            help=f"{side} Wuji Hand 2 SDK address (IP:PORT)",
        )
        parser.add_argument(
            f"--wuji-{side}-serial",
            default="",
            help=f"{side} original Wuji Hand USB serial",
        )
    parser.add_argument("--wuji-kp", type=float, default=8.0)
    parser.add_argument("--wuji-kd", type=float, default=0.1)
    parser.add_argument(
        "--wuji-current-limit",
        type=float,
        default=1.0,
        help="Wuji Hand 2 per-joint current limit in amps",
    )
    from adapters.litchibot.cli import add_options as add_litchibot_options

    add_litchibot_options(parser)
    parser.add_argument("--dry-run", action="store_true",
                        help="LitchiBot hands-only dry-run; initializes no GELLO/FR3 devices")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="LitchiBot dry-run seconds; 0 until Ctrl+C")
    parser.add_argument("--enable-hand-output", action="store_true",
                        help="Explicitly allow LitchiBot UDP hand output after per-side engagement")
    args = parser.parse_args()
    if (args.dry_run or args.enable_hand_output) and args.hand_source != "litchibot":
        parser.error("--dry-run/--enable-hand-output currently require --hand-source litchibot")
    if args.dry_run and args.enable_hand_output:
        parser.error("--dry-run cannot be combined with --enable-hand-output")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must not be negative")
    if args.hand_source == "litchibot" and (
        args.left_hand_model or args.right_hand_model or args.right_hand_strategy_config
    ):
        parser.error("LitchiBot uses independent Sharpa mapping; Manus model/strategy overrides do not apply")
    if args.dry_run:
        from adapters.litchibot.cli import run_dry_run

        raise SystemExit(run_dry_run(args))
    preset_actions = load_preset_actions(
        args.preset_config, args.preset_data_root
    )
    if args.hand_debug_log and args.hand_source == "none":
        parser.error("--hand-debug-log requires a hand source")
    if not 0 < args.hand_telemetry_port < 65536:
        parser.error("--hand-telemetry-port must be in 1..65535")
    if args.arm_source == "vive-trackers" and not args.vive_config:
        parser.error("--arm-source vive-trackers requires --vive-config")
    if args.arm_source != "vive-trackers" and args.vive_config:
        parser.error("--vive-config is only valid with --arm-source vive-trackers")
    if args.right_hand_strategy_config and args.hand_source != "manus":
        parser.error(
            "--right-hand-strategy-config requires --hand-source manus"
        )
    if args.wuji_hand_strategy_config and args.hand_source != "wuji":
        parser.error(
            "--wuji-hand-strategy-config requires --hand-source wuji"
        )
    if args.hand_source != "wuji" and any(
        (
            args.wuji_left_address,
            args.wuji_right_address,
            args.wuji_left_serial,
            args.wuji_right_serial,
        )
    ):
        parser.error("--wuji-*-address/serial requires --hand-source wuji")
    if args.hand_source == "wuji":
        selected_wuji_sides = (
            ("left", "right")
            if args.wuji_sides == "both"
            else (args.wuji_sides,)
        )
        for side in selected_wuji_sides:
            model = getattr(args, f"wuji_{side}_model")
            address = getattr(args, f"wuji_{side}_address")
            if model == "wuji_hand_2" and not address:
                parser.error(
                    f"--wuji-{side}-address IP:PORT is required for "
                    "wuji_hand_2"
                )
        if args.wuji_kp < 0.0 or args.wuji_kd < 0.0:
            parser.error("--wuji-kp and --wuji-kd must not be negative")
        if args.wuji_current_limit <= 0.0:
            parser.error("--wuji-current-limit must be positive")
    if args.record_left_dataset is not None:
        if args.hand_source != "wuji":
            parser.error("--record-left-dataset requires --hand-source wuji")
        if args.wuji_sides == "right":
            parser.error("--record-left-dataset requires the left Wuji side")
        if args.wuji_left_model != "wuji_hand_2":
            parser.error("--record-left-dataset requires left Wuji Hand 2")

    if args.hand_source == "pico" and args.arm_source == "controllers":
        parser.error(
            "--hand-source pico cannot be combined with --arm-source "
            "controllers: holding a controller occupies the operator's "
            "hand, so the optical skeleton cannot describe a grasp"
        )

    debug_logger = None
    if args.debug_log:
        from pico_bimanual_franka_teleop.debug_log import FollowDebugLogger

        debug_logger = FollowDebugLogger(args.debug_log)
        print(f"debug log -> {args.debug_log}")

    config = load_config(args.config)

    from pico_bimanual_franka_teleop.control_server import (
        OperatorConsole,
        OperatorControlServer,
    )

    ui = OperatorConsole()
    server = OperatorControlServer((args.control_host, args.control_port), ui)
    server.start()
    print(
        f"operator control server on {args.control_host}:{args.control_port} "
        "- connect the GUI (apps/operator_gui) to engage"
    )

    pico_session = None
    arm_source = None
    hands = None
    dataset_recorder = None
    try:
        # Wuji SDK connection can hold the Python interpreter for multiple
        # seconds while it negotiates the network device.  Start its pipeline
        # before the deadline-monitored GELLO reader; otherwise healthy GELLO
        # buses can trip their fail-closed heartbeat during Wuji startup and
        # remain terminally stale for the entire session.
        defer_gello_until_hands = (
            args.arm_source == "gello" and args.hand_source == "wuji"
        )
        if args.arm_source == "gello":
            if not defer_gello_until_hands:
                from adapters.gello import DualGelloJointInput, load_gello_config

                arm_source = DualGelloJointInput(
                    load_gello_config(args.gello_config),
                    ui,
                )
        elif args.arm_source == "vive-trackers":
            from vive_tracker_teleop import ViveTrackerInput, load_vive_config

            arm_source = ViveTrackerInput(
                load_vive_config(args.vive_config),
                ui,
            )
        else:
            pico_session = PicoSession()
            arm_source = create_pico_input(
                config.input,
                args.arm_source,
                keyboard=ui,
                xrt_client=pico_session.client,
            )
        hand_pipeline = None
        if args.hand_source == "pico":
            if pico_session is None:
                pico_session = PicoSession()
            assert pico_session.client is not None
            from pico_bimanual_franka_teleop.hand_teleop import HandPipeline

            sides = ("left", "right")
            hand_pipeline = HandPipeline(
                pico_session.client,
                assets_root=REPO_ROOT / "assets",
                host=args.hand_host,
                port=args.hand_port,
                rate=args.hand_rate,
                sides=sides,
                models={
                    side: (getattr(args, f"{side}_hand_model") or "g20")
                    for side in sides
                },
                debug_log=args.hand_debug_log,
            )
        elif args.hand_source == "manus":
            from manus_teleop import ManusHandPipeline
            from manus_teleop.pipeline import (
                DEFAULT_HANDS,
                DEFAULT_METHODS,
                split_legacy_model,
            )

            task_config = None
            if args.right_hand_strategy_config is not None:
                # Task packages are optional and live outside the editable
                # source adapters, so expose only the repository package root.
                sys.path.insert(0, str(REPO_ROOT))
                from tasks.powderweighing.strategy import (
                    load_config as load_task_config,
                )

                try:
                    task_config = load_task_config(
                        args.right_hand_strategy_config
                    )
                except (OSError, KeyError, TypeError, ValueError) as error:
                    parser.error(
                        f"invalid right-hand strategy config: {error}"
                    )

            # Dual O30i + sharpa is the MANUS default. Plain "o30i" still means
            # the right-only landmark retargeter, so only decode CLI overrides.
            hands = dict(DEFAULT_HANDS)
            methods = dict(DEFAULT_METHODS)
            for side, legacy in (
                ("left", args.left_hand_model),
                ("right", args.right_hand_model),
            ):
                if legacy is not None:
                    hands[side], methods[side] = split_legacy_model(legacy)

            hand_pipeline = ManusHandPipeline(
                host=args.hand_host,
                port=args.hand_port,
                rate=args.hand_rate,
                debug_log=args.hand_debug_log,
                dynamic_sides=("left", "right"),
                hands=hands,
                methods=methods,
            )
            if task_config is not None:
                from tasks.powderweighing.strategy import (
                    install_on_manus_pipeline,
                )

                try:
                    install_on_manus_pipeline(hand_pipeline, task_config)
                except (TypeError, ValueError):
                    hand_pipeline.close()
                    raise
                print(
                    "right hand strategy -> powderweighing "
                    f"({args.right_hand_strategy_config})"
                )
        elif args.hand_source == "litchibot":
            from adapters.litchibot import LitchiBotHandPipeline
            from adapters.litchibot.cli import pipeline_options
            from adapters.litchibot.transport import UdpSharpaSender

            hand_pipeline = LitchiBotHandPipeline(
                **pipeline_options(args),
                dry_run=not args.enable_hand_output,
                sender_factory=lambda: UdpSharpaSender(port=args.sharpa_relay_port),
            )
        elif args.hand_source == "wuji":
            sys.path.insert(0, str(REPO_ROOT))
            from adapters.wuji import WujiHandPipeline

            wuji_task_config = None
            if args.wuji_hand_strategy_config is not None:
                from tasks.wuji_pose_switching.strategy import (
                    load_config as load_wuji_task_config,
                )

                try:
                    wuji_task_config = load_wuji_task_config(
                        args.wuji_hand_strategy_config
                    )
                except (OSError, KeyError, TypeError, ValueError) as error:
                    parser.error(f"invalid Wuji hand strategy config: {error}")

            sides = selected_wuji_sides
            models = {
                side: getattr(args, f"wuji_{side}_model") for side in sides
            }
            addresses = {
                side: getattr(args, f"wuji_{side}_address") for side in sides
            }
            serials = {
                side: getattr(args, f"wuji_{side}_serial") for side in sides
            }
            hand_pipeline = WujiHandPipeline(
                sides=sides,
                models=models,
                addresses=addresses,
                serials=serials,
                rate=args.hand_rate,
                kp=args.wuji_kp,
                kd=args.wuji_kd,
                current_limit=args.wuji_current_limit,
                debug_log=args.hand_debug_log,
            )
            if wuji_task_config is not None:
                from tasks.wuji_pose_switching.strategy import (
                    install_on_wuji_pipeline,
                )

                try:
                    install_on_wuji_pipeline(hand_pipeline, wuji_task_config)
                except (OSError, KeyError, TypeError, ValueError):
                    hand_pipeline.close()
                    raise
                print(
                    "Wuji hand strategy -> two-stage pose switching "
                    f"({args.wuji_hand_strategy_config})"
                )
        if defer_gello_until_hands:
            from adapters.gello import DualGelloJointInput, load_gello_config

            arm_source = DualGelloJointInput(
                load_gello_config(args.gello_config),
                ui,
            )
        if hand_pipeline is not None:
            telemetry_sender = None
            if args.hand_source == "wuji" and not args.disable_hand_telemetry:
                from teleop_runtime.hand_telemetry import UdpTelemetrySender

                telemetry_sender = UdpTelemetrySender(
                    args.hand_telemetry_host,
                    args.hand_telemetry_port,
                )
                print(
                    "read-only Wuji telemetry -> "
                    f"udp://{args.hand_telemetry_host}:{args.hand_telemetry_port} "
                    f"session={telemetry_sender.session_id}"
                )
            hands = HandWorker(
                hand_pipeline,
                tick_rate=config.host.control_rate,
                telemetry_sender=telemetry_sender,
            )
        if args.record_left_dataset is not None:
            from apps.left_wuji_dataset_recorder import LeftWujiDatasetRecorder

            dataset_recorder = LeftWujiDatasetRecorder(
                args.record_left_dataset,
                hand_joint_names=hand_pipeline.joint_names["left"],
                control_rate_hz=config.host.control_rate,
            )
            print(f"left teleop dataset -> {dataset_recorder.output}")

        teleop = DualFr3HardwareTeleop(
            command_host=config.udp.command_host,
            command_port=config.udp.command_port,
            state_host=config.udp.state_host,
            state_port=config.udp.state_port,
            state_timeout=config.udp.state_timeout,
            translation_scale=config.host.translation_scale,
            rotation_scale=config.host.rotation_scale,
            control_rate=config.host.control_rate,
            max_joint_speed=config.host.max_joint_speed,
            robot_state_wait_timeout=config.host.robot_state_wait_timeout,
            arm_source=arm_source,
            operator=ui,
            hands=hands,
            debug_logger=debug_logger,
            dataset_recorder=dataset_recorder,
            reset_invoker=invoke_reset,
            capture_home_invoker=invoke_capture_home,
            ready_invoker=invoke_ready,
            ready_to_home_invoker=invoke_ready_to_home,
            hand_home_store=HandHomeStore(args.preset_data_root),
            preset_actions=preset_actions,
            preset_solver=lambda preset: invoke_preset_ik(
                preset, max_joint_speed=config.host.max_joint_speed
            ),
        )
        # A legacy terminal-backed input's `q` (and Ctrl-C) surface as
        # KeyboardInterrupt. Operator GUI `Q` is a preset request and never
        # reaches this path. run() has already closed hardware by this point.
        try:
            teleop.run()
        except KeyboardInterrupt:
            pass
    finally:
        try:
            if dataset_recorder is not None:
                try:
                    dataset_recorder.close()
                except Exception as error:  # noqa: BLE001 - continue safe shutdown
                    print(f"dataset finalization FAILED: {error}", file=sys.stderr)
            if hands is not None:
                hands.close()
            if arm_source is not None:
                arm_source.close()
            if pico_session is not None:
                pico_session.close()
        finally:
            try:
                server.close()
            finally:
                if debug_logger is not None:
                    debug_logger.close()
    print("\nteleop stopped")


if __name__ == "__main__":
    main()
