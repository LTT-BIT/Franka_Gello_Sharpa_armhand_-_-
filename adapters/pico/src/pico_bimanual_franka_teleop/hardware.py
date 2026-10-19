import threading
import time
from dataclasses import replace

import numpy as np
from operator_tasks import DEFAULT_OPERATOR_TASK, task_label
from teleop_core.safety import LOWER_LIMITS, UPPER_LIMITS

from .ik import BimanualPinkIK, IKError, classify_step
from .interfaces import ArmPoseSource, HandController, OperatorState
from .joint_mapping import RelativeJointMapper
from .loop_timing import LoopTimingDiagnostics
from .pose_mapping import RelativePoseMapper
from .relative_action import PresetAction, SolvedRelativeAction, sample_solved_action
from .robot_udp import UdpRobotBackend
from .types import ArmSample, JointTeleopSample, SIDES, TeleopSample


PRESET_INTERRUPT_DELTA_RAD = 0.08
PRESET_LEAD_IN_SEC = 0.50
PRESET_SETTLE_TOLERANCE_RAD = 0.03
PRESET_SETTLE_HOLD_SEC = 0.25
PRESET_SETTLE_TIMEOUT_SEC = 5.0


def reseed_inactive_joints(
    held, measured, activations, mapper_active, arm_holds=None
):
    """Keep every inactive or newly engaging side aligned to hardware."""
    result = np.asarray(held, dtype=float).copy()
    measured = np.asarray(measured, dtype=float)
    arm_holds = arm_holds or {}
    for side, joints in (("left", slice(0, 7)), ("right", slice(7, 14))):
        if arm_holds.get(side, False):
            continue
        if not activations.get(side, False) or not mapper_active.get(side, False):
            result[joints] = measured[joints]
    return result


def latch_new_arm_holds(held, measured, arm_holds, previous_holds):
    """Capture measured joints only when a side enters hold mode."""
    result = np.asarray(held, dtype=float).copy()
    measured = np.asarray(measured, dtype=float)
    for side, joints in (("left", slice(0, 7)), ("right", slice(7, 14))):
        if arm_holds.get(side, False) and not previous_holds.get(side, False):
            result[joints] = measured[joints]
    return result


def disengage_sample_sides(sample: ArmSample | None, sides) -> ArmSample | None:
    if sample is None:
        return None
    activations = dict(sample.activations)
    for side in sides:
        activations[side] = False
    return replace(sample, activations=activations)


class DualFr3HardwareTeleop:
    def __init__(
        self,
        command_host: str,
        command_port: int,
        state_host: str,
        state_port: int,
        state_timeout: float,
        translation_scale: float,
        rotation_scale: float,
        control_rate: float,
        max_joint_speed: float,
        robot_state_wait_timeout: float,
        arm_source: ArmPoseSource,
        operator: OperatorState,
        hands: HandController | None = None,
        debug_logger=None,
        dataset_recorder=None,
        reset_invoker=None,
        capture_home_invoker=None,
        ready_invoker=None,
        ready_to_home_invoker=None,
        preset_actions: dict[str, PresetAction | None] | None = None,
        preset_solver=None,
    ) -> None:
        self.arm_source = arm_source
        self.operator = operator
        self._notify = operator.show
        self.dt = 1.0 / control_rate
        self.robot_state_wait_timeout = robot_state_wait_timeout
        try:
            self.robot = UdpRobotBackend(
                command_host=command_host,
                command_port=command_port,
                state_host=state_host,
                state_port=state_port,
                state_timeout=state_timeout,
            )
        except BaseException:
            if hands is not None:
                hands.close()
            arm_source.close()
            raise
        self.ik = BimanualPinkIK(dt=self.dt, max_joint_speed=max_joint_speed)
        self.joint_input = getattr(arm_source, "output_kind", "pose") == "joint"
        if self.joint_input:
            max_delta_by_side = getattr(arm_source, "max_relative_delta", 0.25)
            max_target_velocity = getattr(
                arm_source, "max_target_velocity", None
            )
            sensitivity_by_side = getattr(arm_source, "joint_sensitivity", {})
            limit_margin_by_side = getattr(arm_source, "joint_limit_margin", {})
            self.mappers = {
                side: RelativeJointMapper(
                    LOWER_LIMITS[index : index + 7],
                    UPPER_LIMITS[index : index + 7],
                    (
                        max_delta_by_side.get(side, np.full(7, 0.25, dtype=float))
                        if isinstance(max_delta_by_side, dict)
                        else max_delta_by_side
                    ),
                    joint_sensitivity=sensitivity_by_side.get(
                        side, np.ones(7, dtype=float)
                    ),
                    max_target_velocity=max_target_velocity,
                    nominal_dt=self.dt,
                    joint_limit_margin=(
                        limit_margin_by_side.get(side, np.zeros(7, dtype=float))
                        if isinstance(limit_margin_by_side, dict)
                        else limit_margin_by_side
                    ),
                )
                for side, index in (("left", 0), ("right", 7))
            }
        else:
            self.mappers = {
                side: RelativePoseMapper(
                    translation_scale=translation_scale,
                    rotation_scale=rotation_scale,
                )
                for side in SIDES
            }
        self.hold_q: np.ndarray | None = None
        self.arm_holds = {side: False for side in SIDES}

        self.hands = hands
        self.debug_logger = debug_logger
        self.dataset_recorder = dataset_recorder
        # Reset to the captured initial pose, requested from the keyboard. The
        # operator process is deliberately ROS-free, so the reset is delegated
        # to a blocking callable (a `ros2 service call` in the container) run on
        # a worker thread; the worker only appends to `reset_outcome`, and every
        # other state change stays on the control thread.
        self.reset_invoker = reset_invoker
        self.reset_thread: threading.Thread | None = None
        self.reset_outcome: list[tuple[bool, str]] = []
        self.reset_preserves_arm_permits = False
        self.capture_home_invoker = capture_home_invoker
        self.ready_invoker = ready_invoker
        self.ready_to_home_invoker = ready_to_home_invoker
        self.capture_thread: threading.Thread | None = None
        self.capture_side: str | None = None
        self.capture_task: str | None = None
        self.capture_outcome: list[tuple[bool, str]] = []
        self.preset_actions = preset_actions or {key: None for key in ("q", "w", "e")}
        self.preset_solver = preset_solver
        self.preset_thread: threading.Thread | None = None
        self.preset_pending: PresetAction | None = None
        self.preset_outcome: list[tuple[bool, object]] = []
        self.preset_cancelled = False
        self.preset_cancel_resume = True
        self.active_preset: SolvedRelativeAction | None = None
        self.preset_started_at: float | None = None
        self.preset_settled_since: float | None = None
        self.preset_resume_active = False
        self.preset_leader_anchor: np.ndarray | None = None

    def _start_reset(
        self, side: str | None = None, task: str = DEFAULT_OPERATOR_TASK
    ) -> None:
        """Both-arm Home retains permits behind a locked Hold. Single-arm
        reset keeps its existing disengagement behavior. The reset trajectory
        exclusively owns the command bus until its measured settling check."""
        if self.reset_invoker is None:
            self._notify("reset requested, but no reset command is configured")
            return
        if (
            self.reset_thread is not None
            or getattr(self, "capture_thread", None) is not None
            or getattr(self, "preset_thread", None) is not None
            or getattr(self, "active_preset", None) is not None
        ):
            self._notify("Home reset/capture is already in progress")
            return
        self.reset_preserves_arm_permits = side is None
        if self.reset_preserves_arm_permits:
            self.operator.begin_arm_home()
        else:
            self.operator.disable_all("resetting to initial pose")
        for mapper in self.mappers.values():
            mapper.reset()
        scope = f"{side} arm" if side else "arms"
        self._notify(f"Home: moving {scope} to {task_label(task)}")

        def worker() -> None:
            try:
                outcome = self.reset_invoker(side, task)
            except Exception as error:  # noqa: BLE001 - report, never crash the loop
                outcome = (False, str(error))
            self.reset_outcome.append(outcome)

        self.reset_thread = threading.Thread(target=worker, daemon=True)
        self.reset_thread.start()


    def _start_episode_home(self, job: dict) -> None:
        # Reuse the existing gateway/reset path and measured settling checks.
        current = self.operator.snapshot().get("episode_home", {})
        if current.get("id") != job["id"] or current.get("state") != "queued":
            return
        if (self.reset_invoker is None or self.reset_thread is not None
                or getattr(self, "capture_thread", None) is not None
                or getattr(self, "preset_thread", None) is not None
                or getattr(self, "active_preset", None) is not None):
            self.operator.update_episode_home(job["id"], "failed", "Home 不可用或另一动作正在执行，请按 B 重试")
            return
        self._episode_home_id = job["id"]
        self.operator.update_episode_home(job["id"], "running", "机械臂正在 Home")
        self._start_reset(task=job["task"])

    def _open_hands(self, sides: tuple[str, ...]) -> None:
        """Stop selected sides following, then stream their open pose."""
        selected = tuple(side for side in SIDES if side in sides)
        if self.hands is None:
            self._notify("hands: not running, start with --hand-source")
            return
        for side in selected:
            self.operator.deny_hand(side, "opening hand")
        self.hands.request_open(sides=selected)
        self._notify("hands: opening " + "/".join(selected))

    def _start_capture_home(
        self, side: str, task: str = DEFAULT_OPERATOR_TASK
    ) -> None:
        """Persist one stopped arm's current measured joints as its Home."""
        if self.capture_home_invoker is None:
            self._notify(
                "Home capture requested, but no capture command is configured"
            )
            return
        if (
            self.reset_thread is not None
            or self.capture_thread is not None
            or getattr(self, "preset_thread", None) is not None
            or getattr(self, "active_preset", None) is not None
        ):
            self._notify("Home reset/capture is already in progress")
            return
        if self.operator.poll().get(side, False):
            self._notify(f"{side} Home capture rejected: stop that arm first")
            return
        self.operator.set_active(side, False, target="arm")
        self.mappers[side].reset()
        self.capture_side = side
        self.capture_task = task
        self._notify(
            f"{side}: recording current measured joints as {task_label(task)} Home"
        )

        def worker() -> None:
            try:
                outcome = self.capture_home_invoker(side, task)
            except Exception as error:  # noqa: BLE001 - report, never crash loop
                outcome = (False, str(error))
            self.capture_outcome.append(outcome)

        self.capture_thread = threading.Thread(target=worker, daemon=True)
        self.capture_thread.start()

    def _service_capture_home(self) -> None:
        if self.capture_thread is None or self.capture_thread.is_alive():
            return
        self.capture_thread.join()
        self.capture_thread = None
        side = self.capture_side
        task = self.capture_task
        self.capture_side = None
        self.capture_task = None
        succeeded, message = (
            self.capture_outcome.pop()
            if self.capture_outcome
            else (False, "no result")
        )
        self._notify(
            f"{side} {task_label(str(task))} Home capture "
            f"{'done' if succeeded else 'FAILED'}: {message}"
        )

    def _start_ready_operation(self, action: str, task: str | None = None) -> None:
        invoker = self.ready_to_home_invoker if action == "trajectory" else self.ready_invoker
        if invoker is None:
            self._notify(f"Ready {action} requested, but no command is configured")
            return
        if (
            self.reset_thread is not None
            or self.capture_thread is not None
            or self.preset_thread is not None
            or self.active_preset is not None
        ):
            self._notify("another Home/Ready/preset operation is already in progress")
            return
        if action == "capture" and any(self.operator.poll().values()):
            self._notify("Ready capture rejected: stop both arms first")
            return
        self.operator.disable_all(f"Ready {action}")
        for mapper in self.mappers.values():
            mapper.reset()
        label = (
            f"Ready to {task_label(str(task))} Home"
            if action == "trajectory"
            else f"Ready {action}"
        )
        self._notify(f"{label}: started")

        def worker() -> None:
            try:
                outcome = invoker(task) if action == "trajectory" else invoker(action)
            except Exception as error:  # noqa: BLE001
                outcome = (False, str(error))
            self.reset_outcome.append(outcome)

        self.reset_thread = threading.Thread(target=worker, daemon=True)
        self.reset_thread.start()

    @staticmethod
    def _side_slice(side: str) -> slice:
        return slice(0, 7) if side == "left" else slice(7, 14)

    def _start_preset(
        self, key: str, measured_q: np.ndarray, sample: ArmSample | None
    ) -> None:
        preset = self.preset_actions.get(key)
        if preset is None:
            self._notify(f"preset {key.upper()}: slot is not configured")
            return
        if self.preset_solver is None:
            self._notify("preset action requested, but MoveIt IK is not configured")
            return
        if (
            self.reset_thread is not None
            or self.capture_thread is not None
            or self.preset_thread is not None
            or self.active_preset is not None
        ):
            self._notify("another Home/preset operation is already in progress")
            return
        if not preset.path.exists():
            self._notify(f"preset {key.upper()}: action file is missing: {preset.path}")
            return
        if not isinstance(sample, JointTeleopSample):
            self._notify("preset actions currently require GELLO joint input")
            return
        side = preset.side
        self.preset_resume_active = self.operator.poll().get(side, False)
        self.operator.set_active(side, False, target="arm")
        self.mappers[side].reset()
        self.preset_leader_anchor = sample.positions[side].copy()
        self.preset_pending = preset
        self.preset_cancelled = False
        self.preset_cancel_resume = True
        self._notify(
            f"preset {key.upper()} ({preset.label}): solving every MoveIt IK frame "
            f"from current measured pose at {preset.speed_scale * 100:.0f}% speed"
        )

        def worker() -> None:
            try:
                outcome = (True, self.preset_solver(preset))
            except Exception as error:  # noqa: BLE001 - report on control thread
                outcome = (False, str(error))
            self.preset_outcome.append(outcome)

        self.preset_thread = threading.Thread(target=worker, daemon=True)
        self.preset_thread.start()

    def _service_preset_solver(self, measured_q: np.ndarray) -> None:
        if self.preset_thread is None or self.preset_thread.is_alive():
            return
        self.preset_thread.join()
        self.preset_thread = None
        preset = self.preset_pending
        succeeded, result = (
            self.preset_outcome.pop()
            if self.preset_outcome
            else (False, "MoveIt IK returned no result")
        )
        if self.preset_cancelled:
            self._finish_preset(
                "IK precheck interrupted", resume=self.preset_cancel_resume
            )
            return
        if not succeeded or not isinstance(result, SolvedRelativeAction):
            self._notify(f"preset IK precheck FAILED: {result}")
            self._finish_preset("IK precheck failed", resume=True)
            return
        assert preset is not None
        if result.side != preset.side:
            self._notify("preset IK precheck FAILED: solved side does not match slot")
            self._finish_preset("IK side mismatch", resume=True)
            return
        joints = self._side_slice(result.side)
        moved = float(np.max(np.abs(measured_q[joints] - result.start_q[joints])))
        if moved > 0.03:
            self._notify(
                f"preset rejected: arm moved {np.degrees(moved):.1f} deg during IK"
            )
            self._finish_preset("arm moved during IK", resume=True)
            return
        self.preset_pending = None
        self.active_preset = result
        self.preset_started_at = time.monotonic()
        self.preset_settled_since = None
        self._notify(
            f"preset IK passed ({len(result.positions)} frames); executing "
            f"{result.name} at {result.speed_scale * 100:.0f}%"
        )

    def _finish_preset(self, reason: str, *, resume: bool) -> None:
        side = None
        if self.active_preset is not None:
            side = self.active_preset.side
        elif self.preset_pending is not None:
            side = self.preset_pending.side
        self.active_preset = None
        self.preset_pending = None
        self.preset_started_at = None
        self.preset_settled_since = None
        self.preset_leader_anchor = None
        self.preset_cancelled = False
        self.preset_cancel_resume = True
        if side is not None:
            self.mappers[side].reset()
            reengage = resume and (self.preset_resume_active or reason == "completed")
            if reengage:
                self.operator.set_active(side, True, target="arm")
                self._notify(f"preset {reason}; {side} GELLO re-anchoring now")
            else:
                self._notify(f"preset {reason}; {side} arm remains stopped")
        self.preset_resume_active = False

    def _stop_preset(self, reason: str, *, resume: bool = True) -> None:
        if self.preset_thread is not None:
            if self.preset_cancelled:
                return
            self.preset_cancelled = True
            self.preset_cancel_resume = resume
            self.preset_leader_anchor = None
            self._notify(f"preset stop requested during IK: {reason}")
            return
        if self.active_preset is not None:
            self._finish_preset(reason, resume=resume)
            return
        self._notify("preset stop requested, but no preset is running")

    def _preset_leader_moved(self, sample: ArmSample | None) -> bool:
        if (
            self.preset_leader_anchor is None
            or not isinstance(sample, JointTeleopSample)
        ):
            return False
        preset = self.active_preset
        side = preset.side if preset is not None else self.preset_pending.side
        delta = float(
            np.max(np.abs(sample.positions[side] - self.preset_leader_anchor))
        )
        return delta >= PRESET_INTERRUPT_DELTA_RAD

    def _service_reset(self) -> None:
        """Fold a finished reset back into the loop, on the control thread."""
        if self.reset_thread is None or self.reset_thread.is_alive():
            return
        self.reset_thread.join()
        self.reset_thread = None
        succeeded, message = (
            self.reset_outcome.pop() if self.reset_outcome else (False, "no result")
        )
        self._notify(f"reset {'done' if succeeded else 'FAILED'}: {message}")
        if getattr(self, "reset_preserves_arm_permits", False):
            self.operator.finish_arm_home(succeeded)
        job_id = getattr(self, "_episode_home_id", None)
        if job_id is not None:
            self.operator.update_episode_home(job_id, "done" if succeeded else "failed", message)
            self._episode_home_id = None
        self.reset_preserves_arm_permits = False
        # The arms are wherever the reset left them, so the pre-reset hold_q is
        # a lie. Dropping it makes the loop re-seed from measured state and
        # re-anchor the IK posture reference before anything can re-engage.
        self.hold_q = None


    @staticmethod
    def _ik_status(worst: dict) -> str:
        """One operator-readable clause per side, worst tick since the last
        report: the IK failure cause, or 'ok' while tracking is transparent."""
        parts = []
        for side in SIDES:
            diagnostics = worst.get(side)
            if diagnostics is None:
                continue
            cause = classify_step(diagnostics)
            if cause == "ok":
                parts.append(f"{side} ok")
                continue
            detail = (
                f"{diagnostics['position_error'] * 1e3:.0f}mm/"
                f"{np.degrees(diagnostics['orientation_error']):.0f}deg"
            )
            if cause == "joint-limit":
                joints = ",".join(
                    f"j{joint}" for joint, _ in diagnostics["limit_joints"]
                )
                parts.append(f"{side} LIMIT {joints} off {detail}")
            elif cause == "speed-clamp":
                parts.append(f"{side} clamped off {detail}")
            else:
                parts.append(f"{side} UNREACHABLE off {detail}")
        return " | ".join(parts)

    def run(self) -> None:
        next_status_report = 0.0
        state_missing_since: float | None = None
        # Worst IK step per side since the last status report; a transient
        # at the report instant must not hide a limit hit seconds earlier.
        ik_worst: dict[str, dict] = {}
        previous_hand_sent = (
            {} if self.hands is None else {side: 0 for side in self.hands.sides}
        )
        status_summary = getattr(self.arm_source, "status_summary", None)
        # Temporary stall probes, enabled by the existing --debug-log launch.
        timing = None
        try:
            debug_path = getattr(self.debug_logger, "path", None)
            if debug_path is not None:
                try:
                    timing = LoopTimingDiagnostics(
                        debug_path.with_name(debug_path.stem + ".loop_timing.jsonl"),
                        session_id=getattr(self.robot, "stream_id", None),
                        period_sec=self.dt,
                    )
                except (OSError, RuntimeError) as error:
                    self._notify(f"Loop timing diagnostics unavailable: {error}")
            # No hardware state is an unavailable subsystem, not a GUI lifecycle failure.
            availability = getattr(self.operator, "set_arm_available", None)
            if self.hands is not None:
                start_hands = getattr(self.hands, "start", None)
                if start_hands is not None:
                    start_hands()
            while True:
                if timing is not None:
                    timing.begin_cycle()
                started_at = time.monotonic()
                # A queued rejection may arrive on the very tick which folds
                # the completed Home worker back into the loop.
                home_owned_bus = (
                    self.reset_thread is not None
                    and self.reset_preserves_arm_permits
                )
                self._service_reset()
                self._service_capture_home()
                if timing is not None:
                    timing.mark("receive_robot_state")
                q = self.robot.receive_state()
                if timing is not None:
                    timing.mark("preset_and_gateway_status")
                if q is None:
                    if self.preset_thread is not None or self.active_preset is not None:
                        self._stop_preset("robot state missing", resume=False)
                    reason = "robot state missing or stale"
                    if state_missing_since is None:
                        state_missing_since = started_at
                        if availability is None:
                            self.operator.disable_all(reason)
                        self.hold_q = None
                        self.arm_holds = {side: False for side in SIDES}
                    if availability is not None:
                        availability(False, "机械臂不可用：状态缺失或过期；请检查连接、Exec 模式及 FCI")
                    missing_for = started_at - state_missing_since
                    self.operator.set_status(
                        f"ARMS UNAVAILABLE | {reason} for {missing_for:.1f}s; 检查连接 / Exec / FCI，修复后在 Connection 菜单重连机械臂"
                    )
                    for mapper in self.mappers.values():
                        mapper.reset()
                    if timing is not None:
                        timing.mark("send_command")
                    self.robot.send_command(
                        self.hold_q if self.hold_q is not None else self.ik.configuration.q,
                        (),
                    )
                    if timing is not None:
                        timing.command_sent(getattr(self.robot, "sequence", 0) - 1, ())
                        timing.mark("missing_state_handling")
                    # The arms are disengaged, so the hands stop following too;
                    # a pending open request still streams.
                    if self.hands is not None:
                        self.hands.set_active(
                            {side: False for side in SIDES}
                        )
                    if timing is not None:
                        timing.mark("loop_wait")
                    time.sleep(self.dt)
                    continue
                if availability is not None:
                    availability(True)
                state_missing_since = None
                self._service_preset_solver(q)
                if self.hold_q is None:
                    self.hold_q = np.asarray(q, dtype=float).copy()
                    # Anchor the IK null-space attractor at the pose the session
                    # started from, normally the captured hardware home.
                    self.ik.set_posture_reference(self.hold_q)
                for fault in self.robot.take_gateway_faults():
                    if home_owned_bus and fault == "reset is active":
                        # Expected ownership rejection of a packet already in
                        # flight at handoff, not a new motion safety failure.
                        continue
                    rejected = tuple(
                        side for side in SIDES if fault.startswith(f"{side} ")
                    )
                    if len(rejected) == 1:
                        self.operator.deny(
                            rejected[0], f"safety gateway: {fault}"
                        )
                    else:
                        self.operator.disable_all(
                            f"safety gateway: {fault}"
                        )
                    if self.preset_thread is not None or self.active_preset is not None:
                        self._stop_preset("safety gateway fault", resume=False)
                if timing is not None:
                    timing.mark("sample_arm_input")
                sample = self.arm_source.sample()
                if timing is not None:
                    timing.mark("operator_requests")
                requests = self.operator.take_requests()
                if requests.get("stop_action"):
                    self._stop_preset("operator STOP")
                if requests.get("abort_action"):
                    self._stop_preset("all followers disengaged", resume=False)
                    if self.hands is not None:
                        self.hands.cancel_pose()
                open_sides = {
                    side
                    for side in SIDES
                    if requests.get("open_hands")
                    or requests.get(f"open_{side}_hand")
                }
                if open_sides:
                    self._open_hands(tuple(open_sides))
                if requests.get("episode_home"):
                    self._start_episode_home(requests["episode_home"])
                elif requests.get("reset"):
                    self._start_reset(task=str(requests["reset"]))
                elif requests.get("reset_left"):
                    self._start_reset("left", str(requests["reset_left"]))
                elif requests.get("reset_right"):
                    self._start_reset("right", str(requests["reset_right"]))
                elif requests.get("capture_home_left"):
                    self._start_capture_home(
                        "left", str(requests["capture_home_left"])
                    )
                elif requests.get("capture_home_right"):
                    self._start_capture_home(
                        "right", str(requests["capture_home_right"])
                    )
                elif requests.get("capture_ready"):
                    self._start_ready_operation("capture")
                elif requests.get("move_ready"):
                    self._start_ready_operation("move")
                elif requests.get("ready_to_home"):
                    self._start_ready_operation(
                        "trajectory", str(requests["ready_to_home"])
                    )
                else:
                    for key in ("q", "w", "e"):
                        if requests.get(f"preset_{key}"):
                            self._start_preset(key, q, sample)
                            break
                if (
                    (self.preset_thread is not None or self.active_preset is not None)
                    and self._preset_leader_moved(sample)
                ):
                    self._stop_preset("GELLO moved more than 0.08 rad")
                if self.reset_thread is not None:
                    # The reset trajectory owns the arms; nothing may engage,
                    # and this tick's sample must not act on stale activations.
                    if not self.reset_preserves_arm_permits:
                        self.operator.disable_all("reset in progress")
                    sample = None
                if self.capture_thread is not None and self.capture_side is not None:
                    self.operator.set_active(
                        self.capture_side, False, target="arm"
                    )
                    sample = disengage_sample_sides(sample, (self.capture_side,))
                preset_side = None
                if self.active_preset is not None:
                    preset_side = self.active_preset.side
                elif self.preset_pending is not None:
                    preset_side = self.preset_pending.side
                if preset_side is not None:
                    self.operator.set_active(preset_side, False, target="arm")
                    sample = disengage_sample_sides(sample, (preset_side,))
                if timing is not None:
                    timing.mark("target_mapping")
                hold_reader = getattr(self.operator, "poll_holds", None)
                arm_holds = (
                    {side: False for side in SIDES}
                    if hold_reader is None
                    else hold_reader()
                )
                # Capture measured joints on the rising edge. Using the last
                # target here could make a lagging robot finish an old move
                # after Hold was pressed, which is not a true stationary hold.
                self.hold_q = latch_new_arm_holds(
                    self.hold_q, q, arm_holds, self.arm_holds
                )
                self.arm_holds = dict(arm_holds)
                if self.reset_thread is None:
                    activations = (
                        {side: False for side in SIDES}
                        if sample is None
                        else sample.activations
                    )
                    self.hold_q = reseed_inactive_joints(
                        self.hold_q,
                        q,
                        activations,
                        {
                            side: self.mappers[side].active
                            for side in SIDES
                        },
                        arm_holds,
                    )
                targets = {}
                if self.joint_input:
                    for side, joints in (
                        ("left", slice(0, 7)),
                        ("right", slice(7, 14)),
                    ):
                        leader = (
                            np.zeros(7)
                            if sample is None
                            else sample.positions[side]
                        )
                        target = self.mappers[side].update(
                            leader,
                            sample is not None and sample.activations[side],
                            q[joints],
                        )
                        if target is not None:
                            self.hold_q[joints] = target
                            targets[side] = target
                    self.ik.last_diagnostics = {}
                else:
                    current_poses = self.ik.frame_poses(self.hold_q)
                    for side in SIDES:
                        current = current_poses[side]
                        if sample is None:
                            self.mappers[side].update(current, False, current)
                            continue
                        target = self.mappers[side].update(
                            sample.poses[side], sample.activations[side], current
                        )
                        if target is not None:
                            targets[side] = target
                    try:
                        if targets:
                            self.hold_q = self.ik.step(self.hold_q, targets)
                    except IKError:
                        self.robot.send_command(q, ())
                        raise
                    for side, diagnostics in self.ik.last_diagnostics.items():
                        worst = ik_worst.get(side)
                        if worst is None or diagnostics["position_error"] > worst[
                            "position_error"
                        ]:
                            ik_worst[side] = diagnostics
                # Holding is intentionally still an active command: preserve
                # hold_q and include that side in every outgoing packet. The
                # mapper was disengaged above, so GELLO motion is ignored and
                # a later Start re-anchors instead of jumping.
                for side, joints in (
                    ("left", slice(0, 7)),
                    ("right", slice(7, 14)),
                ):
                    if arm_holds.get(side, False):
                        targets[side] = self.hold_q[joints].copy()
                finish_preset_after_send: tuple[str, bool] | None = None
                if self.active_preset is not None:
                    assert self.preset_started_at is not None
                    solution = self.active_preset
                    elapsed = time.monotonic() - self.preset_started_at
                    joints = self._side_slice(solution.side)
                    if elapsed < PRESET_LEAD_IN_SEC:
                        blend = elapsed / PRESET_LEAD_IN_SEC
                        action_target = (
                            solution.start_q[joints]
                            + blend
                            * (solution.positions[0] - solution.start_q[joints])
                        )
                    else:
                        action_target = sample_solved_action(
                            solution, elapsed - PRESET_LEAD_IN_SEC
                        )
                    self.hold_q[joints] = action_target
                    targets[solution.side] = action_target
                    scheduled_end = PRESET_LEAD_IN_SEC + solution.duration_sec
                    if elapsed >= scheduled_end:
                        final_error = float(
                            np.max(np.abs(q[joints] - solution.positions[-1]))
                        )
                        if final_error <= PRESET_SETTLE_TOLERANCE_RAD:
                            self.preset_settled_since = (
                                self.preset_settled_since or time.monotonic()
                            )
                            if (
                                time.monotonic() - self.preset_settled_since
                                >= PRESET_SETTLE_HOLD_SEC
                            ):
                                finish_preset_after_send = ("completed", True)
                        else:
                            self.preset_settled_since = None
                        if elapsed >= scheduled_end + PRESET_SETTLE_TIMEOUT_SEC:
                            self._notify(
                                "preset FAILED: final pose did not settle within "
                                f"{PRESET_SETTLE_TIMEOUT_SEC:.0f}s "
                                f"({np.degrees(final_error):.1f} deg max error)"
                            )
                            finish_preset_after_send = (
                                "final pose did not settle",
                                False,
                            )
                # Locked Hold is visible to the operator, but must not contend
                # with the Home trajectory's exclusive command ownership.
                active_sides = tuple(side for side in SIDES if side in targets)
                if timing is not None:
                    timing.mark("send_command")
                if self.reset_thread is None:
                    self.robot.send_command(self.hold_q, active_sides)
                if timing is not None and self.reset_thread is None:
                    timing.command_sent(getattr(self.robot, "sequence", 0) - 1, active_sides)
                if finish_preset_after_send is not None:
                    reason, resume = finish_preset_after_send
                    self._finish_preset(reason, resume=resume)
                if timing is not None:
                    timing.mark("debug_log_and_fk")
                if self.debug_logger is not None:
                    raw_pose_reader = getattr(
                        self.arm_source, "debug_raw_poses", None
                    )
                    raw_poses = (
                        raw_pose_reader() if raw_pose_reader is not None else {}
                    )
                    feed_reader = getattr(
                        self.arm_source, "debug_feed_state", None
                    )
                    pose_sample = sample if isinstance(sample, TeleopSample) else None
                    pose_targets = {} if self.joint_input else targets
                    self.debug_logger.record(
                        time.monotonic(),
                        q,
                        self.hold_q,
                        {} if pose_sample is None else pose_sample.poses,
                        {} if sample is None else sample.activations,
                        pose_targets,
                        self.ik.frame_poses(self.hold_q),
                        raw_tracker_poses=raw_poses,
                        measured_ee_poses=self.ik.frame_poses(q),
                        ik_diagnostics=self.ik.last_diagnostics,
                        feed_state=(
                            feed_reader() if feed_reader is not None else None
                        ),
                    )
                if timing is not None:
                    timing.mark("hand_activation")
                hand_activations = {side: False for side in SIDES}
                if self.hands is not None:
                    hand_activation_reader = getattr(
                        self.operator, "poll_hands", self.operator.poll
                    )
                    hand_activations = hand_activation_reader()
                    self.hands.set_active(hand_activations)
                if timing is not None:
                    timing.mark("optional_dataset_recorder")
                if self.dataset_recorder is not None:
                    try:
                        hand_feedback = (
                            None
                            if self.hands is None
                            else self.hands.feedback_snapshot("left")
                        )
                        recorded_at = time.monotonic()
                        self.dataset_recorder.record(
                            wall_time_ns=time.time_ns(),
                            monotonic_time_s=recorded_at,
                            arm_q=np.asarray(q, dtype=float)[:7],
                            hand_feedback=hand_feedback,
                            ee_pose=self.ik.named_frame_pose(
                                q, "left_fr3v2_link8"
                            ),
                            arm_active=(
                                sample is not None
                                and bool(sample.activations.get("left", False))
                            ),
                            hand_active=bool(hand_activations.get("left", False)),
                        )
                    except Exception as error:  # noqa: BLE001 - never stop control
                        print(f"dataset recording disabled: {error}")
                        try:
                            self.dataset_recorder.close()
                        except Exception as close_error:  # noqa: BLE001
                            print(f"dataset finalization FAILED: {close_error}")
                        self.dataset_recorder = None
                if timing is not None:
                    timing.mark("status_update")
                now = time.monotonic()
                if now >= next_status_report:
                    input_summary = (
                        status_summary() if status_summary is not None else ""
                    )
                    source_name = getattr(self.arm_source, "source_name", "trackers")
                    parts = (
                        [f"{source_name}: {input_summary}"] if input_summary else []
                    )
                    ik_summary = self._ik_status(ik_worst)
                    if ik_summary:
                        parts.append(f"ik: {ik_summary}")
                    ik_worst = {}
                    if self.hands is not None:
                        hand_parts = []
                        for side in self.hands.sides:
                            status = self.hands.status.sides[side]
                            sent = status.sent - previous_hand_sent[side]
                            previous_hand_sent[side] = status.sent
                            state = (
                                f"sending {sent:.0f}Hz"
                                if status.sending
                                else f"stopped ({status.fault})"
                            )
                            hand_parts.append(f"{side}={state}")
                        parts.append("hands: " + " | ".join(hand_parts))
                    if parts:
                        line = "STATE | " + " | ".join(parts)
                        self.operator.set_status(line)
                    next_status_report = now + 1.0
                if timing is not None:
                    timing.mark("loop_wait")
                remaining = self.dt - (time.monotonic() - started_at)
                if remaining > 0.0:
                    time.sleep(remaining)
        finally:
            if timing is not None:
                timing.end_cycle()
            if self.dataset_recorder is not None:
                try:
                    self.dataset_recorder.close()
                    print(
                        f"dataset saved: {self.dataset_recorder.output} "
                        f"({self.dataset_recorder.sample_count} samples, "
                        f"{self.dataset_recorder.dropped_samples} dropped)"
                    )
                except Exception as error:  # noqa: BLE001 - close hardware first
                    print(f"dataset finalization FAILED: {error}")
            # Close the hand pipeline before the SDK client it reads from.
            try:
                if self.hands is not None:
                    self.hands.close()
            finally:
                try:
                    self.robot.close()
                finally:
                    try:
                        self.arm_source.close()
                    finally:
                        try:
                            if self.debug_logger is not None:
                                self.debug_logger.close()
                        finally:
                            if timing is not None:
                                timing.close()
