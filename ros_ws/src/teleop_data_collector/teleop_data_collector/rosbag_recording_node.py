from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import sys
import time
import tempfile
import json
import uuid
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node

from .config import load_collector_config
from .collector_contract import TopicContract, recording_outcome, topic_preflight_failures
from .bag_validation import inspect_bag
from .collector_control import CollectorController, CollectorControlServer
from .keyboard import KeyboardInterface
from .quality_storage import allocate_episode, archive_episode


class RosbagDataCollectorNode(Node):
    def __init__(self):
        super().__init__(
            "teleop_data_collector",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.config = load_collector_config(self)
        self.output_dir = Path(_get_string_parameter(self, "output_dir", self.config.data_root))
        self.bag_prefix = _get_string_parameter(self, "bag_prefix", "episode")
        self.quality_dir = Path(_get_string_parameter(
            self, "quality_dir", "/home/user/franka_teleop_data/数据分类"
        ))
        self.stop_timeout_sec = _get_positive_float_parameter(
            self,
            "bag_stop_timeout_sec",
            30.0,
        )
        self.rosbag_record_args = _get_string_list_parameter(self, "rosbag_record_args", [])
        self.rosbag_record_default_qos = _get_parameter_prefix_values(
            self,
            "rosbag_record_default_qos",
        )
        self.control_bind_host = _get_string_parameter(
            self, "control_bind_host", "127.0.0.1"
        )
        self.control_port = _get_positive_int_parameter(self, "control_port", 5592)
        if self.control_bind_host != "127.0.0.1":
            raise ValueError("control_bind_host must be 127.0.0.1")

        configured_topics = (*self.config.topics, *self.config.static_topics)
        self.topic_names = tuple(dict.fromkeys(topic.topic for topic in configured_topics))
        if not self.topic_names:
            raise ValueError("No topics configured for rosbag recording.")

        for topic in self.config.topics:
            self.get_logger().info(f"Bagging {topic.name}: {topic.topic} [{topic.type_name}]")
        for topic in self.config.static_topics:
            self.get_logger().info(
                f"Bagging static {topic.name}: {topic.topic} [{topic.type_name}]"
            )

    def preflight_failures(self) -> tuple[str, ...]:
        available = dict(self.get_topic_names_and_types())
        configured = tuple(
            TopicContract(topic.name, topic.topic, topic.type_name, topic.required)
            for topic in self.config.topics
        )
        return topic_preflight_failures(configured, available)

    def postflight(
        self, bag_dir: Path
    ) -> tuple[tuple[str, ...], dict[str, Any]]:
        return inspect_bag(
            bag_dir,
            self.config.topics,
            trim_start_sec=self.config.trim_start_sec,
            trim_end_sec=self.config.trim_end_sec,
            continuity_enabled=self.config.continuity_enabled,
        )


class RosbagEpisodeRecorder:
    def __init__(
        self,
        node: RosbagDataCollectorNode,
        output_dir: Path,
        bag_prefix: str,
        topics: tuple[str, ...],
        stop_timeout_sec: float,
        rosbag_record_args: tuple[str, ...],
        default_qos: dict[str, Any] | None = None,
        provenance: dict[str, Any] | None = None,
        preflight=None,
        postflight=None,
    ):
        self._node = node
        self._output_dir = output_dir
        self._bag_prefix = bag_prefix
        self._topics = topics
        self._stop_timeout_sec = stop_timeout_sec
        self._rosbag_record_args = rosbag_record_args
        self._default_qos = default_qos or {}
        self._provenance = dict(provenance or {})
        self._preflight = preflight
        self._postflight = postflight
        self.on_recording_stopped = lambda: None
        self._process: subprocess.Popen[bytes] | None = None
        self._current_bag_dir: Path | None = None
        self._generated_qos_path: Path | None = None
        self._state_path: Path | None = None
        self._last_bag_dir: Path | None = None
        self._validation_report: dict[str, Any] = {}
        self._milestones: list[dict[str, Any]] = []
        self._source_recording_id: str | None = None
        self._live_continuity_report: dict[str, Any] = {}

    @property
    def active(self) -> bool:
        return self._process is not None and self._process.poll() is None

    @property
    def current_bag_dir(self) -> Path | None:
        return self._current_bag_dir

    @property
    def last_bag_dir(self) -> Path | None:
        return self._last_bag_dir

    @property
    def milestones(self) -> list[dict[str, Any]]:
        return [dict(marker) for marker in self._milestones]

    def mark_milestone(self) -> dict[str, Any]:
        if not self.active or self._current_bag_dir is None:
            raise RuntimeError("no episode is currently recording")
        timestamp_ns = int(self._node.get_clock().now().nanoseconds)
        if timestamp_ns <= 0 or (
            self._milestones and timestamp_ns <= self._milestones[-1]["timestamp_ns"]
        ):
            raise RuntimeError("milestone ROS timestamps must be positive and increasing")
        marker = {
            "id": f"milestone_{len(self._milestones) + 1}",
            "timestamp_ns": timestamp_ns,
            "clock": "ros",
        }
        self._milestones.append(marker)
        try:
            self._write_state(
                self._current_bag_dir, state="recording", finalized=False, failures=[]
            )
        except Exception:
            self._milestones.pop()
            raise
        return dict(marker)

    def start(self, *, archive_parent: Path | None = None) -> Path:
        if self.active:
            raise RuntimeError("rosbag recording is already active.")
        ros2 = shutil.which("ros2")
        if ros2 is None:
            raise RuntimeError("Could not find the 'ros2' executable in PATH.")
        failures = tuple(self._preflight() if self._preflight is not None else ())
        if failures:
            raise RuntimeError("Recording preflight failed: " + "; ".join(failures))

        self._output_dir.mkdir(parents=True, exist_ok=True)
        bag_dir = self._next_episode_dir(archive_parent)
        self._milestones = []
        self._source_recording_id = uuid.uuid4().hex
        self._validation_report = {}
        self._live_continuity_report = {}
        self._last_bag_dir = bag_dir
        self._state_path = bag_dir.parent / f".{bag_dir.name}.collection_state.json"
        self._write_state(
            bag_dir,
            state="recording",
            finalized=False,
            failures=[],
        )
        rosbag_record_args = list(self._rosbag_record_args)
        if self._default_qos and not _has_qos_overrides_arg(rosbag_record_args):
            self._generated_qos_path = _write_topic_qos_overrides_file(
                topics=self._topics,
                qos=self._default_qos,
            )
            rosbag_record_args.extend(
                ["--qos-profile-overrides-path", str(self._generated_qos_path)]
            )

        command = [
            ros2,
            "bag",
            "record",
            "--output",
            str(bag_dir),
            *rosbag_record_args,
            *self._topics,
        ]
        self._node.get_logger().info("Starting rosbag record:")
        self._node.get_logger().info(" ".join(command))
        try:
            self._process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception:
            self._remove_generated_qos_file()
            self._write_state(
                bag_dir,
                state="interrupted",
                finalized=False,
                failures=["failed to start ros2 bag record"],
            )
            raise
        self._current_bag_dir = bag_dir
        return bag_dir

    def set_live_continuity(self, report: dict[str, Any]) -> None:
        self._live_continuity_report = report

    def stop(self, *, interrupted: bool = False) -> Path | None:
        process = self._process
        bag_dir = self._current_bag_dir
        if process is None:
            return bag_dir

        if process.poll() is not None:
            self._node.get_logger().warn(
                f"ros2 bag record already exited with code {process.returncode}."
            )
            self._process = None
            self._current_bag_dir = None
            self._remove_generated_qos_file()
            self.on_recording_stopped()
            self._write_state(
                bag_dir,
                state="interrupted",
                finalized=False,
                failures=[f"ros2 bag record exited with code {process.returncode}"],
            )
            return bag_dir

        self._node.get_logger().info("Stopping rosbag record. Waiting for bag finalization...")
        _signal_process_group(process, signal.SIGINT)
        try:
            process.wait(timeout=self._stop_timeout_sec)
        except subprocess.TimeoutExpired:
            self._node.get_logger().warn(
                "ros2 bag record did not exit after SIGINT; sending SIGTERM."
            )
            _signal_process_group(process, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._node.get_logger().error("ros2 bag record did not exit; sending SIGKILL.")
                _signal_process_group(process, signal.SIGKILL)
                process.wait()

        self._process = None
        self._current_bag_dir = None
        self._remove_generated_qos_file()
        self.on_recording_stopped()
        failures = ()
        validation_report: dict[str, Any] = {}
        if self._postflight is not None and int(process.returncode or 0) == 0:
            validation_started = time.monotonic()
            self._node.get_logger().info(f"Validating recorded messages: {bag_dir}")
            try:
                failures, validation_report = self._postflight(bag_dir)
            except Exception as error:  # a validator failure cannot finalize
                failures = (f"post-recording validation failed: {error}",)
            self._node.get_logger().info(
                f"Validation finished in {time.monotonic() - validation_started:.2f}s."
            )
        failures = tuple(failures) + tuple(self._live_continuity_report.get("failures", []))
        self._validation_report = validation_report
        outcome = recording_outcome(
            exit_code=int(process.returncode or 0),
            stream_failures=failures,
            interrupted=interrupted,
        )
        if not outcome.finalized:
            self._node.get_logger().warn(
                f"Bag is {outcome.state}; check {bag_dir}."
            )
        for failure in outcome.failures:
            self._node.get_logger().error(f"[validation FAILED] {failure}")
        for warning in validation_report.get("source_warnings", ()):
            self._node.get_logger().warn(f"[source warning] {warning}")
        boundary_warnings = validation_report.get("boundary_warnings", ())
        transport_warnings = validation_report.get("transport_warnings", ())
        for warning in boundary_warnings:
            self._node.get_logger().warn(f"[boundary warning] {warning}")
        if transport_warnings and outcome.finalized and not boundary_warnings:
            self._node.get_logger().info(
                f"[transport info] Validation passed; {len(transport_warnings)} "
                f"receive-timing warnings. Details: {bag_dir / 'collection_state.json'}"
            )
        else:
            for warning in transport_warnings:
                self._node.get_logger().warn(f"[transport warning] {warning}")
        self._write_state(
            bag_dir,
            state=outcome.state,
            finalized=outcome.finalized,
            failures=list(outcome.failures),
            validation_report=validation_report,
        )
        if outcome.finalized:
            self._node.get_logger().info(
                _terminal_success(f"Bag saved to {bag_dir}.")
            )
        return bag_dir

    def archive_quality(self, parent: Path, quality: str) -> Path:
        if self._process is not None or self._current_bag_dir is not None:
            raise RuntimeError("请等待写盘和校验完成后归档")
        if self._last_bag_dir is None:
            raise RuntimeError("没有可归档的数据")
        destination = archive_episode(self._last_bag_dir, parent, quality)
        self._last_bag_dir = destination
        self._state_path = destination / "collection_state.json"
        return destination

    def mark_discarded(self) -> Path | None:
        if self.active:
            self.stop()
        bag_dir = self._current_bag_dir or self._last_bag_dir
        if bag_dir is not None:
            outcome = recording_outcome(exit_code=0, discarded=True)
            self._write_state(
                bag_dir,
                state=outcome.state,
                finalized=outcome.finalized,
                failures=list(outcome.failures),
                validation_report=self._validation_report,
            )
        return bag_dir

    def _write_state(
        self,
        bag_dir: Path,
        *,
        state: str,
        finalized: bool,
        failures: list[str],
        validation_report: dict[str, Any] | None = None,
    ) -> None:
        report = validation_report or {}
        live = self._live_continuity_report
        # Persist the fault even on unexpected exit/discard and future revalidation.
        failures = list(dict.fromkeys([*failures, *live.get("failures", [])]))
        if live.get("auto_stop_reason") and state in {"finalized", "interrupted"}:
            state, finalized = "incomplete", False
        payload = {
            **self._provenance,
            "source_bag": str(bag_dir.resolve()),
            "source_recording_id": self._source_recording_id,
            "milestones": self.milestones,
            "source_topic_contract": self._provenance.get("topics", {}),
            "state": state,
            "finalized": finalized,
            "failures": failures,
            "validation_report": report,
            "live_continuity": live,
            "auto_stop_reason": live.get("auto_stop_reason"),
            "source_warnings": list(dict.fromkeys([*report.get("source_warnings", []), *live.get("source_warnings", [])])),
            "validation_policy": "source_header_continuity_trimmed_boundary_v3",
            "boundary_warnings": list(report.get("boundary_warnings") or []),
            "transport_warnings": list(report.get("transport_warnings") or []),
            "updated_at_ns": time.time_ns(),
        }
        sidecar = self._state_path or self._output_dir / f".{bag_dir.name}.collection_state.json"
        sidecar.parent.mkdir(parents=True, exist_ok=True)
        temporary_sidecar = sidecar.with_name(sidecar.name + ".tmp")
        with temporary_sidecar.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_sidecar, sidecar)
        if bag_dir.is_dir():
            final_path = bag_dir / "collection_state.json"
            os.replace(sidecar, final_path)
            self._state_path = final_path
        else:
            self._state_path = sidecar

    def _next_episode_dir(self, archive_parent: Path | None = None) -> Path:
        return allocate_episode(self._output_dir, self._bag_prefix, archive_parent)

    def _remove_generated_qos_file(self) -> None:
        qos_path = self._generated_qos_path
        self._generated_qos_path = None
        if qos_path is None:
            return
        try:
            qos_path.unlink(missing_ok=True)
        except OSError as exc:
            self._node.get_logger().warn(f"Failed to remove temporary QoS file {qos_path}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = RosbagDataCollectorNode()
    recorder = RosbagEpisodeRecorder(
        node=node,
        output_dir=node.output_dir,
        bag_prefix=node.bag_prefix,
        topics=node.topic_names,
        stop_timeout_sec=node.stop_timeout_sec,
        rosbag_record_args=tuple(node.rosbag_record_args),
        default_qos=node.rosbag_record_default_qos,
        provenance=node.config.provenance,
        preflight=node.preflight_failures,
        postflight=node.postflight,
    )
    live = None
    if node.config.continuity_enabled or node.config.rgb_preflight_enabled:
        from .live_continuity import LiveContinuity
        live = LiveContinuity(node, recording_active=lambda: recorder.active)
        if node.config.continuity_enabled:
            node.continuity_monitor = live.monitor
        node.collection_readiness = live.readiness
    controller = CollectorController(node, recorder)
    control_server = CollectorControlServer(
        (node.control_bind_host, node.control_port), controller
    )
    control_server.start_in_thread()
    if live is not None:
        live.start(node, controller)

    try:
        with KeyboardInterface() as keyboard:
            node.get_logger().info(f"Output directory: {node.output_dir}")
            node.get_logger().info(f"Configured topics: {len(node.topic_names)}")
            node.get_logger().info(
                f"UI control: {node.control_bind_host}:{node.control_port}"
            )
            if not keyboard.enabled:
                node.get_logger().warn("stdin is not a TTY; keyboard shortcuts are disabled.")
            if _wait_for_required_topics(node):
                controller.set_ready()
                _log_ready_banner(node)

            while rclpy.ok():
                key = keyboard.get_key()
                if key in {"l", "L"}:
                    _toggle_recording(node, controller)
                elif key in {"d", "D"}:
                    try:
                        controller.discard()
                    except RuntimeError as error:
                        node.get_logger().warn(str(error))
                elif key == " ":
                    try:
                        controller.mark_milestone()
                    except (RuntimeError, OSError) as error:
                        node.get_logger().warn(str(error))
                if recorder.current_bag_dir is not None and not recorder.active:
                    controller.reconcile_exited_recorder()
                time.sleep(0.05)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested.")
    finally:
        if recorder.current_bag_dir is not None:
            node.get_logger().info(
                "Shutting down. Stopping active rosbag recording..."
            )
            controller.stop_if_present(interrupted=True)
        else:
            node.get_logger().info("Shutting down.")
        controller.wait_for_finalization()
        if live is not None:
            live.close()
        control_server.shutdown()
        control_server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _toggle_recording(
    node: RosbagDataCollectorNode,
    controller: CollectorController,
) -> None:
    if controller.status()["active"]:
        controller.dispatch("stop")
        _log_ready_banner(node)
        return

    controller.start()


def _log_ready_banner(node: RosbagDataCollectorNode) -> None:
    node.get_logger().info("============================================================")
    node.get_logger().info("READY: 采集器已准备好，可以开始录制。")
    node.get_logger().info("  L      开始录制 / 停止并校验当前 episode")
    node.get_logger().info("  SPACE  记录中间完成标记（继续录制）")
    node.get_logger().info("  D      将最近的 episode 标记为 discarded（不会删除）")
    node.get_logger().info("  Ctrl-C 退出采集程序；录制中退出会标记为 interrupted")
    node.get_logger().info("============================================================")


def _wait_for_required_topics(
    node: RosbagDataCollectorNode,
    *,
    stable_sec: float = 2.0,
    poll_sec: float = 0.1,
    status_interval_sec: float = 5.0,
) -> bool:
    """Wait until every required topic has passed preflight continuously.

    Camera drivers continue printing initialization messages after the collector
    process starts.  Keeping the operator banner behind this gate makes READY
    mean that the complete configured input graph is present, rather than only
    that the collector process itself has entered its keyboard loop.
    """
    node.get_logger().info(
        "WAITING: 正在等待配置的机械臂、手和相机数据源准备完成..."
    )
    ready_since: float | None = None
    last_status_at = float("-inf")
    previous_failures: tuple[str, ...] | None = None

    while rclpy.ok():
        failures = node.preflight_failures()
        now = time.monotonic()
        if failures:
            ready_since = None
            if (
                failures != previous_failures
                or now - last_status_at >= status_interval_sec
            ):
                node.get_logger().info(
                    "Still waiting for required topics: " + "; ".join(failures)
                )
                last_status_at = now
        else:
            if ready_since is None:
                ready_since = now
                node.get_logger().info(
                    f"All required topics found; confirming stability for {stable_sec:.1f}s..."
                )
            elif now - ready_since >= stable_sec:
                return True

        previous_failures = failures
        time.sleep(poll_sec)
    return False


def _signal_process_group(process: subprocess.Popen[bytes], sig: signal.Signals) -> None:
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        return


def _get_string_parameter(node: Node, name: str, default: str) -> str:
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    value = str(node.get_parameter(name).value or "").strip()
    if not value:
        raise ValueError(f"ROS parameter '{name}' must be a non-empty string.")
    return value


def _get_positive_int_parameter(node: Node, name: str, default: int) -> int:
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    value = int(node.get_parameter(name).value)
    if value <= 0 or value > 65_535:
        raise ValueError(f"ROS parameter '{name}' must be in 1..65535.")
    return value


def _terminal_success(text: str, *, color_enabled: bool | None = None) -> str:
    """Render a successful operator-facing message green on an attached TTY."""
    if color_enabled is None:
        color_enabled = sys.stdout.isatty() or sys.stderr.isatty()
    if not color_enabled:
        return text
    return f"\033[1;32m{text}\033[0m"


def _get_positive_float_parameter(node: Node, name: str, default: float) -> float:
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    value = float(node.get_parameter(name).value)
    if value <= 0.0:
        raise ValueError(f"ROS parameter '{name}' must be positive.")
    return value


def _get_string_list_parameter(node: Node, name: str, default: list[str]) -> tuple[str, ...]:
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"ROS parameter '{name}' must be a string list.")
    return tuple(_clean_option(item, name) for item in value)


def _get_parameter_prefix_values(node: Node, prefix: str) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for name, parameter in node.get_parameters_by_prefix(prefix).items():
        if not name:
            continue
        values[name] = parameter.value
    return values


def _clean_option(value: Any, parameter_name: str) -> str:
    option = str(value or "").strip()
    if not option:
        raise ValueError(f"ROS parameter '{parameter_name}' cannot contain empty strings.")
    return option


def _has_qos_overrides_arg(args: list[str]) -> bool:
    return "--qos-profile-overrides-path" in args


def _write_topic_qos_overrides_file(
    *,
    topics: tuple[str, ...],
    qos: dict[str, Any],
) -> Path:
    with tempfile.NamedTemporaryFile(
        "w",
        prefix="teleop_rosbag_qos_",
        suffix=".yaml",
        delete=False,
    ) as file:
        file.write(_topic_qos_overrides_yaml(topics=topics, qos=qos))
        return Path(file.name)


def _topic_qos_overrides_yaml(
    *,
    topics: tuple[str, ...],
    qos: dict[str, Any],
) -> str:
    if not topics:
        raise ValueError("Cannot generate rosbag QoS overrides without topics.")
    if not qos:
        raise ValueError("Cannot generate rosbag QoS overrides without QoS settings.")

    lines: list[str] = []
    for topic in topics:
        clean_topic = _clean_option(topic, "topics")
        lines.append(f"{clean_topic}:")
        for key in sorted(qos):
            clean_key = _clean_qos_key(key)
            lines.append(f"  {clean_key}: {_format_yaml_scalar(qos[key])}")
    lines.append("")
    return "\n".join(lines)


def _clean_qos_key(value: Any) -> str:
    key = str(value or "").strip()
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
        raise ValueError(f"Invalid QoS override key: {value!r}")
    return key


def _format_yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return str(value)
    text = str(value or "").strip()
    if not text:
        raise ValueError("QoS override values cannot be empty.")
    return text


if __name__ == "__main__":
    main()
