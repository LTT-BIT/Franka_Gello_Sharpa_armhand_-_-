import json
import socket
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")

from teleop_data_collector import rosbag_recording_node
from teleop_data_collector.rosbag_recording_node import (
    _has_qos_overrides_arg,
    _terminal_success,
    _topic_qos_overrides_yaml,
    _wait_for_required_topics,
)
from teleop_data_collector.collector_control import (
    CollectorController,
    CollectorControlServer,
)


class _Logger:
    def __init__(self):
        self.messages = []
        self.entries = []

    def info(self, message):
        self.messages.append(message)
        self.entries.append(("info", message))

    def warn(self, message):
        self.messages.append(message)
        self.entries.append(("warn", message))

    def error(self, message):
        self.messages.append(message)
        self.entries.append(("error", message))


class _Node:
    def __init__(self, failures):
        self._failures = iter(failures)
        self.calls = 0
        self.logger = _Logger()
        self.now_ns = 5_000_000_000

    def preflight_failures(self):
        self.calls += 1
        return next(self._failures)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))


class _Recorder:
    def __init__(self, root: Path):
        self.root = root
        self.active = False
        self.current_bag_dir = None
        self.last_bag_dir = None
        self.milestones = []

    def mark_milestone(self):
        if not self.active:
            raise RuntimeError("no episode is currently recording")
        marker = {"id": "milestone_1", "timestamp_ns": 5_000_000_000, "clock": "ros"}
        self.milestones.append(marker)
        return marker

    def start(self):
        self.current_bag_dir = self.root / "episode0"
        self.current_bag_dir.mkdir()
        self.last_bag_dir = self.current_bag_dir
        self.active = True
        return self.current_bag_dir

    def stop(self, interrupted=False):
        self.active = False
        state = {
            "state": "interrupted" if interrupted else "finalized",
            "finalized": not interrupted,
            "failures": [],
            "transport_warnings": ["synthetic warning"],
        }
        (self.current_bag_dir / "collection_state.json").write_text(
            json.dumps(state), encoding="utf-8"
        )
        bag = self.current_bag_dir
        self.current_bag_dir = None
        return bag

    def mark_discarded(self):
        state = {
            "state": "discarded",
            "finalized": False,
            "failures": [],
        }
        (self.last_bag_dir / "collection_state.json").write_text(
            json.dumps(state), encoding="utf-8"
        )
        return self.last_bag_dir


def test_default_qos_expands_to_each_configured_topic():
    yaml_text = _topic_qos_overrides_yaml(
        topics=("/cam0/color/image_raw", "/cam0/depth/image_raw"),
        qos={"history": "keep_last", "depth": 100},
    )

    assert "/cam0/color/image_raw:" in yaml_text
    assert "/cam0/depth/image_raw:" in yaml_text
    assert yaml_text.count("history: keep_last") == 2
    assert yaml_text.count("depth: 100") == 2


def test_milestones_persist_without_stopping_and_reset_for_next_bag(tmp_path, monkeypatch):
    node = _Node([])
    processes = []

    def popen(*args, **kwargs):
        process = SimpleNamespace(poll=lambda: None, wait=lambda **kw: 0, returncode=0)
        processes.append(process)
        return process

    monkeypatch.setattr(rosbag_recording_node.shutil, "which", lambda _: "/usr/bin/ros2")
    monkeypatch.setattr(rosbag_recording_node.subprocess, "Popen", popen)
    monkeypatch.setattr(rosbag_recording_node, "_signal_process_group", lambda *_: None)
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        node, tmp_path, "episode", (), 30.0, (), postflight=lambda _: ((), {}),
    )
    with pytest.raises(RuntimeError, match="no episode"):
        recorder.mark_milestone()
    bag = recorder.start()
    # rosbag creates its directory asynchronously: markers must persist even
    # before it exists, without pre-creating it and breaking ros2 bag record.
    marker = recorder.mark_milestone()
    sidecar = tmp_path / ".episode0.collection_state.json"
    assert json.loads(sidecar.read_text())["milestones"] == [marker]
    assert not bag.exists()
    bag.mkdir()
    node.now_ns += 1_000_000_000
    second = recorder.mark_milestone()
    state = json.loads((bag / "collection_state.json").read_text())
    assert state["milestones"] == [marker, second]
    assert state["state"] == "recording"
    assert recorder.active
    assert len(processes) == 1
    with pytest.raises(RuntimeError, match="increasing"):
        recorder.mark_milestone()
    recorder.stop()
    finalized = json.loads((bag / "collection_state.json").read_text())
    assert finalized["milestones"] == [marker, second]
    assert finalized["finalized"] is True
    recorder.mark_discarded()
    assert json.loads((bag / "collection_state.json").read_text())["milestones"] == [marker, second]
    recorder.start()
    assert recorder.milestones == []
    assert recorder._source_recording_id != state["source_recording_id"]


def test_failed_milestone_write_does_not_report_success(tmp_path, monkeypatch):
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        _Node([]), tmp_path, "episode", (), 30.0, (),
    )
    recorder._current_bag_dir = tmp_path / "episode0"
    recorder._process = SimpleNamespace(poll=lambda: None)

    def fail(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(recorder, "_write_state", fail)
    with pytest.raises(OSError, match="disk full"):
        recorder.mark_milestone()
    assert recorder.milestones == []


def test_explicit_qos_overrides_arg_is_preserved():
    assert _has_qos_overrides_arg(
        ["--qos-profile-overrides-path", "custom_qos.yaml"]
    )
    assert not _has_qos_overrides_arg(["--storage-preset-profile", "fastwrite"])


def test_explicit_qos_file_prevents_generated_global_override(tmp_path, monkeypatch):
    commands = []

    def popen(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(poll=lambda: None, wait=lambda **kw: 0, returncode=0)

    monkeypatch.setattr(rosbag_recording_node.shutil, "which", lambda _: "/usr/bin/ros2")
    monkeypatch.setattr(rosbag_recording_node.subprocess, "Popen", popen)
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        _Node([]),
        tmp_path,
        "episode",
        ("/cam0/color/image_raw",),
        30.0,
        ("--qos-profile-overrides-path", "/config/per_topic.yaml"),
        default_qos={"reliability": "best_effort"},
    )

    recorder.start()

    assert commands[0].count("--qos-profile-overrides-path") == 1
    qos_flag = commands[0].index("--qos-profile-overrides-path")
    assert commands[0][qos_flag + 1] == "/config/per_topic.yaml"
    assert recorder._generated_qos_path is None


def test_success_message_is_green_only_for_an_attached_terminal():
    text = "Bag saved to /collection_data/bags/gello/episode165."
    assert _terminal_success(text, color_enabled=True) == (
        f"\033[1;32m{text}\033[0m"
    )
    assert _terminal_success(text, color_enabled=False) == text


@pytest.mark.parametrize("validator_raises", [False, True])
def test_stop_prints_validation_details_and_preserves_failed_state(tmp_path, monkeypatch, validator_raises):
    node = _Node([])

    def postflight(_path):
        if validator_raises:
            raise ValueError("/cam0: truncated image")
        return ("/cam1: gap exceeds 150ms",), {
            "boundary_warnings": ["/cam2: trimmed leading gap"],
            "transport_warnings": ["/cam0: receive gap"],
        }

    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        node, tmp_path, "episode", (), 30.0, (), postflight=postflight,
    )
    bag = tmp_path / "episode0"
    bag.mkdir()
    recorder._current_bag_dir = bag
    recorder._process = SimpleNamespace(poll=lambda: None, wait=lambda **kwargs: 0, returncode=0)
    monkeypatch.setattr(rosbag_recording_node, "_signal_process_group", lambda *_: None)
    recorder.stop()
    state = json.loads((bag / "collection_state.json").read_text())
    assert state["state"] == "incomplete"
    assert state["finalized"] is False
    assert any("Validation finished in" in text for text in node.logger.messages)
    for failure in state["failures"]:
        assert f"[validation FAILED] {failure}" in node.logger.messages
        assert ("error", f"[validation FAILED] {failure}") in node.logger.entries
    if not validator_raises:
        assert "[boundary warning] /cam2: trimmed leading gap" in node.logger.messages
        assert "[transport warning] /cam0: receive gap" in node.logger.messages


@pytest.mark.parametrize("has_boundary_warning", [False, True])
def test_successful_stop_preserves_report_and_summarizes_only_transport_warnings(
    tmp_path, monkeypatch, has_boundary_warning,
):
    node = _Node([])
    report = {
        "boundary_warnings": ["/cam2: trimmed leading gap"] if has_boundary_warning else [],
        "transport_warnings": ["/cam0: receive gap", "/cam1: receive gap"],
    }
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(
        node, tmp_path, "episode", (), 30.0, (), postflight=lambda _: ((), report),
    )
    bag = tmp_path / "episode0"
    bag.mkdir()
    recorder._current_bag_dir = bag
    recorder._process = SimpleNamespace(poll=lambda: None, wait=lambda **kwargs: 0, returncode=0)
    monkeypatch.setattr(rosbag_recording_node, "_signal_process_group", lambda *_: None)
    recorder.stop()

    state = json.loads((bag / "collection_state.json").read_text())
    assert state["finalized"] is True
    assert state["failures"] == []
    assert state["validation_report"] == report
    assert state["transport_warnings"] == report["transport_warnings"]
    summaries = [(level, text) for level, text in node.logger.entries if "[transport info]" in text]
    if has_boundary_warning:
        assert not summaries
        assert ("warn", "[boundary warning] /cam2: trimmed leading gap") in node.logger.entries
        assert ("warn", "[transport warning] /cam0: receive gap") in node.logger.entries
    else:
        assert len(summaries) == 1
        level, text = summaries[0]
        assert level == "info"
        assert "2 receive-timing warnings" in text
        assert str(bag / "collection_state.json") in text
        assert not any(level in ("warn", "error") for level, _ in node.logger.entries)


def test_ready_waits_for_required_topics_to_stay_stable(monkeypatch):
    clock = {"now": 0.0}
    node = _Node(
        [
            ("/cam2/color/image_raw: missing",),
            (),
            (),
            (),
        ]
    )
    monkeypatch.setattr(rosbag_recording_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        rosbag_recording_node.time,
        "monotonic",
        lambda: clock["now"],
    )
    monkeypatch.setattr(
        rosbag_recording_node.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    assert _wait_for_required_topics(node, stable_sec=2.0, poll_sec=1.0)
    assert node.calls == 4
    assert any("/cam2/color/image_raw: missing" in text for text in node.logger.messages)


def test_ui_controller_keeps_waiting_start_stop_and_discard_states(tmp_path):
    node = _Node([])
    recorder = _Recorder(tmp_path)
    controller = CollectorController(node, recorder, quality_dir=tmp_path / "数据分类")

    assert controller.status()["state"] == "WAITING"
    with pytest.raises(RuntimeError, match="waiting"):
        controller.start()

    controller.set_ready()
    assert controller.status()["can_start"] is True
    started = controller.start()
    assert started["state"] == "RECORDING"
    assert started["current_episode"] == "episode0"
    assert started["can_stop"] is True
    assert started["can_mark_milestone"] is True
    marked = controller.dispatch("mark_milestone")
    assert marked["state"] == "RECORDING"
    assert len(marked["milestones"]) == 1

    stopped = controller.stop()
    assert stopped["state"] == "FINALIZED"
    assert stopped["finalized"] is True
    assert stopped["transport_warnings"] == ["synthetic warning"]
    assert stopped["can_start"] is True
    assert stopped["can_discard"] is True
    assert stopped["can_mark_milestone"] is False
    with pytest.raises(RuntimeError, match="no episode"):
        controller.dispatch("mark_milestone")
    # The main loop can notice active=False just after the UI stop completes.
    # Reconciliation must be idempotent instead of issuing a second stop.
    assert controller.reconcile_exited_recorder()["state"] == "FINALIZED"

    discarded = controller.discard()
    assert discarded["state"] == "DISCARDED"
    assert discarded["finalized"] is False


def test_main_loop_marks_only_an_unexpected_recorder_exit_interrupted(tmp_path):
    recorder = _Recorder(tmp_path)
    controller = CollectorController(_Node([]), recorder, quality_dir=tmp_path / "数据分类")
    controller.set_ready()
    controller.start()

    # Model rosbag exiting on its own while its episode directory is still
    # current. The main-loop reconciliation owns this path.
    recorder.active = False
    status = controller.reconcile_exited_recorder()

    assert status["state"] == "INTERRUPTED"
    assert status["active"] is False
    assert status["current_episode"] is None


def test_collector_control_server_accepts_line_delimited_local_json(tmp_path):
    controller = CollectorController(_Node([]), _Recorder(tmp_path), quality_dir=tmp_path / "数据分类")
    controller.set_ready()
    server = CollectorControlServer(("127.0.0.1", 0), controller)
    thread = server.start_in_thread()
    try:
        with socket.create_connection(server.server_address, timeout=1.0) as client:
            client.sendall(b'{"id":7,"command":"start","arguments":{}}\n')
            response = json.loads(client.makefile("rb").readline())
        assert response["id"] == 7
        assert response["ok"] is True
        assert response["result"]["state"] == "RECORDING"
    finally:
        controller.stop(interrupted=True)
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)


def _monitored_controller(tmp_path, monkeypatch, *, gate=None, timeout=False):
    import threading
    import subprocess
    from teleop_data_collector.continuity import ContinuityMonitor

    node = _Node([])
    node.continuity_monitor = ContinuityMonitor(["/cam1/color/image_raw"])
    signals = []
    entered = threading.Event()

    class Process:
        returncode = 0
        exited = False
        waits = 0

        def poll(self):
            return self.returncode if self.exited else None

        def wait(self, **kwargs):
            entered.set()
            self.waits += 1
            if timeout and self.waits < 3:
                raise subprocess.TimeoutExpired("rosbag", 1)
            if gate is not None:
                assert gate.wait(3)
            self.exited = True
            return self.returncode

    def popen(command, **kwargs):
        bag = Path(command[command.index("--output") + 1])
        bag.mkdir()
        (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
        return Process()

    monkeypatch.setattr(rosbag_recording_node.shutil, "which", lambda _: "/usr/bin/ros2")
    monkeypatch.setattr(rosbag_recording_node.subprocess, "Popen", popen)
    monkeypatch.setattr(rosbag_recording_node, "_signal_process_group", lambda process, sig: signals.append(sig))
    recorder = rosbag_recording_node.RosbagEpisodeRecorder(node, tmp_path, "episode", (), 30, (), postflight=lambda _: ((), {}))
    controller = CollectorController(node, recorder, quality_dir=tmp_path / "quality")
    controller.set_ready()
    controller.start()
    return controller, recorder, node.continuity_monitor, signals, entered


def _severe_gap(monitor):
    monitor.observe("/cam1/color/image_raw", 1_000_000_000, 0)
    monitor.observe("/cam1/color/image_raw", 1_133_333_333, .133333333)


def test_auto_stop_keeps_controller_alive_and_restarts_on_next_start(tmp_path, monkeypatch):
    controller, recorder, monitor, signals, _ = _monitored_controller(tmp_path, monkeypatch)
    _severe_gap(monitor)
    controller.check_continuity()
    controller._worker.join(3)
    assert not controller._worker.is_alive()
    status = controller.status()
    assert status["quality"] == "报错"
    assert status["quality_pending"] is False
    assert status["state"] == "INCOMPLETE"
    assert status["can_start"] and not status["active"]
    state = json.loads((recorder.last_bag_dir / "collection_state.json").read_text())
    assert not state["finalized"] and state["auto_stop_reason"]
    assert state["live_continuity"]["streams"]["/cam1/color/image_raw"]["estimated_missing_frames_max_run"] == 3
    assert len(signals) == 1
    with pytest.raises(ValueError, match="报错或放弃"):
        controller.rate_quality(recorder.last_bag_dir.name, "优等")
    first = recorder.last_bag_dir
    controller.start()
    assert recorder.current_bag_dir != first
    assert controller.status()["auto_stop_reason"] is None
    assert controller.status()["alerts"] == []
    controller.stop()


def test_status_remains_responsive_and_stop_races_are_idempotent(tmp_path, monkeypatch):
    import threading
    gate = threading.Event()
    controller, recorder, monitor, signals, entered = _monitored_controller(tmp_path, monkeypatch, gate=gate)
    _severe_gap(monitor)
    controller.check_continuity()
    assert entered.wait(2)
    assert controller.status()["state"] == "STOPPING"
    assert not controller.status()["can_stop"]
    assert not controller.status()["can_start"]
    assert not controller.status()["can_mark_milestone"]
    controller.dispatch("stop")
    shutdown = threading.Thread(target=lambda: controller.stop_if_present(interrupted=True))
    shutdown.start()
    gate.set()
    controller._worker.join(3)
    shutdown.join(3)
    assert not shutdown.is_alive()
    assert len(signals) == 1
    assert controller.status()["quality"] == "报错"


def test_manual_tactile_stop_is_async(tmp_path, monkeypatch):
    import threading
    gate = threading.Event()
    controller, recorder, monitor, signals, entered = _monitored_controller(tmp_path, monkeypatch, gate=gate)
    assert controller.dispatch("stop")["state"] == "STOPPING"
    assert entered.wait(2)
    assert controller.status()["active"]
    gate.set()
    controller._worker.join(3)
    assert controller.status()["quality_pending"]
    assert controller.status()["auto_stop_reason"] is None
    assert len(signals) == 1


def test_auto_quality_write_failure_is_retryable(tmp_path, monkeypatch):
    from teleop_data_collector import collector_control
    controller, recorder, monitor, signals, _ = _monitored_controller(tmp_path, monkeypatch)
    original = collector_control.save_quality
    monkeypatch.setattr(collector_control, "save_quality", lambda *args: (_ for _ in ()).throw(OSError("disk full")))
    _severe_gap(monitor)
    controller.check_continuity()
    controller._worker.join(3)
    assert controller.status()["quality_pending"]
    assert not controller.status()["can_start"]
    assert controller.status()["auto_stop_reason"]
    with pytest.raises(RuntimeError, match="先保存"):
        controller.start()
    monkeypatch.setattr(collector_control, "save_quality", original)
    controller.rate_quality(recorder.last_bag_dir.name, "报错")
    assert controller.status()["can_start"]


def test_auto_failure_survives_stop_timeout_and_revalidation(tmp_path, monkeypatch):
    from teleop_data_collector.revalidate_bag import revalidate_bag
    controller, recorder, monitor, signals, _ = _monitored_controller(tmp_path, monkeypatch, timeout=True)
    _severe_gap(monitor)
    controller.check_continuity()
    controller._worker.join(3)
    assert len(signals) == 3
    bag = recorder.last_bag_dir
    state = json.loads((bag / "collection_state.json").read_text())
    # Revalidation uses a real contract; source-complete rescan cannot clear a live fault.
    state["source_topic_contract"] = {"cam1": {"topic": "/cam1/color/image_raw", "type": "sensor_msgs/msg/Image"}}
    (bag / "collection_state.json").write_text(json.dumps(state))
    result = revalidate_bag(bag, inspect=lambda *args, **kwargs: ((), {}))
    assert not result["finalized"]
    assert result["auto_stop_reason"]
    assert result["failures"]


def test_shutdown_winning_auto_stop_race_still_preserves_incomplete_state(tmp_path, monkeypatch):
    controller, recorder, monitor, signals, _ = _monitored_controller(tmp_path, monkeypatch)
    _severe_gap(monitor)
    controller.stop_if_present(interrupted=True)
    state = json.loads((recorder.last_bag_dir / "collection_state.json").read_text())
    assert state["state"] == "incomplete"
    assert not state["finalized"] and state["auto_stop_reason"]
    assert controller.status()["quality"] == "报错"
    assert len(signals) == 1


def test_real_ros_logger_warning_to_error_does_not_kill_auto_stop(tmp_path, monkeypatch, capfd):
    from rclpy.logging import get_logger
    controller, recorder, monitor, signals, _ = _monitored_controller(tmp_path, monkeypatch)
    controller._node.logger = get_logger("continuity_logger_transition")
    monitor.observe("/cam1/color/image_raw", 1_000_000_000, 0)
    monitor.observe("/cam1/color/image_raw", 1_066_666_667, .066667)
    controller.check_continuity()  # actual ROS warning at the production call site
    monitor.observe("/cam1/color/image_raw", 1_200_000_000, .2)
    controller.check_continuity()  # actual ROS error must use a different call site
    controller._worker.join(3)
    assert controller.status()["quality"] == "报错"
    assert len(signals) == 1
    assert "diagnostic logging failed" not in capfd.readouterr().err


def test_diagnostic_logging_failure_cannot_prevent_scheduling_stop(tmp_path, monkeypatch):
    controller, recorder, monitor, signals, _ = _monitored_controller(tmp_path, monkeypatch)
    _severe_gap(monitor)
    original = controller._node.logger.error
    def fail_diagnostic(message):
        if message.startswith("/cam1/"):
            raise ValueError("logger failure")
        original(message)
    monkeypatch.setattr(controller._node.logger, "error", fail_diagnostic)
    controller.check_continuity()
    controller._worker.join(3)
    assert not recorder.active
    assert len(signals) == 1


def test_start_gate_rechecks_camera_after_discovery_before_allocating_episode(tmp_path, monkeypatch):
    from teleop_data_collector.camera_stream_health import CameraReadiness
    gate = CameraReadiness(['/cam0/color/image_raw'])
    now = [0.0]
    monkeypatch.setattr('teleop_data_collector.collector_control.time.monotonic', lambda: now[0])
    node = _Node([])
    node.collection_readiness = gate
    recorder = _Recorder(tmp_path)
    controller = CollectorController(node, recorder, quality_dir=tmp_path / 'quality')
    controller.set_ready()
    assert controller.status()['state'] == 'WAITING'
    assert not controller.status()['can_start']
    with pytest.raises(RuntimeError, match='等待相机'):
        controller.start()
    assert recorder.current_bag_dir is None
    for i in range(102):
        gate.observe('/cam0/color/image_raw', 1_000_000_000 + i * 50_000_000, i * .05)
    now[0] = 5.05
    assert controller.status()['can_start']
    # Camera disconnects after the GUI's latest status: reject stale Start too.
    now[0] = 5.3
    with pytest.raises(RuntimeError, match='等待相机'):
        controller.start()
    assert recorder.current_bag_dir is None
    for i in range(102):
        gate.observe('/cam0/color/image_raw', 10_000_000_000 + i * 50_000_000, 10 + i * .05)
    now[0] = 15.05
    assert controller.start()['state'] == 'RECORDING'
    # Losing readiness during recording does not disable the Stop button or
    # mutate the episode. The existing continuity watchdog owns auto-stop.
    now[0] = 16.5
    assert controller.status()['state'] == 'RECORDING'
    assert controller.status()['can_stop']


@pytest.mark.parametrize('home_first', [True, False])
def test_next_episode_requires_both_quality_and_confirmed_home(tmp_path, monkeypatch, home_first):
    controller, recorder, monitor, _, _ = _monitored_controller(tmp_path, monkeypatch)
    # First finish the helper's legacy episode, then enable the new policy.
    controller.stop()
    controller.start({'task': 'assembly'})
    controller.stop()
    status = controller.status()
    home = status['after_home']
    assert home['state'] == 'pending'
    assert not status['can_start']
    episode = status['last_episode']
    result = dict(id=home['id'], attempt=home['attempt'], state='done', message='settled')
    first = lambda: controller.report_home(result)
    second = lambda: controller.rate_quality(episode, '一般')
    if not home_first:
        first, second = second, first
    first()
    assert not controller.status()['can_start']
    with pytest.raises(RuntimeError, match='Home'):
        controller.start()
    second()
    assert controller.status()['can_start']
    state = json.loads((recorder.last_bag_dir / 'collection_state.json').read_text())
    assert state['after_home']['state'] == 'done'
    assert state['quality'] == '一般'
    controller.start({'task': 'assembly'})
    assert controller.status()['after_home']['id'] != home['id']
    controller.stop()


def test_home_runs_during_validation_but_not_while_writer_is_alive(tmp_path, monkeypatch):
    import threading
    controller, recorder, monitor, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    controller.start({'task': 'assembly'})
    entered, release = threading.Event(), threading.Event()
    def validate(path):
        assert not recorder.active
        entered.set()
        assert release.wait(3)
        return (), {}
    recorder._postflight = validate
    assert controller.status()['after_home']['state'] == 'armed'
    controller.request_stop()
    try:
        assert entered.wait(2)
        status = controller.status()
        assert status['state'] == 'VALIDATING'
        assert not status['can_start']
        home = status['after_home']
        assert home['state'] == 'pending'
        controller.report_home(dict(id=home['id'], attempt=0, state='done'))
        assert not controller.status()['can_start']
    finally:
        release.set()
        controller._worker.join(3)
    assert controller.status()['quality_pending']
    assert not controller.status()['can_start']


def test_auto_error_classification_still_waits_for_home_and_retry(tmp_path, monkeypatch):
    controller, recorder, monitor, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    controller.start({'task': 'assembly'})
    _severe_gap(monitor)
    controller.check_continuity()
    controller._worker.join(3)
    status = controller.status()
    assert status['quality'] == '报错'
    assert status['quality_pending'] is False
    assert not status['can_start']
    home = status['after_home']
    controller.report_home(dict(id=home['id'], attempt=0, state='failed', message='settling failed'))
    assert not controller.status()['can_start']
    controller.retry_home(dict(id=home['id']))
    with pytest.raises(ValueError, match='stale'):
        controller.report_home(dict(id=home['id'], attempt=0, state='done'))
    controller.report_home(dict(id=home['id'], attempt=1, state='done'))
    assert controller.status()['can_start']
    assert controller.status()['state'] == 'INCOMPLETE'
    assert not controller.status()['finalized']


def test_saved_home_cannot_bypass_quality_save_failure(tmp_path, monkeypatch):
    from teleop_data_collector import collector_control
    controller, recorder, monitor, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    controller.start({'task': 'assembly'})
    controller.stop()
    home = controller.status()['after_home']
    controller.report_home(dict(id=home['id'], attempt=0, state='done'))
    save = collector_control.save_quality
    def fail(*args):
        raise OSError('classification disk unavailable')
    monkeypatch.setattr(collector_control, 'save_quality', fail)
    with pytest.raises(OSError):
        controller.rate_quality(recorder.last_bag_dir.name, '一般')
    assert not controller.status()['can_start']
    assert controller.status()['quality_pending']
    monkeypatch.setattr(collector_control, 'save_quality', save)
    controller.rate_quality(recorder.last_bag_dir.name, '一般')
    assert controller.status()['can_start']


def test_quality_archiving_moves_full_bag_and_preserves_home_gate(tmp_path, monkeypatch):
    controller, recorder, monitor, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    parent = tmp_path / 'selected'
    controller.start({'task': 'assembly'}, storage_parent=str(parent))
    bag = recorder.current_bag_dir
    assert bag.parent == parent / 'pending'
    (bag / 'raw.mcap').write_bytes(b'raw recording')
    inode = (bag / 'raw.mcap').stat().st_ino
    controller.stop()
    home = controller.status()['after_home']
    status = controller.rate_quality(bag.name, '优等')
    destination = parent / 'A' / bag.name
    assert not bag.exists()
    assert recorder.last_bag_dir == destination
    assert (destination / 'raw.mcap').stat().st_ino == inode
    assert status['storage']['state'] == 'saved'
    assert status['quality'] == '优等'
    assert not status['can_start']  # Home is independently pending.
    controller.report_home(dict(id=home['id'], attempt=0, state='done'))
    assert controller.status()['can_start']
    state = json.loads((destination / 'collection_state.json').read_text())
    assert state['after_home']['state'] == 'done'
    assert state['source_bag'] == str(destination)
    assert state['storage']['path'] == str(destination)
    # Changing quality moves the same data; discard retains everything in D.
    controller.rate_quality(bag.name, '一般')
    assert (parent / 'B' / bag.name / 'raw.mcap').is_file()
    controller.discard()
    assert (parent / 'D' / bag.name / 'raw.mcap').is_file()
    assert controller.status()['storage']['state'] == 'saved'
    assert controller.status()['quality'] == '放弃'


def test_archive_collision_is_retryable_and_blocks_next_episode(tmp_path, monkeypatch):
    controller, recorder, _, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    parent = tmp_path / 'selected'
    controller.start(storage_parent=str(parent))
    bag = recorder.current_bag_dir
    (bag / 'raw.mcap').write_bytes(b'keep')
    controller.stop()
    collision = parent / 'A' / bag.name
    collision.mkdir()
    with pytest.raises(FileExistsError):
        controller.rate_quality(bag.name, '优等')
    assert bag.exists()
    assert controller.status()['storage']['state'] == 'failed'
    assert not controller.status()['can_start']
    with pytest.raises(RuntimeError, match='归档'):
        controller.start()
    collision.rmdir()
    controller.rate_quality(bag.name, '优等')
    assert controller.status()['can_start']


def test_auto_failure_archives_in_c_without_waiting_for_gui(tmp_path, monkeypatch):
    controller, recorder, monitor, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    parent = tmp_path / 'selected'
    controller.start({'task': 'assembly'}, storage_parent=str(parent))
    _severe_gap(monitor)
    controller.check_continuity()
    controller.wait_for_finalization()
    status = controller.status()
    assert status['quality'] == '报错'
    assert Path(status['last_bag']).parent == parent / 'C'
    assert status['storage']['state'] == 'saved'
    assert not status['can_start']
    assert status['state'] == 'INCOMPLETE'
    with pytest.raises(ValueError, match='报错或放弃'):
        controller.rate_quality(status['last_episode'], '优等')


def test_archive_move_then_classification_failure_retries_at_new_path(tmp_path, monkeypatch):
    from teleop_data_collector import collector_control
    controller, recorder, _, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    parent = tmp_path / 'selected'
    controller.start(storage_parent=str(parent))
    controller.stop()
    episode = recorder.last_bag_dir.name
    real_save = collector_control.save_quality
    def fail(*args):
        raise OSError('quality lists unavailable')
    monkeypatch.setattr(collector_control, 'save_quality', fail)
    with pytest.raises(OSError):
        controller.rate_quality(episode, '一般')
    assert recorder.last_bag_dir == parent / 'B' / episode
    assert not controller.status()['can_start']
    assert controller.status()['quality_pending']
    monkeypatch.setattr(collector_control, 'save_quality', real_save)
    controller.rate_quality(episode, '一般')
    assert controller.status()['can_start']


def test_background_deletion_does_not_block_next_recording_or_recreate_bag(tmp_path, monkeypatch):
    import shutil
    controller, recorder, _, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    parent=tmp_path/'selected'
    controller.start({'task':'assembly'},storage_parent=str(parent))
    controller.stop()
    episode=recorder.last_bag_dir.name
    controller.rate_quality(episode,'优等')
    home=controller.status()['after_home']
    controller.report_home(dict(id=home['id'],attempt=0,state='done'))
    bag=recorder.last_bag_dir
    assert json.loads((bag/'collection_state.json').read_text())['transfer_ready']
    shutil.rmtree(bag)  # Simulate an independently verified uploader deletion.
    status=controller.status()
    assert status['can_start'] and not status['can_rate_quality'] and not status['can_discard']
    with pytest.raises(RuntimeError,match='已由后台删除'):
        controller.rate_quality(episode,'一般')
    with pytest.raises(RuntimeError,match='已由后台删除'):
        controller.discard()
    assert not bag.exists() and controller.status()['can_start']
    controller.start({'task':'assembly'},storage_parent=str(parent))
    assert recorder.current_bag_dir.name!=episode
    controller.stop()


def test_node_postflight_does_not_apply_rgb_preflight_policy(monkeypatch):
    config = SimpleNamespace(topics=(), trim_start_sec=1., trim_end_sec=.2, continuity_enabled=True,
        rgb_preflight_enabled=True, rgb_brightness_threshold=25., rgb_preflight_stable_sec=3.)
    received = {}
    def inspect(path, topics, **kwargs):
        received.update(kwargs)
        return (), {}
    monkeypatch.setattr(rosbag_recording_node, 'inspect_bag', inspect)
    rosbag_recording_node.RosbagDataCollectorNode.postflight(SimpleNamespace(config=config), Path('fake'))
    assert not any(key.startswith('rgb') for key in received)


def test_rgb_preflight_blocks_start_before_allocating_new_episode(tmp_path, monkeypatch):
    from teleop_data_collector.camera_stream_health import CameraReadiness
    controller, recorder, _, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    previous = recorder.last_bag_dir
    topic = '/cam1/color/image_raw'
    gate = CameraReadiness([topic], rgb_enabled=True)
    controller._readiness = gate
    clock = [0.]
    monkeypatch.setattr('teleop_data_collector.collector_control.time.monotonic', lambda: clock[0])
    for i in range(101):
        clock[0] = i/30
        gate.observe(topic, 10**9+round(clock[0]*10**9), clock[0], brightness=24)
    assert not controller.status()['can_start']
    with pytest.raises(RuntimeError, match='亮度'):
        controller.start(storage_parent=str(tmp_path/'selected'))
    assert recorder.last_bag_dir == previous
    for i in range(101, 203):
        clock[0] = i/30
        gate.observe(topic, 10**9+round(clock[0]*10**9), clock[0], brightness=25)
    assert controller.status()['can_start']
    controller.start(storage_parent=str(tmp_path/'selected'))
    controller.stop()


def test_custom_task_id_preserved_for_episode_home(tmp_path,monkeypatch):
    controller, recorder, monitor, _, _ = _monitored_controller(tmp_path, monkeypatch)
    controller.stop()
    task='task_'+'a'*32
    controller.start({'task':task,'owner':'test-gui'})
    controller.stop()
    home=controller.status()['after_home']
    assert home['task']==task and home['state']=='pending'
    state=json.loads((recorder.last_bag_dir/'collection_state.json').read_text())
    assert state['after_home']['task']==task
