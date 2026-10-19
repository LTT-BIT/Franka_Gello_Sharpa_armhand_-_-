import multiprocessing
import json
import os
import socket
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(
    0, str(REPO_ROOT / "adapters" / "pico" / "src")
)

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from pico_bimanual_franka_teleop.control_server import (
    OperatorConsole,
    OperatorControlServer,
)
from apps.operator_gui.operator_gui import (
    COLLECTION_SHORTCUTS,
    PEDAL_BINDINGS,
    PRESET_KEYS,
    OperatorWindow,
)


def test_shortcut_bindings_match_the_workcell_layout():
    assert PEDAL_BINDINGS == {
        "A": ("toggle_hold", "arm", "left"),
        "B": ("home", "arm", "both"),
        "C": ("toggle_hold", "arm", "right"),
    }
    assert COLLECTION_SHORTCUTS == {"L": "toggle_recording", "Space": "mark_milestone"}
    assert not set(PEDAL_BINDINGS) & set(COLLECTION_SHORTCUTS)
    assert PRESET_KEYS == ("W", "E")


def test_teleop_shortcuts_toggle_hold_and_home_arms_only(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._apply_status({})
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        for shortcut in window.shortcuts:
            assert not shortcut.autoRepeat()
            shortcut.activated.emit()
            application.processEvents()
        window._apply_status({"arm_hold": {"left": True}})
        assert window.arm_hold_buttons["left"].text() == "Release Hold (A)"
        assert window.hand_engage_buttons["left"].text() == "Start hand"
        assert window.hand_engage_buttons["right"].text() == "Start hand"
        left_shortcut = next(s for s in window.shortcuts if s.key().toString() == "A")
        left_shortcut.activated.emit()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("hold_arm", {"side": "left", "enabled": True}),
        ("home_arm", {"side": "both", "task": "powder_weighing"}),
        ("hold_arm", {"side": "right", "enabled": True}),
        ("hold_arm", {"side": "left", "enabled": False}),
    ]


def test_home_pedal_matches_button_and_hold_stays_locked_until_arrival(tmp_path):
    from PySide6.QtWidgets import QPushButton
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path))
    application=QApplication.instance() or QApplication([])
    window=OperatorWindow('127.0.0.1',_unused_port());sent=[]
    try:
        window.socket.abort();window.reconnect_timer.stop()
        window._send=lambda command,arguments=None:sent.append((command,arguments or {}))
        status={'active':{'left':False,'right':False},'arm_authorized':{'left':True,'right':True},
            'arm_hold':{'left':True,'right':True},'home_locked':True,'hand_active':{'left':True,'right':True}}
        window._apply_status(status)
        for side in ('left','right'):
            assert window.arm_engage_buttons[side].isChecked()
            assert window.arm_hold_buttons[side].isChecked()
            assert not window.arm_hold_buttons[side].isEnabled()
            assert window.hand_engage_buttons[side].isChecked()
            window._shortcut_toggle_hold(side)
        assert not sent
        window._apply_status(dict(status,home_locked=False))
        for side in ('left','right'):
            assert window.arm_engage_buttons[side].isChecked() and window.arm_engage_buttons[side].isEnabled()
            assert window.arm_hold_buttons[side].isChecked() and window.arm_hold_buttons[side].isEnabled()
        b=next(s for s in window.shortcuts if s.key().toString()=='B')
        b.activated.emit()
        next(button for button in window.findChildren(QPushButton) if button.text()=='Home both arms').click()
        assert sent==[('home_arm',{'side':'both','task':window._selected_task()})]*2
        window._shortcut_toggle_hold('left')
        assert sent[-1]==('hold_arm',{'side':'left','enabled':False})
    finally:
        window.poll_timer.stop();window.health_timer.stop();window.reconnect_timer.stop()
        window.socket.abort();window.close();application.processEvents()


def test_hold_button_sends_latched_publish_command(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        window.arm_hold_buttons["left"].setEnabled(True)
        window.arm_hold_buttons["left"].click()
        window.arm_hold_buttons["left"].click()
        application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("hold_arm", {"side": "left", "enabled": True}),
        ("hold_arm", {"side": "left", "enabled": False}),
    ]


def test_record_home_sends_per_side_command_only_while_arm_is_stopped(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )

        window._apply_status({"hand_active": {"left": True}})
        window.hand_engage_buttons["left"].setChecked(True)
        assert window.capture_home_buttons["left"].isEnabled()
        assert window.capture_home_buttons["left"].text() == "Record arm as Home"
        window._record_current_home("left")
        window.arm_engage_buttons["right"].setChecked(True)
        window._record_current_home("right")
        application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("capture_home", {"side": "left", "task": "powder_weighing"})
    ]
    assert "stop the right arm first" in window.feedback.toPlainText()


def test_task_selection_drives_home_and_ready_to_home(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        window.task_selector.setCurrentIndex(
            window.task_selector.findData("bean_picking")
        )
        window._shortcut_home("both")
        window.ready_to_home_button.setEnabled(True)
        window.ready_to_home_button.click()
        application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("home_arm", {"side": "both", "task": "bean_picking"}),
        ("ready_to_home", {"task": "bean_picking"}),
    ]


def test_w_e_shortcuts_run_presets(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        for shortcut in window.preset_shortcuts:
            shortcut.activated.emit()
            application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("run_preset", {"key": "w"}),
        ("run_preset", {"key": "e"}),
    ]


def test_collection_panel_enables_only_valid_episode_actions(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    try:
        window.collection_socket.abort()
        window.collection_reconnect_timer.stop()
        window._apply_collection_status(
            {
                "state": "READY",
                "can_start": True,
                "can_stop": False,
                "can_discard": False,
            }
        )
        assert window.collection_record_button.isEnabled()
        assert window.collection_record_button.text() == "开始录制 (L)"

        window._apply_collection_status(
            {
                "state": "RECORDING",
                "current_episode": "episode5",
                "elapsed_sec": 2.5,
                "can_start": False,
                "can_stop": True,
                "can_discard": False,
            }
        )
        assert "episode5" in window.collection_status_label.text()
        assert "2.5s" in window.collection_status_label.text()
        assert window.collection_record_button.isEnabled()
        assert window.collection_record_button.text() == "停止并校验 (L)"

        window._apply_collection_status(
            {
                "state": "FINALIZED",
                "last_episode": "episode5",
                "transport_warnings": ["bag gap"],
                "can_start": True,
                "can_stop": False,
                "can_discard": True,
            }
        )
        assert "1 warning(s)" in window.collection_status_label.text()
        assert window.collection_record_button.isEnabled()
        assert window.collection_record_button.text() == "开始录制 (L)"
        assert window.collection_discard_button.isEnabled()
    finally:
        window.close()
        application.processEvents()


def test_collection_shortcuts_mark_only_while_recording_and_keep_discard_button(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.collection_socket.abort()
        window.collection_reconnect_timer.stop()
        window._send_collection = lambda command: sent.append(command)
        shortcuts = {
            shortcut.key().toString(): shortcut
            for shortcut in window.collection_shortcuts
        }

        window._apply_collection_status(
            {"state": "READY", "can_start": True, "can_stop": False}
        )
        shortcuts["Space"].activated.emit()
        shortcuts["L"].activated.emit()
        window._apply_collection_status(
            {"state": "RECORDING", "can_start": False, "can_stop": True,
             "can_mark_milestone": True, "milestones": [{"id": "milestone_1"}]}
        )
        assert "已标记 1 个完成点" in window.collection_status_label.text()
        assert not shortcuts["Space"].autoRepeat()
        shortcuts["Space"].activated.emit()
        shortcuts["L"].activated.emit()
        window._apply_collection_status(
            {"state": "FINALIZED", "can_discard": True}
        )
        shortcuts["Space"].activated.emit()
        window.collection_discard_button.click()
        window._set_collection_offline()
        shortcuts["Space"].activated.emit()
    finally:
        window.close()
        application.processEvents()

    assert sent == ["start", "mark_milestone", "stop", "discard"]


def _unused_port() -> int:
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def test_quality_dialog_waits_for_validation_and_saves_explicit_episode(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window._send_collection = lambda command, arguments=None: sent.append((command, arguments)) or True
        window._apply_collection_status({"state": "STOPPING", "last_episode": "episode384"})
        assert window.quality_dialog is None
        status = {"state": "INCOMPLETE", "last_episode": "episode384",
                  "failures": ["missing frames"], "can_start": True,
                  "can_rate_quality": True, "quality_pending": True}
        window._apply_collection_status(status)
        dialog = window.quality_dialog
        assert dialog is not None
        assert "missing frames" in dialog.result_label.text()
        assert not window.collection_record_button.isEnabled()
        window._apply_collection_status(status)
        assert window.quality_dialog is dialog
        dialog.buttons["一般"].click()
        assert sent == [("rate_quality", {"episode": "episode384", "quality": "一般"})]
        assert not dialog.buttons["一般"].isEnabled()
        window._apply_collection_status({**status, "quality_pending": False, "quality": "一般"})
        assert window.quality_dialog is None
        assert window.collection_record_button.isEnabled()
        window._apply_collection_status({**status, "quality_pending": False, "quality": "一般"})
        assert window.quality_dialog is None
        window.collection_quality_button.click()
        assert window.quality_dialog is not None
        window.quality_dialog.reject()
        window._apply_collection_status(status)
        assert window.quality_dialog is None
    finally:
        window.close()
        application.processEvents()


def test_quality_dialog_keeps_error_for_retry_and_recovers_disconnect(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    try:
        status = {"state": "FINALIZED", "last_episode": "episode8",
                  "can_rate_quality": True, "quality_pending": True}
        window._apply_collection_status(status)
        dialog = window.quality_dialog
        window._send_collection = lambda *args: True
        dialog.buttons["优等"].click()
        window.collection_pending[99] = "rate_quality"
        window.collection_buffer = (json.dumps({"id": 99, "ok": False, "error": "No space left"}) + "\n").encode()
        window._read_collection_responses()
        assert window.quality_dialog is dialog
        assert "No space left" in dialog.message_label.text()
        assert dialog.buttons["优等"].isEnabled()
        dialog.buttons["优等"].click()
        window._set_collection_offline()
        assert not dialog.buttons["优等"].isEnabled()
        window._apply_collection_status(status)
        assert dialog.buttons["优等"].isEnabled()
        dialog.buttons["优等"].click()
        window._set_collection_offline()
        window._apply_collection_status({**status, "quality": "优等", "quality_pending": False})
        assert window.quality_dialog is None
    finally:
        window.close()
        application.processEvents()


def _serve(port: int) -> None:
    server = OperatorControlServer(("127.0.0.1", port), OperatorConsole())
    server.serve_forever(poll_interval=0.05)


def _wait(application, predicate, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        application.processEvents()
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("GUI state did not converge before timeout")


def test_backend_crash_displays_disconnect_and_reconnects_safely(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    port = _unused_port()
    context = multiprocessing.get_context("spawn")
    backend = context.Process(target=_serve, args=(port,), daemon=True)
    backend.start()
    window = OperatorWindow("127.0.0.1", port)
    try:
        _wait(application, lambda: window.connection_state == "connected")
        status_before_engage = window.last_status_at
        window.engage_buttons["left"].click()
        _wait(
            application,
            lambda: (
                window.last_status_at is not None
                and window.last_status_at != status_before_engage
                and window.engage_buttons["left"].isChecked()
            ),
        )
        status_before_hand = window.last_status_at
        window.hand_engage_buttons["left"].click()
        _wait(
            application,
            lambda: (
                window.last_status_at is not None
                and window.last_status_at != status_before_hand
                and window.hand_engage_buttons["left"].isChecked()
                and window.arm_engage_buttons["left"].isChecked()
            ),
        )

        backend.terminate()
        backend.join(timeout=2.0)
        _wait(application, lambda: window.connection_state == "disconnected")
        assert window.connection_indicator.text().startswith("DISCONNECTED")
        assert "backend status unavailable" in window.status_label.toPlainText()
        assert window.disconnect_message is not None
        assert window.disconnect_message.isVisible()
        assert window.disconnect_message.text() == (
            "The teleoperation backend connection was lost."
        )
        assert window.connect_action.isEnabled()
        assert not window.engage_buttons["left"].isChecked()
        assert not window.engage_buttons["left"].isEnabled()
        assert not window.hand_engage_buttons["left"].isChecked()
        assert not window.hand_engage_buttons["left"].isEnabled()

        reconnect_port = _unused_port()
        backend = context.Process(
            target=_serve, args=(reconnect_port,), daemon=True
        )
        backend.start()
        window.disconnect_message.accept()
        window.connect_action.trigger()
        _wait(application, lambda: window.connection_dialog is not None)
        assert window.connection_dialog.host_field.text() == "127.0.0.1"
        assert window.connection_dialog.port_field.value() == port
        window.connection_dialog.port_field.setValue(reconnect_port)
        window.connection_dialog._accept_if_valid()
        _wait(application, lambda: window.connection_state == "connected")
        assert window.port == reconnect_port
        assert window.engage_buttons["left"].isEnabled()
        assert not window.engage_buttons["left"].isChecked()
        assert window.disconnect_message is None
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()
        if backend.is_alive():
            backend.terminate()
        backend.join(timeout=2.0)


def test_continuity_alerts_are_nonmodal_deduplicated_and_pedal_can_restart():
    from PySide6.QtCore import Qt
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    try:
        window.socket.abort()
        window.collection_socket.abort()
        window.collection_pending.clear()
        alert = {"id": 1, "session_id": "collector", "episode": "episode7", "topic": "/cam1/color/image_raw",
                 "severity": "warning", "category": "source", "kind": "missing_frames",
                 "message": "source 缺帧", "gap_ms": 66.7, "estimated_missing_frames": 1, "count": 1}
        status = {"state": "RECORDING", "active": True, "current_episode": "episode7", "can_stop": True,
                  "can_mark_milestone": True, "alerts": [alert]}
        window._apply_collection_status(status)
        assert window.continuity_dialog is None
        assert "预警" in window.collection_status_label.text()
        assert "本次间隔估算缺帧 1" in window.collection_status_label.text()
        assert "录制继续" in window.collection_status_label.text()
        window._apply_collection_status(status)
        assert window.continuity_dialog is None
        alert.update(id=2, count=5)
        window._apply_collection_status(status)
        assert window.continuity_dialog is None
        assert window.collection_record_button.isEnabled()
        # Even a failure does not pop up until the episode has stopped.
        alert.update(id=3, severity="failure", gap_ms=133.3, estimated_missing_frames=3)
        status.update(state="STOPPING", can_stop=False, can_mark_milestone=False, auto_stop_reason=alert)
        window._apply_collection_status(status)
        assert window.continuity_dialog is None
        assert not window.collection_record_button.isEnabled()
        assert "正在收尾" in window.collection_status_label.text()
        status.update(state="INCOMPLETE", active=False, current_episode=None, last_episode="episode7",
                      can_start=True, quality="报错", quality_pending=False)
        window._apply_collection_status(status)
        assert window.collection_record_button.isEnabled()
        assert window.quality_dialog is None
        dialog = window.continuity_dialog
        assert dialog is not None
        assert dialog.windowModality() == Qt.NonModal
        assert dialog.testAttribute(Qt.WA_ShowWithoutActivating)
        assert dialog.windowFlags() & Qt.WindowDoesNotAcceptFocus
        assert "本次间隔估算缺帧 3" in window.continuity_message.text()
        window._apply_collection_status(status)
        assert window.continuity_dialog is dialog
        assert "再次踩" in window.continuity_message.text()
        dialog.close()
        application.processEvents()
        window._apply_collection_status(status)
        assert window.continuity_dialog is None
        sent = []
        window._send_collection = lambda cmd, args=None: sent.append(cmd)
        pedal = next(s for s in window.collection_shortcuts if s.key().toString() == "L")
        pedal.activated.emit()
        assert sent == ["start"]
        status.update(state="RECORDING", active=True, current_episode="episode8", quality=None, can_start=False,
                      can_stop=True, alerts=[], auto_stop_reason=None)
        window._apply_collection_status(status)
        assert window.continuity_dialog is None
    finally:
        window.close()


def test_rgb_preflight_reason_disables_start_and_recovers(tmp_path):
    from apps.operator_gui.operator_gui import QualityDialog
    application = QApplication.instance() or QApplication([])
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    reason = '录制前亮度检查：/cam0/color/image_raw：平均像素值 7.0 < 25/255'
    window = OperatorWindow('127.0.0.1', _unused_port())
    try:
        window._apply_collection_status(dict(state='WAITING', active=False, can_start=False,
            start_readiness=dict(ready=False, message=reason)))
        assert reason in window.collection_status_label.text()
        assert not window.collection_record_button.isEnabled()
        window._apply_collection_status(dict(state='READY', active=False, can_start=True,
            start_readiness=dict(ready=True, message='')))
        assert reason not in window.collection_status_label.text()
        assert window.collection_record_button.isEnabled()
        dialog = QualityDialog(dict(state='FINALIZED', last_episode='episode158', can_rate_quality=True))
        assert dialog.buttons['优等'].isEnabled() and dialog.buttons['一般'].isEnabled()
        dialog.close()
    finally:
        window.close()
        application.processEvents()


def test_auto_failure_quality_dialog_only_allows_error_or_discard():
    from apps.operator_gui.operator_gui import QualityDialog
    application = QApplication.instance() or QApplication([])
    dialog = QualityDialog({"state": "INCOMPLETE", "last_episode": "episode1", "can_rate_quality": True,
                            "auto_stop_reason": {"category": "source"}})
    try:
        assert not dialog.buttons["优等"].isEnabled()
        assert not dialog.buttons["一般"].isEnabled()
        assert dialog.buttons["报错"].isEnabled()
        assert dialog.buttons["放弃"].isEnabled()
    finally:
        dialog.close()


def test_open_buttons_use_hand_supervisor_and_opening_keeps_start_disabled(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.delenv('TELEOP_FULL_CONFIG', raising=False)
    monkeypatch.delenv('TELEOP_FULL_EXIT_SOCKET', raising=False)
    application = QApplication.instance() or QApplication([])
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    window = OperatorWindow('127.0.0.1', _unused_port())
    sent = []
    try:
        window.hand_client = SimpleNamespace(request=lambda command, args=None: sent.append((command,args)), close=lambda: None)
        window._send('open_hand', {'side':'left'})
        window._send('open_hand', {'side':'both'})
        assert sent == [('open_hand', {'side':'left'}), ('open_hand', {'side':'both'})]
        assert not window.pending  # No arm command is sent by this action.
        window._apply_hand_status({'hands':{
            'left':{'state':'OPENING','permit':False,'can_engage':False,'open_status':'正在张开'},
            'right':{'state':'ACTIVE','permit':True}}})
        assert window.hand_engage_buttons['left'].text() == 'Opening hand…'
        assert not window.hand_engage_buttons['left'].isEnabled()
        assert not window.hand_engage_buttons['left'].isChecked()
        assert window.hand_engage_buttons['right'].isChecked()
        window._apply_hand_status({'hands':{
            'left':{'state':'READY','permit':False,'can_engage':True,'open_status':'已张开'},
            'right':{'state':'ACTIVE','permit':True}}})
        assert window.hand_engage_buttons['left'].text() == 'Start hand [READY]'
        assert window.hand_engage_buttons['left'].isEnabled()
        assert not window.hand_engage_buttons['left'].isChecked()
        assert window.hand_engage_buttons['right'].isChecked()
    finally:
        window.close()
        application.processEvents()


def test_camera_warmup_is_shown_in_collection_strip_and_blocks_pedal(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = OperatorWindow('127.0.0.1', 1, collection_port=2)
    sent = []
    window._send_collection = lambda *args: sent.append(args)
    try:
        window._apply_collection_status(dict(state='WAITING', can_start=False, active=False,
            continuity_enabled=True, start_readiness=dict(ready=False,
                message='等待相机连续稳定 3 秒：/cam0/color/image_raw')))
        assert '等待相机连续稳定 3 秒' in window.collection_status_label.text()
        assert not window.collection_record_button.isEnabled()
        window._toggle_collection_recording()
        assert not sent
        assert window.continuity_dialog is None
        window._apply_collection_status(dict(state='READY', can_start=True, active=False,
            start_readiness=dict(ready=True, message='')))
        assert window.collection_record_button.isEnabled()
        assert '等待相机' not in window.collection_status_label.text()
    finally:
        window.close()
        window.deleteLater()
        app.processEvents()


def test_new_episode_clears_old_collection_display_before_start_reply(tmp_path):
    from unittest.mock import Mock
    from PySide6.QtNetwork import QAbstractSocket
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow('127.0.0.1', _unused_port(), collection_port=_unused_port())
    window.collection_socket.abort()
    window.collection_socket = Mock()
    window.collection_socket.state.return_value = QAbstractSocket.ConnectedState
    old = dict(state='INCOMPLETE', last_episode='episode52', quality='报错', can_start=True,
               failures=['old failure'], alerts=[dict(id=1, severity='warning', topic='/cam0', message='old gap')])
    try:
        window._apply_collection_status(old)
        assert 'episode52' in window.collection_status_label.text()
        assert 'failure' in window.collection_status_label.text()
        # An earlier poll may be outstanding when L requests the next episode.
        assert window._send_collection('status')
        poll_id = window.collection_next_request_id - 1
        assert window._send_collection('start')
        start_id = window.collection_next_request_id - 1
        assert window.collection_state == 'STARTING'
        assert 'episode52' not in window.collection_status_label.text()
        assert not window.collection_status_label.toolTip()
        window.collection_socket.readAll.return_value = (json.dumps(dict(id=poll_id, ok=True, result=old))+'\n').encode()
        window._read_collection_responses()
        assert window.collection_state == 'STARTING'
        assert 'episode52' not in window.collection_status_label.text()
        new = dict(state='RECORDING', active=True, current_episode='episode53', can_stop=True)
        window.collection_socket.readAll.return_value = (json.dumps(dict(id=start_id, ok=True, result=new))+'\n').encode()
        window._read_collection_responses()
        text = window.collection_status_label.text()
        assert 'episode53' in text and 'episode52' not in text
        assert '报错' not in text and 'failure' not in text and 'old gap' not in text
    finally:
        window.close()
        application.processEvents()
