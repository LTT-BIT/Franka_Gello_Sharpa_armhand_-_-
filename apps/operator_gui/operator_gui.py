"""Operator GUI: a shell over the teleop operator's JSON-TCP control server.

Every button sends one command from the server's dispatch table; a poll
timer refreshes the status panel. No teleop logic lives here - the
backend (pico_bimanual_franka_teleop.control_server) is the single
authority, and this window can disconnect and reconnect at any time
without affecting the session.

    conda activate base && python apps/operator_gui/operator_gui.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from PySide6.QtCore import QSettings, Qt, QTimer, Signal, QEvent
from PySide6.QtGui import QAction, QFontDatabase, QKeySequence, QShortcut
from PySide6.QtNetwork import QAbstractSocket, QTcpSocket
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from operator_tasks import DEFAULT_OPERATOR_TASK, OPERATOR_TASKS

POLL_INTERVAL_MS = 500
RECONNECT_INTERVAL_MS = 2000
STATUS_TIMEOUT_SECONDS = 3.0
SIDES = ("left", "right")
PEDAL_BINDINGS = {
    "A": ("toggle_hold", "arm", "left"),
    "C": ("toggle_hold", "arm", "right"),
}
COLLECTION_SHORTCUTS = {"L": "toggle_recording", "Space": "mark_milestone"}
PRESET_KEYS = ("W", "E")
ARM_HOLD_KEY_HINTS = {"left": "A", "right": "C"}


class ConnectionDialog(QDialog):
    def __init__(self, host: str, port: int, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Connect to teleop backend")
        self.setModal(True)
        self.setMinimumWidth(380)

        self.host_field = QLineEdit(host)
        self.host_field.setPlaceholderText("Host name or IP address")
        self.port_field = QSpinBox()
        self.port_field.setRange(1, 65535)
        self.port_field.setValue(port)

        form = QFormLayout()
        form.addRow("Host", self.host_field)
        form.addRow("Port", self.port_field)
        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.button(QDialogButtonBox.Ok).setText("Connect")
        buttons.accepted.connect(self._accept_if_valid)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(buttons)

    def _accept_if_valid(self) -> None:
        if self.host_field.text().strip():
            self.accept()
            return
        QMessageBox.warning(self, "Invalid host", "Enter a host name or IP address.")

    def endpoint(self) -> tuple[str, int]:
        return self.host_field.text().strip(), self.port_field.value()


class QualityDialog(QDialog):
    quality_selected = Signal(str)

    def __init__(self, status: dict, parent=None) -> None:
        super().__init__(parent)
        self.episode = str(status["last_episode"])
        self.selected_quality: str | None = None
        self.setWindowTitle("数据质量评价")
        self.setMinimumWidth(440)
        layout = QVBoxLayout(self)
        title = QLabel(f"数据编号：{self.episode}")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        layout.addWidget(title)
        self.result_label = QLabel()
        self.result_label.setWordWrap(True)
        self.result_label.setTextFormat(Qt.PlainText)
        layout.addWidget(self.result_label)
        layout.addWidget(QLabel("请选择数据质量，点击后保存："))
        row = QHBoxLayout()
        self.buttons: dict[str, QPushButton] = {}
        for label in ("优等", "一般", "报错", "放弃"):
            button = QPushButton(label)
            button.setMinimumHeight(48)
            button.setAutoDefault(False)
            button.setFocusPolicy(Qt.NoFocus)
            button.clicked.connect(lambda checked=False, label=label: self.quality_selected.emit(label))
            self.buttons[label] = button
            row.addWidget(button)
        layout.addLayout(row)
        self.message_label = QLabel()
        self.message_label.setWordWrap(True)
        self.message_label.setTextFormat(Qt.PlainText)
        layout.addWidget(self.message_label)
        self.update_status(status)

    def update_status(self, status: dict) -> None:
        self.discarded = status.get("state") == "DISCARDED"
        state = str(status.get("state") or "UNKNOWN")
        result = {"FINALIZED": "校验通过", "INCOMPLETE": "校验未通过",
                  "INTERRUPTED": "采集中断", "DISCARDED": "已弃用"}.get(state, state)
        details = list(status.get("failures") or [])
        details += list(status.get("boundary_warnings") or [])
        details += list(status.get("transport_warnings") or [])
        text = f"核验结果：{result}"
        if details:
            text += "\n" + "\n".join(str(item) for item in details[:4])
            if len(details) > 4:
                text += f"\n另有 {len(details) - 4} 条信息，详见采集日志。"
        if status.get("quality"):
            text += f"\n当前评价：{status['quality']}"
        self.result_label.setText(text)
        self.set_available(bool(status.get("can_rate_quality")))

    def set_available(self, available: bool) -> None:
        for label, button in self.buttons.items():
            button.setEnabled(available and self.selected_quality is None
                              and (not self.discarded or label == "放弃"))


class OperatorWindow(QMainWindow):
    def __init__(
        self,
        host: str | None,
        port: int | None,
        collection_host: str | None = None,
        collection_port: int | None = None,
    ) -> None:
        super().__init__()
        self.hand_client=None;self.hand_status={};self.hand_events=[];self.full_config=None
        if os.environ.get('TELEOP_FULL_CONFIG'):
            self.full_config=json.loads(Path(os.environ['TELEOP_FULL_CONFIG']).read_text())
        self.settings = QSettings("HSC", "FrankaUpperBodyTeleop")
        self.host = host or str(self.settings.value("host", "127.0.0.1"))
        self.port = int(
            port if port is not None else self.settings.value("port", 5590)
        )
        self.collection_host = collection_host or "127.0.0.1"
        self.collection_port = int(collection_port or 5592)
        self.setWindowTitle(f"Teleop operator - {self.host}:{self.port}")
        self.next_request_id = 1
        self.pending: dict[int, str] = {}
        self.buffer = b""
        self.connection_state = "disconnected"
        self.connected_at: float | None = None
        self.last_status_at: float | None = None
        self._handling_disconnect = False
        self.was_connected = False
        self.connection_dialog: ConnectionDialog | None = None
        self.disconnect_message: QMessageBox | None = None
        self.capture_dialogs: dict[str, QMessageBox] = {}
        self.ready_capture_dialog: QMessageBox | None = None
        self.collection_state = "OFFLINE"
        self.collection_next_request_id = 1
        self.collection_pending: dict[int, str] = {}
        self.collection_buffer = b""
        self.collection_status: dict = {}
        self.quality_dialog: QualityDialog | None = None
        self.quality_prompted: set[str] = set()

        self.socket = QTcpSocket(self)
        self.socket.readyRead.connect(self._read_responses)
        self.socket.connected.connect(self._connected)
        self.socket.disconnected.connect(self._socket_disconnected)
        self.socket.errorOccurred.connect(self._socket_error)

        self.collection_socket = QTcpSocket(self)
        self.collection_socket.readyRead.connect(self._read_collection_responses)
        self.collection_socket.connected.connect(self._collection_connected)
        self.collection_socket.disconnected.connect(self._collection_disconnected)
        self.collection_socket.errorOccurred.connect(self._collection_socket_error)

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(POLL_INTERVAL_MS)
        self.poll_timer.timeout.connect(self._poll_status)
        self.reconnect_timer = QTimer(self)
        self.reconnect_timer.setInterval(RECONNECT_INTERVAL_MS)
        self.reconnect_timer.timeout.connect(self._connect)
        self.health_timer = QTimer(self)
        self.health_timer.setInterval(POLL_INTERVAL_MS)
        self.health_timer.timeout.connect(self._check_connection_health)
        self.collection_poll_timer = QTimer(self)
        self.collection_poll_timer.setInterval(POLL_INTERVAL_MS)
        self.collection_poll_timer.timeout.connect(self._poll_collection_status)
        self.collection_reconnect_timer = QTimer(self)
        self.collection_reconnect_timer.setInterval(RECONNECT_INTERVAL_MS)
        self.collection_reconnect_timer.timeout.connect(self._connect_collection)

        self._build_ui()
        self._install_shortcuts()
        self._set_connection_state("disconnected", "backend is not connected")
        self._connect()
        self._connect_collection()
        if self.full_config:
            from apps.operator_gui.hand_client import HandClient
            self.hand_client=HandClient(self.full_config['control_port'],self.full_config['token'],self,
                normal_forwarding=self.full_config.get('source')=='litchibot' and self.full_config.get('mode') in ('normal','fake'))
            self.hand_client.status.connect(self._apply_hand_status)
            self.hand_client.event.connect(self._hand_event)
            if self.full_config['mode']=='supervised_hardware_validation':
                QApplication.instance().installEventFilter(self)
            self._apply_hand_status({'hands':{s:{'state':'OFFLINE','permit':False} for s in SIDES}})

    # ------------------------------------------------------------------ ui
    def _build_ui(self) -> None:
        connection_menu = self.menuBar().addMenu("&Connection")
        self.connect_action = QAction("Connect...", self)
        self.connect_action.setShortcut("Ctrl+K")
        self.connect_action.triggered.connect(self._open_connection_dialog)
        connection_menu.addAction(self.connect_action)
        connection_menu.addSeparator()
        quit_action = QAction("Quit", self)
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)
        connection_menu.addAction(quit_action)

        root = QWidget(self)
        layout = QVBoxLayout(root)

        layout.addWidget(QLabel("System status"))
        self.status_label = QPlainTextEdit("-")
        self.status_label.setReadOnly(True)
        self.status_label.setMinimumHeight(240)
        self.status_label.setFont(
            QFontDatabase.systemFont(QFontDatabase.FixedFont)
        )
        layout.addWidget(self.status_label, stretch=2)

        collection_box = QGroupBox("数据采集（独立采集终端）")
        collection_layout = QHBoxLayout(collection_box)
        self.collection_status_label = QLabel("OFFLINE — 请启动采集终端")
        self.collection_status_label.setMinimumWidth(330)
        self.collection_status_label.setStyleSheet(
            "color: #777; font-weight: bold;"
        )
        collection_layout.addWidget(self.collection_status_label, stretch=2)
        self.collection_record_button = QPushButton("开始录制 (L)")
        self.collection_record_button.setFocusPolicy(Qt.NoFocus)
        self.collection_record_button.clicked.connect(
            self._toggle_collection_recording
        )
        self.collection_milestone_button = QPushButton("中间完成标记 (Space)")
        self.collection_milestone_button.setFocusPolicy(Qt.NoFocus)
        self.collection_milestone_button.clicked.connect(self._mark_collection_milestone)
        self.collection_discard_button = QPushButton("丢弃最近一次")
        self.collection_discard_button.setFocusPolicy(Qt.NoFocus)
        self.collection_discard_button.setStyleSheet("color: #a35b00;")
        self.collection_discard_button.clicked.connect(
            lambda: self._send_collection("discard")
        )
        self.collection_quality_button = QPushButton("数据质量")
        self.collection_quality_button.setFocusPolicy(Qt.NoFocus)
        self.collection_quality_button.clicked.connect(self._open_quality_dialog)
        for button in (
            self.collection_record_button,
            self.collection_milestone_button,
            self.collection_discard_button,
            self.collection_quality_button,
        ):
            button.setEnabled(False)
            collection_layout.addWidget(button)
        layout.addWidget(collection_box)

        sides_row = QHBoxLayout()
        self.arm_engage_buttons: dict[str, QPushButton] = {}
        self.arm_hold_buttons: dict[str, QPushButton] = {}
        self.hand_engage_buttons: dict[str, QPushButton] = {}
        self.capture_home_buttons: dict[str, QPushButton] = {}
        # Compatibility alias used by existing integrations and tests.
        self.engage_buttons = self.arm_engage_buttons
        for side in SIDES:
            box = QGroupBox(side.capitalize())
            grid = QGridLayout(box)
            hold_key = ARM_HOLD_KEY_HINTS[side]
            arm_engage = QPushButton("Start arm")
            arm_engage.setCheckable(True)
            arm_engage.setFocusPolicy(Qt.NoFocus)
            arm_engage.setMinimumHeight(56)
            arm_engage.setToolTip("Click to toggle arm following")
            arm_engage.clicked.connect(
                lambda checked, side=side: self._send(
                    "engage_arm" if checked else "disengage_arm", {"side": side}
                )
            )
            self.arm_engage_buttons[side] = arm_engage
            grid.addWidget(arm_engage, 0, 0)

            arm_hold = QPushButton(f"Hold arm ({hold_key})")
            arm_hold.setCheckable(True)
            arm_hold.setFocusPolicy(Qt.NoFocus)
            arm_hold.setMinimumHeight(48)
            arm_hold.setToolTip(
                f"Shortcut: {hold_key} toggles a stationary hold while "
                "continuing to publish this arm's measured joints"
            )
            arm_hold.clicked.connect(
                lambda checked, side=side: self._send(
                    "hold_arm", {"side": side, "enabled": checked}
                )
            )
            self.arm_hold_buttons[side] = arm_hold
            grid.addWidget(arm_hold, 1, 0)

            hand_engage = QPushButton("Start hand")
            hand_engage.setCheckable(True)
            hand_engage.setFocusPolicy(Qt.NoFocus)
            hand_engage.setMinimumHeight(56)
            hand_engage.setToolTip(
                "Click to toggle MANUS hand following"
            )
            hand_engage.clicked.connect(
                lambda checked, side=side: self._send(
                    "engage_hand" if checked else "disengage_hand",
                    {"side": side},
                )
            )
            self.hand_engage_buttons[side] = hand_engage
            grid.addWidget(hand_engage, 2, 0)

            capture_home = QPushButton("Record arm + hand as Home")
            capture_home.setFocusPolicy(Qt.NoFocus)
            capture_home.setToolTip(
                "Save this arm and its Wuji Hand 2 measured joints as task Home. "
                "This does not move the robot."
            )
            capture_home.clicked.connect(
                lambda _checked=False, side=side: self._confirm_capture_home(side)
            )
            self.capture_home_buttons[side] = capture_home
            self.action_buttons = getattr(self, "action_buttons", [])
            self.action_buttons.append(capture_home)
            grid.addWidget(capture_home, 3, 0)

            grid.addWidget(
                self._button("Open hand", "open_hand", {"side": side}), 4, 0
            )
            sides_row.addWidget(box)
        layout.addLayout(sides_row)

        actions = QHBoxLayout()
        disengage_all = self._button("DISENGAGE ALL", "disengage_all")
        self.disengage_all_button=disengage_all
        disengage_all.setMinimumHeight(64)
        disengage_all.setStyleSheet(
            "background-color: #a83232; color: white; font-weight: bold;"
        )
        actions.addWidget(disengage_all, stretch=2)
        actions.addWidget(
            self._button("Open both hands", "open_hand", {"side": "both"})
        )
        self.task_selector = QComboBox()
        for task, label in OPERATOR_TASKS.items():
            self.task_selector.addItem(label, task)
        saved_task = str(self.settings.value("operator_task", DEFAULT_OPERATOR_TASK))
        selected_index = self.task_selector.findData(saved_task)
        self.task_selector.setCurrentIndex(max(0, selected_index))
        self.task_selector.currentIndexChanged.connect(
            lambda: self.settings.setValue("operator_task", self._selected_task())
        )
        self.task_selector.setToolTip("Select which task Home pose and trajectory to use")
        actions.addWidget(self.task_selector)
        home_both = QPushButton("Home both arms")
        home_both.setFocusPolicy(Qt.NoFocus)
        home_both.clicked.connect(
            lambda: self._send(
                "home_arm", {"side": "both", "task": self._selected_task()}
            )
        )
        self.action_buttons.append(home_both)
        home_both.setToolTip("Homes both arms and disengages followers")
        actions.addWidget(home_both)
        layout.addLayout(actions)

        ready_box = QGroupBox("Shared Ready and task Home trajectory")
        ready_layout = QHBoxLayout(ready_box)
        self.capture_ready_button = QPushButton("Record both arms as Ready")
        self.capture_ready_button.setFocusPolicy(Qt.NoFocus)
        self.capture_ready_button.clicked.connect(self._confirm_capture_ready)
        self.action_buttons.append(self.capture_ready_button)
        ready_layout.addWidget(self.capture_ready_button)
        ready_layout.addWidget(self._button("Move both arms to Ready", "move_ready"))
        self.ready_to_home_button = QPushButton("Ready to Home")
        self.ready_to_home_button.setFocusPolicy(Qt.NoFocus)
        self.ready_to_home_button.clicked.connect(
            lambda: self._send(
                "ready_to_home", {"task": self._selected_task()}
            )
        )
        self.ready_to_home_button.setToolTip(
            "Execute the selected task's absolute dual-arm trajectory only when "
            "both arms are already within 0.05 rad of Ready."
        )
        self.action_buttons.append(self.ready_to_home_button)
        ready_layout.addWidget(self.ready_to_home_button)
        layout.addWidget(ready_box)

        preset_box = QGroupBox("Preset relative actions — 65% speed")
        preset_layout = QHBoxLayout(preset_box)
        self.preset_buttons: dict[str, QPushButton] = {}
        preset_labels = {
            "q": "Preset 1: left kuai1",
            "w": "Preset 2: unconfigured (W)",
            "e": "Preset 3: unconfigured (E)",
        }
        for key in ("q", "w", "e"):
            button = self._button(
                preset_labels[key], "run_preset", {"key": key}
            )
            button.setMinimumHeight(48)
            button.setToolTip(
                "Rebase the saved relative tool path at the current measured "
                "end-effector pose, precheck every IK frame, then execute once."
            )
            self.preset_buttons[key] = button
            preset_layout.addWidget(button)
        stop_action = self._button("STOP PRESET", "stop_action")
        stop_action.setMinimumHeight(48)
        stop_action.setStyleSheet(
            "background-color: #d47b22; color: white; font-weight: bold;"
        )
        stop_action.setToolTip("Interrupt preset execution and re-anchor GELLO")
        preset_layout.addWidget(stop_action)
        layout.addWidget(preset_box)

        shortcut_hint = QLabel(
            "Shortcuts: L 开始/停止录制 · Space 中间完成标记  |  "
            "A left Hold · C right Hold"
        )
        shortcut_hint.setStyleSheet("color: #666;")
        layout.addWidget(shortcut_hint)

        layout.addWidget(QLabel("Event log"))
        self.feedback = QPlainTextEdit()
        self.feedback.setReadOnly(True)
        self.feedback.setMaximumBlockCount(200)
        self.feedback.setPlaceholderText("Backend messages will appear here.")
        layout.addWidget(self.feedback, stretch=1)

        self.setCentralWidget(root)
        self.connection_indicator = QLabel()
        self.connection_indicator.setContentsMargins(4, 0, 4, 0)
        self.statusBar().addPermanentWidget(self.connection_indicator)
        self.resize(980, 900)

    def _confirm_capture_home(self, side: str) -> None:
        """Confirm before overwriting one arm's persisted Home pose."""
        current = self.capture_dialogs.get(side)
        if current is not None:
            current.raise_()
            current.activateWindow()
            return
        message = QMessageBox(self)
        self.capture_dialogs[side] = message
        message.setIcon(QMessageBox.Warning)
        task_label = OPERATOR_TASKS[self._selected_task()]
        message.setWindowTitle(f"Replace {side} arm {task_label} Home?")
        message.setText(
            f"Record the {side} arm's current measured joint angles as the "
            f"{task_label} Home?"
        )
        message.setInformativeText(
            "The arm and hand must be stopped. Recording does not move hardware; "
            "the next Home command moves the arm first, then the Wuji hand."
        )
        message.setStandardButtons(QMessageBox.Save | QMessageBox.Cancel)
        message.setDefaultButton(QMessageBox.Cancel)
        message.button(QMessageBox.Save).setText("Record Home")
        message.finished.connect(
            lambda result, selected=side, task=self._selected_task(), current=message: (
                self._capture_home_dialog_finished(selected, task, current, result)
            )
        )
        message.open()

    def _capture_home_dialog_finished(
        self, side: str, task: str, message: QMessageBox, result: int
    ) -> None:
        if self.capture_dialogs.get(side) is message:
            del self.capture_dialogs[side]
        if result == QMessageBox.Save:
            self._record_current_home(side, task)
        message.deleteLater()

    def _record_current_home(self, side: str, task: str | None = None) -> None:
        if self.connection_state != "connected":
            return
        if self.arm_engage_buttons[side].isChecked():
            self.feedback.appendPlainText(
                f"[capture_home] rejected: stop the {side} arm first"
            )
            return
        if self.hand_engage_buttons[side].isChecked():
            self.feedback.appendPlainText(
                f"[capture_home] rejected: stop the {side} hand first"
            )
            return
        self._send(
            "capture_home", {"side": side, "task": task or self._selected_task()}
        )

    def _selected_task(self) -> str:
        return str(self.task_selector.currentData())

    def _confirm_capture_ready(self) -> None:
        if self.ready_capture_dialog is not None:
            self.ready_capture_dialog.raise_()
            self.ready_capture_dialog.activateWindow()
            return
        message = QMessageBox(self)
        self.ready_capture_dialog = message
        message.setIcon(QMessageBox.Warning)
        message.setWindowTitle("Replace shared Ready pose?")
        message.setText("Record both arms' current measured joints as Ready?")
        message.setInformativeText(
            "Both arms must be stopped. This overwrites the one shared Ready pose."
        )
        message.setStandardButtons(QMessageBox.Save | QMessageBox.Cancel)
        message.setDefaultButton(QMessageBox.Cancel)
        message.button(QMessageBox.Save).setText("Record Ready")
        message.finished.connect(
            lambda result, current=message: self._capture_ready_dialog_finished(
                current, result
            )
        )
        message.open()

    def _capture_ready_dialog_finished(
        self, message: QMessageBox, result: int
    ) -> None:
        if self.ready_capture_dialog is message:
            self.ready_capture_dialog = None
        if result == QMessageBox.Save:
            if any(button.isChecked() for button in self.arm_engage_buttons.values()):
                self.feedback.appendPlainText(
                    "[capture_ready] rejected: stop both arms first"
                )
            else:
                self._send("capture_ready")
        message.deleteLater()

    def _button(self, text: str, command: str, arguments=None) -> QPushButton:
        button = QPushButton(text)
        # Keep keyboard input from activating a previously focused button.
        button.setFocusPolicy(Qt.NoFocus)
        button.clicked.connect(
            lambda: self._send(command, dict(arguments or {}))
        )
        self.action_buttons = getattr(self, "action_buttons", [])
        self.action_buttons.append(button)
        return button

    def _install_shortcuts(self) -> None:
        # Foot pedals appear as ordinary keyboard keys. Auto-repeat is disabled
        # so holding a pedal cannot retrigger an action.
        self.shortcuts = []
        for key, (action, target, side) in PEDAL_BINDINGS.items():
            if action == "toggle":
                slot = lambda side=side, target=target: (
                    self._shortcut_toggle_engage(side, target)
                )
            elif action == "toggle_hold":
                slot = lambda side=side: self._shortcut_toggle_hold(side)
            else:
                slot = lambda side=side: self._shortcut_home(side)
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.WindowShortcut)
            shortcut.setAutoRepeat(False)
            shortcut.activated.connect(slot)
            self.shortcuts.append(shortcut)
        self.preset_shortcuts = []
        for key in PRESET_KEYS:
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.WindowShortcut)
            shortcut.setAutoRepeat(False)
            shortcut.activated.connect(
                lambda selected=key.lower(): self._shortcut_preset(selected)
            )
            self.preset_shortcuts.append(shortcut)
        self.collection_shortcuts = []
        for key, action in COLLECTION_SHORTCUTS.items():
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.WindowShortcut)
            shortcut.setAutoRepeat(False)
            if action == "toggle_recording":
                shortcut.activated.connect(self._toggle_collection_recording)
            else:
                shortcut.activated.connect(self._mark_collection_milestone)
            self.collection_shortcuts.append(shortcut)

    def _shortcut_preset(self, key: str) -> None:
        if self.connection_state == "connected":
            self._send("run_preset", {"key": key})

    def _shortcut_toggle_engage(self, side: str, target: str = "arm") -> None:
        if self.connection_state != "connected":
            return
        buttons = (
            self.arm_engage_buttons
            if target == "arm"
            else self.hand_engage_buttons
        )
        button = buttons[side]
        if button.isChecked():
            self._send(f"disengage_{target}", {"side": side})
        else:
            self._send(f"engage_{target}", {"side": side})

    def _shortcut_toggle_hold(self, side: str) -> None:
        if self.connection_state != "connected":
            return
        held = self.arm_hold_buttons[side].isChecked()
        self._send("hold_arm", {"side": side, "enabled": not held})

    def _shortcut_home(self, side: str) -> None:
        if self.connection_state == "connected":
            self._send(
                "home_arm", {"side": side, "task": self._selected_task()}
            )

    # -------------------------------------------------------------- socket
    def _connect(self) -> None:
        if self.socket.state() == QAbstractSocket.UnconnectedState:
            self._set_connection_state("connecting")
            self.socket.connectToHost(self.host, self.port)

    def _connected(self) -> None:
        self.reconnect_timer.stop()
        self.connected_at = time.monotonic()
        self.last_status_at = None
        self.pending.clear()
        self.buffer = b""
        self._set_connection_state("syncing")
        self.poll_timer.start()
        self.health_timer.start()
        self._poll_status()

    def _socket_error(self, _error) -> None:
        self._disconnected(self.socket.errorString())

    def _socket_disconnected(self) -> None:
        self._disconnected(self.socket.errorString())

    def _disconnected(self, reason: str = "") -> None:
        if self._handling_disconnect:
            return
        self._handling_disconnect = True
        self.poll_timer.stop()
        self.health_timer.stop()
        self.pending.clear()
        self.buffer = b""
        self.connected_at = None
        self.last_status_at = None
        detail = reason.strip() or "backend connection closed"
        changed = self.connection_state != "disconnected"
        self._set_connection_state("disconnected", detail)
        if changed:
            self.feedback.appendPlainText(f"[connection] {detail}")
        if self.was_connected:
            message = QMessageBox(self)
            self.disconnect_message = message
            message.setIcon(QMessageBox.Warning)
            message.setWindowTitle("Connection lost")
            message.setText("The teleoperation backend connection was lost.")
            message.finished.connect(
                lambda _result, current=message: self._forget_message(current)
            )
            message.open()
        self.was_connected = False
        if self.socket.state() != QAbstractSocket.UnconnectedState:
            self.socket.abort()
        if self.connection_dialog is None:
            self.reconnect_timer.start()
        self._handling_disconnect = False

    def _set_connection_state(self, state: str, detail: str = "") -> None:
        self.connection_state = state
        if state == "connected":
            text = f"CONNECTED  {self.host}:{self.port}"
            style = "color: #248a3d; font-weight: bold;"
        elif state == "syncing":
            text = f"SYNCING  {self.host}:{self.port}"
            style = "color: #9a6b00; font-weight: bold;"
        elif state == "connecting":
            text = f"CONNECTING  {self.host}:{self.port}"
            style = "color: #9a6b00; font-weight: bold;"
        else:
            text = f"DISCONNECTED  {self.host}:{self.port}"
            style = "color: #b02020; font-weight: bold;"
        self.connection_indicator.setText(text)
        self.connection_indicator.setStyleSheet(style)
        self.statusBar().showMessage(detail if state == "disconnected" else "")
        ready = state == "connected"
        for button in getattr(self, "action_buttons", []):
            button.setEnabled(ready)
        for side, button in self.arm_engage_buttons.items():
            button.setEnabled(ready)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                button.setText("Start arm")
                button.blockSignals(False)
        for side, button in self.arm_hold_buttons.items():
            button.setEnabled(ready)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                key_hint = ARM_HOLD_KEY_HINTS[side]
                button.setText(f"Hold arm ({key_hint})")
                button.blockSignals(False)
        for button in getattr(self, "capture_home_buttons", {}).values():
            button.setEnabled(ready)
        for side, button in self.hand_engage_buttons.items():
            if self.hand_client is not None:continue
            button.setEnabled(ready)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                button.setText("Start hand")
                button.blockSignals(False)
        if self.hand_client is not None:self._apply_hand_status(self.hand_status)
        self.connect_action.setEnabled(state != "connected")
        self.task_selector.setEnabled(ready)
        if state == "disconnected":
            self.status_label.setPlainText(
                "DISCONNECTED — backend status unavailable"
            )
        elif state == "connected":
            self.was_connected = True

    def _forget_message(self, message: QMessageBox) -> None:
        if self.disconnect_message is message:
            self.disconnect_message = None

    def _open_connection_dialog(self) -> None:
        if self.connection_dialog is not None:
            self.connection_dialog.raise_()
            self.connection_dialog.activateWindow()
            return
        dialog = ConnectionDialog(self.host, self.port, self)
        self.connection_dialog = dialog
        self.reconnect_timer.stop()
        if self.socket.state() != QAbstractSocket.UnconnectedState:
            self.socket.abort()
        dialog.finished.connect(self._connection_dialog_finished)
        dialog.open()

    def _connection_dialog_finished(self, result: int) -> None:
        dialog = self.connection_dialog
        self.connection_dialog = None
        if dialog is None:
            return
        if result == QDialog.Accepted:
            self.host, self.port = dialog.endpoint()
            self.settings.setValue("host", self.host)
            self.settings.setValue("port", self.port)
            self.setWindowTitle(f"Teleop operator - {self.host}:{self.port}")
            self._connect()
        elif self.connection_state == "disconnected":
            self.reconnect_timer.start()
        dialog.deleteLater()

    def _check_connection_health(self) -> None:
        if self.socket.state() != QAbstractSocket.ConnectedState:
            return
        baseline = self.last_status_at or self.connected_at
        if baseline is None:
            return
        if time.monotonic() - baseline <= STATUS_TIMEOUT_SECONDS:
            return
        self._disconnected(
            f"no status response for {STATUS_TIMEOUT_SECONDS:.0f}s"
        )

    def _send(self, command: str, arguments=None) -> None:
        if self.hand_client:
            if command in ('engage_hand','disengage_hand','open_hand'):
                self.hand_client.request(command,arguments)
                self._apply_hand_status(self.hand_status)
                return
            if command=='disengage_all':self.hand_client.request('disengage_all')
        if self.socket.state() != QAbstractSocket.ConnectedState:
            return
        request_id = self.next_request_id
        self.next_request_id += 1
        self.pending[request_id] = command
        payload = {
            "id": request_id,
            "command": command,
            "arguments": arguments or {},
        }
        self.socket.write((json.dumps(payload) + "\n").encode("utf-8"))

    def _poll_status(self) -> None:
        if "status" in self.pending.values():
            return
        self._send("status")

    def _read_responses(self) -> None:
        self.buffer += bytes(self.socket.readAll())
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                response = json.loads(line)
            except json.JSONDecodeError:
                continue
            command = self.pending.pop(response.get("id"), "")
            if not response.get("ok"):
                self.feedback.appendPlainText(
                    f"[{command}] rejected: {response.get('error')}"
                )
            elif command == "status":
                self._apply_status(response.get("result", {}))

    # ---------------------------------------------------- collection socket
    def _connect_collection(self) -> None:
        if self.collection_socket.state() == QAbstractSocket.UnconnectedState:
            self.collection_socket.connectToHost(
                self.collection_host, self.collection_port
            )

    def _collection_connected(self) -> None:
        self.collection_reconnect_timer.stop()
        self.collection_pending.clear()
        self.collection_buffer = b""
        self.collection_poll_timer.start()
        self._poll_collection_status()

    def _collection_socket_error(self, _error) -> None:
        self._set_collection_offline()

    def _collection_disconnected(self) -> None:
        self._set_collection_offline()

    def _set_collection_offline(self) -> None:
        self.collection_state = "OFFLINE"
        self.collection_poll_timer.stop()
        self.collection_pending.clear()
        self.collection_buffer = b""
        self.collection_status_label.setText("OFFLINE — 请启动采集终端")
        self.collection_status_label.setStyleSheet(
            "color: #777; font-weight: bold;"
        )
        self.collection_record_button.setText("开始录制 (L)")
        self.collection_record_button.setEnabled(False)
        self.collection_milestone_button.setEnabled(False)
        self.collection_discard_button.setEnabled(False)
        self.collection_quality_button.setEnabled(False)
        if self.quality_dialog is not None:
            self.quality_dialog.set_available(False)
            self.quality_dialog.message_label.setText("采集连接已断开，重新连接后可重试保存。")
        if self.collection_socket.state() != QAbstractSocket.UnconnectedState:
            self.collection_socket.abort()
        if not self.collection_reconnect_timer.isActive():
            self.collection_reconnect_timer.start()

    def _send_collection(self, command: str, arguments: dict | None = None) -> bool:
        if self.collection_socket.state() != QAbstractSocket.ConnectedState:
            return False
        request_id = self.collection_next_request_id
        self.collection_next_request_id += 1
        self.collection_pending[request_id] = command
        if command == "start":
            self.collection_status_label.setText("STARTING — 正在启动 rosbag…")
        elif command == "stop":
            self.collection_status_label.setText("STOPPING — 正在结束并校验…")
        elif command == "discard":
            self.collection_status_label.setText("DISCARDING — 正在标记…")
        elif command == "mark_milestone":
            self.collection_status_label.setText("RECORDING — 正在保存中间完成标记…")
        if command != "status":
            for button in (
                self.collection_record_button,
                self.collection_milestone_button,
                self.collection_discard_button,
                self.collection_quality_button,
            ):
                button.setEnabled(False)
        payload = {"id": request_id, "command": command, "arguments": arguments or {}}
        self.collection_socket.write(
            (json.dumps(payload) + "\n").encode("utf-8")
        )
        return True

    def _open_quality_dialog(self) -> None:
        status = self.collection_status
        if not status.get("can_rate_quality") or not status.get("last_episode"):
            return
        if self.quality_dialog is not None:
            self.quality_dialog.raise_()
            self.quality_dialog.activateWindow()
            return
        dialog = QualityDialog(status, self)
        self.quality_dialog = dialog
        self.quality_prompted.add(dialog.episode)
        dialog.quality_selected.connect(self._save_collection_quality)
        dialog.finished.connect(lambda: self._quality_dialog_finished(dialog))
        dialog.show()

    def _quality_dialog_finished(self, dialog: QualityDialog) -> None:
        if self.quality_dialog is dialog:
            self.quality_dialog = None
        dialog.deleteLater()

    def _save_collection_quality(self, quality: str) -> None:
        dialog = self.quality_dialog
        if dialog is None or dialog.selected_quality is not None:
            return
        if self._send_collection("rate_quality", {"episode": dialog.episode, "quality": quality}):
            dialog.selected_quality = quality
            dialog.set_available(False)
            dialog.message_label.setText("正在保存…")
        else:
            dialog.message_label.setText("采集端未连接，评价尚未保存。请连接后重试。")

    def _toggle_collection_recording(self) -> None:
        if not self.collection_record_button.isEnabled():
            return
        command = "stop" if self.collection_state == "RECORDING" else "start"
        self._send_collection(command)

    def _mark_collection_milestone(self) -> None:
        if self.collection_milestone_button.isEnabled():
            self._send_collection("mark_milestone")

    def _poll_collection_status(self) -> None:
        if self.collection_pending:
            return
        self._send_collection("status")

    def _read_collection_responses(self) -> None:
        self.collection_buffer += bytes(self.collection_socket.readAll())
        while b"\n" in self.collection_buffer:
            line, self.collection_buffer = self.collection_buffer.split(b"\n", 1)
            try:
                response = json.loads(line)
            except json.JSONDecodeError:
                continue
            command = self.collection_pending.pop(response.get("id"), "")
            if not response.get("ok"):
                self.feedback.appendPlainText(
                    f"[collection:{command}] rejected: {response.get('error')}"
                )
                if command == "rate_quality" and self.quality_dialog is not None:
                    self.quality_dialog.selected_quality = None
                    self.quality_dialog.set_available(True)
                    self.quality_dialog.message_label.setText(
                        f"保存失败：{response.get('error')}\n请处理后重新选择以重试。"
                    )
                self._poll_collection_status()
                continue
            self._apply_collection_status(response.get("result", {}))
            if command in {"rate_quality", "discard"}:
                result = response.get("result", {})
                if result.get("quality"):
                    self.feedback.appendPlainText(
                        f"[collection] {result.get('last_episode')} 已归类为“{result['quality']}”"
                    )
            if command == "mark_milestone":
                markers = response.get("result", {}).get("milestones") or []
                if markers:
                    self.feedback.appendPlainText(
                        f"[collection] 已保存 {markers[-1]['id']}，继续录制"
                    )

    def _apply_collection_status(self, status: dict) -> None:
        self.collection_status = dict(status)
        state = str(status.get("state") or "UNKNOWN").upper()
        self.collection_state = state
        episode = status.get("current_episode") or status.get("last_episode") or "-"
        elapsed = status.get("elapsed_sec")
        detail = f" · {episode}"
        if elapsed is not None:
            detail += f" · {float(elapsed):.1f}s"
        if status.get("quality"):
            detail += f" · {status['quality']}"
        elif status.get("quality_pending"):
            detail += " · 待评价"
        markers = status.get("milestones") or []
        if markers:
            detail += f" · 已标记 {len(markers)} 个完成点"
        failures = list(status.get("failures") or [])
        warnings = len(status.get("boundary_warnings") or []) + len(
            status.get("transport_warnings") or []
        )
        if failures:
            detail += f" · {len(failures)} failure(s)"
        elif warnings:
            detail += f" · {warnings} warning(s)"
        self.collection_status_label.setText(f"{state}{detail}")
        if state == "RECORDING":
            style = "color: #c22525; font-weight: bold;"
        elif state in {"READY", "FINALIZED"}:
            style = "color: #248a3d; font-weight: bold;"
        elif state in {"INCOMPLETE", "INTERRUPTED", "DISCARDED"}:
            style = "color: #a35b00; font-weight: bold;"
        else:
            style = "color: #777; font-weight: bold;"
        self.collection_status_label.setStyleSheet(style)
        can_start = bool(status.get("can_start"))
        can_stop = bool(status.get("can_stop"))
        self.collection_record_button.setText(
            "停止并校验 (L)" if can_stop else "开始录制 (L)"
        )
        self.collection_record_button.setEnabled(
            can_stop or (can_start and not status.get("quality_pending"))
        )
        self.collection_milestone_button.setEnabled(bool(status.get("can_mark_milestone")))
        self.collection_discard_button.setEnabled(bool(status.get("can_discard")))
        self.collection_quality_button.setEnabled(bool(status.get("can_rate_quality")))
        if any(command != "status" for command in self.collection_pending.values()):
            for button in (
                self.collection_record_button, self.collection_milestone_button,
                self.collection_discard_button, self.collection_quality_button,
            ):
                button.setEnabled(False)
        dialog = self.quality_dialog
        if dialog is not None:
            if status.get("active") or status.get("last_episode") != dialog.episode:
                dialog.reject()
            elif dialog.selected_quality is not None and status.get("quality") == dialog.selected_quality:
                dialog.accept()
            else:
                # A lost response can be reconciled by status polling after reconnect.
                if "rate_quality" not in self.collection_pending.values():
                    dialog.selected_quality = None
                dialog.update_status(status)
        episode = status.get("last_episode")
        if status.get("quality_pending") and episode and episode not in self.quality_prompted:
            self._open_quality_dialog()

    def closeEvent(self, event) -> None:
        if self.full_config:
            from teleop_runtime.full_launcher import notify_gui_session_exit
            notify_gui_session_exit(self.full_config['token'])
        if self.hand_client:self.hand_client.close()
        for timer in (
            self.poll_timer,
            self.health_timer,
            self.reconnect_timer,
            self.collection_poll_timer,
            self.collection_reconnect_timer,
        ):
            timer.stop()
        self.socket.abort()
        self.collection_socket.abort()
        super().closeEvent(event)

    # -------------------------------------------------------------- status
    def _apply_status(self, status: dict) -> None:
        self.last_status_at = time.monotonic()
        if self.connection_state != "connected":
            self._set_connection_state("connected")
        self.status_label.setPlainText(str(status.get("status_line", "-")))
        active = status.get("active", {})
        arm_hold = status.get("arm_hold", {})
        for side, button in self.arm_engage_buttons.items():
            engaged = bool(active.get(side))
            button.blockSignals(True)
            button.setChecked(engaged)
            button.setText(
                "Arm running"
                if engaged
                else "Start arm"
            )
            button.setEnabled(not bool(arm_hold.get(side)))
            button.blockSignals(False)
        for side, button in self.arm_hold_buttons.items():
            held = bool(arm_hold.get(side))
            key_hint = ARM_HOLD_KEY_HINTS[side]
            button.blockSignals(True)
            button.setChecked(held)
            button.setText(
                f"Release Hold ({key_hint})"
                if held
                else f"Hold arm ({key_hint})"
            )
            button.blockSignals(False)
        hand_active = status.get("hand_active", {})
        for side, button in self.hand_engage_buttons.items():
            if self.hand_client:continue
            engaged = bool(hand_active.get(side))
            button.blockSignals(True)
            button.setChecked(engaged)
            button.setText(
                "Hand running"
                if engaged
                else "Start hand"
            )
            button.blockSignals(False)
        for side, button in self.capture_home_buttons.items():
            button.setEnabled(
                not self.arm_engage_buttons[side].isChecked()
                and not self.arm_hold_buttons[side].isChecked()
                and not self.hand_engage_buttons[side].isChecked()
            )
        both_stopped = not any(
            button.isChecked() for button in self.arm_engage_buttons.values()
        ) and not any(
            button.isChecked() for button in self.arm_hold_buttons.values()
        )
        self.capture_ready_button.setEnabled(both_stopped)
        feedback = status.get("feedback", [])
        # Re-render the ring wholesale: the server caps it at 50 lines, so
        # replacing the text is the simplest correct display.
        if self.hand_client:feedback=list(feedback)+self.hand_events[-50:]
        if feedback and self.feedback.toPlainText().splitlines() != feedback:
            self.feedback.setPlainText("\n".join(feedback))
            self.feedback.verticalScrollBar().setValue(
                self.feedback.verticalScrollBar().maximum()
            )

    def _hand_event(self,text):
        if not self.hand_events or self.hand_events[-1]!=text:
            self.hand_events.append(text);self.feedback.appendPlainText(text)

    def _apply_hand_status(self,status):
        previous=self.hand_status
        old_reason=previous.get('reason');self.hand_status=status
        if status.get('reason') and status.get('reason')!=old_reason:self._hand_event(status['reason'])
        for side,button in self.hand_engage_buttons.items():
            data=status.get('hands',{}).get(side,{})
            state=data.get('state','OFFLINE');permit=bool(data.get('permit'))
            old_data=previous.get('hands',{}).get(side,{})
            if data.get('reason') and (data.get('reason'),state)!=(old_data.get('reason'),old_data.get('state')):
                self._hand_event(f'{side}: {state}: '+data['reason'])
            button.blockSignals(True);button.setChecked(permit)
            button.setText(('Stop hand' if permit else 'Start hand')+f' [{state}]')
            button.setEnabled((state not in ('FAULT','HARD_FAULT','OFFLINE','RECOVERING')) and (state!='SOFT_HOLD' or permit))
            button.setToolTip('Supervisor state: '+state+' | '+str(data.get('reason') or status.get('reason','')))
            button.blockSignals(False)
        if self.full_config and self.full_config['mode']=='supervised_hardware_validation':
            self.statusBar().showMessage('Validation: hold F12 to run; release/focus loss latches STOP')
        self.disengage_all_button.setEnabled(self.connection_state=='connected' or any(
            d.get('state')!='OFFLINE' for d in status.get('hands',{}).values()))

    def eventFilter(self,watched,event):
        if self.hand_client and self.full_config and self.full_config['mode']=='supervised_hardware_validation':
            if event.type() in (QEvent.KeyPress,QEvent.KeyRelease) and event.key()==Qt.Key_F12:
                if not event.isAutoRepeat():
                    self.hand_client.held=event.type()==QEvent.KeyPress
                    self.hand_client.request('heartbeat',{'hold':self.hand_client.held})
                return True
            if event.type()==QEvent.WindowDeactivate:
                self.hand_client.held=False
                self.hand_client.request('heartbeat',{'hold':False})
        return super().eventFilter(watched,event)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default=None,
        help="initial backend host (default: saved value or 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="initial backend port (default: saved value or 5590)",
    )
    parser.add_argument(
        "--collection-host",
        default="127.0.0.1",
        help="data collector control host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--collection-port",
        type=int,
        default=5592,
        help="data collector control port (default: 5592)",
    )
    args = parser.parse_args()
    application = QApplication(sys.argv)
    window = OperatorWindow(
        args.host,
        args.port,
        collection_host=args.collection_host,
        collection_port=args.collection_port,
    )
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
