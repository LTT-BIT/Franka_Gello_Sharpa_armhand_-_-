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
import threading
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from PySide6.QtCore import QSettings, Qt, QTimer, Signal, QEvent, QUrl
from PySide6.QtGui import QDesktopServices, QAction, QFontDatabase, QKeySequence, QShortcut
from PySide6.QtNetwork import QAbstractSocket, QTcpSocket
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QFrame,
    QScrollArea,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QInputDialog,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from teleop_runtime.transfer_settings import read_settings as read_transfer_settings, update_settings as update_transfer_settings

from operator_tasks import (DEFAULT_OPERATOR_TASK, get_operator_tasks, task_label,
                            add_operator_task, delete_operator_task)

POLL_INTERVAL_MS = 500
RECONNECT_INTERVAL_MS = 2000
STATUS_TIMEOUT_SECONDS = 3.0
SIDES = ("left", "right")
PEDAL_BINDINGS = {
    "A": ("toggle_hold", "arm", "left"),
    "B": ("home", "arm", "both"),
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
        self.auto_failed = bool(status.get("auto_stop_reason"))
        state = str(status.get("state") or "UNKNOWN")
        result = {"FINALIZED": "校验通过", "INCOMPLETE": "校验未通过",
                  "INTERRUPTED": "采集中断", "DISCARDED": "已弃用"}.get(state, state)
        details = list(status.get("failures") or [])
        details += list(status.get("source_warnings") or [])
        details += list(status.get("boundary_warnings") or [])
        details += list(status.get("transport_warnings") or [])
        text = f"核验结果：{result}"
        if details:
            text += "\n" + "\n".join(str(item) for item in details[:4])
            if len(details) > 4:
                text += f"\n另有 {len(details) - 4} 条信息，详见采集日志。"
        if status.get("quality"):
            text += f"\n当前评价：{status['quality']}"
        storage = status.get("storage") or {}
        if storage:
            text += f"\n保存父目录：{storage.get('parent', '')}\n优等→A　一般→B　报错→C　放弃→D"
        home = status.get("after_home") or {}
        if home:
            text += "\n" + {"pending": "机械臂即将 Home，可同时保存评价。",
                "running": "机械臂正在 Home，可同时保存评价。",
                "done": "机械臂已 Home；评价保存成功后可开始下一段。",
                "failed": "机械臂 Home 未完成，请通过 Home 按钮或中间踏板重试。"}.get(home.get("state"), "等待停止写盘后 Home。")
        self.result_label.setText(text)
        self.set_available(bool(status.get("can_rate_quality")))

    def set_available(self, available: bool) -> None:
        for label, button in self.buttons.items():
            button.setEnabled(available and self.selected_quality is None
                              and (not self.discarded or label == "放弃")
                              and (not self.auto_failed or label in {"报错", "放弃"}))


class OperatorWindow(QMainWindow):
    task_edit_finished = Signal(str, str, str)
    def __init__(
        self,
        host: str | None,
        port: int | None,
        collection_host: str | None = None,
        collection_port: int | None = None,
    ) -> None:
        super().__init__()
        self.task_edit_pending=False
        self.task_edit_finished.connect(self._task_edit_completed)
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
        self.continuity_dialog: QDialog | None = None
        self.continuity_message: QLabel | None = None
        self.continuity_notice_keys: set[tuple] = set()
        self.continuity_episode_key: tuple | None = None

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
                normal_forwarding=self.full_config.get('source') in ('litchibot','manus') and self.full_config.get('mode') in ('normal','fake'))
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
        if self.full_config:
            self.arm_reconnect_action = QAction("重新连接机械臂（修复连接 / Exec / FCI 后）", self)
            self.arm_reconnect_action.triggered.connect(self._reconnect_arms)
            connection_menu.addAction(self.arm_reconnect_action)
        connection_menu.addSeparator()
        quit_action = QAction("Quit", self)
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)
        connection_menu.addAction(quit_action)

        root = QWidget(self)
        layout = QVBoxLayout(root)

        self.system_status_heading = QLabel("System status")
        layout.addWidget(self.system_status_heading)
        self.status_label = QPlainTextEdit("-")
        self.status_label.setReadOnly(True)
        self.status_label.setFixedHeight(100)
        self.status_label.setFont(
            QFontDatabase.systemFont(QFontDatabase.FixedFont)
        )
        layout.addWidget(self.status_label)

        collection_box = QGroupBox("数据采集（独立采集终端）")
        collection_box.setFixedHeight(210)
        collection_box.setStyleSheet("QGroupBox { background-color: white; }")
        collection_layout = QHBoxLayout(collection_box)
        self.collection_status_label = QLabel(self._collection_offline_text())
        self.collection_status_label.setMinimumWidth(330)
        self.collection_status_label.setWordWrap(True)
        self.collection_status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
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
            collection_layout.addWidget(button, alignment=Qt.AlignBottom)
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

            capture_home = QPushButton("Record arm as Home")
            capture_home.setFocusPolicy(Qt.NoFocus)
            capture_home.setToolTip(
                "Save this arm's measured joints as task Home. "
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
        self.task_selector.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.task_selector.setMinimumContentsLength(6)
        for task, label in get_operator_tasks().items():
            self.task_selector.addItem(label, task)
        saved_task = str(self.settings.value("operator_task", DEFAULT_OPERATOR_TASK))
        selected_index = self.task_selector.findData(saved_task)
        self.task_selector.setCurrentIndex(max(0, selected_index))
        self.task_selector.currentIndexChanged.connect(
            lambda: self.settings.setValue("operator_task", self._selected_task())
        )
        self.task_selector.setToolTip("Select which task Home pose and trajectory to use")
        self.task_selector.addItem("＋ 新增任务…", "__add_task__")
        self.task_selector.addItem("－ 删除当前任务…", "__delete_task__")
        self.task_selector.activated.connect(self._task_action_selected)
        self.current_task = str(self.task_selector.currentData())
        actions.addWidget(self.task_selector)
        home_both = QPushButton("Home both arms")
        self.home_both_button = home_both
        home_both.setFocusPolicy(Qt.NoFocus)
        home_both.clicked.connect(
            lambda: self._shortcut_home("both")
        )
        self.action_buttons.append(home_both)
        home_both.setToolTip("B: Home both arms; retain arm permits and stay in Hold after arrival")
        actions.addWidget(home_both)
        layout.addLayout(actions)

        ready_box = QGroupBox("Shared Ready and task Home trajectory")
        ready_layout = QHBoxLayout(ready_box)
        self.capture_ready_button = QPushButton("Record both arms as Ready")
        self.capture_ready_button.setFocusPolicy(Qt.NoFocus)
        self.capture_ready_button.clicked.connect(self._confirm_capture_ready)
        self.action_buttons.append(self.capture_ready_button)
        ready_layout.addWidget(self.capture_ready_button)
        self.move_ready_button=self._button("Move both arms to Ready", "move_ready")
        ready_layout.addWidget(self.move_ready_button)
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
            "A left Hold · B Home both arms · C right Hold"
        )
        shortcut_hint.setStyleSheet("color: #666;")
        layout.addWidget(shortcut_hint)

        layout.addWidget(QLabel("Event log"))
        self.feedback = QPlainTextEdit()
        self.feedback.setReadOnly(True)
        self.feedback.setMaximumBlockCount(200)
        self.feedback.setPlaceholderText("Backend messages will appear here.")
        layout.addWidget(self.feedback, stretch=1)

        container = QWidget(self)
        columns = QHBoxLayout(container)
        columns.setContentsMargins(0, 0, 0, 0)
        controls_scroll = QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QFrame.NoFrame)
        controls_scroll.setWidget(root)
        columns.addWidget(controls_scroll, stretch=1)
        storage_scroll = QScrollArea()
        storage_scroll.setMinimumWidth(310)
        storage_scroll.setMaximumWidth(360)
        storage_scroll.setWidgetResizable(True)
        storage_scroll.setFrameShape(QFrame.NoFrame)
        storage_scroll.setWidget(self._build_storage_sidebar())
        columns.addWidget(storage_scroll)
        self.setCentralWidget(container)
        self.connection_indicator = QLabel()
        self.connection_indicator.setContentsMargins(4, 0, 4, 0)
        self.statusBar().addPermanentWidget(self.connection_indicator)
        self.resize(1320, 960)

    def _build_storage_sidebar(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(290)
        panel.setMaximumWidth(350)
        column = QVBoxLayout(panel)
        self.storage_heading = QLabel("本地采集设置")
        self.storage_heading.setFont(self.system_status_heading.font())
        heading_font = self.storage_heading.font()
        heading_font.setBold(True)
        self.storage_heading.setFont(heading_font)
        self.storage_heading.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        # Match the left caption and the document's first-line inset, without
        # retaining the old full-height title frame.
        self.storage_heading.setFixedHeight(
            self.system_status_heading.sizeHint().height()
            + self.status_label.frameWidth() + int(self.status_label.document().documentMargin())
        )
        column.addWidget(self.storage_heading)
        self.storage_parent_label = QLabel("保存父目录")
        self.storage_parent_label.setFont(self.status_label.font())
        column.addWidget(self.storage_parent_label)
        row = QHBoxLayout()
        try:
            transfer_parent = read_transfer_settings()["local_parent"]
        except (OSError, ValueError):
            transfer_parent = ""
        self.storage_parent_edit = QLineEdit(transfer_parent or str(self.settings.value("collection_storage_parent", "")))
        self.storage_parent_edit.setPlaceholderText("选择本机文件夹…")
        self.storage_parent_edit.setFocusPolicy(Qt.ClickFocus)
        self.storage_parent_edit.returnPressed.connect(self._finish_storage_edit)
        self.storage_parent_edit.editingFinished.connect(self._remember_storage_parent)
        self.storage_parent_edit.textEdited.connect(self._persist_storage_parent_edit)
        self.storage_parent_edit.textChanged.connect(self._update_storage_preview)
        row.addWidget(self.storage_parent_edit, stretch=1)
        self.storage_browse_button = QPushButton("浏览…")
        self.storage_browse_button.setFocusPolicy(Qt.NoFocus)
        self.storage_browse_button.clicked.connect(self._browse_storage_parent)
        row.addWidget(self.storage_browse_button)
        column.addLayout(row)
        self.storage_preview = QLabel()
        self.storage_preview.setWordWrap(True)
        self.storage_preview.setTextFormat(Qt.PlainText)
        self.storage_preview.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.storage_preview.setStyleSheet("color: #285d49; background: white; border: 1px solid #b8b8b8; padding: 8px;")
        column.addWidget(self.storage_preview)
        column.addSpacing(18)
        column.addWidget(QLabel("本段保存状态"))
        self.storage_episode_label = QLabel("等待采集器连接")
        self.storage_episode_label.setWordWrap(True)
        self.storage_episode_label.setTextFormat(Qt.PlainText)
        self.storage_episode_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        column.addWidget(self.storage_episode_label)
        self.storage_open_button = QPushButton("打开本段文件夹")
        self.storage_open_button.setFocusPolicy(Qt.NoFocus)
        self.storage_open_button.setEnabled(False)
        self.storage_open_button.clicked.connect(self._open_episode_folder)
        column.addWidget(self.storage_open_button)
        column.addSpacing(18)
        from apps.operator_gui.transfer_panel import TransferPanel
        self.transfer_panel = TransferPanel(self)
        self.transfer_panel.local_parent_changed.connect(self._transfer_local_parent_changed)
        column.addWidget(self.transfer_panel)
        column.addStretch(1)
        self._update_storage_preview()
        return panel

    def _update_storage_preview(self, *_args) -> None:
        parent = self.storage_parent_edit.text().strip()
        parent = str(Path(parent).expanduser()) if parent else ""
        lines = [f"A  优等  →  {Path(parent) / 'A' if parent else '待选择'}"]
        lines += [f"{folder}  {quality}  →  ../{folder}"
                  for quality, folder in (("一般", "B"), ("报错", "C"), ("放弃", "D"))]
        self.storage_preview.setText("\n".join(lines))

    def _persist_storage_parent_edit(self, value: str) -> None:
        value = value.strip()
        value = str(Path(value).expanduser()) if value else ''
        self.settings.setValue("collection_storage_parent", value)
        try:
            update_transfer_settings(local_parent=value)
        except (OSError, ValueError) as error:
            if hasattr(self, 'transfer_panel'):
                self.transfer_panel.status_label.setText(f'设置保存失败：{error}')

    def _remember_storage_parent(self) -> None:
        value = self.storage_parent_edit.text().strip()
        if value:
            value = str(Path(value).expanduser())
            self.storage_parent_edit.setText(value)
        self.settings.setValue("collection_storage_parent", value)
        self.storage_parent_edit.setToolTip(value)
        self.storage_parent_edit.setCursorPosition(0)
        try:
            if read_transfer_settings()["local_parent"] != value:
                update_transfer_settings(local_parent=value)
        except (OSError, ValueError) as error:
            self.transfer_panel.status_label.setText(f"后台监控目录设置未保存：{error}")

    def _transfer_local_parent_changed(self, value: str) -> None:
        if value and not self.storage_parent_edit.hasFocus():
            if self.storage_parent_edit.text() != value:
                self.storage_parent_edit.setText(value)
                self.storage_parent_edit.setCursorPosition(0)
            self.settings.setValue("collection_storage_parent", value)

    def _finish_storage_edit(self) -> None:
        self._remember_storage_parent()
        self.storage_parent_edit.clearFocus()
        self.setFocus(Qt.OtherFocusReason)

    def _browse_storage_parent(self) -> None:
        chosen = QFileDialog.getExistingDirectory(self, "选择 episode 保存父目录", self.storage_parent_edit.text())
        if chosen:
            self.storage_parent_edit.setText(chosen)
            self._finish_storage_edit()

    def _open_episode_folder(self) -> None:
        path = (self.collection_status.get("storage") or {}).get("path")
        if path:
            QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    def _apply_storage_status(self, status: dict) -> None:
        settings = status.get("storage_settings") or {}
        if not self.storage_parent_edit.text().strip() and settings.get("default_parent"):
            self.storage_parent_edit.setText(settings["default_parent"])
            self._remember_storage_parent()
        storage = status.get("storage") or {}
        self.storage_open_button.setEnabled(bool(storage.get("path")) and status.get("local_bag_available", True))
        if storage:
            state = storage.get("state")
            label = {"recording": "录制中 · 暂存", "pending": "校验完成 · 待选择质量",
                     "archiving": "正在归档", "saved": "已归档", "failed": "归档未完成 · 请重新选择质量重试"}.get(state, str(state))
            folder = storage.get("folder") or (Path(storage["path"]).parent.name if state == "saved" and storage.get("path") else "")
            suffix = f" {folder}" if state == "saved" and folder in {"A", "B", "C", "D"} else ""
            text = f"{storage.get('episode', '')} · {label}{suffix}"
            color = "#c22525" if state == "failed" else "#285d49"
        else:
            text = "尚未开始本段" if settings.get("supported") else "等待支持归档的采集器连接"
            color = "#666"
        self.storage_episode_label.setText(text)
        details = [str(storage.get("path", "")), str(storage.get("message", ""))]
        self.storage_episode_label.setToolTip("\n".join(value for value in details if value))
        self.storage_open_button.setToolTip(str(storage.get("path", "")))
        self.storage_episode_label.setStyleSheet(f"color: {color};")

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
        task_label = self.task_selector.currentText()
        message.setWindowTitle(f"Replace {side} arm {task_label} Home?")
        message.setText(
            f"Record the {side} arm's current measured joint angles as the "
            f"{task_label} Home?"
        )
        message.setInformativeText(
            "The arm must be stopped. Recording does not move hardware; the next Home command moves this arm to the recorded pose."
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
        self._send(
            "capture_home", {"side": side, "task": task or self._selected_task()}
        )

    def _selected_task(self) -> str:
        task=str(self.task_selector.currentData())
        if task.startswith('__'):
            return getattr(self, 'current_task', DEFAULT_OPERATOR_TASK)
        self.current_task=task
        return task

    def _task_edit_allowed(self) -> bool:
        home=self.collection_status.get('after_home') or {}
        return (not self.task_edit_pending and not getattr(self,'arm_home_locked',False)
                and self.collection_state not in {'RECORDING','STOPPING','VALIDATING'}
                and home.get('state') not in {'armed','pending','running','failed'}
                and not any(button.isChecked() for button in self.arm_engage_buttons.values())
                and not any(button.isChecked() for button in self.arm_hold_buttons.values()))

    def _reload_task_selector(self, selected: str | None = None) -> None:
        tasks=get_operator_tasks()
        selected=selected if selected in tasks else next(iter(tasks))
        self.task_selector.blockSignals(True)
        self.task_selector.clear()
        for task,label in tasks.items():self.task_selector.addItem(label,task)
        self.task_selector.addItem('＋ 新增任务…','__add_task__')
        self.task_selector.addItem('－ 删除当前任务…','__delete_task__')
        self.task_selector.setCurrentIndex(self.task_selector.findData(selected))
        self.task_selector.blockSignals(False)
        self.current_task=selected
        self.settings.setValue('operator_task',selected)

    def _task_action_selected(self, index: int) -> None:
        action=str(self.task_selector.itemData(index))
        if not action.startswith('__'):
            self.current_task=action
            return
        selected=self.current_task
        self._reload_task_selector(selected)
        if not self._task_edit_allowed():
            self.feedback.appendPlainText('请先停止臂使能、完成本段录制和 Home 后再增删任务。')
            return
        if action=='__add_task__':
            name,ok=QInputDialog.getText(self,'新增任务','任务名称（新任务需要分别记录左右臂 Home）：')
            if not ok or not self._task_edit_allowed():return
            self._begin_task_edit('add',name)
        else:
            message=QMessageBox(self)
            message.setWindowTitle('永久删除当前任务')
            message.setText('永久删除任务“'+task_label(selected)+'”？')
            message.setInformativeText('将同时永久删除该任务的左右臂 Home 和对应的 Ready-to-Home 轨迹，无法通过本界面恢复。其他任务、共用 Ready 和已录制 episode 不受影响。')
            message.setStandardButtons(QMessageBox.Yes|QMessageBox.No)
            message.setDefaultButton(QMessageBox.No)
            message.finished.connect(lambda result, task=selected: self._delete_task_confirmed(task,result))
            message.open()
            self.task_delete_dialog=message

    def _delete_task_confirmed(self, task: str, result: int) -> None:
        if result!=QMessageBox.Yes or not self._task_edit_allowed():return
        self._begin_task_edit('delete',task)

    def _begin_task_edit(self, action: str, value: str) -> None:
        self.task_edit_pending=True
        self.task_selector.setEnabled(False)
        for button in [*self.arm_engage_buttons.values(),*self.arm_hold_buttons.values(),
                       *self.capture_home_buttons.values(),self.home_both_button,self.capture_ready_button,
                       self.move_ready_button,self.ready_to_home_button,*self.preset_buttons.values()]:
            button.setEnabled(False)
        if self.collection_state!='RECORDING':self.collection_record_button.setEnabled(False)
        self.feedback.appendPlainText('正在保存任务…' if action=='add' else '正在永久删除任务、Home 和对应轨迹…')
        def worker():
            result='';error=''
            try:
                if action=='add':result=add_operator_task(value)
                else:delete_operator_task(value)
            except Exception as exception:
                error=str(exception)
            try:self.task_edit_finished.emit(action,result,error)
            except RuntimeError:pass  # Window already closed; metadata cleanup can finish independently.
        threading.Thread(target=worker,name='operator-task-files',daemon=True).start()

    def _task_edit_completed(self, action: str, result: str, error: str) -> None:
        self.task_edit_pending=False
        if error:
            self.feedback.appendPlainText(('删除未完成，可再次删除以重试清理：' if action=='delete' else '新增任务失败：')+error)
        else:
            self._reload_task_selector(result or None)
            self.feedback.appendPlainText('任务已保存，请分别记录左右臂 Home。' if action=='add' else '任务及其 Home、对应轨迹已永久删除。')
        self.task_selector.setEnabled(not getattr(self,'arm_home_locked',False))
        self._poll_status()
        self._poll_collection_status()

    def _reconnect_arms(self) -> None:
        if not self.full_config or getattr(self,'arms_available',True):
            self.feedback.appendPlainText('机械臂已就绪或状态尚未确认，无需重连。')
            return
        from teleop_runtime.full_launcher import notify_full_session
        sent=notify_full_session(self.full_config['token'],command='reconnect_arms')
        self.feedback.appendPlainText('已请求重连机械臂；就绪后仍需手动使能。' if sent else '重连请求失败，请检查统一启动入口。')

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
        if self.connection_state != "connected" or not self.arm_hold_buttons[side].isEnabled():
            return
        held = self.arm_hold_buttons[side].isChecked()
        self._send("hold_arm", {"side": side, "enabled": not held})

    def _shortcut_home(self, side: str) -> None:
        if self.connection_state == "connected":
            home = self.collection_status.get("after_home") or {}
            if side == "both" and home.get("state") == "armed" and self.collection_state in {"STOPPING", "VALIDATING"}:
                return
            if side == "both" and home.get("state") in {"pending", "running", "failed"}:
                owner = self.full_config["token"][:16] if self.full_config else ""
                self._send_collection("retry_home", {"id": home["id"], "owner": owner})
                return
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
            button.setEnabled(ready and not self.task_edit_pending)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                button.setText("Start arm")
                button.blockSignals(False)
        for side, button in self.arm_hold_buttons.items():
            button.setEnabled(ready and not self.task_edit_pending)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                key_hint = ARM_HOLD_KEY_HINTS[side]
                button.setText(f"Hold arm ({key_hint})")
                button.blockSignals(False)
        for button in getattr(self, "capture_home_buttons", {}).values():
            button.setEnabled(ready and not self.task_edit_pending)
        for side, button in self.hand_engage_buttons.items():
            if self.hand_client is not None:continue
            button.setEnabled(ready)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                button.setText("Start hand")
                button.blockSignals(False)
        if self.task_edit_pending:
            for button in [self.home_both_button,self.capture_ready_button,self.move_ready_button,
                           self.ready_to_home_button,*self.preset_buttons.values()]:
                button.setEnabled(False)
        if self.hand_client is not None:self._apply_hand_status(self.hand_status)
        self.connect_action.setEnabled(state != "connected")
        self.task_selector.setEnabled(not getattr(self,"arm_home_locked",False) and not self.task_edit_pending)
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
        if self.task_edit_pending and command in {'engage','engage_arm','hold_arm','home_arm',
                'capture_home','capture_ready','move_ready','ready_to_home','run_preset'}:
            self.feedback.appendPlainText('任务文件处理中，请完成后再操作机械臂。')
            return
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

    def _collection_offline_text(self) -> str:
        if self.full_config:
            return "OFFLINE — 请启动触觉采集终端以启用缺帧监控"
        return "OFFLINE — 请启动采集终端"

    def _set_collection_offline(self) -> None:
        self.collection_state = "OFFLINE"
        self.collection_poll_timer.stop()
        self.collection_pending.clear()
        self.collection_buffer = b""
        self.collection_status_label.setText(self._collection_offline_text())
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
            self.collection_state = "STARTING"
            self.collection_status_label.setText("STARTING — 正在启动新 episode…")
            self.collection_status_label.setToolTip("")
            self.collection_status_label.setStyleSheet("color: #777; font-weight: bold;")
            if self.continuity_dialog is not None:
                self.continuity_dialog.close()
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
        arguments = {"return_arm_home": {"task": self._selected_task(), "owner": self.full_config["token"][:16]}} if command == "start" and self.full_config else None
        if command == "start" and (self.collection_status.get("storage_settings") or {}).get("supported"):
            parent = self.storage_parent_edit.text().strip()
            if not parent:
                self.feedback.appendPlainText("请选择右侧的保存父目录后再开始录制")
                return
            self._remember_storage_parent()
            arguments = dict(arguments or {}, storage_parent=self.storage_parent_edit.text().strip())
        if arguments is None:
            self._send_collection(command)
        else:
            self._send_collection(command, arguments)

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
            # A status request sent just before L may still describe the old
            # episode. Keep the cleared STARTING display until start replies.
            if command == "status" and "start" in self.collection_pending.values():
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
        self._apply_storage_status(status)
        state = str(status.get("state") or "UNKNOWN").upper()
        self.collection_state = state
        episode = status.get("current_episode") or status.get("last_episode") or "-"
        elapsed = status.get("elapsed_sec")
        home = status.get("after_home") or {}
        home_waiting = bool(home) and home.get("state") not in {"armed", "done"}
        detail = f" · {episode}"
        if status.get("continuity_enabled") is True:
            detail += " · 实时缺帧监控：开启"
        elif self.full_config and status.get("continuity_enabled") is False:
            detail += " · 实时缺帧监控：关闭（请使用触觉采集器）"
        readiness = status.get("start_readiness") or {}
        if not status.get("active") and readiness.get("message"):
            detail += " · " + readiness["message"]
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
        warnings = len(status.get("source_warnings") or []) + len(status.get("boundary_warnings") or []) + len(
            status.get("transport_warnings") or []
        )
        if failures:
            detail += f" · {len(failures)} failure(s)"
        elif warnings:
            detail += f" · {warnings} warning(s)"
        alerts = status.get("alerts") or []
        if status.get("auto_stop_reason"):
            detail += " · 本段已停止，正在收尾" if state in {"STOPPING", "VALIDATING"} else (
                (" · 已归类为报错，等待相机稳定" if readiness.get("ready") is False
                 else " · 已归类为报错，等待机械臂 Home" if home_waiting
                 else " · 已归类为报错，可再次踩踏板开始")
                if status.get("quality") == "报错" else " · 请保存报错或放弃分类")
        elif alerts:
            latest = max(alerts, key=lambda a: a.get("id", 0))
            detail += " · 预警：" + self._continuity_alert_text(latest)
            if state == "RECORDING":
                detail += " · 录制继续"
        if home.get("state") != "armed" and home:
            home_text = {"pending": "等待机械臂 Home", "running": "机械臂 Home 中",
                         "done": "机械臂已 Home", "failed": "Home 未完成：请点 Home both arms 或踩中间踏板重试"}
            detail += " · " + home_text.get(home.get("state"), "Home 状态未知")
            if home.get("state") == "failed" and home.get("message"):
                detail += "（" + home["message"] + "）"
        self.collection_status_label.setToolTip("\n".join(self._continuity_alert_text(a) for a in alerts))
        self.collection_status_label.setText(f"{state}{detail}")
        self._update_continuity_alerts(status)
        if state == "RECORDING" and alerts and not status.get("auto_stop_reason"):
            style = "color: #a35b00; font-weight: bold;"
        elif state == "RECORDING":
            style = "color: #c22525; font-weight: bold;"
        elif status.get("auto_stop_reason"):
            style = "color: #c22525; font-weight: bold;"
        elif alerts:
            style = "color: #a35b00; font-weight: bold;"
        elif self.full_config and status.get("continuity_enabled") is False:
            style = "color: #a35b00; font-weight: bold;"
        elif home_waiting:
            style = "color: #a35b00; font-weight: bold;"
        elif state in {"READY", "FINALIZED"}:
            style = "color: #248a3d; font-weight: bold;"
        elif state in {"INCOMPLETE", "INTERRUPTED", "DISCARDED"}:
            style = "color: #a35b00; font-weight: bold;"
        else:
            style = "color: #777; font-weight: bold;"
        self.collection_status_label.setStyleSheet(style)
        can_start = bool(status.get("can_start")) and not getattr(self, "arm_home_locked", False) and not self.task_edit_pending
        if (status.get("storage_settings") or {}).get("supported") and not self.storage_parent_edit.text().strip():
            can_start = False
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

    @staticmethod
    def _continuity_alert_text(alert: dict) -> str:
        text = f"{alert.get('topic')}: {alert.get('message')}"
        if alert.get("gap_ms") is not None:
            text += f"，间隔 {alert['gap_ms']:.1f} ms"
        if alert.get("estimated_missing_frames") is not None:
            kind = "本次间隔估算缺帧" if alert.get("kind") == "missing_frames" else "本次间隔缺失 30 Hz 时间步"
            text += f"，{kind} {alert['estimated_missing_frames']}"
        return text

    def _update_continuity_alerts(self, status: dict) -> None:
        # Warnings stay in the collection strip. Show the error dialog only
        # after finalization, independently of confirmation from the operator.
        alerts = [a for a in (status.get("alerts") or []) if a.get("severity") == "failure"]
        reason = status.get("auto_stop_reason")
        if reason and not any(a.get("id") == reason.get("id") for a in alerts):
            alerts.append(reason)
        if not alerts or status.get("active") or status.get("state") in {"RECORDING", "STOPPING", "VALIDATING"}:
            if self.continuity_dialog is not None:
                self.continuity_dialog.close()
            return
        episode_key = (alerts[0].get("session_id"), alerts[0].get("episode"))
        if self.continuity_episode_key != episode_key:
            if self.continuity_dialog is not None:
                self.continuity_dialog.close()
            self.continuity_episode_key = episode_key
            self.continuity_notice_keys.clear()
        keys = {(*episode_key, a.get("topic"), a.get("category"), a.get("severity")) for a in alerts}
        fresh = keys - self.continuity_notice_keys
        self.continuity_notice_keys.update(keys)
        if self.continuity_dialog is None and fresh:
            dialog = QDialog(self, Qt.Window | Qt.WindowDoesNotAcceptFocus)
            dialog.setWindowModality(Qt.NonModal)
            dialog.setAttribute(Qt.WA_ShowWithoutActivating)
            dialog.setMinimumWidth(560)
            layout = QVBoxLayout(dialog)
            label = QLabel()
            label.setTextFormat(Qt.PlainText)
            label.setWordWrap(True)
            layout.addWidget(label)
            buttons = QDialogButtonBox(QDialogButtonBox.Close)
            buttons.rejected.connect(dialog.close)
            layout.addWidget(buttons)
            self.continuity_dialog = dialog
            self.continuity_message = label
            dialog.finished.connect(lambda: self._continuity_dialog_closed(dialog))
            dialog.show()
        if self.continuity_dialog is None:
            return
        self.continuity_dialog.setWindowTitle("录制已因连续性错误停止")
        self.continuity_message.setStyleSheet("color: #c22525;")
        lines = [f"数据编号：{episode_key[1]}"]
        camera_waiting = (status.get("start_readiness") or {}).get("ready") is False
        home = status.get("after_home") or {}
        home_waiting = bool(home) and home.get("state") != "done"
        lines.append(("本段已归类为报错，等待机械臂 Home 完成。" if home_waiting
                      else "本段已归类为报错，等待相机稳定后再踩 L 开始。" if camera_waiting
                      else "本段已归类为报错，可再次踩 L 踏板开始。") if status.get("quality") == "报错"
                     else "本段失败，请保存报错或放弃分类后重新开始。")
        for alert in sorted(alerts, key=lambda a: (a.get("severity") != "failure", a.get("topic", "")))[:12]:
            text = self._continuity_alert_text(alert)
            text += f"（该类事件 {alert.get('count', 1)} 次；次数不作为退出阈值）"
            lines.append(text)
        if len(alerts) > 12:
            lines.append(f"另有 {len(alerts) - 12} 项，详见 collection_state.json。")
        self.continuity_message.setText("\n".join(lines))

    def _continuity_dialog_closed(self, dialog: QDialog) -> None:
        if self.continuity_dialog is dialog:
            self.continuity_dialog = None
            self.continuity_message = None
        dialog.deleteLater()

    def closeEvent(self, event) -> None:
        self._remember_storage_parent()
        self.transfer_panel.save_remote()
        if self.full_config:
            from teleop_runtime.full_launcher import notify_gui_session_exit
            notify_gui_session_exit(self.full_config['token'])
        if self.hand_client:self.hand_client.close()
        self.transfer_panel.timer.stop()
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
        self.arms_available = bool(status.get("arm_available", True))
        active = status.get("active", {})
        authorized = status.get("arm_authorized", active)
        home_locked = bool(status.get("home_locked", False))
        self.arm_home_locked = home_locked
        if home_locked and self.collection_state != "RECORDING":
            self.collection_record_button.setEnabled(False)
        arm_hold = status.get("arm_hold", {})
        for side, button in self.arm_engage_buttons.items():
            engaged = bool(authorized.get(side))
            button.blockSignals(True)
            button.setChecked(engaged)
            button.setText(
                "Arm running"
                if engaged and active.get(side)
                else "Arm enabled (Hold)" if engaged
                else "Start arm" if self.arms_available else "Arm unavailable [连接 / Exec / FCI]"
            )
            button.setEnabled(self.arms_available and not home_locked and not self.task_edit_pending)
            button.blockSignals(False)
        for side, button in self.arm_hold_buttons.items():
            held = bool(arm_hold.get(side))
            key_hint = ARM_HOLD_KEY_HINTS[side]
            button.blockSignals(True)
            button.setChecked(held)
            button.setEnabled(self.arms_available and not home_locked and not self.task_edit_pending)
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
                self.arms_available and not home_locked and not self.task_edit_pending
                and not self.arm_engage_buttons[side].isChecked()
                and not self.arm_hold_buttons[side].isChecked()
            )
        both_stopped = not any(
            button.isChecked() for button in self.arm_engage_buttons.values()
        ) and not any(
            button.isChecked() for button in self.arm_hold_buttons.values()
        )
        self.capture_ready_button.setEnabled(self.arms_available and both_stopped and not home_locked and not self.task_edit_pending)
        for button in [self.home_both_button,self.move_ready_button,self.ready_to_home_button,*self.preset_buttons.values()]:
            button.setEnabled(self.arms_available and not home_locked and not self.task_edit_pending)
        # Task setup remains available while arms are offline.
        self.task_selector.setEnabled(not home_locked and not self.task_edit_pending)
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
            if data.get('open_status') and data.get('open_status')!=old_data.get('open_status'):
                self._hand_event(f'{side}: '+data['open_status'])
            if data.get('reason') and (data.get('reason'),state)!=(old_data.get('reason'),old_data.get('state')):
                self._hand_event(f'{side}: {state}: '+data['reason'])
            button.blockSignals(True);button.setChecked(permit)
            button.setText('Opening hand…' if state=='OPENING' else ('Stop hand' if permit else 'Start hand')+f' [{state}]')
            button.setEnabled((state not in ('FAULT','HARD_FAULT','OFFLINE','RECOVERING')) and (state!='SOFT_HOLD' or permit) and (permit or data.get('can_engage',True)))
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
