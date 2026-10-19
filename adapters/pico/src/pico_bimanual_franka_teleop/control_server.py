"""The teleop operator's control backend: console state + JSON-TCP server.

One unified backend, swappable frontends. `OperatorConsole` is the single
activation/request/status object the input sources and the control loop
share; `OperatorControlServer` exposes it over line-delimited JSON
({"id", "command", "arguments"} answered by {"id", "ok",
"result"|"error"}, gluon's demonstration-server protocol). The PySide6
GUI in apps/operator_gui is a shell over this protocol and is the
operator frontend; the process itself is headless.

The server thread never touches robot state directly. Commands mutate the
console's activation and one-shot request flags; the 100 Hz loop reads
them at its own pace, exactly as it read the old keyboard, and `status`
returns the loop's latest published snapshot.
"""

from __future__ import annotations

import json
import socketserver
import threading
import time

from operator_tasks import DEFAULT_OPERATOR_TASK, require_operator_task

from .types import SIDES


class OperatorConsole:
    """Thread-safe operator state shared by the GUI and coordinator.

    State arrives from the control server, and feedback accumulates in a ring
    the `status` command serves to frontends.
    """

    def __init__(self) -> None:
        self.sides = SIDES
        # `active` remains the arm activation map for compatibility with every
        # existing arm input source. Hands have independent ownership so a
        # pedal can start/stop MANUS following without moving the FR3.
        self.active = {side: False for side in SIDES}
        # A held arm remains an active command source, but ignores leader
        # motion and repeatedly sends the target captured by the coordinator.
        self.arm_hold = {side: False for side in SIDES}
        self._arm_hold_resume = {side: False for side in SIDES}
        self.arm_available = True
        self.arm_unavailable_reason = ""
        self.home_locked = False
        self.episode_home: dict = {}
        self.hand_active = {side: False for side in SIDES}
        self.requests = {
            "open_hands": False,
            "open_left_hand": False,
            "open_right_hand": False,
            "reset": False,
            "episode_home": False,
            "reset_left": False,
            "reset_right": False,
            "capture_home_left": False,
            "capture_home_right": False,
            "capture_ready": False,
            "move_ready": False,
            "ready_to_home": False,
            "preset_q": False,
            "preset_w": False,
            "preset_e": False,
            "stop_action": False,
            "abort_action": False,
        }
        self._status_line = "starting..."
        self._feedback: list[str] = []
        self._lock = threading.Lock()

    # -- operator-state interface ----------------------------------------
    # Every mutation happens under the lock: the server thread writes while
    # the 100 Hz control loop reads, and the keyboard-era code was safe only
    # because both sides ran on one thread. Without the lock a request set
    # between take_requests' read and swap lands in the discarded dict and
    # the operator's click is silently lost.

    def poll(self) -> dict[str, bool]:
        with self._lock:
            return dict(self.active)

    def poll_hands(self) -> dict[str, bool]:
        with self._lock:
            return dict(self.hand_active)

    def poll_holds(self) -> dict[str, bool]:
        with self._lock:
            return dict(self.arm_hold)

    def take_requests(self) -> dict[str, bool]:
        with self._lock:
            taken = self.requests
            self.requests = {name: False for name in taken}
        return taken

    def set_active(self, side: str, engaged: bool, *, target: str = "both") -> None:
        if side not in self.sides:
            raise ValueError(f"{side} is not configured for this run")
        if target not in {"arm", "hand", "both"}:
            raise ValueError(f"unknown activation target: {target}")
        with self._lock:
            if not self.arm_available and engaged and target in {"arm", "both"}:
                raise ValueError(self.arm_unavailable_reason or "机械臂不可用")
            if self.home_locked and engaged and target in {"arm", "both"}:
                raise ValueError("Home not confirmed complete; arm following is locked")
            if target in {"arm", "both"}:
                self.active[side] = engaged
                # Start/stop is an explicit exit from hold. Re-engagement is
                # re-anchored by the relative mapper on the next control tick.
                self.arm_hold[side] = False
                self._arm_hold_resume[side] = False
            if target in {"hand", "both"}:
                self.hand_active[side] = engaged

    def set_hold(self, side: str, held: bool) -> None:
        if side not in self.sides:
            raise ValueError(f"{side} is not configured for this run")
        if not isinstance(held, bool):
            raise ValueError("enabled must be a boolean")
        with self._lock:
            if not self.arm_available:
                raise ValueError(self.arm_unavailable_reason or "机械臂不可用")
            if self.home_locked and not held:
                raise ValueError("Home not confirmed complete; arm Hold is locked")
            if held:
                if not self.arm_hold[side]:
                    self._arm_hold_resume[side] = self.active[side]
                self.active[side] = False
                self.arm_hold[side] = True
            else:
                self.arm_hold[side] = False
                self.active[side] = self._arm_hold_resume[side]
                self._arm_hold_resume[side] = False
        self.show(f"{side} arm: {'holding target' if held else 'hold released'}")

    def set_arm_available(self, available: bool, reason: str = "") -> None:
        with self._lock:
            self.arm_available = available
            self.arm_unavailable_reason = reason
            if not available:
                for side in SIDES:
                    self.active[side] = False
                    self.arm_hold[side] = False
                    self._arm_hold_resume[side] = False
                for name in self.requests:
                    if not name.startswith("open_"):
                        self.requests[name] = False
                if self.episode_home.get("state") in {"queued", "running"}:
                    self.episode_home.update(state="failed", message=reason)

    def begin_arm_home(self) -> None:
        """Keep existing arm permits behind Hold while Home owns the bus."""
        with self._lock:
            self.home_locked = True
            for side in SIDES:
                self._arm_hold_resume[side] = self.active[side] or self._arm_hold_resume[side]
                self.active[side] = False
                self.arm_hold[side] = True

    def finish_arm_home(self, succeeded: bool) -> None:
        with self._lock:
            # Only a confirmed successful reset permits a new pedal release.
            self.home_locked = not succeeded
            # begin_arm_home already latched Hold. Preserve any explicit
            # Stop/DISENGAGE ALL issued while the reset was running.
            for side in SIDES:
                self.active[side] = False

    def queue_episode_home(self, arguments: dict) -> dict:
        job_id = arguments.get("id")
        if not isinstance(job_id, str) or not job_id or len(job_id) > 100:
            raise ValueError("invalid episode Home id")
        task = require_operator_task(arguments.get("task", DEFAULT_OPERATOR_TASK))
        with self._lock:
            if not self.arm_available:
                raise ValueError(self.arm_unavailable_reason or "机械臂不可用")
            if self.episode_home.get("id") == job_id:
                return dict(self.episode_home)
            if self.episode_home.get("state") in {"queued", "running"}:
                raise ValueError("episode Home is already in progress")
            self.episode_home = {"id": job_id, "task": task, "state": "queued", "message": "等待 Home"}
            self.requests["episode_home"] = dict(self.episode_home)
            return dict(self.episode_home)

    def update_episode_home(self, job_id: str, state: str, message: str = "") -> None:
        with self._lock:
            if self.episode_home.get("id") == job_id and self.episode_home.get("state") in {"queued", "running"}:
                self.episode_home.update(state=state, message=message)

    def request(self, name: str, value=True) -> None:
        with self._lock:
            if name not in self.requests:
                raise ValueError(f"Unknown request: {name}")
            self.requests[name] = value

    def disable_all(self, reason: str) -> None:
        with self._lock:
            if self.episode_home.get("state") in {"queued", "running"}:
                self.episode_home.update(state="failed", message=reason)
            self.active = {side: False for side in SIDES}
            self.arm_hold = {side: False for side in SIDES}
            self._arm_hold_resume = {side: False for side in SIDES}
            self.hand_active = {side: False for side in SIDES}
        self.show(f"all sides disengaged: {reason}")

    def deny(self, side: str, reason: str) -> None:
        """Deny the selected arm path without changing independent hand state."""
        with self._lock:
            self.active[side] = False
            self.arm_hold[side] = False
            self._arm_hold_resume[side] = False
        self.show(f"{side} arm: {reason}")

    def deny_hand(self, side: str, reason: str) -> None:
        with self._lock:
            self.hand_active[side] = False
        self.show(f"{side} hand: {reason}")

    def show(self, message: str) -> None:
        with self._lock:
            self._feedback.append(time.strftime("%H:%M:%S ") + str(message))
            del self._feedback[:-50]

    def set_status(self, text: str) -> None:
        with self._lock:
            self._status_line = str(text)

    def close(self) -> None:
        pass

    # -- frontend-facing snapshot ----------------------------------------


    def snapshot(self) -> dict:
        with self._lock:
            return {
                "status_line": self._status_line,
                "arm_available": self.arm_available,
                "arm_unavailable_reason": self.arm_unavailable_reason,
                "feedback": list(self._feedback),
                "active": dict(self.active),
                "arm_hold": dict(self.arm_hold),
                "arm_authorized": {side: self.active[side] or self._arm_hold_resume[side] for side in SIDES},
                "home_locked": self.home_locked,
                "episode_home": dict(self.episode_home),
                "hand_active": dict(self.hand_active),
                "sides": list(self.sides),
            }


class _RequestHandler(socketserver.StreamRequestHandler):
    def setup(self) -> None:
        super().setup()
        self.server.client_connected()

    def finish(self) -> None:
        try:
            super().finish()
        finally:
            self.server.client_disconnected()

    def handle(self) -> None:
        while True:
            line = self.rfile.readline()
            if not line:
                return
            request_id = None
            try:
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise ValueError("Request must be a JSON object.")
                request_id = request.get("id")
                command = str(request.get("command", ""))
                arguments = request.get("arguments", {})
                if not isinstance(arguments, dict):
                    raise ValueError("arguments must be an object.")
                result = self.server.dispatch(command, arguments)
                response = {"id": request_id, "ok": True, "result": result}
            except Exception as exception:  # noqa: BLE001 - report to client
                response = {"id": request_id, "ok": False, "error": str(exception)}
            self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))
            self.wfile.flush()


def _require_side(arguments) -> str:
    side = str(arguments.get("side", ""))
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    return side


class OperatorControlServer(socketserver.ThreadingTCPServer):
    """Serves engage/disengage, arm-home, and hand-open commands."""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, console: OperatorConsole):
        super().__init__(address, _RequestHandler)
        self.keyboard = console
        self.snapshot = console.snapshot
        self._thread: threading.Thread | None = None
        self._clients = 0
        self._clients_lock = threading.Lock()

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True
        )
        self._thread.start()

    def close(self) -> None:
        self.shutdown()
        self.server_close()
        self.keyboard.disable_all("operator server stopped")
        self.keyboard.request("abort_action")

    def client_connected(self) -> None:
        with self._clients_lock:
            self._clients += 1

    def client_disconnected(self) -> None:
        with self._clients_lock:
            self._clients = max(0, self._clients - 1)
            last_client = self._clients == 0
        if last_client:
            self.keyboard.disable_all("operator frontend disconnected")
            self.keyboard.request("abort_action")

    # -- commands ---------------------------------------------------------

    def _queue_episode_home(self, arguments: dict) -> dict:
        # The short-lived coordinator connection must never substitute for
        # the persistent operator frontend or mask its disconnect guard.
        with self._clients_lock:
            if self._clients < 2:
                raise ValueError("自动 Home 需要 Operator GUI 保持连接，请连接后按 B 重试")
        return self.keyboard.queue_episode_home(arguments)

    def _disengage_all(self) -> None:
        self.keyboard.disable_all("operator frontend")
        self.keyboard.request("abort_action")

    def _scoped_request(
        self,
        arguments,
        *,
        both: str,
        left: str,
        right: str,
        value=True,
    ) -> None:
        scope = str(arguments.get("side", "both"))
        if scope == "both":
            self.keyboard.request(both, value)
        elif scope == "left":
            self.keyboard.request(left, value)
        elif scope == "right":
            self.keyboard.request(right, value)
        else:
            raise ValueError("side must be left, right, or both")

    def dispatch(self, command: str, arguments: dict):
        if command in {"home_arm", "capture_home", "capture_ready", "move_ready",
                       "ready_to_home", "run_preset", "episode_home"}:
            snapshot=self.snapshot()
            if not snapshot.get("arm_available", True):
                raise ValueError(snapshot.get("arm_unavailable_reason") or "机械臂不可用")
        commands = {
            "status": self.snapshot,
            "engage": lambda: self.keyboard.set_active(
                _require_side(arguments), True, target="both"
            ),
            "disengage": lambda: self.keyboard.set_active(
                _require_side(arguments), False, target="both"
            ),
            "engage_arm": lambda: self.keyboard.set_active(
                _require_side(arguments), True, target="arm"
            ),
            "disengage_arm": lambda: self.keyboard.set_active(
                _require_side(arguments), False, target="arm"
            ),
            "hold_arm": lambda: self.keyboard.set_hold(
                _require_side(arguments), arguments.get("enabled")
            ),
            "engage_hand": lambda: self.keyboard.set_active(
                _require_side(arguments), True, target="hand"
            ),
            "disengage_hand": lambda: self.keyboard.set_active(
                _require_side(arguments), False, target="hand"
            ),
            "disengage_all": self._disengage_all,
            "open_hand": lambda: self._scoped_request(
                arguments,
                both="open_hands",
                left="open_left_hand",
                right="open_right_hand",
            ),
            "episode_home": lambda: self._queue_episode_home(arguments),
            "home_arm": lambda: self._scoped_request(
                arguments,
                both="reset",
                left="reset_left",
                right="reset_right",
                value=require_operator_task(
                    arguments.get("task", DEFAULT_OPERATOR_TASK)
                ),
            ),
            "capture_home": lambda: self.keyboard.request(
                f"capture_home_{_require_side(arguments)}",
                require_operator_task(
                    arguments.get("task", DEFAULT_OPERATOR_TASK)
                ),
            ),
            "capture_ready": lambda: self.keyboard.request("capture_ready"),
            "move_ready": lambda: self.keyboard.request("move_ready"),
            "ready_to_home": lambda: self.keyboard.request(
                "ready_to_home",
                require_operator_task(
                    arguments.get("task", DEFAULT_OPERATOR_TASK)
                ),
            ),
            "run_preset": lambda: self.keyboard.request(
                f"preset_{_require_preset(arguments)}"
            ),
            "stop_action": lambda: self.keyboard.request("stop_action"),
        }
        handler = commands.get(command)
        if handler is None:
            raise ValueError(f"Unknown command: {command}")
        result = handler()
        return {} if result is None else result


def _require_preset(arguments) -> str:
    key = str(arguments.get("key", "")).lower()
    if key not in ("q", "w", "e"):
        raise ValueError("preset key must be q, w, or e")
    return key
