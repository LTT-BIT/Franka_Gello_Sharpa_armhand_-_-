from __future__ import annotations

import json
import sys
import socketserver
import threading
import time
from pathlib import Path
from typing import Any

from .quality_lists import DEFAULT_QUALITY_DIR, QUALITY_LABELS, save_quality


class CollectorController:
    """Thread-safe command facade around the independent episode recorder."""

    def __init__(self, node: Any, recorder: Any, *, quality_dir: Path | None = None) -> None:
        self._node = node
        self._recorder = recorder
        self._operation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._phase = "WAITING"
        self._ready = False
        self._started_at: float | None = None
        self._last_state: dict[str, Any] = {}
        self._quality_dir = Path(
            quality_dir if quality_dir is not None else getattr(node, "quality_dir", DEFAULT_QUALITY_DIR)
        )
        self._quality: str | None = None
        self._monitor = getattr(node, "continuity_monitor", None)
        self._auto_stop_reason: dict[str, Any] | None = None
        self._seen_alerts: set[int] = set()
        self._worker: threading.Thread | None = None

    def set_ready(self) -> None:
        with self._state_lock:
            self._ready = True
            if not self._recorder.active:
                self._phase = "READY"

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            phase = self._phase
            ready = self._ready
            started_at = self._started_at
            last_state = dict(self._last_state)
            quality = self._quality
            auto_stop_reason = self._auto_stop_reason
        active = bool(self._recorder.active)
        current = self._recorder.current_bag_dir
        last_bag = self._recorder.last_bag_dir
        elapsed_sec = (
            max(0.0, time.monotonic() - started_at)
            if active and started_at is not None
            else None
        )
        health = self._monitor.snapshot() if self._monitor is not None else {}
        busy = phase in {"STOPPING", "VALIDATING"}
        return {
            "state": phase if busy else "RECORDING" if active else phase,
            "continuity_health": health,
            "alerts": health.get("alerts", []),
            "auto_stop_reason": auto_stop_reason,
            "source_warnings": list(last_state.get("source_warnings") or health.get("source_warnings") or []),
            "ready": ready,
            "active": active,
            "current_episode": current.name if current is not None else None,
            "current_bag": str(current) if current is not None else None,
            "last_episode": last_bag.name if last_bag is not None else None,
            "last_bag": str(last_bag) if last_bag is not None else None,
            "quality": quality,
            "quality_pending": bool(last_state) and quality is None and not active and not busy,
            "can_rate_quality": bool(last_state) and not active and phase not in {"STOPPING", "VALIDATING"},
            "elapsed_sec": elapsed_sec,
            "finalized": bool(last_state.get("finalized", False)),
            "failures": list(last_state.get("failures") or []),
            "boundary_warnings": list(last_state.get("boundary_warnings") or []),
            "transport_warnings": list(last_state.get("transport_warnings") or []),
            "can_start": ready and not active and not busy and not (auto_stop_reason and quality is None),
            "can_stop": active and not busy,
            "can_mark_milestone": active and phase == "RECORDING",
            "milestones": self._recorder.milestones,
            "can_discard": not active and last_bag is not None and not busy,
        }

    def start(self) -> dict[str, Any]:
        with self._operation_lock:
            with self._state_lock:
                if not self._ready:
                    raise RuntimeError("collector is still waiting for required topics")
                if self._phase in {"STOPPING", "VALIDATING"}:
                    raise RuntimeError("collector is still finalizing the previous episode")
            if self._monitor is not None and self._auto_stop_reason and self._quality is None:
                raise RuntimeError("请先保存上一段的报错或放弃分类")
            if self._recorder.active:
                raise RuntimeError("an episode is already recording")
            bag_dir = self._recorder.start()
            if self._monitor is not None:
                self._monitor.begin(bag_dir.name, time.monotonic())
            with self._state_lock:
                self._auto_stop_reason = None
                self._seen_alerts.clear()
                self._phase = "RECORDING"
                self._started_at = time.monotonic()
                self._last_state = {}
                self._quality = None
            self._node.get_logger().info(
                f"RECORDING: {bag_dir} (UI Stop or L will stop and validate)"
            )
            return self.status()

    def stop(self, *, interrupted: bool = False) -> dict[str, Any]:
        with self._operation_lock:
            if self._recorder.current_bag_dir is None:
                if self._auto_stop_reason is not None:
                    return self.status()
                raise RuntimeError("no episode is currently recording")
            return self._stop_locked(interrupted=interrupted)

    def mark_milestone(self) -> dict[str, Any]:
        with self._operation_lock:
            marker = self._recorder.mark_milestone()
            self._node.get_logger().info(
                f"Marked {marker['id']} at ROS time {marker['timestamp_ns']} ns; recording continues."
            )
            return self.status()

    def reconcile_exited_recorder(self) -> dict[str, Any]:
        """Finalize an unexpectedly exited recorder without racing a normal stop.

        The main loop may observe ``active=False`` during the brief interval in
        which a UI request is already finalizing the same bag. Waiting for the
        operation lock and rechecking the recorder makes that observation a
        harmless no-op instead of issuing a second stop.
        """
        with self._operation_lock:
            if self._recorder.current_bag_dir is None or self._recorder.active:
                return self.status()
            return self._stop_locked(interrupted=True)

    def stop_if_present(self, *, interrupted: bool, expected_episode: str | None = None) -> dict[str, Any]:
        """Idempotent shutdown stop used when Ctrl-C races a UI request."""
        with self._operation_lock:
            current = self._recorder.current_bag_dir
            if current is None or (expected_episode is not None and current.name != expected_episode):
                return self.status()
            return self._stop_locked(interrupted=interrupted)

    def wait_for_finalization(self) -> None:
        worker = self._worker
        if worker is not None:
            worker.join()

    def check_continuity(self) -> None:
        if self._monitor is None or not self._monitor.active:
            return
        report = self._monitor.snapshot()
        # At most one retained alert per stream/category, so this set is bounded
        # by episode events only while we periodically prune superseded IDs.
        # Schedule safety finalization before diagnostic logging: logging must
        # never prevent an already detected failure from stopping the bag.
        reason = report.get("auto_stop_reason")
        if reason is not None:
            self.request_stop(reason=reason)
        alerts = report.get("alerts", [])
        ids = {alert["id"] for alert in alerts}
        for alert in alerts:
            if alert["id"] not in self._seen_alerts:
                from .continuity import format_alert
                logger = self._node.get_logger()
                # Humble caches severity per Python call site. Keep each
                # severity on its own line rather than invoking a conditional.
                try:
                    if alert["severity"] == "failure":
                        logger.error(format_alert(alert))
                    else:
                        logger.warn(format_alert(alert))
                except Exception as error:
                    print(f"Continuity diagnostic logging failed: {error}; {format_alert(alert)}",
                          file=sys.stderr, flush=True)
        self._seen_alerts = ids

    def request_stop(self, *, reason: dict[str, Any] | None = None) -> dict[str, Any]:
        """Schedule a single finalization without blocking status polling."""
        with self._state_lock:
            if self._phase != "RECORDING":
                return_status = True
            else:
                if reason is not None and reason.get("episode") != self._monitor.episode:
                    return_status = True
                else:
                    self._phase = "STOPPING"
                    self._auto_stop_reason = reason
                    return_status = False
        if not return_status:
            episode = self._monitor.episode
            self._monitor.end()

            def finalize():
                try:
                    self.stop_if_present(interrupted=False, expected_episode=episode)
                except Exception as error:
                    # Retain failure/quality pending status, including classification
                    # write errors; the UI can retry rate_quality without restarting.
                    self._node.get_logger().error(f"Episode finalization failed: {error}")
                    with self._state_lock:
                        self._phase = "INCOMPLETE"

            self._worker = threading.Thread(target=finalize, name="episode-finalization", daemon=True)
            self._worker.start()
        return self.status()

    def _stop_locked(self, *, interrupted: bool) -> dict[str, Any]:
        with self._state_lock:
            self._phase = "STOPPING"
        self._node.get_logger().info(
            "STOPPING: finalizing and validating the active episode; please wait..."
        )
        if self._monitor is not None:
            report = self._monitor.end()
            with self._state_lock:
                self._auto_stop_reason = self._auto_stop_reason or report.get("auto_stop_reason")
            self._recorder.set_live_continuity(report)
        bag_dir = self._recorder.stop(interrupted=interrupted)
        state = _read_collection_state(bag_dir)
        with self._state_lock:
            self._started_at = None
            self._last_state = state
            final_phase = str(state.get("state") or "INCOMPLETE").upper()
            self._phase = "STOPPING" if self._auto_stop_reason is not None else final_phase
        if self._auto_stop_reason is not None and bag_dir is not None:
            try:
                self._save_quality_locked(bag_dir.name, "报错")
            finally:
                with self._state_lock:
                    self._phase = final_phase
        return self.status()

    def discard(self) -> dict[str, Any]:
        with self._operation_lock:
            if self._recorder.current_bag_dir is not None:
                self._stop_locked(interrupted=False)
            if self._recorder.last_bag_dir is None:
                raise RuntimeError("there is no episode to discard")
            bag_dir = self._recorder.mark_discarded()
            state = _read_collection_state(bag_dir)
            with self._state_lock:
                self._last_state = state
                self._phase = "DISCARDED"
            self._node.get_logger().warn(
                f"Bag marked discarded (not deleted): {bag_dir}"
            )
            self._save_quality_locked(bag_dir.name, "放弃")
            return self.status()

    def _save_quality_locked(self, episode: str, quality: str) -> None:
        # Do not keep advertising an earlier rating if saving a change fails.
        with self._state_lock:
            self._quality = None
        path = save_quality(self._quality_dir, episode, quality)
        with self._state_lock:
            self._quality = quality
        self._node.get_logger().info(f"数据质量已保存：{episode} → {quality} ({path})")

    def rate_quality(self, episode: str, quality: str) -> dict[str, Any]:
        with self._operation_lock:
            bag_dir = self._recorder.last_bag_dir
            if self._recorder.active or self._recorder.current_bag_dir is not None:
                raise RuntimeError("请等待录制结束并完成校验后再评价")
            if bag_dir is None or bag_dir.name != episode or not self._last_state:
                raise ValueError("数据编号已变化，请刷新后评价当前数据")
            if quality not in QUALITY_LABELS:
                raise ValueError(f"未知数据质量：{quality}")
            if self._last_state.get("auto_stop_reason") and quality not in {"报错", "放弃"}:
                raise ValueError("自动连续性失败仅允许归类为报错或放弃")
            if self._phase == "DISCARDED" and quality != "放弃":
                raise ValueError("该数据已弃用，不能通过质量评价恢复")
            if quality == "放弃":
                self._recorder.mark_discarded()
                with self._state_lock:
                    self._last_state = _read_collection_state(bag_dir)
                    self._phase = "DISCARDED"
            self._save_quality_locked(episode, quality)
            return self.status()

    def dispatch(self, command: str, arguments: dict[str, Any] | None = None) -> dict[str, Any]:
        if command == "rate_quality":
            arguments = arguments or {}
            return self.rate_quality(str(arguments.get("episode") or ""), str(arguments.get("quality") or ""))
        if command == "status":
            return self.status()
        if command == "start":
            return self.start()
        if command == "stop":
            return self.request_stop() if self._monitor is not None else self.stop()
        if command == "discard":
            return self.discard()
        if command == "mark_milestone":
            return self.mark_milestone()
        raise ValueError(f"unknown collector command: {command}")


def _read_collection_state(bag_dir: Path | None) -> dict[str, Any]:
    if bag_dir is None:
        return {}
    path = bag_dir / "collection_state.json"
    if not path.is_file():
        return {
            "state": "incomplete",
            "finalized": False,
            "failures": ["collection_state.json is missing"],
        }
    return json.loads(path.read_text(encoding="utf-8"))


class _CollectorRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        while True:
            line = self.rfile.readline(65_537)
            if not line:
                return
            request_id = None
            try:
                if len(line) > 65_536:
                    raise ValueError("request is too large")
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise ValueError("request must be a JSON object")
                request_id = request.get("id")
                command = str(request.get("command") or "").strip()
                arguments = request.get("arguments") or {}
                if not isinstance(arguments, dict):
                    raise ValueError("arguments must be a JSON object")
                result = self.server.controller.dispatch(command, arguments)
                response = {"id": request_id, "ok": True, "result": result}
            except Exception as error:
                response = {"id": request_id, "ok": False, "error": str(error)}
            self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))
            self.wfile.flush()


class CollectorControlServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], controller: CollectorController):
        host, _ = address
        if host != "127.0.0.1":
            raise ValueError("collector control server must bind to 127.0.0.1")
        self.controller = controller
        super().__init__(address, _CollectorRequestHandler)

    def start_in_thread(self) -> threading.Thread:
        thread = threading.Thread(
            target=self.serve_forever,
            kwargs={"poll_interval": 0.1},
            name="collector-control-server",
            daemon=True,
        )
        thread.start()
        return thread
