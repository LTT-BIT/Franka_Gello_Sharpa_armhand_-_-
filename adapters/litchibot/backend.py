"""Headless LitchiBot lifecycle compatible with HandWorker; dry-run by default."""
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
import importlib
import json
import math
import os
import sys
import threading
import time

import numpy as np

from .retarget import JOINT_NAMES, SharpaRetargeter


@dataclass
class SideStatus:
    sending: bool = False
    fault: str | None = None
    sent: int = 0
    solve_seconds: float = 0.0


@dataclass
class Status:
    sides: dict = field(default_factory=lambda: {s: SideStatus() for s in ("left", "right")})
    errors: int = 0
    last_error: str | None = None


def load_sdk(sdk_root=None):
    if sdk_root:
        root = Path(sdk_root).resolve()
        package_root = root / "src" if (root / "src/litchibot_glove").is_dir() else root
        if "litchibot_glove" in sys.modules:
            loaded = Path(sys.modules["litchibot_glove"].__file__).resolve()
            if not loaded.is_relative_to(root):
                raise RuntimeError(f"Another LitchiBot SDK is already loaded: {loaded}")
        sys.path.insert(0, str(package_root))
    sdk = importlib.import_module("litchibot_glove")
    if sdk_root and not Path(sdk.__file__).resolve().is_relative_to(Path(sdk_root).resolve()):
        raise RuntimeError("SDK import did not resolve to selected SDK root")
    return sdk


class LitchiBotHandPipeline:
    sides = ("left", "right")

    def __init__(self, *, profile="LYG226360006", data_root=None, sdk_root=None,
                 receiver_id=None, rate=30.0, dry_run=True, debug_log=None,
                 stale_timeout=0.25, sender_factory=None, session=None, sdk=None,
                 retarget_configs=None):
        if not math.isfinite(rate) or not 0 < rate <= 60:
            raise ValueError("LitchiBot rate must be in (0, 60] Hz")
        if not math.isfinite(stale_timeout) or stale_timeout <= 0:
            raise ValueError("stale_timeout must be positive")
        if not dry_run and sender_factory is None:
            raise ValueError("Hardware output needs an explicit sender factory")
        self.status = Status()
        self.dry_run = bool(dry_run)
        self.rate = rate
        self.stale_timeout = stale_timeout
        self.profile = profile
        self.sdk = sdk or load_sdk(sdk_root)
        self.sdk_joint_names = tuple(self.sdk.CANONICAL_JOINT_NAMES)
        self.joint_names = {s: JOINT_NAMES for s in self.sides}
        self.mapper = {s: SharpaRetargeter(s, (retarget_configs or {}).get(s)) for s in self.sides}
        self._lock = threading.Lock()
        self._latest = {}
        self._processed = {}
        self._previous = {}
        self._previous_ns = {}
        self._active = {s: False for s in self.sides}
        self._sender_factory = sender_factory
        self._sender = None
        self._last_send = {}
        self._closed = False
        self.last_command = {s: None for s in self.sides}
        self.statistics = {s: {"count": 0, "dt_s": [], "max_delta_rad": 0.0,
                              "statuses": Counter(), "invalid_glove_joints": Counter()}
                           for s in self.sides}
        self._log = Path(debug_log).open("x", encoding="utf-8") if debug_log else None
        self._diagnostics = None
        self.session = None
        try:
            diagnostic_path = os.environ.get('LITCHIBOT_DIAGNOSTICS_FILE')
            if diagnostic_path:
                if not self.dry_run:
                    raise ValueError('Diagnostic capture is dry-run only')
                from .diagnostics import DiagnosticLog
                self._diagnostics = DiagnosticLog(diagnostic_path)
            self.session = session or self.sdk.GloveSession.from_profile(profile, data_root=data_root,
                receiver_id=receiver_id, side="both", solver="v3.5", solve_rate_hz=rate,
                pose_output="both_type_hand_pose")
            self.session.subscribe_solved(self._on_solved)
            self.session.subscribe_errors(self._on_error)
            if self._diagnostics is not None:
                self.session.subscribe_raw(self._diagnostics.raw)
            self._write({"type": "metadata", "source": "litchibot", "dry_run": self.dry_run,
                "profile": profile, "sdk_path": getattr(self.sdk, "__file__", None),
                "joint_names": list(JOINT_NAMES), "unit": "rad"})
            self.session.start()
        except Exception:
            self.close()
            raise

    def _write(self, row):
        if self._log:
            self._log.write(json.dumps(row, allow_nan=False) + "\n")

    def _on_solved(self, frame):
        if self._diagnostics is not None:
            self._diagnostics.solved(frame)
        with self._lock:
            for pose in frame.hands:
                side = getattr(pose.side, "value", pose.side)
                if side in self.sides:
                    self._latest[side] = (frame, pose)

    def _on_error(self, error):
        # Error callbacks never throw into SDK threads or the arm loop.
        with self._lock:
            self.status.errors += 1
            self.status.last_error = str(error)
        if self._diagnostics is not None:
            self._diagnostics.write('sdk_error',str(error))

    def tick(self, *, active):
        now = time.monotonic_ns()
        with self._lock:
            latest = dict(self._latest)
        for side in self.sides:
            status = self.status.sides[side]
            engaged = bool(active.get(side, False))
            status.sending = False
            if self._active[side] and not engaged and self._sender is not None:
                try:
                    self._sender.disengage(side)
                except Exception as error:
                    self._fault(side, error)
            self._active[side] = engaged
            sample = latest.get(side)
            if sample is None:
                status.fault = f"waiting for {side} LitchiBot data"
                continue
            frame, pose = sample
            age_s = (now - frame.source_received_monotonic_ns) / 1e9
            if age_s < 0 or age_s > self.stale_timeout:
                status.fault = f"{side} LitchiBot data stale ({age_s:.3f}s)"
                if self._sender is not None:
                    try:
                        self._sender.disengage(side)
                    except Exception as error:
                        self._fault(side, error)
                continue
            identity = (frame.source_id, frame.sequence, frame.source_raw_sequence)
            if self._processed.get(side) == identity:
                continue
            self._processed[side] = identity
            try:
                started = time.monotonic()
                row = self.mapper[side].map(pose, self.sdk_joint_names, frame.source_received_monotonic_ns,
                                          diagnostics=self._diagnostics is not None)
                positions = np.asarray(row["positions_rad"])
                previous = self._previous.get(side)
                delta = positions-previous if previous is not None else np.zeros(22)
                stats = self.statistics[side]
                if side in self._previous_ns:
                    stats["dt_s"].append((now-self._previous_ns[side])/1e9)
                stats["count"] += 1
                stats["max_delta_rad"] = max(stats["max_delta_rad"], float(np.max(np.abs(delta))))
                stats["statuses"][getattr(pose.status, "value", pose.status)] += 1
                stats["invalid_glove_joints"].update(row["invalid_glove_joints"])
                self._previous[side] = positions
                self._previous_ns[side] = now
                row.update(type="target", source="litchibot", profile=frame.profile_id,
                    source_id=frame.source_id, sequence=frame.sequence,
                    source_raw_sequence=frame.source_raw_sequence,
                    source_received_monotonic_ns=frame.source_received_monotonic_ns,
                    source_device_timestamp_ns=frame.source_device_timestamp_ns,
                    solved_monotonic_ns=frame.solved_monotonic_ns,
                    generated_monotonic_ns=now, source_age_s=age_s,
                    glove_status=getattr(pose.status, "value", pose.status),
                    glove_joint_names=list(self.sdk_joint_names),
                    glove_joint_angles_rad=[float(v) if math.isfinite(v) else None for v in pose.joint_angles_rad],
                    valid_glove_joint_mask=[bool(v) for v in pose.valid_joint_mask],
                    node_positions_root_m=np.asarray(pose.node_positions_root_m).tolist(),
                    delta_rad=delta.tolist(), min_rad=float(positions.min()), max_rad=float(positions.max()),
                    dry_run=self.dry_run, engaged=engaged, sent=False)
                # Only this branch can reach a transport; dry-run cannot even create a sender.
                if not self.dry_run and engaged and now-self._last_send.get(side, 0) >= 1e9/self.rate:
                    if self._sender is None:
                        self._sender = self._sender_factory()
                    self._sender.send(row)
                    self._last_send[side] = now
                    status.sending = True
                    status.sent += 1
                    row["sent"] = True
                self.last_command[side] = row
                status.fault = None
                status.solve_seconds = time.monotonic()-started
                self._write(row)
                if self._diagnostics is not None:
                    self._diagnostics.write('target',row)
            except Exception as error:
                self._fault(side, error)

    def _fault(self, side, error):
        self.status.errors += 1
        self.status.last_error = f"{side} LitchiBot: {error}"
        self.status.sides[side].fault = str(error)
        self.status.sides[side].sending = False

    def request_open(self, *, sides=None, duration=2.0):
        # No invented Sharpa open pose. An operator opening policy needs separate validation.
        for side in sides or self.sides:
            self.status.sides[side].fault = "LitchiBot Sharpa open action is not implemented"

    def summary(self):
        result = {}
        for side, stats in self.statistics.items():
            intervals = np.asarray(stats["dt_s"])
            result[side] = {k: dict(v) if isinstance(v, Counter) else v
                           for k, v in stats.items() if k != "dt_s"}
            result[side].update(mean_dt_s=float(intervals.mean()) if intervals.size else None,
                p50_dt_s=float(np.percentile(intervals, 50)) if intervals.size else None,
                p95_dt_s=float(np.percentile(intervals, 95)) if intervals.size else None,
                max_dt_s=float(intervals.max()) if intervals.size else None,
                target_hz=float(1/intervals.mean()) if intervals.size and intervals.mean() else None)
        return {"type": "summary", "dry_run": self.dry_run, "sides": result,
                "errors": self.status.errors, "last_error": self.status.last_error,
                "sent": sum(s.sent for s in self.status.sides.values())}

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self.session is not None:
                self.session.close()
        finally:
            try:
                if self._sender is not None:
                    self._sender.close()
            finally:
                if self._log:
                    self._write(self.summary())
                    self._log.close()
                    self._log = None
                if self._diagnostics is not None:
                    self._diagnostics.close()
