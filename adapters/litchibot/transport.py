"""Named Sharpa target transport to a separate ROS relay. No vendor SDK here."""
import json
import math
import socket
import time
import uuid

import numpy as np

from .retarget import JOINT_LIMITS, JOINT_NAMES


def normalize_feedback(side, names, positions):
    if side not in ("left", "right") or len(names) != len(positions):
        raise ValueError("Invalid feedback side/shape")
    # Existing sharpa_driver publishes left_/right_ prefixed joint names.
    names = [n[len(side)+1:] if n.startswith(side+'_') else n for n in names]
    if len(set(names)) != len(names) or set(names) != set(JOINT_NAMES):
        raise ValueError("Sharpa feedback names do not match selected side")
    values = dict(zip(names, positions))
    return [values[n] for n in JOINT_NAMES]


def validate_packet(packet, *, now_ns=None, max_age_s=0.25, allow_dry_run=False):
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    if packet.get("schema") != "litchibot.sharpa_target.v1" or (
        packet.get("dry_run") is not False and not (allow_dry_run and packet.get("dry_run") is True)
    ):
        raise ValueError("Not an enabled LitchiBot Sharpa packet")
    if packet.get("side") not in ("left", "right"):
        raise ValueError("Invalid target side")
    age = (now_ns-int(packet["source_received_monotonic_ns"]))/1e9
    if not 0 <= age <= max_age_s:
        raise ValueError("Stale/future hand input")
    if tuple(packet.get("joint_names", ())) != JOINT_NAMES:
        raise ValueError("Sharpa joint order mismatch")
    q = np.asarray(packet.get("positions_rad"), dtype=float)
    if q.shape != (22,) or not np.isfinite(q).all():
        raise ValueError("Expected 22 finite joint radians")
    if np.any(q < JOINT_LIMITS[:, 0]-1e-8) or np.any(q > JOINT_LIMITS[:, 1]+1e-8):
        raise ValueError("Sharpa joint limit violation")
    if packet.get("engaged") is not True:
        raise ValueError("Hand is not engaged")
    if not isinstance(packet.get("valid_glove_joint_count"), int) or packet["valid_glove_joint_count"] <= 0:
        raise ValueError("No valid glove joints")
    if not packet.get("session_id"):
        raise ValueError("Missing session identity")
    if not isinstance(packet.get("sequence"), int) or packet["sequence"] < 0:
        raise ValueError("Invalid sequence")
    return q


class UdpSharpaSender:
    def __init__(self, host="127.0.0.1", port=5572):
        if host not in ("127.0.0.1", "localhost"):
            raise ValueError("Sharpa relay is local-host only")
        if not 0 < int(port) < 65536:
            raise ValueError("Invalid Sharpa relay port")
        self.address = (host, int(port))
        self.session_id = str(uuid.uuid4())
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, row):
        fields = ("side", "sequence", "source_id", "source_received_monotonic_ns",
                  "source_device_timestamp_ns", "solved_monotonic_ns", "joint_names", "positions_rad",
                  "valid_glove_joint_count", "valid_target_mask", "held_joints", "initial_fallback_joints",
                  "dry_run", "engaged")
        packet = {key: row[key] for key in fields}
        packet.update(schema="litchibot.sharpa_target.v1", session_id=self.session_id)
        validate_packet(packet)
        self.socket.sendto(json.dumps(packet, allow_nan=False).encode(), self.address)

    def disengage(self, side):
        self.socket.sendto(json.dumps({"schema": "litchibot.sharpa_stop.v1", "side": side,
                           "session_id": self.session_id}).encode(), self.address)

    def close(self):
        try:
            for side in ("left", "right"):
                self.disengage(side)
        finally:
            self.socket.close()


class TargetGate:
    """Validate freshness, ordering, feedback acquisition and slew before ROS output."""
    def __init__(self, *, max_age_s=0.25, max_speed_rad_s=2.0, allow_dry_run=False):
        if max_speed_rad_s is not None and (not math.isfinite(max_speed_rad_s) or max_speed_rad_s <= 0):
            raise ValueError("Positive finite slew limit required")
        self.max_age_s = max_age_s
        self.max_speed = max_speed_rad_s
        self.allow_dry_run = bool(allow_dry_run)
        self.previous = {}
        self.sequence = {}

    def clear(self, side):
        self.previous.pop(side, None)

    def accept(self, packet, *, feedback, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        target = validate_packet(packet, now_ns=now_ns, max_age_s=self.max_age_s,
                                 allow_dry_run=self.allow_dry_run)
        side = packet["side"]
        if feedback is None:
            raise ValueError("No Sharpa feedback for acquisition")
        measured, feedback_ns = feedback
        measured = np.asarray(measured, dtype=float)
        if measured.shape != (22,) or not np.isfinite(measured).all():
            raise ValueError("Invalid Sharpa feedback")
        if not 0 <= (now_ns-feedback_ns)/1e9 <= self.max_age_s:
            raise ValueError("Stale Sharpa feedback")
        if np.any(measured < JOINT_LIMITS[:, 0]-0.02) or np.any(measured > JOINT_LIMITS[:, 1]+0.02):
            raise ValueError("Sharpa feedback outside joint limits")
        identity = (packet["session_id"], side)
        if packet["sequence"] <= self.sequence.get(identity, -1):
            raise ValueError("Duplicate/out-of-order target")
        prev = self.previous.get(side)
        if prev is None or prev[2] != packet["session_id"] or (now_ns-prev[1])/1e9 > self.max_age_s:
            base, dt = np.clip(measured, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]), 1/30
        else:
            base, dt = prev[0], min(0.05, max(0, (now_ns-prev[1])/1e9))
        # None explicitly selects value-preserving validation, not a higher speed cap.
        q = target.copy() if self.max_speed is None else base + np.clip(target-base, -self.max_speed*dt, self.max_speed*dt)
        # Never let unavailable channels jump to the startup fallback on real hardware.
        mask = np.asarray(packet.get("valid_target_mask"), dtype=bool)
        if mask.shape != (22,):
            raise ValueError("Invalid target validity mask")
        q[~mask] = base[~mask]
        q = np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        self.sequence[identity] = packet["sequence"]
        self.previous[side] = (q.copy(), now_ns, packet["session_id"])
        return q
