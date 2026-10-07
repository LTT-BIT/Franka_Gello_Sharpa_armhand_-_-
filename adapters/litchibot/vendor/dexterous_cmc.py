"""V3 CMC attitude -> legacy human CMC coordinates, only for robot hands.

The old URDF chain is Ry(-mirror*yaw) Rz(mirror*(swing-alpha)),
with metacarpal along local +Y. Match that bone direction, not Euler labels.
No visualization mapping, angular gain, or position limit is applied here.
Optional temporal damping and speed limits stabilize the projected coordinates.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import time

import numpy as np

from .rotations import matrix, ordered, rotation, unit


CONFIG_PATH = Path(__file__).resolve().parents[3] / "config/litchibot/dexterous_hand_cmc_mapping.json"
CHANNELS = ("dexterous_cmc_yaw", "dexterous_cmc_swing")
ALPHA = .7853982  # Fixed swing-joint origin in the original left/right URDFs.
STABILIZATION_DEFAULTS = dict(
    enabled=False, damping_start_deg=25., damping_strength_deg=15.,
    damping_time_s=.08, max_yaw_speed_deg_s=120., max_swing_speed_deg_s=180.,
    max_dt_s=.05,
)


def _bounded_minimum(cost, lower, upper):
    """Small, deterministic scalar fit over one frame's allowed yaw interval."""
    ratio = (math.sqrt(5.)-1.)/2.
    a, b = upper-ratio*(upper-lower), lower+ratio*(upper-lower)
    fa, fb = cost(a), cost(b)
    for _ in range(28):
        if fa < fb:
            upper, b, fb = b, a, fa
            a = upper-ratio*(upper-lower)
            fa = cost(a)
        else:
            lower, a, fa = a, b, fb
            b = lower+ratio*(upper-lower)
            fb = cost(b)
    return (lower+upper)/2.


def legacy_rotation(side, yaw, swing):
    mirror = 1. if side == "right" else -1.
    return rotation([0, 1, 0], -mirror*yaw) @ rotation([0, 0, 1], mirror*(swing-ALPHA))


def _align_direction(source, target):
    """Shortest proper rotation aligning two unit directions (no added roll)."""
    cross = np.cross(source, target)
    sine, cosine = float(np.linalg.norm(cross)), float(source @ target)
    if sine < 1e-10:
        if cosine < 0:
            raise ValueError("CMC neutral directions are antiparallel; configure an explicit neutral frame")
        return np.eye(3)
    return rotation(cross, math.atan2(sine, cosine))


class DexterousCMCConverter:
    def __init__(self, config=None):
        self.config = deepcopy(config) if config is not None else json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        self.stabilization = {**STABILIZATION_DEFAULTS, **self.config.get("stabilization", {})}
        if not isinstance(self.stabilization["enabled"], bool):
            raise ValueError("CMC stabilization.enabled must be a boolean")
        for key in STABILIZATION_DEFAULTS.keys() - {"enabled"}:
            value = float(self.stabilization[key])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"CMC stabilization.{key} must be finite and positive")
            self.stabilization[key] = value
        if not 1 < self.stabilization["damping_start_deg"] < 90 or self.stabilization["damping_strength_deg"] >= 90:
            raise ValueError("CMC damping angles must be below 90 degrees; start must exceed 1 degree")
        self.frames = {}
        for side in ("left", "right"):
            entry = self.config["sides"][side]
            basis = np.asarray(entry["solver_to_thumb_basis"], dtype=float)
            if basis.shape != (3, 3) or not np.isfinite(basis).all() or not np.allclose(basis @ basis.T, np.eye(3), atol=1e-7):
                raise ValueError("CMC basis must be a finite orthogonal 3x3 matrix")
            rpy = np.asarray(entry["thumb_neutral_rpy_rad"], dtype=float)
            neutral = np.asarray(entry["legacy_neutral_yaw_swing_rad"], dtype=float)
            if rpy.shape != (3,) or neutral.shape != (2,) or not np.isfinite(rpy).all() or not np.isfinite(neutral).all():
                raise ValueError("CMC neutral angles must be finite")
            root = ordered([[0, 0, 1], [0, 1, 0], [1, 0, 0]], rpy[::-1])
            source_axis = unit(entry["solver_metacarpal_axis"])
            old_neutral = legacy_rotation(side, *neutral)
            bone = old_neutral[:, 1]
            # Keep the Unity-aligned palm axes, then align its neutral bone to
            # the old open pose. Axial twist remains along the bone after this.
            transform = _align_direction(root @ basis @ source_axis, bone) @ root @ basis
            self.frames[side] = (transform, old_neutral, neutral)
        self.reset()

    def reset(self):
        self.previous = {}
        self.diagnostics = {}
        self.timestamps = {}

    def _stabilize(self, side, direction, candidate, timestamp_s):
        cfg = self.stabilization
        info = {"stabilization_enabled": cfg["enabled"], "damping_lambda": 0., "rate_limited": False}
        if not cfg["enabled"]:
            return candidate, info
        try:
            now = float(timestamp_s)
            if not math.isfinite(now):
                raise ValueError("Invalid timestamp")
            clock = "pose"
        except (TypeError, ValueError):
            now, clock = time.monotonic(), "monotonic"
        last = self.timestamps.get(side)
        self.timestamps[side] = (now, clock)
        if side not in self.previous or last is None:
            return candidate, info  # Initialize from the first valid attitude.
        dt = min(cfg["max_dt_s"], max(0., now-last[0])) if clock == last[1] else 0.
        info["dt_s"] = dt
        previous = self.previous[side]
        if dt == 0:
            return previous.copy(), {**info, "time_held": True}

        x, y, z = direction
        mirror = 1. if side == "right" else -1.
        radius = math.hypot(x, z)
        start = math.sin(math.radians(cfg["damping_start_deg"]))
        hold = math.sin(math.radians(1.))
        u = float(np.clip((start-radius)/(start-hold), 0., 1.))
        activation = u*u*(3.-2.*u)
        damping = (math.sin(math.radians(cfg["damping_strength_deg"]))*activation)**2 * cfg["damping_time_s"]/dt
        yaw_step = math.radians(cfg["max_yaw_speed_deg_s"])*dt
        swing_step = math.radians(cfg["max_swing_speed_deg_s"])*dt
        yaw_low, yaw_high = previous[0]-yaw_step, previous[0]+yaw_step
        swing_low, swing_high = previous[1]-swing_step, previous[1]+swing_step

        def fit_swing(yaw):
            signed_radius = mirror*x*math.cos(yaw) + z*math.sin(yaw)
            swing = ALPHA-math.atan2(signed_radius, y)
            swing += 2*math.pi*round((previous[1]-swing)/(2*math.pi))
            return min(swing_high, max(swing_low, swing))

        def cost(yaw):
            spread = ALPHA-fit_swing(yaw)
            fitted = np.array([mirror*math.sin(spread)*math.cos(yaw),
                               math.cos(spread), math.sin(spread)*math.sin(yaw)])
            return float(np.sum((fitted-direction)**2)) + damping*(yaw-previous[0])**2

        if radius < hold:
            yaw = previous[0]
        elif damping > 0:
            best = _bounded_minimum(cost, yaw_low, yaw_high)
            # Include exact previous/raw commands to avoid numerical creep and
            # to handle a minimum at the speed boundary.
            yaw = min((best, previous[0], min(yaw_high, max(yaw_low, candidate[0]))), key=cost)
        else:
            yaw = min(yaw_high, max(yaw_low, candidate[0]))
        swing = fit_swing(yaw)
        info.update(damping_lambda=damping,
                    rate_limited=bool(abs(yaw-previous[0]) >= yaw_step-1e-8 or
                                      abs(swing-previous[1]) >= swing_step-1e-8))
        return np.array([yaw, swing]), info

    def convert(self, state):
        side = state.side
        if side not in self.frames:
            raise ValueError(f"Unsupported CMC hand side: {side!r}")
        transform, neutral_rotation, neutral = self.frames[side]
        pose = getattr(state, "thumb_cmc_pose", {}) or {}
        try:
            q = pose.get("quaternion_wxyz")
            if q is not None:
                measured = matrix(q)
            else:
                axes = getattr(state, "solver_context", {}).get("calibration_snapshot", {}).get("thumb", {}).get("cmc_axes")
                angles = [pose.get(name + "_rad") for name in ("yaw", "swing", "twist")]
                if axes is None or len(axes) != 3 or any(a is None for a in angles):
                    raise ValueError("Missing complete CMC attitude")
                measured = ordered(axes, angles)
                if not np.isfinite(measured).all():
                    raise ValueError("Invalid CMC attitude")
        except (ValueError, TypeError):
            self.diagnostics[side] = {"status": "missing_attitude"}
            return {}  # Missing measurement must not become a zero motor command.

        target = transform @ measured @ transform.T @ neutral_rotation
        x, y, z = target[:, 1]
        mirror = 1. if side == "right" else -1.
        previous = self.previous.get(side, neutral)
        radius = math.hypot(x, z)
        singular = radius < math.sin(math.radians(1.))
        if singular:
            yaw = float(previous[0])
            signed_radius = mirror*x*math.cos(yaw) + z*math.sin(yaw)
            swing = ALPHA-math.atan2(signed_radius, y)
        else:
            yaw = math.atan2(z, mirror*x)
            spread = math.atan2(radius, y)
            candidates = []
            for a, b in ((yaw, ALPHA-spread), (yaw+math.pi, ALPHA+spread)):
                a += 2*math.pi*round((float(previous[0])-a)/(2*math.pi))
                candidates.append(np.array([a, b]))
            # Start on the old open-thumb branch (swing <= alpha). Once
            # tracking, allow crossing the pole without a pi jump in yaw.
            yaw, swing = (min(candidates, key=lambda candidate: float(np.sum((candidate-previous)**2)))
                          if side in self.previous else candidates[0])
        (yaw, swing), stability = self._stabilize(
            side, target[:, 1], np.array([yaw, swing]), getattr(state, "timestamp_s", None))
        self.previous[side] = np.array([yaw, swing])
        fitted = legacy_rotation(side, yaw, swing)
        self.diagnostics[side] = {
            **stability,
            "status": "yaw_unobservable" if singular else "valid",
            "direction_error_deg": math.degrees(math.acos(float(np.clip(fitted[:, 1] @ target[:, 1], -1., 1.)))),
            "orientation_residual_deg": math.degrees(math.acos(float(np.clip((np.trace(fitted.T @ target)-1)/2, -1., 1.)))),
        }
        return dict(zip(CHANNELS, (float(yaw), float(swing))))
