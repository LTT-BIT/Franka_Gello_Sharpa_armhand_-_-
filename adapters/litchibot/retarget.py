"""Adapt SDK HandPose to the staged manufacturer Sharpa mapping."""
from pathlib import Path
from types import SimpleNamespace
import json
import math

import numpy as np

from .vendor.dexterous_cmc import CHANNELS, DexterousCMCConverter
from .vendor.visual_angle import VisualAngleRetargeter, _source_weight_items

CONFIG_ROOT = Path(__file__).resolve().parents[2] / "config/litchibot"
SCHEMA = json.loads((CONFIG_ROOT / "sharpa_schema.json").read_text())
JOINT_NAMES = tuple(SCHEMA["joint_names"])
JOINT_LIMITS = np.asarray(SCHEMA["joint_limits"], dtype=float)


class SharpaRetargeter:
    def __init__(self, side, config_path=None):
        if side not in ("left", "right"):
            raise ValueError("Invalid hand side")
        self.side = side
        self.config_path = Path(config_path or CONFIG_ROOT / "wave.json")
        config = json.loads(self.config_path.read_text())
        if tuple(config["joint_names"]) != JOINT_NAMES:
            raise ValueError("Sharpa joint order must match sharpa_wave_22_v1")
        self.mapper = VisualAngleRetargeter(config, side=side, config_path=self.config_path)
        self.cmc = DexterousCMCConverter()
        self.ever_valid = np.zeros(len(JOINT_NAMES), dtype=bool)

    def map(self, pose, joint_names, timestamp_ns, *, diagnostics=False):
        side = getattr(pose.side, "value", pose.side)
        if side != self.side:
            raise ValueError(f"Cross-side pose: expected {self.side}, received {side}")
        angles = np.asarray(pose.joint_angles_rad, dtype=float)
        mask = np.asarray(pose.valid_joint_mask, dtype=bool)
        if angles.shape != (len(joint_names),) or mask.shape != angles.shape:
            raise ValueError("Invalid SDK joint/mask shape")
        valid = dict(zip(joint_names, mask & np.isfinite(angles)))
        # GUI uses SDK kinematic_hand_pose, not the canonical array or Unity angles.
        inputs = {k: float(v) for k, v in (pose.kinematic_hand_pose or {}).items()
                  if valid.get(k, False) and math.isfinite(float(v))}
        q = pose.thumb_cmc_orientation_palm_xyzw
        if valid.get("thumb_cmc_yaw") and valid.get("thumb_cmc_swing") and q is not None:
            q = np.asarray(q, dtype=float)
            if q.shape == (4,) and np.isfinite(q).all() and np.linalg.norm(q) > 1e-10:
                state = SimpleNamespace(side=self.side, timestamp_s=timestamp_ns / 1e9,
                    thumb_cmc_pose={"quaternion_wxyz": q[[3, 0, 1, 2]].tolist()})
                inputs.update(self.cmc.convert(state))
        previous = np.asarray(self.mapper.current_values, dtype=float).copy()
        target = self.mapper.map_visual_joints(inputs, timestamp_s=timestamp_ns / 1e9)
        values = np.asarray(target.values, dtype=float)
        vendor_values = values.copy() if diagnostics else None
        target_valid = np.zeros(len(JOINT_NAMES), dtype=bool)
        held = []
        fallback = []
        mappings = self.mapper._side_joint_mappings()
        for index, name in enumerate(JOINT_NAMES):
            rule = mappings.get(name, {})
            sources = [n for n, w in _source_weight_items(rule) if w != 0]
            available = ("fixed_value" in rule or bool(sources) and all(n in inputs for n in sources))
            if available and math.isfinite(values[index]):
                target_valid[index] = True
                self.ever_valid[index] = True
            else:
                # Vendor mapping defaults absent scalars to zero. Override only
                # unavailable dependencies; never interpret an invalid scalar as a measurement.
                values[index] = previous[index]
                (held if self.ever_valid[index] else fallback).append(name)
        if values.shape != (22,) or not np.isfinite(values).all():
            raise ValueError("Sharpa target requires 22 finite radians")
        clipped = np.clip(values, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        self.mapper.current_values = clipped.tolist()
        result = {
            "side": self.side, "joint_names": list(JOINT_NAMES), "positions_rad": clipped.tolist(),
            "valid_target_mask": target_valid.tolist(), "held_joints": held,
            "initial_fallback_joints": fallback,
            "invalid_glove_joints": [n for n in joint_names if not valid[n]],
            "valid_glove_joint_count": int(sum(valid.values())),
            "mapping_inputs_rad": inputs, "cmc_diagnostics": dict(self.cmc.diagnostics.get(self.side, {})),
            "clipped_joints": [n for n, a, b in zip(JOINT_NAMES, values, clipped) if a != b],
        }
        if diagnostics:
            rules = {}
            for name, rule in mappings.items():
                if 'fixed_value' in rule:
                    rules[name] = {'fixed_value': rule['fixed_value']}
                    continue
                start, end = rule.get('input_range_deg', (0.0,90.0))
                source_deg = target.source_values.get(name)
                if source_deg is None:
                    continue
                normalized = (source_deg-start)/(end-start) if abs(end-start)>1e-9 else 0.0
                rules[name] = {'source_value_deg': source_deg,
                    'normalized_before_vendor_clamp': normalized,
                    'vendor_input_saturation': bool(self.mapper.config.get('clamp_inputs',True))
                                               and (normalized<0 or normalized>1),
                    'input_range_deg': [start,end], 'output_range': rule.get('output_range')}
            result['diagnostic_stages'] = {'vendor_output_before_hold_rad': vendor_values.tolist(),
                'previous_target_rad': previous.tolist(), 'after_hold_before_schema_clip_rad': values.tolist(),
                'final_target_rad': clipped.tolist(), 'mapping_rules': rules}
        return result


def joint_table(config_path=None):
    config = json.loads(Path(config_path or CONFIG_ROOT / "wave.json").read_text())
    table = []
    for side in ("left", "right"):
        mapper = VisualAngleRetargeter(config, side=side)
        for index, name in enumerate(JOINT_NAMES):
            rule = mapper._side_joint_mappings().get(name, {})
            start, end = rule.get("input_range_deg", (0, 1))
            low, high = rule.get("output_range", (0, 0))
            slope = (high-low) / (end-start) if end != start else 0
            table.append({"side": side, "index": index, "joint": name, "unit": "rad",
                "sdk_min": float(JOINT_LIMITS[index, 0]), "sdk_max": float(JOINT_LIMITS[index, 1]),
                "mapping_min": min(low, high), "mapping_max": max(low, high),
                "source_weights": dict(_source_weight_items(rule)),
                "source_signs": {n: int(np.sign(w * slope * rule.get('scale', 1)))
                                 for n, w in _source_weight_items(rule)},
                "fixed_value": rule.get("fixed_value")})
    return table
