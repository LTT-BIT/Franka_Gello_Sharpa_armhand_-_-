from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from .base import RetargetTarget
from .dexterous_cmc import CHANNELS, DexterousCMCConverter


Number = int | float


@dataclass
class JointMappingResult:
    value: Number
    source_value_deg: float


def clamp(value: float, lower: float, upper: float) -> float:
    lo = min(float(lower), float(upper))
    hi = max(float(lower), float(upper))
    return min(max(float(value), lo), hi)


def _as_float_pair(value: object, default: tuple[float, float]) -> tuple[float, float]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return float(value[0]), float(value[1])
    return default


def _source_weight_items(payload: Mapping[str, object]) -> list[tuple[str, float]]:
    sources = payload.get("sources", {})
    if isinstance(sources, Mapping):
        return [(str(name), float(weight)) for name, weight in sources.items()]
    if isinstance(sources, list):
        items: list[tuple[str, float]] = []
        for item in sources:
            if not isinstance(item, Mapping):
                continue
            name = item.get("joint")
            if name is None:
                continue
            items.append((str(name), float(item.get("weight", 1.0))))
        return items
    return []


class VisualAngleRetargeter:
    """
    Config-driven retargeter from clamped human visual joint angles to robot outputs.

    V3 CMC inputs are dedicated legacy-axis coordinates from the full attitude;
    other joints retain the kinematic mapping. SDK frames share this contract.
    """

    def __init__(
        self,
        config: Mapping[str, object],
        *,
        side: str = "right",
        config_path: Path | None = None,
        initial_values: list[Number] | None = None,
        joint_names: list[str] | None = None,
    ):
        self.config = dict(config)
        self.side = self._normalize_side(side)
        self.config_path = config_path
        self.manufacturer = str(self.config.get("manufacturer", ""))
        self.hand_joint = str(self.config.get("hand_joint", ""))
        self.joint_names = list(joint_names or self.config.get("joint_names", []))
        self.initial_values = list(initial_values or self.config.get("initial_values", []))
        if self.joint_names and len(self.initial_values) != len(self.joint_names):
            raise ValueError("initial_values length must match joint_names length.")
        self.current_values = list(self.initial_values)
        self.cmc_converter = None

    def map_visual_joints(
        self,
        joint_values: Mapping[str, float],
        *,
        timestamp_s: float | None = None,
    ) -> RetargetTarget:
        values = list(self.current_values)
        source_values: dict[str, float] = {}

        for joint_name, mapping in self._side_joint_mappings().items():
            if not isinstance(mapping, Mapping):
                continue
            if "fixed_value" not in mapping and any(
                source in CHANNELS and (source not in joint_values or not math.isfinite(float(joint_values[source])))
                for source, weight in _source_weight_items(mapping) if weight != 0
            ):
                continue
            result = self._map_one_joint(mapping, joint_values)
            source_values[joint_name] = result.source_value_deg
            self._set_value(values, joint_name, result.value)

        self.current_values = list(values)
        return RetargetTarget(
            manufacturer=self.manufacturer,
            hand_joint=self.hand_joint,
            side=self.side,
            values=values,
            joint_names=list(self.joint_names),
            value_by_joint={
                name: values[index]
                for index, name in enumerate(self.joint_names)
                if index < len(values)
            },
            source_values=source_values,
            timestamp_s=timestamp_s,
            config_path=self.config_path,
        )

    def reset(self) -> None:
        self.current_values = list(self.initial_values)
        if self.cmc_converter is not None:
            self.cmc_converter.reset()

    def _side_joint_mappings(self) -> Mapping[str, object]:
        sides = self.config.get("sides", {})
        if isinstance(sides, Mapping):
            side_payload = sides.get(self.side) or sides.get("default") or {}
            if isinstance(side_payload, Mapping):
                joints = side_payload.get("joints", side_payload)
                if isinstance(joints, Mapping):
                    return joints
        joints = self.config.get("joints", {})
        return joints if isinstance(joints, Mapping) else {}

    def _map_one_joint(
        self,
        mapping: Mapping[str, object],
        joint_values: Mapping[str, float],
    ) -> JointMappingResult:
        if "fixed_value" in mapping:
            value = self._coerce_output_value(float(mapping["fixed_value"]))
            return JointMappingResult(value=value, source_value_deg=0.0)

        input_range = _as_float_pair(mapping.get("input_range_deg"), (0.0, 90.0))
        source_weights = _source_weight_items(mapping)
        scale = float(mapping.get("scale", 1.0))
        anchor_at_input_start = mapping.get("gain_about_input_start") is True
        if anchor_at_input_start and len(source_weights) != 1:
            raise ValueError("gain_about_input_start requires exactly one source")
        gain_origin_deg = input_range[0] if anchor_at_input_start else 0.0
        gain_origin_rad = math.radians(gain_origin_deg)
        source_rad = sum(
            (float(joint_values.get(source_name, 0.0)) - gain_origin_rad) * weight
            for source_name, weight in source_weights
        )
        source_deg = gain_origin_deg + math.degrees(source_rad) * scale
        source_deg += float(mapping.get("offset_deg", 0.0))

        output_range = _as_float_pair(mapping.get("output_range"), (0.0, 1.0))
        normalized = self._normalize_value(source_deg, input_range)
        output = output_range[0] + normalized * (output_range[1] - output_range[0])
        return JointMappingResult(
            value=self._coerce_output_value(output),
            source_value_deg=source_deg,
        )

    def _normalize_value(self, value: float, input_range: tuple[float, float]) -> float:
        start, end = input_range
        if abs(end - start) <= 1e-9:
            return 0.0
        normalized = (float(value) - start) / (end - start)
        if bool(self.config.get("clamp_inputs", True)):
            normalized = clamp(normalized, 0.0, 1.0)
        return normalized

    def _coerce_output_value(self, value: float) -> Number:
        output = self.config.get("output", {})
        output_kind = ""
        if isinstance(output, Mapping):
            output_kind = str(output.get("kind", "float")).lower()
        if output_kind in {"uint8", "uint8_pose", "int"}:
            return int(round(clamp(value, 0.0, 255.0)))
        return float(value)

    def _set_value(self, values: list[Number], joint_name: str, value: Number) -> None:
        try:
            index = self.joint_names.index(joint_name)
        except ValueError:
            return
        if 0 <= index < len(values):
            values[index] = value

    @staticmethod
    def _normalize_side(side: object) -> str:
        normalized = str(side or "right").strip().lower()
        if normalized not in ("left", "right"):
            raise ValueError(f"Unsupported retargeting side: {side!r}")
        return normalized
