from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Protocol


@dataclass
class RetargetTarget:
    """Generic robot-hand retargeting result."""

    manufacturer: str
    hand_joint: str
    side: str
    values: list[int | float]
    joint_names: list[str]
    value_by_joint: dict[str, int | float]
    source_values: dict[str, float] = field(default_factory=dict)
    timestamp_s: float | None = None
    config_path: Path | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "manufacturer": self.manufacturer,
            "hand_joint": self.hand_joint,
            "side": self.side,
            "values": list(self.values),
            "joint_names": list(self.joint_names),
            "value_by_joint": dict(self.value_by_joint),
            "source_values": dict(self.source_values),
            "timestamp_s": self.timestamp_s,
            "config_path": None if self.config_path is None else str(self.config_path),
        }


class VisualJointRetargeter(Protocol):
    """Retargeter consuming mapped human joint radians for its config version."""

    def map_hand_frame(self, frame: object, *, solver_variant: str | None = None) -> RetargetTarget:
        ...

    def map_visual_joints(
        self,
        joint_values: Mapping[str, float],
        *,
        timestamp_s: float | None = None,
    ) -> RetargetTarget:
        ...
