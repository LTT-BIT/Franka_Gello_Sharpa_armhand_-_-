"""Validated storage for dual-arm absolute Ready-to-Home trajectories."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable, Mapping, Sequence

import yaml

from operator_tasks import require_operator_task, task_label, task_registry_lock

SIDES = ("left", "right")
JOINT_COUNT = 7


def _joints(values: Iterable[object]) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != JOINT_COUNT or not all(math.isfinite(value) for value in result):
        raise ValueError("双臂绝对轨迹的每侧必须包含7个有限关节角")
    return result


@dataclass(frozen=True)
class DualArmSample:
    time_sec: float
    positions: Mapping[str, tuple[float, ...]]

    @classmethod
    def create(cls, time_sec: object, positions: Mapping[str, Iterable[object]]):
        timestamp = float(time_sec)
        if not math.isfinite(timestamp) or timestamp < 0.0:
            raise ValueError("轨迹采样时间必须是非负有限数值")
        if set(positions) != set(SIDES):
            raise ValueError("轨迹采样必须同时包含左右臂")
        return cls(timestamp, {side: _joints(positions[side]) for side in SIDES})


@dataclass(frozen=True)
class AbsoluteTrajectory:
    task: str
    label: str
    created_at: str
    joint_names: Mapping[str, tuple[str, ...]]
    samples: tuple[DualArmSample, ...]

    @property
    def duration_sec(self) -> float:
        return self.samples[-1].time_sec


def build_absolute_trajectory(
    task: object,
    samples: Sequence[DualArmSample],
    joint_names: Mapping[str, Iterable[object]],
) -> AbsoluteTrajectory:
    selected = require_operator_task(task)
    captured = tuple(samples)
    if len(captured) < 3:
        raise ValueError("双臂绝对轨迹至少需要3帧")
    start = captured[0].time_sec
    normalized = tuple(
        DualArmSample.create(
            sample.time_sec - start,
            sample.positions,
        )
        for sample in captured
    )
    times = tuple(sample.time_sec for sample in normalized)
    if any(times[index] <= times[index - 1] for index in range(1, len(times))):
        raise ValueError("双臂绝对轨迹时间必须严格递增")
    if times[-1] < 0.2 or times[-1] > 120.0:
        raise ValueError("双臂绝对轨迹时长必须在0.2到120秒之间")
    names = {side: tuple(str(value) for value in joint_names[side]) for side in SIDES}
    if any(len(names[side]) != JOINT_COUNT for side in SIDES):
        raise ValueError("双臂绝对轨迹的每侧必须包含7个关节名")
    return AbsoluteTrajectory(
        task=selected,
        label=task_label(selected),
        created_at=datetime.now(timezone.utc).isoformat(),
        joint_names=names,
        samples=normalized,
    )


class AbsoluteTrajectoryStore:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root) / "absolute_trajectories"

    def path(self, task: object) -> Path:
        return self.root / f"{require_operator_task(task)}.yaml"

    def save(self, trajectory: AbsoluteTrajectory, *, overwrite: bool = True) -> Path:
        with task_registry_lock():
            # A recording finalized after task deletion cannot recreate its YAML.
            require_operator_task(trajectory.task)
            return self._save(trajectory,overwrite=overwrite)

    def _save(self, trajectory: AbsoluteTrajectory, *, overwrite: bool = True) -> Path:
        path = self.path(trajectory.task)
        if path.exists() and not overwrite:
            raise ValueError(f"轨迹已存在: {trajectory.label}")
        data = {
            "version": 1,
            "kind": "dual_arm_absolute_joint_trajectory",
            "task": trajectory.task,
            "label": trajectory.label,
            "created_at": trajectory.created_at,
            "joint_names": {
                side: list(trajectory.joint_names[side]) for side in SIDES
            },
            "samples": [
                {
                    "time_sec": sample.time_sec,
                    "positions": {
                        side: list(sample.positions[side]) for side in SIDES
                    },
                }
                for sample in trajectory.samples
            ],
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except BaseException:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise
        return path
