from dataclasses import dataclass
from typing import Any

from rclpy.node import Node

from .collector_contract import validate_recording_contract


@dataclass(frozen=True)
class TopicConfig:
    name: str
    topic: str
    type_name: str
    required: bool = True
    min_frequency_hz: float = 1.0
    max_gap_ms: float = 150.0


@dataclass(frozen=True)
class CollectorConfig:
    data_root: str
    topics: tuple[TopicConfig, ...]
    static_topics: tuple[TopicConfig, ...]
    trim_start_sec: float
    trim_end_sec: float
    provenance: dict[str, Any]
    continuity_enabled: bool = False
    rgb_preflight_enabled: bool = False
    rgb_brightness_threshold: float = 25.0
    rgb_preflight_stable_sec: float = 3.0


def load_collector_config(node: Node) -> CollectorConfig:
    data_root = _get_required_string(node, "data_root", "data")
    topics = _load_topic_configs(node, "topics", default_required=True)
    static_topics = _load_topic_configs(node, "static_topics", default_required=False)
    if not topics:
        raise ValueError("No topics configured. Add entries under ros__parameters.topics.")
    trim_start_sec = _get_nonnegative_float(node, "trim_start_sec", 0.0)
    trim_end_sec = _get_nonnegative_float(node, "trim_end_sec", 0.0)

    continuity_enabled = _coerce_bool(_get_or_declare(node, "continuity_enabled", False), False)
    rgb_enabled = _coerce_bool(_get_or_declare(node, "rgb_preflight_enabled", False), False)
    rgb_threshold = _get_nonnegative_float(node, "rgb_brightness_threshold", 25.0)
    rgb_duration = _get_nonnegative_float(node, "rgb_preflight_stable_sec", 3.0)
    from .rgb_brightness import validate_policy
    validate_policy(rgb_threshold, rgb_duration)
    provenance = {
        "rgb_preflight_policy": dict(enabled=rgb_enabled, threshold=rgb_threshold, stable_sec=rgb_duration),
        "continuity_enabled": continuity_enabled,
        "bag_contract_version": _get_or_declare(node, "bag_contract_version", 1),
        "teleoperator": _get_required_string(node, "teleoperator", ""),
        "workcell_id": _get_required_string(node, "workcell_id", ""),
        "workcell_config_hash": _get_required_string(node, "workcell_config_hash", ""),
        "control_config_id": _get_required_string(node, "control_config_id", ""),
        "timestamp_policy": _get_required_string(node, "timestamp_policy", ""),
        "trim_start_sec": trim_start_sec,
        "trim_end_sec": trim_end_sec,
        "calibration_ids": _prefix_values(node, "calibration_ids"),
        "device_identities": _prefix_values(node, "device_identities"),
        "topics": {
            topic.name: {
                "topic": topic.topic,
                "type": topic.type_name,
                "required": topic.required,
                "min_frequency_hz": topic.min_frequency_hz,
                "max_gap_ms": topic.max_gap_ms,
            }
            for topic in topics
        },
    }
    validate_recording_contract(provenance)

    return CollectorConfig(
        data_root=data_root,
        topics=tuple(topics),
        static_topics=tuple(static_topics),
        trim_start_sec=trim_start_sec,
        trim_end_sec=trim_end_sec,
        provenance=provenance,
        continuity_enabled=continuity_enabled,
        rgb_preflight_enabled=rgb_enabled,
        rgb_brightness_threshold=rgb_threshold,
        rgb_preflight_stable_sec=rgb_duration,
    )


def _load_topic_configs(
    node: Node,
    prefix: str,
    default_required: bool,
) -> list[TopicConfig]:
    raw_params = node.get_parameters_by_prefix(prefix)
    grouped: dict[str, dict[str, Any]] = {}

    for suffix, parameter in raw_params.items():
        name, _, field = suffix.partition(".")
        if not name or not field:
            continue
        grouped.setdefault(name, {})[field] = parameter.value

    topics = []
    for name in sorted(grouped):
        fields = grouped[name]
        topic = _clean_string(fields.get("topic"))
        type_name = _clean_string(fields.get("type"))
        if not topic or not type_name:
            raise ValueError(f"Topic entry '{name}' must define non-empty topic and type fields.")

        required = _coerce_bool(fields.get("required"), default_required)
        min_frequency_hz = float(fields.get("min_frequency_hz", 1.0))
        max_gap_ms = float(fields.get("max_gap_ms", 150.0))
        if min_frequency_hz <= 0.0 or max_gap_ms <= 0.0:
            raise ValueError(
                f"Topic entry '{name}' frequency and max gap must be positive."
            )
        topics.append(
            TopicConfig(
                name=name,
                topic=topic,
                type_name=type_name,
                required=required,
                min_frequency_hz=min_frequency_hz,
                max_gap_ms=max_gap_ms,
            )
        )

    return topics


def _get_required_string(node: Node, name: str, default: str) -> str:
    value = _clean_string(_get_or_declare(node, name, default))
    if not value:
        raise ValueError(f"ROS parameter '{name}' must be a non-empty string.")
    return value


def _get_or_declare(node: Node, name: str, default: Any) -> Any:
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def _get_nonnegative_float(node: Node, name: str, default: float) -> float:
    value = float(_get_or_declare(node, name, default))
    if not value >= 0.0:
        raise ValueError(f"ROS parameter '{name}' must be non-negative.")
    return value


def _clean_string(value: Any) -> str:
    return str(value or "").strip()


def _coerce_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
    return bool(value)


def _prefix_values(node: Node, prefix: str) -> dict[str, Any]:
    return {
        name: parameter.value
        for name, parameter in node.get_parameters_by_prefix(prefix).items()
        if name
    }
