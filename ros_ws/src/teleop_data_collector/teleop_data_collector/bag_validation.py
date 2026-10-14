from __future__ import annotations

from bisect import bisect_left
from collections import defaultdict
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Iterable

from .image_metadata import read_image_metadata

from teleop_core.contract import (
    COMMAND_STATUS_TOPIC,
    DATA_LEFT_ARM_JOINT_NAMES,
    DATA_RIGHT_ARM_JOINT_NAMES,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
    WUJI_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
    WUJI_STATE_TOPIC,
    WUJI_TELEMETRY_STATUS_TOPIC,
)


@dataclass
class StreamSamples:
    bag_times_ns: list[int] = field(default_factory=list)
    header_times_ns: list[int] = field(default_factory=list)


def _clock_timing_report(times: list[int]) -> dict[str, Any]:
    gaps = [current - previous for previous, current in zip(times, times[1:])]
    observed_duration_ns = max(0, times[-1] - times[0]) if times else 0
    frequency_hz = (
        (len(times) - 1) * 1_000_000_000 / observed_duration_ns
        if len(times) > 1 and observed_duration_ns > 0
        else 0.0
    )
    return {
        "timestamp_count": len(times),
        "first_time_ns": times[0] if times else None,
        "last_time_ns": times[-1] if times else None,
        "observed_duration_ns": observed_duration_ns,
        "frequency_hz": frequency_hz,
        "max_internal_gap_ms": (max(gaps) / 1_000_000 if gaps else None),
        "nonmonotonic_count": sum(gap <= 0 for gap in gaps),
    }


def stream_timing_report(
    samples: StreamSamples,
    *,
    global_start_ns: int,
    global_end_ns: int,
    global_source_start_ns: int | None = None,
    global_source_end_ns: int | None = None,
    trim_start_sec: float = 0.0,
    trim_end_sec: float = 0.0,
) -> dict[str, Any]:
    times = samples.bag_times_ns
    bag_timing = _clock_timing_report(times)
    source_full_timing = _clock_timing_report(samples.header_times_ns)
    effective_source_start_ns = (
        global_source_start_ns + int(trim_start_sec * 1_000_000_000)
        if global_source_start_ns is not None
        else None
    )
    effective_source_end_ns = (
        global_source_end_ns - int(trim_end_sec * 1_000_000_000)
        if global_source_end_ns is not None
        else None
    )
    effective_source_times = [
        timestamp
        for timestamp in samples.header_times_ns
        if (
            effective_source_start_ns is None
            or timestamp >= effective_source_start_ns
        )
        and (
            effective_source_end_ns is None
            or timestamp <= effective_source_end_ns
        )
    ]
    source_timing = _clock_timing_report(effective_source_times)
    duration_ns = max(0, global_end_ns - global_start_ns)
    leading_gap_ms = (
        (times[0] - global_start_ns) / 1_000_000 if times else None
    )
    trailing_gap_ms = (
        (global_end_ns - times[-1]) / 1_000_000 if times else None
    )
    bag_timing.update(
        {
            "leading_gap_ms": leading_gap_ms,
            "trailing_gap_ms": trailing_gap_ms,
        }
    )
    source_times = effective_source_times
    source_timing.update(
        {
            "leading_gap_ms": (
                (source_times[0] - effective_source_start_ns) / 1_000_000
                if source_times and effective_source_start_ns is not None
                else None
            ),
            "trailing_gap_ms": (
                (effective_source_end_ns - source_times[-1]) / 1_000_000
                if source_times and effective_source_end_ns is not None
                else None
            ),
        }
    )
    return {
        "message_count": len(times),
        "first_bag_time_ns": times[0] if times else None,
        "last_bag_time_ns": times[-1] if times else None,
        "recording_duration_ns": duration_ns,
        # Keep the original flat bag-time fields for compatibility with older
        # reports and tooling.
        "frequency_hz": bag_timing["frequency_hz"],
        "max_internal_gap_ms": bag_timing["max_internal_gap_ms"],
        "leading_gap_ms": leading_gap_ms,
        "trailing_gap_ms": trailing_gap_ms,
        "source_header_count": len(samples.header_times_ns),
        "first_source_header_ns": (
            samples.header_times_ns[0] if samples.header_times_ns else None
        ),
        "last_source_header_ns": (
            samples.header_times_ns[-1] if samples.header_times_ns else None
        ),
        "source_header_nonmonotonic": source_full_timing["nonmonotonic_count"],
        "bag_timing": bag_timing,
        "source_full_timing": source_full_timing,
        "source_timing": source_timing,
    }


def timing_failures(
    name: str,
    report: dict[str, Any],
    *,
    min_frequency_hz: float,
    max_gap_ms: float,
    boundary_gap_ms: float | None = None,
    trim_start_sec: float = 0.0,
    trim_end_sec: float = 0.0,
) -> list[str]:
    if report["message_count"] == 0:
        return [f"{name}: no messages in bag"]
    failures = []
    if report["message_count"] < 2:
        failures.append(f"{name}: fewer than two messages")
    source = report["source_timing"]
    if report["source_header_count"] != report["message_count"]:
        failures.append(f"{name}: source header timestamps are missing")
    if report["source_full_timing"]["nonmonotonic_count"]:
        failures.append(f"{name}: source header timestamps are not monotonic")
    if source["timestamp_count"] < 2:
        failures.append(f"{name}: fewer than two source timestamps")
    elif source["frequency_hz"] < min_frequency_hz:
        failures.append(
            f"{name}: source_frequency_hz {source['frequency_hz']:.2f} Hz below "
            f"{min_frequency_hz:.2f} Hz"
        )
    gap = source["max_internal_gap_ms"]
    if gap is not None and gap > max_gap_ms:
        failures.append(
            f"{name}: source_max_internal_gap_ms {gap:.1f} ms exceeds "
            f"{max_gap_ms:.1f} ms"
        )

    # Header timestamps describe acquisition continuity, but the recording
    # window itself is opened and closed in the rosbag receive-time domain.
    # Allow one nominal source period at either edge because an asynchronous
    # stream can legitimately cross the boundary between two samples.
    nominal_period_ms = 1_000.0 / min_frequency_hz if min_frequency_hz > 0 else 0.0
    base_boundary_allowance_ms = (max_gap_ms if boundary_gap_ms is None else boundary_gap_ms) + nominal_period_ms
    bag = report["bag_timing"]
    for field_name, trim_sec in (
        ("leading_gap_ms", trim_start_sec),
        ("trailing_gap_ms", trim_end_sec),
    ):
        boundary_allowance_ms = base_boundary_allowance_ms + trim_sec * 1_000.0
        gap = bag[field_name]
        if gap is not None and gap > boundary_allowance_ms:
            failures.append(
                f"{name}: bag_{field_name} {gap:.1f} ms exceeds boundary "
                f"allowance {boundary_allowance_ms:.1f} ms"
            )
    return failures


def boundary_timing_warnings(
    name: str,
    report: dict[str, Any],
    *,
    min_frequency_hz: float,
    max_gap_ms: float,
    trim_start_sec: float,
    trim_end_sec: float,
) -> list[str]:
    """Describe edge gaps accepted only because an edge will be trimmed."""
    nominal_period_ms = 1_000.0 / min_frequency_hz if min_frequency_hz > 0 else 0.0
    base_allowance_ms = max_gap_ms + nominal_period_ms
    bag = report["bag_timing"]
    warnings = []
    for field_name, trim_sec in (
        ("leading_gap_ms", trim_start_sec),
        ("trailing_gap_ms", trim_end_sec),
    ):
        gap = bag[field_name]
        if (
            gap is not None
            and trim_sec > 0.0
            and base_allowance_ms < gap <= base_allowance_ms + trim_sec * 1_000.0
        ):
            warnings.append(
                f"{name}: bag_{field_name} {gap:.1f} ms is outside the "
                f"untrimmed allowance {base_allowance_ms:.1f} ms; accepted by "
                f"the {trim_sec:.3f} s edge trim"
            )
    return warnings


def transport_timing_warnings(
    name: str,
    report: dict[str, Any],
    *,
    max_gap_ms: float,
) -> list[str]:
    warnings = []
    bag = report["bag_timing"]
    for field_name in ("max_internal_gap_ms", "leading_gap_ms", "trailing_gap_ms"):
        gap = bag[field_name]
        if gap is not None and gap > max_gap_ms:
            warnings.append(
                f"{name}: bag_{field_name} {gap:.1f} ms exceeds "
                f"{max_gap_ms:.1f} ms"
            )
    return warnings


def _message_header_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    timestamp = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return timestamp if timestamp > 0 else None


def _validate_named_message(
    message: Any,
    expected_names: tuple[str, ...],
    *,
    fields: tuple[str, ...],
    label: str,
    allow_extra_names: bool = False,
) -> None:
    names = tuple(str(name) for name in message.name)
    if len(names) != len(set(names)):
        raise ValueError(f"{label}: duplicate joint names")
    if allow_extra_names:
        if not set(expected_names).issubset(names):
            raise ValueError(f"{label}: required joint group is partial")
    elif set(names) != set(expected_names):
        missing = sorted(set(expected_names).difference(names))
        unknown = sorted(set(names).difference(expected_names))
        raise ValueError(
            f"{label}: joint-name mismatch missing={missing}, unknown={unknown}"
        )
    for selected in fields:
        values = getattr(message, selected)
        if len(values) != len(names):
            raise ValueError(
                f"{label}: {selected} length does not match joint names"
            )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"{label}: {selected} contains a non-finite value")


def inspect_bag(
    bag_path: Path,
    topic_contracts: Iterable[Any],
    *,
    trim_start_sec: float = 0.0,
    trim_end_sec: float = 0.0,
    continuity_enabled: bool = False,
) -> tuple[tuple[str, ...], dict[str, Any]]:
    """Read every recorded message and return completeness failures/report."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    configured = {item.topic: item for item in topic_contracts}
    from .continuity import ContinuityMonitor, policy_for_topic
    continuity = ContinuityMonitor([item.topic for item in configured.values() if item.required]) if continuity_enabled else None
    if continuity is not None:
        continuity.begin(bag_path.name, 0.0)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id=_storage_id(bag_path),
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    actual_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_classes = {
        topic: get_message(type_name)
        for topic, type_name in actual_types.items()
        if topic in configured
    }
    streams: dict[str, StreamSamples] = defaultdict(StreamSamples)
    content_failures: list[str] = []
    telemetry_counters = {"velocity_sources": set()}
    engagement_samples: dict[str, list[tuple[int, bool]]] = defaultdict(list)

    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in configured or topic not in message_classes:
            continue
        bag_time_ns = int(bag_time_ns)
        stream = streams[topic]
        stream.bag_times_ns.append(bag_time_ns)
        try:
            message = (
                read_image_metadata(serialized)
                if actual_types[topic] == "sensor_msgs/msg/Image"
                else None
            )
            if message is None:
                message = deserialize_message(serialized, message_classes[topic])
                header_ns = _message_header_ns(message)
            else:
                header_ns = message.header_time_ns
        except ValueError as error:
            if continuity is not None:
                continuity.invalid(topic, str(error), bag_time_ns / 1e9)
            if len(content_failures) < 100:
                content_failures.append(f"{topic}: {error}")
            continue
        if continuity is not None:
            continuity.observe_message(topic, message, header_ns, bag_time_ns / 1e9)
        if header_ns is not None:
            stream.header_times_ns.append(header_ns)
        try:
            _inspect_message_content(
                topic,
                message,
                bag_time_ns,
                header_ns,
                streams,
                telemetry_counters,
                engagement_samples,
            )
        except ValueError as error:
            if len(content_failures) < 100:
                content_failures.append(str(error))

    required = [item for item in configured.values() if item.required]
    all_times = [
        timestamp
        for item in required
        for timestamp in streams[item.topic].bag_times_ns
    ]
    if not all_times:
        return ("bag contains no required messages",), {"streams": {}}
    global_start_ns = min(all_times)
    global_end_ns = max(all_times)
    all_source_times = [
        timestamp
        for item in required
        for timestamp in streams[item.topic].header_times_ns
    ]
    global_source_start_ns = min(all_source_times) if all_source_times else None
    global_source_end_ns = max(all_source_times) if all_source_times else None
    required_source_streams = [
        streams[item.topic].header_times_ns
        for item in required
        if streams[item.topic].header_times_ns
    ]
    common_source_start_ns = (
        max(times[0] for times in required_source_streams)
        if required_source_streams
        else None
    )
    common_source_end_ns = (
        min(times[-1] for times in required_source_streams)
        if required_source_streams
        else None
    )
    failures = list(content_failures)
    source_effective_start_ns = (
        common_source_start_ns + int(trim_start_sec * 1_000_000_000)
        if common_source_start_ns is not None
        else None
    )
    source_effective_end_ns = (
        common_source_end_ns - int(trim_end_sec * 1_000_000_000)
        if common_source_end_ns is not None
        else None
    )
    if (
        common_source_start_ns is not None
        and common_source_end_ns is not None
        and (
            source_effective_start_ns is None
            or source_effective_end_ns is None
            or source_effective_end_ns < source_effective_start_ns
        )
    ):
        failures.append(
            "configured edge trim removes the complete required source-time interval"
        )
    reports = {}
    transport_warnings: list[str] = []
    boundary_warnings: list[str] = []
    mode_aware_action_topics = {
        VALIDATED_COMMAND_TOPIC,
        *(WUJI_COMMAND_TOPIC.format(side=side) for side in ("left", "right")),
    }
    for item in required:
        report = stream_timing_report(
            streams[item.topic],
            global_start_ns=global_start_ns,
            global_end_ns=global_end_ns,
            global_source_start_ns=common_source_start_ns,
            global_source_end_ns=common_source_end_ns,
            trim_start_sec=trim_start_sec,
            trim_end_sec=trim_end_sec,
        )
        reports[item.topic] = report
        transport_warnings.extend(
            transport_timing_warnings(
                item.topic,
                report,
                max_gap_ms=float(item.max_gap_ms),
            )
        )
        boundary_warnings.extend(
            boundary_timing_warnings(
                item.topic,
                report,
                min_frequency_hz=float(item.min_frequency_hz),
                max_gap_ms=float(item.max_gap_ms),
                trim_start_sec=trim_start_sec,
                trim_end_sec=trim_end_sec,
            )
        )
        if item.topic not in mode_aware_action_topics:
            failures.extend(
                timing_failures(
                    item.topic,
                    report,
                    min_frequency_hz=float(item.min_frequency_hz),
                    max_gap_ms=float("inf") if continuity_enabled else float(item.max_gap_ms),
                    boundary_gap_ms=float(item.max_gap_ms),
                    trim_start_sec=trim_start_sec,
                    trim_end_sec=trim_end_sec,
                )
            )

    # The combined validated action bus must independently cover both sides.
    arm_policy = configured.get(VALIDATED_COMMAND_TOPIC)
    if arm_policy is not None:
        for side in ("left", "right"):
            name = f"{VALIDATED_COMMAND_TOPIC}#{side}"
            report = stream_timing_report(
                streams[name],
                global_start_ns=global_start_ns,
                global_end_ns=global_end_ns,
                global_source_start_ns=common_source_start_ns,
                global_source_end_ns=common_source_end_ns,
                trim_start_sec=trim_start_sec,
                trim_end_sec=trim_end_sec,
            )
            reports[name] = report
            failures.extend(
                _engaged_action_failures(
                    name=name,
                    engagement=engagement_samples[f"arm:{side}"],
                    action_times=streams[name].header_times_ns,
                    max_gap_ms=policy_for_topic(VALIDATED_COMMAND_TOPIC).failure_gap_ms if continuity_enabled else float(arm_policy.max_gap_ms),
                    window_start_ns=source_effective_start_ns,
                    window_end_ns=source_effective_end_ns,
                    allow_never_engaged=continuity_enabled,
                )
            )

    hand_policy = configured.get(WUJI_TELEMETRY_STATUS_TOPIC)
    for side in ("left", "right"):
        status_name = f"{WUJI_TELEMETRY_STATUS_TOPIC}#{side}"
        status_report = stream_timing_report(
            streams[status_name],
            global_start_ns=global_start_ns,
            global_end_ns=global_end_ns,
            global_source_start_ns=common_source_start_ns,
            global_source_end_ns=common_source_end_ns,
            trim_start_sec=trim_start_sec,
            trim_end_sec=trim_end_sec,
        )
        reports[status_name] = status_report
        if hand_policy is not None:
            failures.extend(
                timing_failures(
                    status_name,
                    status_report,
                    min_frequency_hz=float(hand_policy.min_frequency_hz),
                    max_gap_ms=float(hand_policy.max_gap_ms),
                    trim_start_sec=trim_start_sec,
                    trim_end_sec=trim_end_sec,
                )
            )
        command_topic = WUJI_COMMAND_TOPIC.format(side=side)
        command_policy = configured.get(command_topic)
        if command_policy is not None:
            failures.extend(
                _engaged_action_failures(
                    name=command_topic,
                    engagement=engagement_samples[f"hand:{side}"],
                    action_times=streams[command_topic].header_times_ns,
                    max_gap_ms=float(command_policy.max_gap_ms),
                    window_start_ns=source_effective_start_ns,
                    window_end_ns=source_effective_end_ns,
                )
            )

    for field_name in _TELEMETRY_COUNTER_FIELDS:
        delta = int(telemetry_counters.get(f"delta_{field_name}", 0))
        if delta > 0:
            failures.append(
                f"hand telemetry reports {field_name}_delta={delta}"
            )
    velocity_sources = sorted(telemetry_counters["velocity_sources"])
    if velocity_sources and velocity_sources != ["finite_difference"]:
        failures.append(
            "hand state velocity provenance is not exclusively finite_difference"
        )

    continuity_report = continuity.end() if continuity is not None else {}
    failures.extend(continuity_report.get("failures", []))
    report = {
        "continuity": continuity_report,
        "source_warnings": continuity_report.get("source_warnings", []),
        "bag_start_time_ns": global_start_ns,
        "bag_end_time_ns": global_end_ns,
        "source_start_time_ns": global_source_start_ns,
        "source_end_time_ns": global_source_end_ns,
        "source_common_start_time_ns": common_source_start_ns,
        "source_common_end_time_ns": common_source_end_ns,
        "source_effective_start_time_ns": source_effective_start_ns,
        "source_effective_end_time_ns": source_effective_end_ns,
        "streams": reports,
        "trim_start_sec": trim_start_sec,
        "trim_end_sec": trim_end_sec,
        "effective_bag_start_time_ns": global_start_ns + int(trim_start_sec * 1e9),
        "effective_bag_end_time_ns": global_end_ns - int(trim_end_sec * 1e9),
        "boundary_warnings": list(dict.fromkeys(boundary_warnings)),
        "transport_warnings": list(dict.fromkeys(transport_warnings)),
        "telemetry": {
            **{
                key: value
                for key, value in telemetry_counters.items()
                if key != "velocity_sources"
            },
            "velocity_sources": velocity_sources,
        },
        "engagement": {
            key: {
                "sample_count": len(samples),
                "engaged_sample_count": sum(active for _, active in samples),
                "disengaged_sample_count": sum(not active for _, active in samples),
            }
            for key, samples in sorted(engagement_samples.items())
        },
    }
    return tuple(dict.fromkeys(failures)), report


def _inspect_message_content(
    topic: str,
    message: Any,
    bag_time_ns: int,
    header_ns: int | None,
    streams: dict[str, StreamSamples],
    telemetry_counters: dict[str, Any],
    engagement_samples: dict[str, list[tuple[int, bool]]] | None = None,
) -> None:
    if engagement_samples is None:
        engagement_samples = defaultdict(list)
    hand_names = {
        "left": WUJI_LEFT_JOINT_NAMES,
        "right": WUJI_RIGHT_JOINT_NAMES,
    }
    if topic == "/cam0/depth/image_raw":
        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        if (width, height) != (640, 400):
            raise ValueError(
                f"head depth shape must be 640x400, got {width}x{height}"
            )
        if str(message.encoding) != "16UC1":
            raise ValueError(
                f"head depth encoding must be 16UC1, got {message.encoding!r}"
            )
        if step != width * 2 or len(message.data) != step * height:
            raise ValueError("head depth byte layout is not packed uint16")
        return
    if topic == VALIDATED_COMMAND_TOPIC:
        names = set(str(name) for name in message.name)
        for side, expected in (
            ("left", LEFT_COMMAND_JOINT_NAMES),
            ("right", RIGHT_COMMAND_JOINT_NAMES),
        ):
            overlap = names.intersection(expected)
            if overlap and not set(expected).issubset(names):
                raise ValueError(f"validated arm action has partial {side} side")
            if set(expected).issubset(names):
                _validate_named_message(
                    message,
                    expected,
                    fields=("position",),
                    label=f"validated {side} arm action",
                    allow_extra_names=True,
                )
                virtual = streams[f"{VALIDATED_COMMAND_TOPIC}#{side}"]
                virtual.bag_times_ns.append(bag_time_ns)
                if header_ns is not None:
                    virtual.header_times_ns.append(header_ns)
        return
    if topic == COMMAND_STATUS_TOPIC:
        accepted = {str(side) for side in message.accepted_sides}
        unknown = accepted.difference({"left", "right"})
        if unknown:
            raise ValueError(f"arm command status has unknown sides: {sorted(unknown)}")
        if message.faults:
            raise ValueError(
                "arm command status reports faults: "
                + "; ".join(str(fault) for fault in message.faults)
            )
        for side in ("left", "right"):
            engagement_samples[f"arm:{side}"].append(
                (header_ns if header_ns is not None else bag_time_ns, side in accepted)
            )
        return
    for side in ("left", "right"):
        if topic == f"/{side}/franka/joint_states":
            expected = (
                DATA_LEFT_ARM_JOINT_NAMES
                if side == "left"
                else DATA_RIGHT_ARM_JOINT_NAMES
            )
            _validate_named_message(
                message,
                expected,
                fields=("position", "velocity"),
                label=f"{side} arm state",
            )
            return
        if topic == WUJI_COMMAND_TOPIC.format(side=side):
            _validate_named_message(
                message,
                hand_names[side],
                fields=("position",),
                label=f"{side} hand command",
            )
            return
        if topic == WUJI_STATE_TOPIC.format(side=side):
            _validate_named_message(
                message,
                hand_names[side],
                fields=("position", "velocity"),
                label=f"{side} hand state",
            )
            return
    if topic == WUJI_TELEMETRY_STATUS_TOPIC:
        side = str(message.side)
        if side not in {"left", "right"}:
            raise ValueError(f"hand telemetry status has unknown side: {side!r}")
        engagement_samples[f"hand:{side}"].append(
            (header_ns if header_ns is not None else bag_time_ns, bool(message.engaged))
        )
        virtual = streams[f"{WUJI_TELEMETRY_STATUS_TOPIC}#{side}"]
        virtual.bag_times_ns.append(bag_time_ns)
        if header_ns is not None:
            virtual.header_times_ns.append(header_ns)
        for field_name in _TELEMETRY_COUNTER_FIELDS:
            _update_cumulative_counter(
                telemetry_counters,
                field_name,
                int(getattr(message, field_name)),
            )
        if bool(message.state_valid):
            telemetry_counters["velocity_sources"].add(
                str(message.state_velocity_source)
            )


_TELEMETRY_COUNTER_FIELDS = (
    "sender_dropped_packets",
    "receiver_lost_packets",
    "receiver_duplicate_packets",
    "receiver_out_of_order_packets",
    "receiver_stale_packets",
    "receiver_invalid_packets",
)


def _update_cumulative_counter(
    counters: dict[str, Any], field_name: str, current: int
) -> None:
    """Track only counter growth observed inside this episode.

    Receiver counters live for the complete collector process, so the first
    observed value is an episode baseline rather than an episode failure.
    When a sender/session reset makes a counter decrease, its new value is the
    growth since reset and is therefore included in the episode delta.
    """
    if current < 0:
        raise ValueError(f"hand telemetry counter {field_name} is negative")
    first_key = f"first_{field_name}"
    last_key = f"last_{field_name}"
    delta_key = f"delta_{field_name}"
    maximum_key = f"max_{field_name}"
    previous = counters.get(last_key)
    if first_key not in counters:
        counters[first_key] = current
    if previous is not None:
        counters[delta_key] = int(counters.get(delta_key, 0)) + (
            current - int(previous) if current >= int(previous) else current
        )
    else:
        counters.setdefault(delta_key, 0)
    counters[last_key] = current
    counters[maximum_key] = max(int(counters.get(maximum_key, 0)), current)


def _timestamp_in_window(
    timestamp: int,
    window_start_ns: int | None,
    window_end_ns: int | None,
) -> bool:
    if window_start_ns is not None and timestamp < window_start_ns:
        return False
    if window_end_ns is not None and timestamp > window_end_ns:
        return False
    return True


def _engaged_action_failures(
    *,
    name: str,
    engagement: list[tuple[int, bool]],
    action_times: list[int],
    max_gap_ms: float,
    window_start_ns: int | None = None,
    window_end_ns: int | None = None,
    allow_never_engaged: bool = False,
) -> list[str]:
    """Require fresh actions only while the corresponding side is engaged."""
    if not engagement:
        return [f"{name}: no engagement status in bag"]
    engaged_times = [
        timestamp
        for timestamp, active in engagement
        if active
        and _timestamp_in_window(timestamp, window_start_ns, window_end_ns)
    ]
    if not engaged_times:
        return [] if allow_never_engaged else [f"{name}: side was never engaged during the episode"]
    if not action_times:
        return [f"{name}: no messages while side was engaged"]

    max_gap_ns = int(max_gap_ms * 1_000_000)
    # Arm status is published immediately before the validated action in the
    # same callback, so a small future allowance is needed for bag ordering.
    # It must stay much smaller than the freshness window; otherwise two action
    # samples 300 ms apart could incorrectly cover the status samples between.
    future_grace_ns = min(max_gap_ns, 20_000_000)
    uncovered = 0
    worst_distance_ns = 0
    for timestamp in engaged_times:
        index = bisect_left(action_times, timestamp)
        previous_distance = (
            timestamp - action_times[index - 1]
            if index > 0
            else max_gap_ns + 1
        )
        future_distance = (
            action_times[index] - timestamp
            if index < len(action_times)
            else future_grace_ns + 1
        )
        covered = (
            previous_distance <= max_gap_ns
            or future_distance <= future_grace_ns
        )
        if not covered:
            uncovered += 1
            worst_distance_ns = max(
                worst_distance_ns,
                min(previous_distance, future_distance),
            )
    if uncovered:
        return [
            f"{name}: {uncovered} engaged status samples have no action within "
            f"{max_gap_ms:.1f} ms (worst {worst_distance_ns / 1_000_000:.1f} ms)"
        ]
    return []


def _storage_id(bag_path: Path) -> str:
    import yaml

    metadata = yaml.safe_load(
        (bag_path / "metadata.yaml").read_text(encoding="utf-8")
    )
    return str(
        metadata["rosbag2_bagfile_information"].get(
            "storage_identifier", "sqlite3"
        )
    )
