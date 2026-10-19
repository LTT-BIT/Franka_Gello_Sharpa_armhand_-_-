from __future__ import annotations

import argparse
import threading
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Any


DEFAULT_STABLE_SEC = 3.0
DEFAULT_MAX_GAP_MS = 150.0
CAMERA_STREAMS = (
    ("/cam0/color/image_raw", 18.0),
    ("/cam0/depth/image_raw", 18.0),
    ("/cam1/color/image_raw", 25.0),
    ("/cam2/color/image_raw", 25.0),
)


class CameraReadiness:
    """Continuously maintained pre-episode gate; never relaxes episode checks."""

    def __init__(self, topics: list[str], *, rgb_enabled=False, rgb_threshold=25.0, rgb_stable_sec=3.0):
        frequencies = dict(CAMERA_STREAMS)
        self._windows = {topic: StreamWindow(topic, frequencies[topic], require_source_timestamps=True)
                         for topic in topics if topic in frequencies}
        from teleop_core.contract import CAMERA_COLOR_TOPIC
        from .rgb_brightness import RgbReadiness
        colors = {CAMERA_COLOR_TOPIC.format(index=index) for index in range(3)}
        self._rgb = RgbReadiness([topic for topic in self._windows if topic in colors],
                                 threshold=rgb_threshold, stable_sec=rgb_stable_sec) if rgb_enabled else None
        self._lock = threading.Lock()

    def observes(self, topic: str) -> bool:
        return topic in self._windows

    def checks_brightness(self, topic: str) -> bool:
        return self._rgb is not None and topic in self._rgb.topics

    def set_brightness_paused(self, paused: bool) -> None:
        with self._lock:
            if self._rgb is not None:
                self._rgb.set_paused(paused)

    def observe(self, topic: str, header_ns: int | None, now: float, *, brightness=None, brightness_error=None) -> None:
        with self._lock:
            self._windows[topic].observe(now, max_gap_sec=DEFAULT_MAX_GAP_MS / 1000,
                                         header_ns=header_ns)
            if self._rgb is not None:
                self._rgb.observe(topic, header_ns, now, brightness, brightness_error)

    def snapshot(self, now: float) -> dict[str, Any]:
        with self._lock:
            waiting = [topic for topic, window in self._windows.items()
                       if not window.ready(now, stable_sec=DEFAULT_STABLE_SEC,
                                           max_gap_sec=DEFAULT_MAX_GAP_MS / 1000)]
            result = {"ready": not waiting, "waiting_topics": waiting,
                    "stable_sec": DEFAULT_STABLE_SEC, "max_receive_gap_ms": DEFAULT_MAX_GAP_MS,
                    "message": (f"等待相机连续稳定 {DEFAULT_STABLE_SEC:g} 秒：" + ", ".join(waiting)) if waiting else ""}
            if self._rgb is not None:
                rgb = self._rgb.snapshot(now)
                result['rgb_brightness'] = rgb
                result['ready'] = result['ready'] and rgb['ready']
                result['message'] = ' · '.join(text for text in (result['message'], rgb['message']) if text)
            return result


@dataclass
class StreamWindow:
    topic: str
    min_frequency_hz: float
    first_time: float | None = None
    last_time: float | None = None
    sample_count: int = 0
    reset_count: int = 0
    worst_gap_sec: float = 0.0
    require_source_timestamps: bool = False
    last_source_ns: int | None = None
    first_source_ns: int | None = None
    source_reset_count: int = 0

    def observe(self, now: float, *, max_gap_sec: float, header_ns: int | None = None) -> None:
        if self.require_source_timestamps:
            previous = self.last_source_ns
            self.last_source_ns = header_ns
            if header_ns is None or header_ns <= 0:
                self.source_reset_count += 1
                self._reset(now, count_reset=True)
                self.first_time = self.last_time = None
                self.last_source_ns = self.first_source_ns = None
                return
            if previous is not None and (header_ns <= previous or (header_ns - previous) / 1e9 > max_gap_sec):
                self.source_reset_count += 1
                self._reset(now, count_reset=True)
                return
        if self.last_time is None:
            self._reset(now, count_reset=False)
            return

        gap_sec = now - self.last_time
        self.worst_gap_sec = max(self.worst_gap_sec, gap_sec)
        if gap_sec > max_gap_sec:
            self._reset(now, count_reset=True)
            return

        self.last_time = now
        self.sample_count += 1

    def ready(
        self,
        now: float,
        *,
        stable_sec: float,
        max_gap_sec: float,
    ) -> bool:
        if self.first_time is None or self.last_time is None:
            return False
        if now - self.last_time > max_gap_sec:
            return False
        duration_sec = self.last_time - self.first_time
        return (
            duration_sec >= stable_sec
            and self.frequency_hz >= self.min_frequency_hz
            and (not self.require_source_timestamps or self.source_frequency_hz >= self.min_frequency_hz)
        )

    @property
    def frequency_hz(self) -> float:
        if (
            self.first_time is None
            or self.last_time is None
            or self.sample_count < 2
        ):
            return 0.0
        duration_sec = self.last_time - self.first_time
        if duration_sec <= 0.0:
            return 0.0
        return (self.sample_count - 1) / duration_sec

    @property
    def source_frequency_hz(self) -> float:
        if self.first_source_ns is None or self.last_source_ns is None or self.last_source_ns <= self.first_source_ns:
            return 0.0
        return (self.sample_count - 1) * 1e9 / (self.last_source_ns - self.first_source_ns)

    def describe(self, now: float, *, max_gap_sec: float) -> str:
        if self.last_time is None:
            return "no frames"
        age_sec = now - self.last_time
        stale = " STALE" if age_sec > max_gap_sec else ""
        return (
            f"{self.frequency_hz:.2f} Hz (minimum {self.min_frequency_hz:.2f}), "
            f"age={age_sec * 1000:.1f} ms, "
            f"worst_gap={self.worst_gap_sec * 1000:.1f} ms, "
            f"resets={self.reset_count}{stale}"
            + (f", source={self.source_frequency_hz:.2f} Hz, source_resets={self.source_reset_count}" if self.require_source_timestamps else "")
        )

    def _reset(self, now: float, *, count_reset: bool) -> None:
        self.first_time = now
        self.first_source_ns = self.last_source_ns
        self.last_time = now
        self.sample_count = 1
        if count_reset:
            self.reset_count += 1


def wait_for_streams(
    *,
    timeout_sec: float,
    stable_sec: float,
    max_gap_ms: float,
    require_source_timestamps: bool = False,
) -> int:
    import rclpy
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image

    max_gap_sec = max_gap_ms / 1000.0
    windows = {
        topic: StreamWindow(topic, min_frequency_hz, require_source_timestamps=require_source_timestamps)
        for topic, min_frequency_hz in CAMERA_STREAMS
    }
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    rclpy.init()
    node = rclpy.create_node("camera_stream_readiness")
    subscriptions = []
    for topic, window in windows.items():

        def callback(_message: bytes, *, stream: StreamWindow = window) -> None:
            header_ns = None
            if require_source_timestamps:
                from .image_metadata import read_image_metadata
                try:
                    metadata = read_image_metadata(_message)
                    header_ns = metadata.header_time_ns if metadata is not None else None
                except ValueError:
                    pass
            stream.observe(time.monotonic(), max_gap_sec=max_gap_sec, header_ns=header_ns)

        subscriptions.append(
            node.create_subscription(Image, topic, callback, qos, raw=True)
        )

    started_at = time.monotonic()
    deadline = started_at + timeout_sec
    next_status_at = started_at
    try:
        while rclpy.ok():
            now = time.monotonic()
            if all(
                window.ready(
                    now,
                    stable_sec=stable_sec,
                    max_gap_sec=max_gap_sec,
                )
                for window in windows.values()
            ):
                print(
                    f"Camera streams stable for {stable_sec:.1f}s; "
                    "recorder may start.",
                    flush=True,
                )
                _print_status(windows, now, max_gap_sec=max_gap_sec)
                return 0
            if now >= deadline:
                print(
                    f"Camera readiness timed out after {timeout_sec:.1f}s.",
                    file=sys.stderr,
                    flush=True,
                )
                _print_status(
                    windows,
                    now,
                    max_gap_sec=max_gap_sec,
                    stream=sys.stderr,
                )
                return 1
            if now >= next_status_at:
                print("Waiting for stable camera frames:", flush=True)
                _print_status(windows, now, max_gap_sec=max_gap_sec)
                next_status_at = now + 5.0
            rclpy.spin_once(node, timeout_sec=min(0.05, deadline - now))
    except KeyboardInterrupt:
        return 130
    finally:
        subscriptions.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1


def validate_bag(bag_path: Path) -> int:
    bag_path = bag_path.expanduser().resolve()
    if not (bag_path / "metadata.yaml").is_file():
        print(f"Not a finalized ROS bag: {bag_path}", file=sys.stderr)
        return 2

    failures, report = inspect_camera_bag(bag_path)
    streams: dict[str, dict[str, Any]] = report.get("streams", {})
    print("Camera source-timestamp validation:")
    for topic, _minimum_hz in CAMERA_STREAMS:
        timing = streams.get(topic, {}).get("source_timing", {})
        frequency_hz = float(timing.get("frequency_hz") or 0.0)
        gap_ms = timing.get("max_internal_gap_ms")
        gap_text = "n/a" if gap_ms is None else f"{float(gap_ms):.1f} ms"
        print(f"  {topic}: {frequency_hz:.2f} Hz, max gap={gap_text}")

    for warning in report.get("transport_warnings", ()):
        print(f"  TRANSPORT WARNING: {warning}")
    for failure in failures:
        print(f"  FAILED: {failure}", file=sys.stderr)
    if failures:
        print(
            "Camera validation failed; keep this bag out of the training set.",
            file=sys.stderr,
        )
        return 1
    print("Camera source streams passed continuity validation.")
    return 0


def camera_contracts() -> tuple[Any, ...]:
    from .collector_contract import TopicContract

    return tuple(
        TopicContract(
            name=topic.removeprefix("/").replace("/", "_"),
            topic=topic,
            type_name="sensor_msgs/msg/Image",
            min_frequency_hz=min_frequency_hz,
            max_gap_ms=DEFAULT_MAX_GAP_MS,
        )
        for topic, min_frequency_hz in CAMERA_STREAMS
    )


def inspect_camera_bag(bag_path: Path) -> tuple[tuple[str, ...], dict[str, Any]]:
    from .bag_validation import inspect_bag

    return inspect_bag(Path(bag_path), camera_contracts())


def _print_status(
    windows: dict[str, StreamWindow],
    now: float,
    *,
    max_gap_sec: float,
    stream: Any = sys.stdout,
) -> None:
    for topic, window in windows.items():
        print(
            f"  {topic}: {window.describe(now, max_gap_sec=max_gap_sec)}",
            file=stream,
            flush=True,
        )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Gate recording on live camera health and validate camera bags."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    wait_parser = subparsers.add_parser("wait")
    wait_parser.add_argument("--require-source-timestamps", action="store_true")
    wait_parser.add_argument("--timeout-sec", type=float, default=90.0)
    wait_parser.add_argument("--stable-sec", type=float, default=DEFAULT_STABLE_SEC)
    wait_parser.add_argument("--max-gap-ms", type=float, default=DEFAULT_MAX_GAP_MS)
    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("bag", type=Path)
    args = parser.parse_args(argv)

    if args.command == "wait":
        if (
            args.timeout_sec <= 0.0
            or args.stable_sec <= 0.0
            or args.max_gap_ms <= 0.0
        ):
            parser.error("timeout, stability duration, and maximum gap must be positive")
        raise SystemExit(
            wait_for_streams(
                timeout_sec=args.timeout_sec,
                stable_sec=args.stable_sec,
                max_gap_ms=args.max_gap_ms,
                require_source_timestamps=args.require_source_timestamps,
            )
        )
    raise SystemExit(validate_bag(args.bag))


if __name__ == "__main__":
    main()
