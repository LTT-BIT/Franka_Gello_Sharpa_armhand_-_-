#!/usr/bin/env python3
"""Subscribe to camera metadata without recording images or commanding hardware."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from teleop_data_collector.continuity import ContinuityMonitor

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=180)
    parser.add_argument("--output-report", type=Path, required=True)
    args = parser.parse_args()
    if args.duration_sec <= 0:
        parser.error("duration must be positive")
    output = args.output_report.expanduser().resolve()
    repo = Path(__file__).resolve().parents[2]
    if output == repo or repo in output.parents:
        parser.error("report must be outside the repository")
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("camera_clock_repair_validation")
    topics = ["/cam0/color/image_raw", "/cam0/depth/image_raw", "/cam1/color/image_raw", "/cam2/color/image_raw"]
    monitor = ContinuityMonitor(topics)
    started = time.monotonic()
    monitor.begin("camera_clock_validation", started)
    subscriptions = []
    for topic in topics:
        def callback(raw, topic=topic):
            now = time.monotonic()
            if len(raw) < 12 or raw[:2] not in (b"\x00\x00", b"\x00\x01"):
                monitor.invalid(topic, "unsupported or truncated CDR header", now)
                return
            sec, nano = struct.unpack_from(("<" if raw[:2] == b"\x00\x01" else ">") + "iI", raw, 4)
            monitor.observe(topic, sec * 1_000_000_000 + nano, now)
        subscriptions.append(node.create_subscription(Image, topic, callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), raw=True))
    next_status = 30
    try:
        while time.monotonic() - started < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            monitor.watchdog(now)
            elapsed = now - started
            if elapsed >= next_status:
                print(json.dumps({"elapsed_s": round(elapsed, 1), "streams": {
                    topic: {key: stats[key] for key in ("sample_count", "timestamp_failure_count", "source_gap_failure_count", "max_source_gap_ms")}
                    for topic, stats in monitor.snapshot()["streams"].items()}}, ensure_ascii=False), flush=True)
                next_status += 30
    finally:
        report = monitor.end()
        report["duration_sec"] = time.monotonic() - started
        report["validation_kind"] = "camera_subscriptions_only_no_robot_commands_or_image_recording"
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        node.destroy_node()
        rclpy.shutdown()
        print(f"Report: {output}", flush=True)
    return int(any(stats["sample_count"] < 2 or stats["timestamp_failure_count"] or stats["source_gap_failure_count"] or stats["silence_failure_count"] for stats in report["streams"].values()))


if __name__ == "__main__":
    raise SystemExit(main())
