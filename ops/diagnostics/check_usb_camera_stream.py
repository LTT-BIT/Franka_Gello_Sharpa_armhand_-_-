#!/usr/bin/env python3
"""Measure delivered V4L2 frames without recording or changing the device."""

import argparse
import json
import statistics
import time

import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("device")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--format", default="MJPG")
    args = parser.parse_args()

    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise SystemExit(f"cannot open {args.device}")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.format))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30)
    actual_fourcc = int(camera.get(cv2.CAP_PROP_FOURCC))
    actual_format = "".join(chr((actual_fourcc >> (8 * i)) & 255) for i in range(4))
    profile = {
        "device": args.device,
        "format": actual_format,
        "width": camera.get(cv2.CAP_PROP_FRAME_WIDTH),
        "height": camera.get(cv2.CAP_PROP_FRAME_HEIGHT),
        "reported_fps": camera.get(cv2.CAP_PROP_FPS),
    }
    stamps = []
    failures = 0
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            ok, _ = camera.read()
            now = time.monotonic()
            if ok:
                stamps.append(now)
            else:
                failures += 1
    finally:
        camera.release()
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    elapsed = time.monotonic() - start
    print(json.dumps({
        **profile,
        "elapsed_sec": round(elapsed, 3),
        "frames": len(stamps),
        "read_failures": failures,
        "delivered_fps": round(len(stamps) / elapsed, 2),
        "mean_gap_ms": round(statistics.mean(gaps) * 1000, 2) if gaps else None,
        "max_gap_ms": round(max(gaps) * 1000, 2) if gaps else None,
        "gaps_over_100ms": sum(g > 0.1 for g in gaps),
        "gaps_over_200ms": sum(g > 0.2 for g in gaps),
    }, sort_keys=True))


if __name__ == "__main__":
    main()
