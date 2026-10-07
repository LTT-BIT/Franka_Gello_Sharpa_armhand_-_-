"""Hands-only observation and diagnostics, with no FR3/GELLO initialization."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time

from .backend import LitchiBotHandPipeline
from .retarget import joint_table

DEFAULT_SDK_ROOT = "/home/user/litchibot_glove/litchibot-glove-0.1.0-ubuntu24.04-x86_64-cp310"
DEFAULT_DATA_ROOT = "/home/user/litchibot_glove/validation/user_data"


def add_options(parser):
    parser.add_argument("--litchibot-profile", default="LYG226360006")
    parser.add_argument("--litchibot-sdk-root", default=os.environ.get("LITCHIBOT_SDK_ROOT", DEFAULT_SDK_ROOT))
    parser.add_argument("--litchibot-data-root", default=os.environ.get("LITCHIBOT_DATA_DIR", DEFAULT_DATA_ROOT))
    parser.add_argument("--litchibot-receiver-id")
    parser.add_argument("--litchibot-left-retarget", type=Path)
    parser.add_argument("--litchibot-right-retarget", type=Path)
    parser.add_argument("--sharpa-relay-port", type=int, default=5572)


def pipeline_options(args):
    return dict(profile=args.litchibot_profile, sdk_root=args.litchibot_sdk_root,
        data_root=args.litchibot_data_root, receiver_id=args.litchibot_receiver_id,
        rate=args.hand_rate, debug_log=args.hand_debug_log,
        retarget_configs={s: getattr(args, 'litchibot_'+s+'_retarget') for s in ('left','right')})


def run_dry_run(args):
    """Used by the unified CLI *before* any arm/server/device construction."""
    pipeline = LitchiBotHandPipeline(**pipeline_options(args), dry_run=True)
    started = time.monotonic()
    report_at = started+1
    try:
        while not args.duration or time.monotonic()-started < args.duration:
            pipeline.tick(active={"left": False, "right": False})
            if time.monotonic() >= report_at:
                print(json.dumps({"type": "progress", "summary": pipeline.summary(),
                    "latest": pipeline.last_command}, allow_nan=False), flush=True)
                report_at += 1
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.close()
    result = pipeline.summary()
    print(json.dumps(result, allow_nan=False), flush=True)
    return 0 if all(s["count"] > 0 for s in result["sides"].values()) else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand-source", choices=("litchibot",), default="litchibot")
    parser.add_argument("--dry-run", action="store_true", help="Always the default in this hands-only tool")
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--hand-rate", type=float, default=30)
    parser.add_argument("--hand-debug-log")
    parser.add_argument("--joint-table", action="store_true")
    add_options(parser)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0 or not math.isfinite(args.hand_rate) or not 0 < args.hand_rate <= 60:
        parser.error("duration must be >= 0; hand-rate must be in (0, 60]")
    if args.joint_table:
        print(json.dumps(joint_table(), indent=2))
        return 0
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    return run_dry_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
