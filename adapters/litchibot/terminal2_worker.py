"""Python 3.10 glove worker. Emits targets over stdout; never creates a sender."""
import argparse
import json
from pathlib import Path
import signal
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from adapters.litchibot.backend import LitchiBotHandPipeline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--sdk-root', required=True)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--receiver-id', default='')
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--log', required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    identity = str(uuid.uuid4())
    pipeline = LitchiBotHandPipeline(profile=args.profile, sdk_root=args.sdk_root,
        data_root=args.data_root, receiver_id=args.receiver_id or None,
        dry_run=True, debug_log=args.log)
    started = time.monotonic()
    previous = {}
    try:
        while not args.duration or time.monotonic()-started < args.duration:
            pipeline.tick(active={'left': False, 'right': False})
            for side, row in pipeline.last_command.items():
                if row is None or previous.get(side) == row['sequence']:
                    continue
                previous[side] = row['sequence']
                # Producer-only envelope: this worker never owns a hardware sender.
                # Real acceptance permission is enforced separately by bridge/guard.
                packet = {**row, 'schema': 'litchibot.sharpa_target.v1',
                          'session_id': identity, 'engaged': True, 'dry_run': True}
                print(json.dumps(packet, allow_nan=False), flush=True)
            time.sleep(0.005)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        pipeline.close()
    print(json.dumps(pipeline.summary()), file=sys.stderr, flush=True)


if __name__ == '__main__':
    main()
