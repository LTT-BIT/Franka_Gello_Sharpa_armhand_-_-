"""Read-only host preflight for the shared CycloneDDS image transport."""
from pathlib import Path
import sys

MIN_RECEIVE_BUFFER_BYTES = 8 * 1024 * 1024
RECEIVE_LIMIT_PATH = Path('/proc/sys/net/core/rmem_max')


def check_receive_buffer(path=RECEIVE_LIMIT_PATH):
    try:
        actual = int(Path(path).read_text().strip())
    except (OSError, ValueError) as error:
        raise RuntimeError(f'Cannot read DDS UDP receive buffer limit: {error}') from error
    if actual < MIN_RECEIVE_BUFFER_BYTES:
        raise RuntimeError(
            f'DDS UDP receive buffer too small: net.core.rmem_max={actual}; '
            f'requires at least {MIN_RECEIVE_BUFFER_BYTES} (8 MiB). '
            'Run once on the host: sudo bash ops/setup/install_dds_buffers.sh'
        )
    return actual


def main():
    try:
        actual = check_receive_buffer()
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 2
    print(f'DDS receive buffer preflight OK: net.core.rmem_max={actual} (minimum 8 MiB)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
