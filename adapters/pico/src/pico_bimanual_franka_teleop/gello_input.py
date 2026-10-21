"""Dual GELLO input with independent reads, freshness, and jump checks."""

from __future__ import annotations

import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import yaml

from .types import JointTeleopSample, SIDES


@dataclass(frozen=True)
class GelloSideConfig:
    port: str
    expected_serial: str
    direction_correction: tuple[int, ...]
    joint_sensitivity: tuple[float, ...]
    max_relative_delta: tuple[float, ...]
    joint_limit_margin: tuple[float, ...] = (0.0,) * 7


@dataclass(frozen=True)
class GelloConfig:
    left: GelloSideConfig
    right: GelloSideConfig
    baudrate: int
    joint_ids: tuple[int, ...]
    standard_signs: tuple[int, ...]
    ready_timeout: float
    stale_timeout: float
    max_joint_jump: float
    max_target_velocity: float


def _seven_signs(value, field: str) -> tuple[int, ...]:
    result = tuple(int(item) for item in value)
    if len(result) != 7 or any(item not in (-1, 1) for item in result):
        raise ValueError(f"{field} must contain seven +1/-1 values")
    return result


def _seven_sensitivities(value, field: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value)
    if (
        len(result) != 7
        or not all(np.isfinite(item) for item in result)
        or any(item < 0.1 or item > 2.2 for item in result)
    ):
        raise ValueError(f"{field} must contain seven finite values in [0.1, 2.2]")
    return result


def _seven_positive_values(value, field: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value)
    if (
        len(result) != 7
        or not all(np.isfinite(item) for item in result)
        or any(item <= 0.0 for item in result)
    ):
        raise ValueError(f"{field} must contain seven finite positive values")
    return result


def _seven_nonnegative_values(value, field: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value)
    if (
        len(result) != 7
        or not all(np.isfinite(item) for item in result)
        or any(item < 0.0 for item in result)
    ):
        raise ValueError(f"{field} must contain seven finite nonnegative values")
    return result


def load_gello_config(path: str | Path) -> GelloConfig:
    """Load the strict, hardware-identity-aware GELLO configuration."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    required = {
        "left",
        "right",
        "baudrate",
        "joint_ids",
        "standard_signs",
        "ready_timeout",
        "stale_timeout",
        "max_joint_jump",
        "max_target_velocity",
    }
    if not isinstance(raw, dict) or set(raw) != required:
        raise ValueError(f"GELLO config keys must be exactly {sorted(required)}")

    def side_config(side: str) -> GelloSideConfig:
        selected = raw[side]
        expected = {
            "port",
            "expected_serial",
            "direction_correction",
            "joint_sensitivity",
            "max_relative_delta",
            "joint_limit_margin",
        }
        if not isinstance(selected, dict) or set(selected) != expected:
            raise ValueError(f"{side} GELLO keys must be exactly {sorted(expected)}")
        port = str(selected["port"])
        serial = str(selected["expected_serial"])
        if not port.startswith("/dev/serial/by-id/") or serial not in port:
            raise ValueError(f"{side} GELLO port must contain expected_serial")
        return GelloSideConfig(
            port=port,
            expected_serial=serial,
            direction_correction=_seven_signs(
                selected["direction_correction"],
                f"{side}.direction_correction",
            ),
            joint_sensitivity=_seven_sensitivities(
                selected["joint_sensitivity"],
                f"{side}.joint_sensitivity",
            ),
            max_relative_delta=_seven_positive_values(
                selected["max_relative_delta"],
                f"{side}.max_relative_delta",
            ),
            joint_limit_margin=_seven_nonnegative_values(
                selected["joint_limit_margin"],
                f"{side}.joint_limit_margin",
            ),
        )

    joint_ids = tuple(int(value) for value in raw["joint_ids"])
    if joint_ids != (1, 2, 3, 4, 5, 6, 7):
        raise ValueError("joint_ids must be [1..7]; motor 8 is reserved")
    numeric = {
        name: float(raw[name])
        for name in (
            "ready_timeout",
            "stale_timeout",
            "max_joint_jump",
            "max_target_velocity",
        )
    }
    if any(value <= 0 for value in numeric.values()) or int(raw["baudrate"]) <= 0:
        raise ValueError("GELLO timing, limits, and baudrate must be positive")
    return GelloConfig(
        left=side_config("left"),
        right=side_config("right"),
        baudrate=int(raw["baudrate"]),
        joint_ids=joint_ids,
        standard_signs=_seven_signs(raw["standard_signs"], "standard_signs"),
        **numeric,
    )


class DynamixelJointReader:
    """Read GELLO motors 1-7 directly; motor 8 is never opened.

    The upstream driver tries to kill a process using the serial port, change
    device permissions with sudo, and fall back to a fake device. All three
    behaviours are unsafe for hardware control, so this adapter disables them.
    """

    def __init__(self, port: str, baudrate: int, joint_ids: tuple[int, ...]) -> None:
        from gello.dynamixel.driver import DynamixelDriver

        try:
            occupied = subprocess.run(
                ["fuser", port],
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(
                f"GELLO port ownership check timed out for {port}"
            ) from error
        except OSError as error:
            raise RuntimeError(
                "fuser is required for safe GELLO port ownership"
            ) from error
        if occupied.returncode == 0 and occupied.stdout.strip():
            pids = ", ".join(occupied.stdout.split())
            raise RuntimeError(f"GELLO port {port} is already in use by PID(s) {pids}")
        if occupied.returncode not in (0, 1):
            raise RuntimeError(f"could not verify GELLO port ownership for {port}")

        class FailClosedDynamixelDriver(DynamixelDriver):
            def __init__(self, *args, **kwargs) -> None:
                self._read_started_at = time.monotonic()
                self._last_successful_read_at: float | None = None
                super().__init__(*args, **kwargs)

            def _start_reading_thread(self) -> None:
                owner = self
                group_sync_read = self._groupSyncRead

                class HeartbeatGroupSyncRead:
                    """Forward SDK calls while recording real bus transactions."""

                    def txRxPacket(self):
                        result = group_sync_read.txRxPacket()
                        if result == 0:  # Dynamixel SDK COMM_SUCCESS
                            owner._last_successful_read_at = time.monotonic()
                        return result

                    def __getattr__(self, name):
                        return getattr(group_sync_read, name)

                self._groupSyncRead = HeartbeatGroupSyncRead()
                super()._start_reading_thread()

            def _check_port_availability(self) -> bool:
                return True

            def _prepare_port(self) -> None:
                return None

            def _kill_processes_using_port(self) -> bool:
                return False

            def _fix_port_permissions(self) -> bool:
                return False

        self._driver = FailClosedDynamixelDriver(
            list(joint_ids),
            port=port,
            baudrate=baudrate,
            max_retries=1,
            use_fake_fallback=False,
        )
        self._terminal_error: str | None = None

    def read(self) -> np.ndarray:
        return np.asarray(self._driver.get_joints(), dtype=float)

    def health_error(self, max_age: float) -> str | None:
        """Return a fault when the upstream cache is no longer hardware-backed."""
        if self._terminal_error is not None:
            return self._terminal_error
        thread = getattr(self._driver, "_reading_thread", None)
        if thread is not None and not thread.is_alive():
            return self._set_terminal_error("Dynamixel read thread stopped")
        last_success = getattr(self._driver, "_last_successful_read_at", None)
        if last_success is None:
            age = time.monotonic() - float(self._driver._read_started_at)
            if age > max_age:
                return self._set_terminal_error(
                    f"no successful Dynamixel bus read for {age:.3f}s"
                )
            return "waiting for first successful Dynamixel bus read"
        age = time.monotonic() - float(last_success)
        if age > max_age:
            return self._set_terminal_error(
                f"Dynamixel bus read stale for {age:.3f}s"
            )
        return None

    @property
    def terminal_error(self) -> str | None:
        return self._terminal_error

    def _set_terminal_error(self, message: str) -> str:
        self._terminal_error = message
        stop = getattr(self._driver, "_stop_thread", None)
        if stop is not None:
            stop.set()
        return message

    def close(self) -> None:
        self._driver.close()


class _SideReader:
    _RECOVERY_SAMPLES = 10

    def __init__(
        self,
        side: str,
        config: GelloSideConfig,
        *,
        baudrate: int,
        joint_ids: tuple[int, ...],
        signs: np.ndarray,
        max_joint_jump: float,
        stale_timeout: float,
        reader_factory: Callable[..., object],
    ) -> None:
        self.side = side
        self._signs = signs
        self._max_joint_jump = max_joint_jump
        self._stale_timeout = stale_timeout
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest: np.ndarray | None = None
        self._latest_at: float | None = None
        self._error: str | None = None
        self._samples = 0
        self._reader = reader_factory(config.port, baudrate, joint_ids)
        self._thread = threading.Thread(
            target=self._run, name=f"Gello-{side}", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        previous_raw: np.ndarray | None = None
        recovery_candidate: np.ndarray | None = None
        recovery_samples = 0
        while not self._stop.is_set():
            try:
                health_reader = getattr(self._reader, "health_error", None)
                if health_reader is not None:
                    health_error = health_reader(self._stale_timeout)
                    if health_error is not None:
                        raise RuntimeError(health_error)
                raw = np.asarray(self._reader.read(), dtype=float)
                if raw.shape != (7,) or not np.all(np.isfinite(raw)):
                    raise ValueError("reader did not return seven finite joints")
                if previous_raw is None:
                    continuous = raw
                else:
                    continuous = previous_raw + (raw - previous_raw + np.pi) % (
                        2.0 * np.pi
                    ) - np.pi
                    delta = np.abs(continuous - previous_raw)
                    if np.max(delta) > self._max_joint_jump:
                        if recovery_candidate is None:
                            recovery_candidate = raw.copy()
                            recovery_samples = 1
                        else:
                            candidate = recovery_candidate + (
                                raw - recovery_candidate + np.pi
                            ) % (2.0 * np.pi) - np.pi
                            if (
                                np.max(np.abs(candidate - recovery_candidate))
                                <= self._max_joint_jump
                            ):
                                recovery_candidate = candidate
                                recovery_samples += 1
                            else:
                                recovery_candidate = raw.copy()
                                recovery_samples = 1
                        joint = int(np.argmax(delta)) + 1
                        if recovery_samples < self._RECOVERY_SAMPLES:
                            raise ValueError(
                                f"joint {joint} jump {delta[joint - 1]:.3f} rad "
                                f"exceeds {self._max_joint_jump:.3f} rad; "
                                f"reacquiring stable input "
                                f"({recovery_samples}/{self._RECOVERY_SAMPLES})"
                            )
                        # The side has already been fail-closed by sample().
                        # Rebase only after a stable run; re-engagement remains
                        # an explicit operator action and the relative mapper
                        # anchors that action to the measured robot pose.
                        continuous = recovery_candidate
                    recovery_candidate = None
                    recovery_samples = 0
                previous_raw = continuous
                now = time.monotonic()
                with self._lock:
                    self._latest = continuous * self._signs
                    self._latest_at = now
                    self._error = None
                    self._samples += 1
                time.sleep(0.001)
            except Exception as error:  # noqa: BLE001 - report hardware faults
                with self._lock:
                    self._error = str(error)
                time.sleep(0.01)

    def snapshot(self) -> tuple[np.ndarray | None, float | None, str | None, int]:
        with self._lock:
            return (
                None if self._latest is None else self._latest.copy(),
                self._latest_at,
                self._error,
                self._samples,
            )

    @property
    def terminal_error(self) -> str | None:
        return getattr(self._reader, "terminal_error", None)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._reader.close()


class DualGelloJointInput:
    """Produce calibrated bimanual joint samples without blocking the arm loop."""

    output_kind = "joint"
    source_name = "gello"

    def __init__(
        self,
        config: GelloConfig,
        operator,
        reader_factory: Callable[..., object] = DynamixelJointReader,
    ) -> None:
        self.config = config
        self.operator = operator
        self.max_relative_delta = {
            side: np.asarray(
                getattr(config, side).max_relative_delta, dtype=float
            )
            for side in SIDES
        }
        self.joint_limit_margin = {
            side: np.asarray(
                getattr(config, side).joint_limit_margin, dtype=float
            )
            for side in SIDES
        }
        self.max_target_velocity = config.max_target_velocity
        self.joint_sensitivity = {
            side: np.asarray(
                getattr(config, side).joint_sensitivity, dtype=float
            )
            for side in SIDES
        }
        standard = np.asarray(config.standard_signs, dtype=float)
        self._readers = {}
        try:
            for side in SIDES:
                self._readers[side] = _SideReader(
                    side,
                    getattr(config, side),
                    baudrate=config.baudrate,
                    joint_ids=config.joint_ids,
                    signs=standard
                    * np.asarray(
                        getattr(config, side).direction_correction, dtype=float
                    ),
                    max_joint_jump=config.max_joint_jump,
                    stale_timeout=config.stale_timeout,
                    reader_factory=reader_factory,
                )
        except BaseException:
            for reader in self._readers.values():
                reader.close()
            raise
        self._denied = {side: False for side in SIDES}
        deadline = time.monotonic() + config.ready_timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            snapshots = {
                side: self._readers[side].snapshot() for side in SIDES
            }
            side_ready = {
                side: values is not None
                and error is None
                and updated_at is not None
                and now - updated_at <= config.stale_timeout
                for side, (values, updated_at, error, _samples) in snapshots.items()
            }
            if all(side_ready.values()):
                break
            terminal = {
                side: self._readers[side].terminal_error is not None for side in SIDES
            }
            if any(terminal.values()) and all(
                terminal[side] or side_ready[side] for side in SIDES
            ):
                break
            time.sleep(0.01)
        snapshots = {
            side: self._readers[side].snapshot() for side in SIDES
        }
        now = time.monotonic()
        ready = all(
            values is not None
            and error is None
            and updated_at is not None
            and now - updated_at <= config.stale_timeout
            for values, updated_at, error, _samples in snapshots.values()
        )
        if not ready:
            details = []
            for side in SIDES:
                values, updated_at, error, _samples = self._readers[side].snapshot()
                if error:
                    details.append(f"{side}: {error}")
                elif values is None or updated_at is None:
                    details.append(f"{side}: no joint sample")
                elif time.monotonic() - updated_at > config.stale_timeout:
                    details.append(f"{side}: joint sample is stale")
            self.close()
            raise RuntimeError(
                "GELLO startup failed before first dual-arm sample: "
                + "; ".join(details)
            )

    def sample(self) -> JointTeleopSample | None:
        now = time.monotonic()
        requested = self.operator.poll()
        hold_reader = getattr(self.operator, "poll_holds", None)
        held = (
            {side: False for side in SIDES}
            if hold_reader is None
            else hold_reader()
        )
        positions = {}
        activations = {}
        for side in SIDES:
            values, updated_at, error, _samples = self._readers[side].snapshot()
            if values is None:
                return None
            fresh = (
                error is None
                and updated_at is not None
                and now - updated_at <= self.config.stale_timeout
            )
            activations[side] = bool(requested.get(side, False) and fresh)
            positions[side] = values
            acquiring = requested.get(side, False) or held.get(side, False)
            if acquiring and not fresh and not self._denied[side]:
                detail = "GELLO input missing or stale"
                if error:
                    detail += f": {error}"
                self.operator.deny(side, detail)
                self._denied[side] = True
            elif fresh:
                self._denied[side] = False
        return JointTeleopSample(positions, activations, now)

    def status_summary(self) -> str:
        now = time.monotonic()
        parts = []
        for side in SIDES:
            _values, updated_at, error, samples = self._readers[side].snapshot()
            age = float("inf") if updated_at is None else now - updated_at
            state = (
                "stale"
                if age > self.config.stale_timeout
                else f"{age * 1e3:.0f}ms"
            )
            if error:
                state += f" error={error}"
            parts.append(f"{side}={state} n={samples}")
        return " | ".join(parts)

    def debug_feed_state(self) -> dict:
        """Structured hook reserved for a later recorder/debug logger."""
        now = time.monotonic()
        result = {"source": "gello"}
        for side in SIDES:
            values, updated_at, error, samples = self._readers[side].snapshot()
            result[side] = {
                "q": None if values is None else values.tolist(),
                "age": None if updated_at is None else now - updated_at,
                "error": error,
                "samples": samples,
            }
        return result

    def close(self) -> None:
        for reader in self._readers.values():
            reader.close()
