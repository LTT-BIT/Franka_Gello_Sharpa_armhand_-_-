import time
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
import pytest

from pico_bimanual_franka_teleop import hardware
from pico_bimanual_franka_teleop.gello_input import (
    _SideReader,
    DynamixelJointReader,
    DualGelloJointInput,
    GelloConfig,
    GelloSideConfig,
    load_gello_config,
)
from pico_bimanual_franka_teleop.joint_mapping import RelativeJointMapper
from pico_bimanual_franka_teleop.types import JointTeleopSample


REPO_ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('extra_fault',[None,'left joint limit violation','dual-arm state is stale','unowned_busy'])
def test_home_loop_yields_bus_then_holds_measured_home_until_next_pedal(monkeypatch,extra_fault):
    import threading
    from pico_bimanual_franka_teleop.control_server import OperatorConsole
    console=OperatorConsole()
    for side in ('left','right'):console.set_active(side,True,target='arm')
    initial=(hardware.LOWER_LIMITS+hardware.UPPER_LIMITS)/2
    home=initial+.03
    finish=threading.Event();sent=[]
    class Source:
        output_kind='joint'
        def sample(self):
            return JointTeleopSample({'left':np.zeros(7),'right':np.zeros(7)},console.poll(),time.monotonic())
        def close(self):pass
    class Robot:
        def __init__(self,**kwargs):self.cycles=0
        def wait_for_state(self,timeout):return initial.copy()
        def receive_state(self):
            self.cycles+=1
            if self.cycles==2:
                assert not sent and console.home_locked
                assert all(console.snapshot()['arm_authorized'].values())
                finish.set();teleop.reset_thread.join(1)
            return (home if self.cycles>=2 else initial).copy()
        def take_gateway_faults(self):
            # Includes a queued response on the exact worker-completion tick.
            faults=('reset is active',) if self.cycles in (2,3) else ()
            if self.cycles==2 and extra_fault not in (None,'unowned_busy'):
                faults+=(extra_fault,)
            if self.cycles==4 and extra_fault=='unowned_busy':faults+=('reset is active',)
            return faults
        def send_command(self,q,sides):
            sent.append((np.asarray(q).copy(),tuple(sides)))
            if len(sent)==1:
                assert not console.home_locked
                assert not any(console.poll().values())
                expected={'left':extra_fault in (None,'unowned_busy'),'right':extra_fault!='dual-arm state is stale'}
                assert console.snapshot()['arm_authorized']==expected
                assert console.poll_holds()==expected
                np.testing.assert_allclose(q,home)
                console.set_hold('right' if extra_fault=='left joint limit violation' else 'left',False)
            else:
                expected={'left':extra_fault is None,'right':extra_fault=='left joint limit violation'}
                assert console.poll()==expected
                np.testing.assert_allclose(q,home)  # Fresh mapper anchor, no jump.
                raise KeyboardInterrupt
        def close(self):pass
    class IK:
        def __init__(self,**kwargs):
            self.configuration=types.SimpleNamespace(q=initial.copy());self.last_diagnostics={}
        def set_posture_reference(self,q):pass
    monkeypatch.setattr(hardware,'UdpRobotBackend',Robot)
    monkeypatch.setattr(hardware,'BimanualPinkIK',IK)
    teleop=hardware.DualFr3HardwareTeleop(command_host='unused',command_port=1,state_host='unused',state_port=2,
        state_timeout=.1,translation_scale=1.,rotation_scale=1.,control_rate=100.,max_joint_speed=.5,
        robot_state_wait_timeout=.1,arm_source=Source(),operator=console,
        reset_invoker=lambda side,task:(finish.wait(2),'settled'))
    console.request('reset','assembly')
    try:
        with pytest.raises(KeyboardInterrupt):teleop.run()
    finally:
        finish.set()
        if teleop.reset_thread is not None:teleop.reset_thread.join(1)
    expected_sides=() if extra_fault=='dual-arm state is stale' else ('right',) if extra_fault=='left joint limit violation' else ('left','right')
    assert [sides for _,sides in sent]==[expected_sides,() if extra_fault=='unowned_busy' else expected_sides]


def test_checked_in_config_preserves_verified_identities_and_directions():
    config = load_gello_config(REPO_ROOT / "config" / "modes" / "gello.yaml")

    assert config.left.expected_serial == "3523CE1C5157375037202020FF102718"
    assert config.right.expected_serial == "CBB557875157375037202020FF0D3429"
    assert config.joint_ids == (1, 2, 3, 4, 5, 6, 7)
    assert config.left.direction_correction == (1, -1, 1, 1, 1, -1, 1)
    assert config.right.direction_correction == (-1, 1, 1, 1, 1, 1, -1)
    assert config.left.joint_sensitivity == (0.7, 0.7, 1.0, 1.0, 1.0, 1.0, 1.0)
    assert config.right.joint_sensitivity == (0.7, 0.7, 1.0, 1.0, 1.0, 1.0, 1.0)
    assert config.left.max_relative_delta == (1.5,) * 7
    physical_spans = hardware.UPPER_LIMITS[7:14] - hardware.LOWER_LIMITS[7:14]
    np.testing.assert_allclose(
        config.right.max_relative_delta,
        physical_spans,
    )
    np.testing.assert_allclose(
        config.left.joint_limit_margin,
        physical_spans * 0.01,
    )
    np.testing.assert_allclose(
        config.right.joint_limit_margin,
        physical_spans * 0.01,
    )
    assert config.max_target_velocity == 0.8


def test_relative_mapper_has_no_engage_jump_and_reanchors():
    mapper = RelativeJointMapper(
        lower_limits=np.full(7, -2.0),
        upper_limits=np.full(7, 2.0),
        max_relative_delta=0.25,
    )
    leader = np.linspace(-1.0, 1.0, 7)
    measured = np.linspace(-0.5, 0.5, 7)

    np.testing.assert_array_equal(mapper.update(leader, True, measured), measured)
    target = mapper.update(leader + 0.1, True, measured + 0.02)
    np.testing.assert_allclose(target, measured + 0.1)

    assert mapper.update(leader + 1.0, False, measured) is None
    moved_robot = measured - 0.2
    np.testing.assert_array_equal(
        mapper.update(leader + 1.0, True, moved_robot), moved_robot
    )


def test_relative_mapper_clips_displacement_and_robot_limits():
    mapper = RelativeJointMapper(
        lower_limits=np.full(7, -0.2),
        upper_limits=np.full(7, 0.2),
        max_relative_delta=0.1,
    )
    mapper.update(np.zeros(7), True, np.full(7, 0.15))
    target = mapper.update(np.ones(7), True, np.full(7, 0.15))
    np.testing.assert_array_equal(target, np.full(7, 0.2))


def test_relative_mapper_applies_per_joint_sensitivity_before_displacement_limit():
    mapper = RelativeJointMapper(
        lower_limits=np.full(7, -2.0),
        upper_limits=np.full(7, 2.0),
        max_relative_delta=0.25,
        joint_sensitivity=np.array([0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0]),
    )
    mapper.update(np.zeros(7), True, np.zeros(7))

    target = mapper.update(np.full(7, 0.1), True, np.zeros(7))
    np.testing.assert_allclose(target, [0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.2])

    limited = mapper.update(np.ones(7), True, np.zeros(7))
    np.testing.assert_allclose(limited, np.full(7, 0.25))


def test_relative_mapper_applies_per_joint_displacement_limits():
    limits = np.linspace(0.1, 0.7, 7)
    mapper = RelativeJointMapper(
        lower_limits=np.full(7, -2.0),
        upper_limits=np.full(7, 2.0),
        max_relative_delta=limits,
    )
    mapper.update(np.zeros(7), True, np.zeros(7))

    target = mapper.update(np.ones(7), True, np.zeros(7))

    np.testing.assert_allclose(target, limits)


def test_relative_mapper_insets_each_absolute_joint_limit():
    margins = np.linspace(0.05, 0.35, 7)
    mapper = RelativeJointMapper(
        lower_limits=np.full(7, -2.0),
        upper_limits=np.full(7, 2.0),
        max_relative_delta=5.0,
        joint_limit_margin=margins,
    )
    mapper.update(np.zeros(7), True, np.zeros(7))

    upper_target = mapper.update(np.full(7, 10.0), True, np.zeros(7))
    np.testing.assert_allclose(upper_target, 2.0 - margins)

    mapper.reset()
    mapper.update(np.zeros(7), True, np.zeros(7))
    lower_target = mapper.update(np.full(7, -10.0), True, np.zeros(7))
    np.testing.assert_allclose(lower_target, -2.0 + margins)


def test_relative_mapper_rejects_margin_that_expands_or_eliminates_range():
    with pytest.raises(ValueError, match="nonnegative"):
        RelativeJointMapper(
            lower_limits=np.full(7, -1.0),
            upper_limits=np.full(7, 1.0),
            max_relative_delta=1.0,
            joint_limit_margin=-0.1,
        )
    with pytest.raises(ValueError, match="no usable joint range"):
        RelativeJointMapper(
            lower_limits=np.full(7, -1.0),
            upper_limits=np.full(7, 1.0),
            max_relative_delta=1.0,
            joint_limit_margin=1.0,
        )


def test_relative_mapper_holds_outside_soft_range_and_preserves_retreat():
    mapper = RelativeJointMapper(
        lower_limits=np.full(7, -2.0),
        upper_limits=np.full(7, 2.0),
        max_relative_delta=2.0,
        joint_limit_margin=0.5,
    )
    signs = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
    measured = signs * 1.8
    np.testing.assert_array_equal(mapper.update(np.zeros(7), True, measured), measured)

    held = mapper.update(np.zeros(7), True, measured)
    np.testing.assert_array_equal(held, measured)
    blocked_outward = mapper.update(signs * 0.5, True, measured)
    np.testing.assert_array_equal(blocked_outward, measured)
    retreat = mapper.update(signs * -0.5, True, measured)
    np.testing.assert_allclose(retreat, signs * 1.3)

    # While the robot is still outside, an outward reversal can never command
    # beyond its live measured position. Once measured inside, the configured
    # soft boundary applies again.
    still_outside = signs * 1.7
    blocked_at_measurement = mapper.update(signs * 0.5, True, still_outside)
    np.testing.assert_allclose(blocked_at_measurement, still_outside)
    inside = signs * 1.4
    clipped_to_soft_limit = mapper.update(signs * 0.5, True, inside)
    np.testing.assert_allclose(clipped_to_soft_limit, signs * 1.5)


def test_relative_mapper_applies_gello_target_velocity_limit():
    mapper = RelativeJointMapper(
        lower_limits=np.full(7, -2.0),
        upper_limits=np.full(7, 2.0),
        max_relative_delta=0.25,
        max_target_velocity=0.5,
        nominal_dt=0.01,
    )
    mapper.update(np.zeros(7), True, np.zeros(7), now=1.0)
    target = mapper.update(np.ones(7), True, np.zeros(7), now=1.1)
    np.testing.assert_allclose(target, np.full(7, 0.05))


class _Operator:
    def __init__(self):
        self.active = {"left": True, "right": True}
        self.denied = []

    def poll(self):
        return dict(self.active)

    def deny(self, side, reason):
        self.denied.append((side, reason))
        self.active[side] = False


class _FakeReader:
    created = []

    def __init__(self, port, baudrate, joint_ids):
        self.port = port
        self.baudrate = baudrate
        self.joint_ids = joint_ids
        self.closed = False
        _FakeReader.created.append(self)

    def read(self):
        base = 1.0 if "LEFT" in self.port else 2.0
        return np.full(7, base)

    def close(self):
        self.closed = True


class _JumpThenStableReader:
    def __init__(self, *_args):
        self.calls = 0

    def read(self):
        self.calls += 1
        return np.zeros(7) if self.calls <= 20 else np.ones(7)

    def close(self):
        return None


def test_side_reader_reacquires_stable_input_after_fail_closed_jump():
    side = _SideReader(
        "right",
        GelloSideConfig(
            "/dev/RIGHT",
            "RIGHT",
            (1, 1, 1, 1, 1, 1, 1),
            (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
            (1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5),
        ),
        baudrate=57600,
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        signs=np.ones(7),
        max_joint_jump=0.35,
        stale_timeout=0.25,
        reader_factory=_JumpThenStableReader,
    )
    try:
        deadline = time.monotonic() + 0.5
        observed_fault = None
        while time.monotonic() < deadline:
            _values, _updated_at, error, _samples = side.snapshot()
            if error is not None:
                observed_fault = error
                break
            time.sleep(0.001)
        assert observed_fault is not None
        assert "exceeds 0.350 rad" in observed_fault

        while time.monotonic() < deadline:
            values, _updated_at, error, _samples = side.snapshot()
            if error is None and values is not None and np.allclose(values, 1.0):
                break
            time.sleep(0.001)
        else:
            raise AssertionError("stable GELLO input was not reacquired")
    finally:
        side.close()


def _config(stale_timeout=0.25, ready_timeout=0.2):
    return GelloConfig(
        left=GelloSideConfig(
            "/dev/LEFT",
            "LEFT",
            (1, 1, 1, -1, 1, -1, 1),
            (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0),
            (1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5),
        ),
        right=GelloSideConfig(
            "/dev/RIGHT",
            "RIGHT",
            (1, 1, 1, -1, 1, -1, 1),
            (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0),
            (1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5),
        ),
        baudrate=57600,
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        standard_signs=(1, -1, 1, -1, 1, 1, 1),
        ready_timeout=ready_timeout,
        stale_timeout=stale_timeout,
        max_joint_jump=0.35,
        max_target_velocity=0.5,
    )


def test_dual_input_opens_only_arm_motors_and_applies_configured_signs():
    _FakeReader.created = []
    operator = _Operator()
    source = DualGelloJointInput(_config(), operator, reader_factory=_FakeReader)
    try:
        sample = source.sample()
        assert sample is not None
        assert sample.activations == {"left": True, "right": True}
        np.testing.assert_array_equal(
            sample.positions["left"], [1, -1, 1, 1, 1, -1, 1]
        )
        np.testing.assert_array_equal(
            sample.positions["right"], [2, -2, 2, 2, 2, -2, 2]
        )
        assert [reader.joint_ids for reader in _FakeReader.created] == [
            (1, 2, 3, 4, 5, 6, 7),
            (1, 2, 3, 4, 5, 6, 7),
        ]
    finally:
        source.close()
    assert all(reader.closed for reader in _FakeReader.created)


def test_hardware_reader_fails_closed_when_port_is_occupied(monkeypatch):
    class Driver:
        def __init__(self, *_args, **_kwargs):
            raise AssertionError("occupied port must fail before driver construction")

    driver_module = types.ModuleType("gello.dynamixel.driver")
    driver_module.DynamixelDriver = Driver
    monkeypatch.setitem(sys.modules, "gello.dynamixel.driver", driver_module)
    monkeypatch.setattr(
        "pico_bimanual_franka_teleop.gello_input.subprocess.run",
        lambda *_args, **_kwargs: types.SimpleNamespace(
            returncode=0, stdout="123\n456\n"
        ),
    )

    with pytest.raises(RuntimeError, match="PID\\(s\\) 123, 456"):
        DynamixelJointReader("/dev/test-gello", 57600, tuple(range(1, 8)))


def test_hardware_reader_disables_upstream_recovery_and_fake_fallback(monkeypatch):
    constructed = {}

    class Driver:
        def __init__(self, ids, **kwargs):
            constructed["instance"] = self
            constructed["ids"] = ids
            constructed["kwargs"] = kwargs

        def get_joints(self):
            return np.arange(7, dtype=float)

        def close(self):
            constructed["closed"] = True

    driver_module = types.ModuleType("gello.dynamixel.driver")
    driver_module.DynamixelDriver = Driver
    monkeypatch.setitem(sys.modules, "gello.dynamixel.driver", driver_module)
    monkeypatch.setattr(
        "pico_bimanual_franka_teleop.gello_input.subprocess.run",
        lambda *_args, **_kwargs: types.SimpleNamespace(returncode=1, stdout=""),
    )

    reader = DynamixelJointReader("/dev/test-gello", 57600, tuple(range(1, 8)))
    np.testing.assert_array_equal(reader.read(), np.arange(7, dtype=float))
    assert constructed["ids"] == list(range(1, 8))
    assert constructed["kwargs"]["max_retries"] == 1
    assert constructed["kwargs"]["use_fake_fallback"] is False
    instance = constructed["instance"]
    assert instance._check_port_availability() is True
    assert instance._kill_processes_using_port() is False
    assert instance._fix_port_permissions() is False
    assert instance._prepare_port() is None
    reader.close()
    assert constructed["closed"] is True


def test_hardware_reader_uses_bus_heartbeat_instead_of_cached_joint_values(
    monkeypatch,
):
    constructed = {}

    class SyncRead:
        def txRxPacket(self):
            return 0

    class ReadingThread:
        def is_alive(self):
            return True

    class StopEvent:
        def __init__(self):
            self.stopped = False

        def set(self):
            self.stopped = True

    class Driver:
        def __init__(self, *_args, **_kwargs):
            constructed["instance"] = self
            self._groupSyncRead = SyncRead()
            self._stop_thread = StopEvent()
            self._start_reading_thread()

        def _start_reading_thread(self):
            self._groupSyncRead.txRxPacket()
            self._reading_thread = ReadingThread()

        def get_joints(self):
            return np.zeros(7)

        def close(self):
            return None

    driver_module = types.ModuleType("gello.dynamixel.driver")
    driver_module.DynamixelDriver = Driver
    monkeypatch.setitem(sys.modules, "gello.dynamixel.driver", driver_module)
    monkeypatch.setattr(
        "pico_bimanual_franka_teleop.gello_input.subprocess.run",
        lambda *_args, **_kwargs: types.SimpleNamespace(returncode=1, stdout=""),
    )

    reader = DynamixelJointReader("/dev/test-gello", 57600, tuple(range(1, 8)))
    assert reader.health_error(0.25) is None
    driver = constructed["instance"]
    driver._last_successful_read_at = None
    driver._read_started_at = time.monotonic() - 1.0
    assert reader.health_error(0.25).startswith(
        "no successful Dynamixel bus read for "
    )
    assert driver._stop_thread.stopped is True
    reader.close()


class _OneFrameReader(_FakeReader):
    def __init__(self, *args):
        super().__init__(*args)
        self.failed = False

    def read(self):
        if self.failed:
            raise RuntimeError("disconnected")
        return super().read()


class _NoFrameReader(_FakeReader):
    def read(self):
        raise RuntimeError("no successful Dynamixel bus read")


class _CachedAfterDisconnectReader(_FakeReader):
    def __init__(self, *args):
        super().__init__(*args)
        self.failed = False

    def health_error(self, _max_age):
        if self.failed:
            return "Dynamixel bus read stale for 0.250s"
        return None


def test_stale_side_is_disengaged_without_stopping_the_other_control_contract():
    _FakeReader.created = []
    operator = _Operator()
    source = DualGelloJointInput(
        _config(stale_timeout=0.01),
        operator,
        reader_factory=_OneFrameReader,
    )
    try:
        for reader in _FakeReader.created:
            reader.failed = True
        time.sleep(0.03)
        sample = source.sample()
        assert sample is not None
        assert sample.activations == {"left": False, "right": False}
        assert {side for side, _reason in operator.denied} == {"left", "right"}
    finally:
        source.close()


def test_dual_input_fails_startup_when_no_hardware_sample_arrives():
    with pytest.raises(
        RuntimeError,
        match=(
            "GELLO startup failed before first dual-arm sample: "
            "left: no successful Dynamixel bus read; "
            "right: no successful Dynamixel bus read"
        ),
    ):
        DualGelloJointInput(
            _config(ready_timeout=0.02),
            _Operator(),
            reader_factory=_NoFrameReader,
        )


def test_cached_driver_data_cannot_keep_disconnected_side_engaged():
    _FakeReader.created = []
    operator = _Operator()
    source = DualGelloJointInput(
        _config(),
        operator,
        reader_factory=_CachedAfterDisconnectReader,
    )
    try:
        right_reader = next(
            reader for reader in _FakeReader.created if "RIGHT" in reader.port
        )
        right_reader.failed = True
        deadline = time.monotonic() + 0.1
        while source.debug_feed_state()["right"]["error"] is None:
            assert time.monotonic() < deadline
            time.sleep(0.001)
        sample = source.sample()
        assert sample is not None
        assert sample.activations == {"left": True, "right": False}
        assert operator.denied == [
            (
                "right",
                "GELLO input missing or stale: "
                "Dynamixel bus read stale for 0.250s",
            )
        ]
        debug = source.debug_feed_state()
        assert debug["left"]["error"] is None
        assert debug["right"]["error"] == "Dynamixel bus read stale for 0.250s"
    finally:
        source.close()


def test_hardware_coordinator_anchors_joint_input_to_measured_state(monkeypatch):
    measured = np.linspace(-0.7, 0.7, 14)
    sent = []
    closed = {"source": False, "robot": False}

    class Source:
        output_kind = "joint"
        max_relative_delta = {
            "left": np.full(7, 0.25),
            "right": np.linspace(0.3, 0.9, 7),
        }
        joint_sensitivity = {
            "left": np.full(7, 0.5),
            "right": np.full(7, 2.0),
        }
        joint_limit_margin = {
            "left": np.linspace(0.01, 0.07, 7),
            "right": np.linspace(0.08, 0.14, 7),
        }
        def sample(self):
            return JointTeleopSample(
                {"left": np.full(7, 1.0), "right": np.full(7, -1.0)},
                {"left": True, "right": True},
                time.monotonic(),
            )

        def close(self):
            closed["source"] = True

    class Operator:
        def take_requests(self):
            return {}

        def disable_all(self, _reason):
            return None

        def deny(self, _side, _reason):
            return None

        def show(self, _message):
            return None

        def set_status(self, _status):
            return None

    class Robot:
        def __init__(self, **_kwargs):
            pass

        def wait_for_state(self, timeout):
            return measured.copy()

        def receive_state(self):
            return measured.copy()

        def take_gateway_faults(self):
            return ()

        def send_command(self, q, sides):
            sent.append((np.asarray(q).copy(), tuple(sides)))
            raise KeyboardInterrupt

        def close(self):
            closed["robot"] = True

    class IK:
        def __init__(self, **_kwargs):
            self.configuration = type("Configuration", (), {"q": np.zeros(14)})()
            self.last_diagnostics = {}

        def set_posture_reference(self, _q):
            return None

    monkeypatch.setattr(hardware, "UdpRobotBackend", Robot)
    monkeypatch.setattr(hardware, "BimanualPinkIK", IK)
    teleop = hardware.DualFr3HardwareTeleop(
        command_host="unused",
        command_port=1,
        state_host="unused",
        state_port=2,
        state_timeout=0.1,
        translation_scale=1.0,
        rotation_scale=1.0,
        control_rate=100.0,
        max_joint_speed=0.5,
        robot_state_wait_timeout=0.1,
        arm_source=Source(),
        operator=Operator(),
    )
    np.testing.assert_array_equal(
        teleop.mappers["left"].joint_sensitivity, np.full(7, 0.5)
    )
    np.testing.assert_array_equal(
        teleop.mappers["right"].joint_sensitivity, np.full(7, 2.0)
    )
    np.testing.assert_array_equal(
        teleop.mappers["left"].max_relative_delta, np.full(7, 0.25)
    )
    np.testing.assert_allclose(
        teleop.mappers["right"].max_relative_delta, np.linspace(0.3, 0.9, 7)
    )
    np.testing.assert_allclose(
        teleop.mappers["left"].joint_limit_margin,
        np.linspace(0.01, 0.07, 7),
    )
    np.testing.assert_allclose(
        teleop.mappers["right"].joint_limit_margin,
        np.linspace(0.08, 0.14, 7),
    )
    with pytest.raises(KeyboardInterrupt):
        teleop.run()

    np.testing.assert_array_equal(sent[0][0], measured)
    assert sent[0][1] == ("left", "right")
    assert closed == {"source": True, "robot": True}


def test_hardware_reader_fails_closed_when_ownership_check_times_out(monkeypatch):
    driver_module = types.ModuleType("gello.dynamixel.driver")
    driver_module.DynamixelDriver = lambda *a, **kw: pytest.fail("driver must not open on timeout")
    monkeypatch.setitem(sys.modules, "gello.dynamixel.driver", driver_module)

    def timed_out(command, **kwargs):
        assert command == ["fuser", "/dev/test-gello"]
        assert kwargs["timeout"] == 5.0
        raise subprocess.TimeoutExpired(command, kwargs["timeout"])

    monkeypatch.setattr("pico_bimanual_franka_teleop.gello_input.subprocess.run", timed_out)
    with pytest.raises(RuntimeError, match="ownership check timed out"):
        DynamixelJointReader("/dev/test-gello", 57600, tuple(range(1, 8)))
