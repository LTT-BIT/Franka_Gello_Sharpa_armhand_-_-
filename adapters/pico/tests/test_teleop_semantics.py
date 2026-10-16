"""Lock the teleoperation semantics the operator relies on.

On engage, the mapper anchors the current hand pose and the current end-effector
pose; thereafter deltas map one to one: rotate the hand 90 degrees about a world
axis and the commanded end effector rotates 90 degrees about that same world
axis. And the IK that executes those targets must neither jump configurations
nor let the elbow drift: with a 7-DoF arm every end-effector pose has a
one-parameter family of elbow configurations, and without an attractor the
configuration random-walked 0.88 rad in a single closed end-effector loop.
"""

import numpy as np
import pinocchio as pin
import pytest

from pico_bimanual_franka_teleop.hardware import (
    DualFr3HardwareTeleop,
    disengage_sample_sides,
    latch_new_arm_holds,
    reseed_inactive_joints,
)
from pico_bimanual_franka_teleop.ik import BimanualPinkIK, classify_step
from pico_bimanual_franka_teleop.pose_mapping import RelativePoseMapper
from pico_bimanual_franka_teleop.relative_action import SolvedRelativeAction
from pico_bimanual_franka_teleop.types import Pose, TeleopSample

# A captured hardware home (2026-07-25). The live initial_pose.yaml may be
# newer; these tests only need a physically plausible dual-arm posture.
HOME_Q = np.array([
    -0.110970355570, -0.493441432714, 0.114729285240, -2.441572666170,
    0.119077377021, 2.436770200730, 0.340127378702,
    0.917802035809, -0.843023777008, -1.136612057686, -2.587259292603,
    0.054329961538, 3.444272756577, -1.538317084312,
])


def make_ik() -> BimanualPinkIK:
    ik = BimanualPinkIK(dt=0.01, max_joint_speed=0.5)
    ik.set_posture_reference(HOME_Q)
    return ik


def engage(mapper, hand, robot):
    assert mapper.update(hand, True, robot) is not None


def test_inactive_and_newly_engaging_sides_reseed_independently():
    held = np.arange(14, dtype=float)
    measured = held + 100.0

    # Left is already tracking and must retain its command. Right is newly
    # engaging, so its anchor must use measured hardware even though left
    # remains active.
    result = reseed_inactive_joints(
        held,
        measured,
        {"left": True, "right": True},
        {"left": True, "right": False},
    )
    np.testing.assert_array_equal(result[:7], held[:7])
    np.testing.assert_array_equal(result[7:], measured[7:])

    # An explicitly inactive side is continuously healed from measured state.
    result = reseed_inactive_joints(
        held,
        measured,
        {"left": False, "right": True},
        {"left": True, "right": True},
    )
    np.testing.assert_array_equal(result[:7], measured[:7])
    np.testing.assert_array_equal(result[7:], held[7:])

    # A held side must retain the latched command instead of drifting with
    # measured state while it continues to be published.
    result = reseed_inactive_joints(
        held,
        measured,
        {"left": False, "right": False},
        {"left": False, "right": False},
        {"left": True, "right": False},
    )
    np.testing.assert_array_equal(result[:7], held[:7])
    np.testing.assert_array_equal(result[7:], measured[7:])


def test_hold_rising_edge_latches_measured_joints_once():
    commanded = np.arange(14, dtype=float)
    measured = commanded + 100.0
    holds = {"left": True, "right": False}

    latched = latch_new_arm_holds(
        commanded, measured, holds, {"left": False, "right": False}
    )
    np.testing.assert_array_equal(latched[:7], measured[:7])
    np.testing.assert_array_equal(latched[7:], commanded[7:])

    # Subsequent ticks retain the first latched pose even if feedback changes.
    retained = latch_new_arm_holds(
        latched, measured + 50.0, holds, {"left": True, "right": False}
    )
    np.testing.assert_array_equal(retained, latched)


def test_open_hand_disengages_only_selected_side_before_opening():
    denied = []
    opened = []
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator",
        (),
        {"deny_hand": lambda _self, side, reason: denied.append((side, reason))},
    )()
    teleop.hands = type(
        "Hands",
        (),
        {"request_open": lambda _self, *, sides: opened.append(sides)},
    )()
    teleop._notify = lambda _message: None

    teleop._open_hands(("right",))

    assert denied == [("right", "opening hand")]
    assert opened == [("right",)]


def test_home_arm_does_not_also_open_the_hand():
    opened = []
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator",
        (),
        {"disable_all": lambda _self, _reason: None},
    )()
    teleop.mappers = {
        side: type("Mapper", (), {"reset": lambda _self: None})()
        for side in ("left", "right")
    }
    teleop.hands = type(
        "Hands",
        (),
        {"request_open": lambda _self, **_kwargs: opened.append(True)},
    )()
    teleop.reset_invoker = lambda side, task: (True, f"{side or 'both'}:{task}")
    teleop.reset_thread = None
    teleop.reset_outcome = []
    teleop._notify = lambda _message: None

    teleop._start_reset("left")
    teleop.reset_thread.join(timeout=1.0)

    assert opened == []


@pytest.mark.parametrize('succeeded',[True,False])
def test_both_home_retains_arm_permits_and_waits_for_reset_confirmation(succeeded):
    import threading
    from pico_bimanual_franka_teleop.control_server import OperatorConsole
    console=OperatorConsole()
    for side in ('left','right'):console.set_active(side,True)
    finish=threading.Event()
    teleop=object.__new__(DualFr3HardwareTeleop)
    teleop.operator=console;teleop.reset_thread=None;teleop.reset_outcome=[]
    teleop.mappers={side:type('Mapper',(),{'reset':lambda self:None})() for side in ('left','right')}
    teleop._notify=lambda message:None
    teleop.reset_invoker=lambda side,task:(finish.wait(2) and succeeded,'measured settle result')
    try:
        teleop._start_reset(task='assembly')
        assert console.home_locked and all(console.snapshot()['arm_authorized'].values())
        assert all(console.poll_hands().values())
        teleop._service_reset()
        assert console.home_locked
        with pytest.raises(ValueError,match='locked'):console.set_hold('left',False)
        finish.set();teleop.reset_thread.join(1)
        assert console.home_locked  # Worker completion alone cannot unlock.
        teleop._service_reset()
        assert console.home_locked is not succeeded
        assert all(console.snapshot()['arm_authorized'].values())
        assert all(console.poll_holds().values()) and not any(console.poll().values())
        assert teleop.hold_q is None
        if succeeded:
            console.set_hold('left',False)
            assert console.poll()=={'left':True,'right':False}
    finally:
        finish.set()
        if teleop.reset_thread is not None:teleop.reset_thread.join(1)


def test_home_moves_only_arm_and_never_requests_a_hand_pose():
    order = []
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator", (), {"disable_all": lambda _self, _reason: None}
    )()
    teleop.mappers = {
        side: type("Mapper", (), {"reset": lambda _self: None})()
        for side in ("left", "right")
    }
    teleop.hands = type(
        "Hands",
        (),
        {"request_pose": lambda _self, poses: order.append(("hands", poses))},
    )()
    teleop.reset_invoker = lambda side, task: (
        order.append(("arms", side, task)) or (True, "arm done")
    )
    teleop.reset_thread = None
    teleop.reset_outcome = []
    teleop.capture_thread = None
    teleop.preset_thread = None
    teleop.active_preset = None
    teleop.hold_q = np.zeros(14)
    notifications = []
    teleop._notify = notifications.append

    teleop._start_reset("left", "assembly")
    teleop.reset_thread.join(timeout=1.0)
    teleop._service_reset()

    assert order == [("arms", "left", "assembly")]
    assert notifications[-1] == "reset done: arm done"


def test_ready_to_home_disengages_and_invokes_selected_task():
    invoked = []
    disabled = []
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator",
        (),
        {
            "poll": lambda _self: {"left": False, "right": False},
            "disable_all": lambda _self, reason: disabled.append(reason),
        },
    )()
    teleop.mappers = {
        side: type("Mapper", (), {"reset": lambda _self: None})()
        for side in ("left", "right")
    }
    teleop.ready_invoker = None
    teleop.ready_to_home_invoker = lambda task: (
        invoked.append(task) or (True, "played")
    )
    teleop.reset_thread = None
    teleop.reset_outcome = []
    teleop.hands = None
    teleop.capture_thread = None
    teleop.preset_thread = None
    teleop.active_preset = None
    teleop._notify = lambda _message: None

    teleop._start_ready_operation("trajectory", "assembly")
    teleop.reset_thread.join(timeout=1.0)

    assert invoked == ["assembly"]
    assert disabled == ["Ready trajectory"]
    assert teleop.reset_outcome == [(True, "played")]


def _capture_test_teleop(active: bool, invoker):
    notifications = []
    activation_changes = []
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator",
        (),
        {
            "poll": lambda _self: {"left": active, "right": False},
            "set_active": lambda _self, side, engaged, *, target: (
                activation_changes.append((side, engaged, target))
            ),
        },
    )()
    teleop.mappers = {
        side: type("Mapper", (), {"reset": lambda _self: None})()
        for side in ("left", "right")
    }
    teleop.reset_thread = None
    teleop.capture_home_invoker = invoker
    teleop.capture_thread = None
    teleop.capture_side = None
    teleop.capture_task = None
    teleop.capture_outcome = []
    teleop.hands = None
    teleop._notify = notifications.append
    return teleop, notifications, activation_changes


def test_capture_home_rejects_an_arm_that_is_still_following():
    invoked = []
    teleop, notifications, activation_changes = _capture_test_teleop(
        True, lambda side: invoked.append(side)
    )

    teleop._start_capture_home("left")

    assert invoked == []
    assert activation_changes == []
    assert teleop.capture_thread is None
    assert notifications == ["left Home capture rejected: stop that arm first"]


def test_capture_home_records_only_the_selected_stopped_arm():
    invoked = []
    teleop, notifications, activation_changes = _capture_test_teleop(
        False, lambda side, task: (invoked.append((side, task)) or (True, "saved"))
    )

    teleop._start_capture_home("left")
    teleop.capture_thread.join(timeout=1.0)
    teleop._service_capture_home()

    assert invoked == [("left", "powder_weighing")]
    assert activation_changes == [("left", False, "arm")]
    assert teleop.capture_thread is None
    assert notifications == [
        "left: recording current measured joints as 粉末称量 Home",
        "left 粉末称量 Home capture done: saved",
    ]


def test_capture_home_ignores_active_hand_and_never_reads_its_feedback():
    invoked = []
    teleop, notifications, _ = _capture_test_teleop(
        False, lambda side, task: (invoked.append((side, task)) or (True, "arm saved"))
    )
    teleop.operator.poll_hands = lambda: {"left": True, "right": True}

    class Hands:
        def feedback_position(self, side):
            raise AssertionError("arm Home capture must not read hand joints")

    teleop.hands = Hands()
    teleop._start_capture_home("left", "bean_picking")
    teleop.capture_thread.join(timeout=1.0)
    teleop._service_capture_home()

    assert invoked == [("left", "bean_picking")]
    assert "夹豆 Home capture done" in notifications[-1]


def test_finishing_preset_resumes_through_a_fresh_gello_anchor():
    changes = []
    notifications = []
    mapper = type(
        "Mapper", (), {"reset": lambda self: setattr(self, "reset_called", True)}
    )()
    mapper.reset_called = False
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator",
        (),
        {
            "set_active": lambda _self, side, active, *, target: changes.append(
                (side, active, target)
            )
        },
    )()
    teleop.mappers = {"left": mapper}
    teleop._notify = notifications.append
    teleop.active_preset = SolvedRelativeAction.from_dict(
        {
            "name": "test",
            "side": "left",
            "time_sec": [0.0, 0.5, 1.0],
            "positions": [[0.0] * 7] * 3,
            "start_q": [0.0] * 14,
            "speed_scale": 0.5,
        }
    )
    teleop.preset_pending = None
    teleop.preset_started_at = 1.0
    teleop.preset_leader_anchor = np.zeros(7)
    teleop.preset_cancelled = False
    teleop.preset_resume_active = True

    teleop._finish_preset("completed", resume=True)

    assert mapper.reset_called
    assert changes == [("left", True, "arm")]
    assert notifications == ["preset completed; left GELLO re-anchoring now"]


def test_completed_preset_reengages_even_if_arm_was_stopped() -> None:
    changes = []
    notifications = []
    mapper = type(
        "Mapper", (), {"reset": lambda self: setattr(self, "reset_called", True)}
    )()
    mapper.reset_called = False
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator",
        (),
        {
            "set_active": lambda _self, side, active, *, target: changes.append(
                (side, active, target)
            )
        },
    )()
    teleop.mappers = {"left": mapper}
    teleop._notify = notifications.append
    teleop.active_preset = SolvedRelativeAction.from_dict(
        {
            "name": "kuai1",
            "side": "left",
            "time_sec": [0.0, 0.5, 1.0],
            "positions": [[0.0] * 7] * 3,
            "start_q": [0.0] * 14,
            "speed_scale": 0.5,
        }
    )
    teleop.preset_pending = None
    teleop.preset_started_at = 1.0
    teleop.preset_leader_anchor = np.zeros(7)
    teleop.preset_cancelled = False
    teleop.preset_resume_active = False

    teleop._finish_preset("completed", resume=True)

    assert mapper.reset_called
    assert changes == [("left", True, "arm")]
    assert notifications == ["preset completed; left GELLO re-anchoring now"]


def test_failed_preset_does_not_engage_a_stopped_arm() -> None:
    changes = []
    notifications = []
    mapper = type(
        "Mapper", (), {"reset": lambda self: setattr(self, "reset_called", True)}
    )()
    mapper.reset_called = False
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = type(
        "Operator",
        (),
        {
            "set_active": lambda _self, side, active, *, target: changes.append(
                (side, active, target)
            )
        },
    )()
    teleop.mappers = {"left": mapper}
    teleop._notify = notifications.append
    teleop.active_preset = None
    teleop.preset_pending = type("Preset", (), {"side": "left"})()
    teleop.preset_started_at = None
    teleop.preset_leader_anchor = None
    teleop.preset_cancelled = False
    teleop.preset_resume_active = False

    teleop._finish_preset("IK precheck failed", resume=True)

    assert changes == []
    assert notifications == ["preset IK precheck failed; left arm remains stopped"]


def test_open_hand_removes_stale_same_tick_activation():
    sample = TeleopSample(
        poses={
            "left": Pose(np.zeros(3), np.eye(3)),
            "right": Pose(np.ones(3), np.eye(3)),
        },
        activations={"left": True, "right": True},
        timestamp=1.0,
    )

    updated = disengage_sample_sides(sample, ("right",))

    assert updated.activations == {"left": True, "right": False}
    assert sample.activations == {"left": True, "right": True}


def test_translation_maps_one_to_one():
    mapper = RelativePoseMapper(translation_scale=1.0, rotation_scale=1.0)
    hand = Pose(np.array([0.1, 0.2, 1.0]), np.eye(3))
    robot = Pose(np.array([0.4, 0.5, 0.4]), np.eye(3))
    engage(mapper, hand, robot)
    moved = Pose(hand.position + np.array([0.07, -0.03, 0.11]), hand.rotation)
    target = mapper.update(moved, True, robot)
    np.testing.assert_allclose(
        target.position - robot.position, [0.07, -0.03, 0.11], atol=1e-12
    )


def test_rotation_maps_to_the_same_world_axis():
    # Rotate the hand 90 degrees about a world axis: the commanded end effector
    # rotates 90 degrees about that same world axis, regardless of either
    # frame's initial orientation.
    mapper = RelativePoseMapper(translation_scale=1.0, rotation_scale=1.0)
    hand = Pose(np.zeros(3), pin.exp3(np.array([0.2, 0.7, -0.4])))
    robot = Pose(np.array([0.4, 0.5, 0.4]), pin.exp3(np.array([-1.0, 0.3, 0.5])))
    engage(mapper, hand, robot)
    for world_axis in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])):
        rotation_vector = (np.pi / 2) * world_axis
        rotated = Pose(hand.position, pin.exp3(rotation_vector) @ hand.rotation)
        target = mapper.update(rotated, True, robot)
        delta = target.rotation @ robot.rotation.T
        np.testing.assert_allclose(pin.log3(delta), rotation_vector, atol=1e-9)


def test_reengage_reanchors_at_the_current_robot_pose():
    # Disengage, move the hand freely, re-engage: no jump. The new anchor is
    # wherever the robot currently is.
    mapper = RelativePoseMapper(translation_scale=1.0, rotation_scale=1.0)
    hand = Pose(np.zeros(3), np.eye(3))
    robot = Pose(np.array([0.4, 0.5, 0.4]), np.eye(3))
    engage(mapper, hand, robot)
    assert mapper.update(hand, False, robot) is None
    far_hand = Pose(np.array([5.0, 5.0, 5.0]), pin.exp3(np.array([0, 0, 2.0])))
    target = mapper.update(far_hand, True, robot)
    np.testing.assert_allclose(target.position, robot.position, atol=1e-12)
    np.testing.assert_allclose(target.rotation, robot.rotation, atol=1e-12)


def test_ik_reaches_static_targets_exactly():
    ik = make_ik()
    home = {side: ik.frame_pose(HOME_Q, side) for side in ("left", "right")}
    q = HOME_Q.copy()
    targets = {
        side: Pose(home[side].position + np.array([0.2, 0.0, 0.1]), home[side].rotation)
        for side in ("left", "right")
    }
    for _ in range(600):
        q = ik.step(q, targets)
    for side in ("left", "right"):
        error = np.linalg.norm(ik.frame_pose(q, side).position - targets[side].position)
        assert error < 0.005, f"{side}: {error * 1e3:.2f} mm"


def test_closed_ee_loops_do_not_drift_the_configuration():
    # The regression this locks: without the posture attractor, one loop drifted
    # the configuration by 0.88 rad while the end effector returned exactly.
    ik = make_ik()
    home = {side: ik.frame_pose(HOME_Q, side) for side in ("left", "right")}
    q = HOME_Q.copy()
    waypoints = (
        (np.array([0.15, 0.0, 0.0]), 0.0),
        (np.array([0.15, 0.0, 0.15]), np.radians(45)),
        (np.array([0.0, 0.0, 0.15]), 0.0),
        (np.zeros(3), 0.0),
    )
    for _ in range(3):
        for offset, angle in waypoints:
            rotation = pin.exp3(np.array([0.0, angle, 0.0]))
            targets = {
                side: Pose(home[side].position + offset, rotation @ home[side].rotation)
                for side in ("left", "right")
            }
            for _ in range(150):
                q = ik.step(q, targets)
    for _ in range(600):
        q = ik.step(q, home)
    drift = np.abs(q - HOME_Q).max()
    assert drift < 0.05, f"null-space drift {drift:.3f} rad"
    for side in ("left", "right"):
        error = np.linalg.norm(ik.frame_pose(q, side).position - home[side].position)
        assert error < 0.005


def test_configuration_evolves_continuously():
    # Differential IK integrates velocity under a hard clamp, so consecutive
    # configurations can never jump by more than max_joint_speed * dt.
    ik = make_ik()
    home = {side: ik.frame_pose(HOME_Q, side) for side in ("left", "right")}
    q = HOME_Q.copy()
    targets = {
        side: Pose(home[side].position + np.array([0.3, -0.2, 0.2]), home[side].rotation)
        for side in ("left", "right")
    }
    bound = 0.5 * 0.01 + 1e-9
    for _ in range(400):
        q_next = ik.step(q, targets)
        assert np.abs(q_next - q).max() <= bound
        q = q_next


def test_ik_diagnostics_report_ok_for_a_reached_target():
    ik = make_ik()
    home = {side: ik.frame_pose(HOME_Q, side) for side in ("left", "right")}
    q = HOME_Q.copy()
    targets = {
        side: Pose(home[side].position + np.array([0.05, 0.0, 0.05]), home[side].rotation)
        for side in ("left", "right")
    }
    for _ in range(600):
        q = ik.step(q, targets)
    for side in ("left", "right"):
        diagnostics = ik.last_diagnostics[side]
        assert classify_step(diagnostics) == "ok"
        assert diagnostics["position_error"] < 0.005
        assert not diagnostics["saturated_joints"]
        assert not diagnostics["limit_joints"]


def test_ik_diagnostics_flag_the_speed_clamp_while_catching_up():
    # A large instantaneous target step cannot be covered in one clamped tick;
    # the deficit is transient and must be attributed to the speed clamp.
    ik = make_ik()
    home = {side: ik.frame_pose(HOME_Q, side) for side in ("left", "right")}
    targets = {
        side: Pose(home[side].position + np.array([0.3, 0.0, 0.0]), home[side].rotation)
        for side in ("left", "right")
    }
    ik.step(HOME_Q.copy(), targets)
    for side in ("left", "right"):
        diagnostics = ik.last_diagnostics[side]
        assert diagnostics["position_error"] > 0.2
        assert diagnostics["saturated_joints"]
        assert classify_step(diagnostics) == "speed-clamp"


def test_ik_diagnostics_attribute_an_out_of_reach_target():
    # A target far outside the workspace: the configuration settles and the
    # residual must be attributed to a pinned joint or the workspace edge,
    # never reported as ok or still-catching-up.
    ik = make_ik()
    home = {side: ik.frame_pose(HOME_Q, side) for side in ("left", "right")}
    q = HOME_Q.copy()
    targets = {
        side: Pose(
            home[side].position + np.array([1.5, 0.0, 0.0]), home[side].rotation
        )
        for side in ("left", "right")
    }
    for _ in range(1200):
        q = ik.step(q, targets)
    for side in ("left", "right"):
        diagnostics = ik.last_diagnostics[side]
        assert diagnostics["position_error"] > 0.1
        assert classify_step(diagnostics) in ("joint-limit", "workspace")


def test_ik_diagnostics_clear_without_targets():
    ik = make_ik()
    ik.step(HOME_Q.copy(), {})
    assert ik.last_diagnostics == {}


def test_fk_pose_snapshots_do_not_change_after_a_later_fk_update():
    ik = make_ik()
    before = ik.frame_poses(HOME_Q)
    saved = {
        side: (pose.position.copy(), pose.rotation.copy())
        for side, pose in before.items()
    }
    moved = HOME_Q.copy()
    moved[0] += 0.1
    moved[7] -= 0.1
    ik.frame_poses(moved)
    for side, pose in before.items():
        np.testing.assert_array_equal(pose.position, saved[side][0])
        np.testing.assert_array_equal(pose.rotation, saved[side][1])


def test_uncommanded_side_never_moves():
    # Regression: with only one side engaged, the posture attractor used to
    # walk the other side's joints toward the reference at the speed clamp
    # (0.35 rad of phantom drift in 0.75 s, measured 2026-07-29), while the
    # real, uncommanded arm stayed put - deadlocking its re-engage against
    # the gateway's initial-delta check.
    ik = make_ik()
    home = {side: ik.frame_pose(HOME_Q, side) for side in ("left", "right")}
    q = HOME_Q.copy()
    # Drive both arms away from the posture reference first.
    targets = {
        side: Pose(home[side].position + np.array([0.2, 0.0, 0.1]), home[side].rotation)
        for side in ("left", "right")
    }
    for _ in range(400):
        q = ik.step(q, targets)
    # Now command only the left side for a while.
    left_only = {"left": targets["left"]}
    right_before = q[7:].copy()
    for _ in range(200):
        q = ik.step(q, left_only)
    np.testing.assert_array_equal(q[7:], right_before)


@pytest.mark.parametrize('succeeded', [True, False])
def test_episode_home_reports_only_measured_worker_completion(succeeded):
    import threading
    from pico_bimanual_franka_teleop.control_server import OperatorConsole
    console = OperatorConsole()
    finish = threading.Event()
    teleop = object.__new__(DualFr3HardwareTeleop)
    teleop.operator = console
    teleop.reset_thread = None
    teleop.reset_outcome = []
    teleop.mappers = {}
    teleop._notify = lambda message: None
    invoked = []
    def reset(side, task):
        invoked.append((side, task))
        finish.wait(2)
        return succeeded, 'measured settle result'
    teleop.reset_invoker = reset
    job = console.queue_episode_home(dict(id='episode-1:0', task='assembly'))
    try:
        teleop._start_episode_home(job)
        assert console.snapshot()['episode_home']['state'] == 'running'
        teleop._service_reset()
        assert console.snapshot()['episode_home']['state'] == 'running'
        finish.set()
        teleop.reset_thread.join(1)
        assert console.snapshot()['episode_home']['state'] == 'running'
        teleop._service_reset()
        assert console.snapshot()['episode_home']['state'] == ('done' if succeeded else 'failed')
        assert invoked == [(None, 'assembly')]
        # Same queued job can never trigger another reset.
        teleop._start_episode_home(job)
        assert teleop.reset_thread is None
    finally:
        finish.set()
        if teleop.reset_thread is not None:
            teleop.reset_thread.join(1)
