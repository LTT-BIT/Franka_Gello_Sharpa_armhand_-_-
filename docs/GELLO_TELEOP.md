# Dual GELLO incremental teleoperation

This first stage changes only the arm input. The inherited workcell remains:

- left/right FR3 controlled by the existing ROS 2 impedance controllers;
- existing UDP bridge and safety gateway;
- left G20 and right O30i;
- MANUS hand input and independent hand worker.

No new dataset recorder is enabled in this stage. `JointTeleopSample`, mapper
diagnostics, and `debug_feed_state()` retain timestamps and calibrated leader
joints so a recorder can be added later without changing the control contract.

## Incremental mapping

Each side anchors independently on its GUI engage edge:

```text
raw_delta = calibrated_gello_now - gello_at_engage
scaled_delta = raw_delta * joint_sensitivity
relative_target = robot_at_engage + clip(scaled_delta, -max_relative_delta, max_relative_delta)
soft_lower = physical_lower + joint_limit_margin
soft_upper = physical_upper - joint_limit_margin
target = clip(relative_target, soft_lower, soft_upper)
```

Disengaging and re-engaging captures fresh anchors, so an absolute GELLO/FR3
pose match is unnecessary and the first command equals measured robot state.
Each side has seven positive `joint_sensitivity` values in `config/modes/gello.yaml`,
ordered by GELLO motor IDs 1-7. A value of `2.0` maps one degree of calibrated
GELLO displacement to two degrees of FR3 target displacement; `0.5` provides
half-scale fine control. Direction remains exclusively controlled by
`standard_signs` and `direction_correction`. Sensitivity is restricted to
0.1-2.2 and is applied before the relative-displacement limit.

The left-side `max_relative_delta` is 1.5 rad per joint. The right-side values
remain the seven different full FR3 physical spans. Independently, both arms
keep 1% of each physical span clear at each limit end, retaining the central
98% absolute range. If engagement starts outside that soft range, a stationary
GELLO holds the measured pose and outward commands are blocked while retreat
toward the safe range remains available. The GELLO mapper is configured at
0.8 rad/s, while the unchanged ROS safety gateway enforces a 0.7 rad/s outer
slew ceiling plus the hard FR3 joint limits, first-target distance, command
freshness, reset exclusion, and contact-torque gating.

GELLO motor 8 is never opened. MANUS exclusively owns both dexterous hands.

## Current verified identities

`config/modes/gello.yaml` identifies the current OpenRB-150 pair by stable USB
serial rather than transient `ttyACM` enumeration:

- left: `3523CE1C5157375037202020FF102718`
- right: `CBB557875157375037202020FF0D3429`

Both identities, motor IDs 1-7, model number 1200, and live joint streams were
verified on 2026-08-17. Never infer left/right from `ttyACM0` or `ttyACM1`.
The current replacement-pair `direction_correction` vectors are
`[1,-1,1,1,1,-1,1]` on the left and `[-1,1,1,1,1,1,-1]` on the right (left
J2/J6 and right J1/J7 flipped on 2026-09-16) before `standard_signs` is applied.
The GELLO mapper uses `0.8 rad/s`; the outer safety gateway retains the
effective `0.7 rad/s` ceiling.

Run the read-only check after reconnecting both units:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python ops/diagnostics/check_gello_ports.py --config config/modes/gello.yaml
```

If permission fails, add the operator to `dialout`, log out completely, and
log in again. Do not weaken the preflight or replace by-id paths with ttyUSB
paths.

## Driver setup

The dedicated environment is `gello-upper-body-teleop`. To reproduce the
external driver installation, create or reuse a `gello_software` checkout:

```bash
git clone https://github.com/wuphilipp/gello_software.git /path/to/gello_software
git -C /path/to/gello_software submodule update --init third_party/DynamixelSDK
GELLO_SOFTWARE_ROOT=/path/to/gello_software ops/setup/setup_gello_driver.sh
```

The runtime uses `gello.dynamixel.driver.DynamixelDriver` directly with IDs
1-7 at 57600 baud. It does not use GELLO's gripper configuration.

Before starting either Franka, hold both GELLOs still and inspect their live
joint streams:

```bash
sg dialout -c 'conda run --no-capture-output -n gello-upper-body-teleop python ops/diagnostics/inspect_gello_joints.py --duration 5'
```

The upstream driver writes torque-disable once during initialization, then
reads present position and velocity. This diagnostic never enables torque,
sends a goal position/current, or connects to either Franka. The serial IDs,
joint ordering, standard signs, and per-arm direction corrections are reused
from `/home/descfly/llx/gello_franka`; no recalibration is required while the
hardware and assembly remain unchanged.

## Start without recording

After the ordinary Docker build and GELLO preflight:

```bash
cd docker
docker compose up franka-control teleop-control moveit-ik preset-ik gello-bridge hand-control
```

In another terminal:

```bash
ops/run/run_teleop.sh
```

Then start the existing operator GUI. The backend starts disengaged.

The two PCsensor foot-pedal units are programmed with distinct keyboard keys:

| Pedal | Key | Action |
|---|---|---|
| Arm pedal | `A` | Toggle left arm Hold |
| Arm pedal (middle) | `B` | Same action as GUI `Home both arms`; hands are unaffected |
| Arm pedal | `C` | Toggle right arm Hold |
| Collection pedal | `L` | Start/stop the current episode |
| Collection pedal | `Space` | Mark a milestone while recording |

Pedal auto-repeat is disabled. The GUI window must have keyboard focus for these
ordinary keyboard-emulating pedals. `DISENGAGE ALL` remains the common software
stop for every follower; hardware emergency stopping remains separate.

`B` and `Home both arms` use the currently selected task's saved Home, including
positions manually updated with Record Home. Existing arm permits are retained
behind a forced Hold during reset. Both Hold controls are locked in the backend
until the reset service confirms measured arrival and settling. The teleop loop
does not publish active arm commands while that service owns the trajectory.
It sends no teleop command packets during reset, including empty inactive packets:
the gateway rejects those too while reset is active. The specific `reset is active`
response from a packet already in flight is treated as expected only while this
backend owns both-arm Home (including its completion tick). Other gateway faults,
and this same response outside the owned Home operation, still revoke permits.
After success the arms remain held, with their Start authorization preserved;
press `A`/`C` to release the corresponding Hold for the next teleoperation round.
A previously unauthorized arm is not automatically authorized by Home.

If reset fails, Hold release remains locked; correct the cause and retry Home.
`DISENGAGE ALL`, frontend disconnection and other safety denials still revoke
authorization, and completing Home never restores a revoked permit. Single-arm
Home, Ready trajectories, and manual Home capture retain their existing behavior.
To manually teach a new Home, explicitly stop the corresponding arm, drag it in
the existing teaching mode, and use its Record Home button as before.

### Preset relative actions

The Operator GUI exposes preset slots `Q`, `W`, and `E`. Slot configuration is
in `config/preset_actions.yaml`; `Q` currently resolves the Arm UI recording
`left__kuai1.yaml` at 65% speed, while `W` and `E` are intentionally empty. A
missing or empty slot reports in the event log and sends no arm command.

On trigger, the selected arm temporarily disengages from GELLO. The long-lived
`preset-ik` service (on `127.0.0.1:5591`, off the Franka realtime CPUs) asks
the read-only `moveit-ik` `move_group` to rebase the recorded tool-relative
path at the current measured `link8` pose, then Arm UI's MoveIt KDL
`/compute_ik` path checks every frame with collision checking and sequential
seeds. Only a complete successful solution enters the existing 100 Hz UDP
teleop/safety-gateway command path; no second controller owns the arm.
`moveit-ik` starts no ros2_control node. Pressing Q does not spawn
`docker compose run`.

Completion, `STOP PRESET`, `DISENGAGE ALL`, GUI loss, or at least 0.08 rad of
GELLO movement interrupts ownership and resets the relative mapper. If that arm
was following before the trigger it is immediately enabled again and anchors
the current GELLO joints to the current measured robot joints on the next tick;
otherwise it remains stopped. `Q` does not conflict with the existing
`Ctrl+Q` window-close shortcut.

Normal completion holds the last target until measured joint error is within
0.03 rad for 0.25 s. Failure to settle within 5 s stops the arm and does not
automatically restore following.

Each arm panel also has `Record current as Home`. Stop that arm, place it at
the desired start posture, click the button, and confirm the overwrite. The
backend reads a fresh measured seven-joint state and updates only that side in
`ros_ws/src/franka_fr3_arm_controllers/config/initial_pose.yaml`; the other
arm's Home is preserved. Recording causes no motion. Use `Home arm` later to
test the saved posture at the reset service's limited speed, with the physical
emergency stop ready.

For first hardware motion:

1. Keep the emergency stop reachable and clear both workspaces.
2. Hold both GELLO units still.
3. Start only one arm; keep both hands stopped for the first arm smoke test.
4. Make one small known motion as a smoke test of the reused mapping.
5. Disengage before moving the leader to another comfortable pose.
6. Smoke-test the other side, then test both sides.

Any stale sample, serial error, gateway rejection, robot-state loss, GUI loss,
or reset disengages the affected path. Re-engage always creates a new anchor.
