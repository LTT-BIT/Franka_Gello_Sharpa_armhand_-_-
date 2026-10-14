# Repository rules

- Existing sibling repositories are read-only references. Copy required code into this repository before changing it.
- `teleop_core.contract` is the single source of truth for robot topics and joint names.
- Every teleoperation source publishes `teleop_interfaces/ArmCommand`; only the safety gateway may publish the FR3 command bus.
- Hardware output defaults to disabled. Never weaken freshness, acquisition, limit, or slew checks to make a test pass.
- Keep source adapters, safety/control, and data tooling independent.
- Do not add glue-dispenser terminology or functionality.

- Operator GUI changes default to `ops/run/run_gello_full_teleop.sh --hand-source manus --hardware`. This unified entry shares `apps/operator_gui/operator_gui.py` with the legacy arms entry; do not create a separate legacy-only UI unless explicitly requested. Keep collection independent of arm/hand enable and lifecycle.
