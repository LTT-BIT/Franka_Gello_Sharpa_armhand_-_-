"""GUI permits around the standalone MANUS sender, without validation latches.

Opening is a separate explicit action. Supervised validation and LitchiBot keep
their own policies; ordinary MANUS uses the original HAL's final joint limits.
"""
import json
import time
import numpy as np

from adapters.litchibot.full_runtime import FullHandRuntime
from adapters.litchibot.retarget import JOINT_NAMES, JOINT_LIMITS
from adapters.litchibot.validation_safety import TIMEOUT_S


class ManusForwardingAuthority:
    limit_output_slew = False
    feedback_limit_tolerance = .02

    def __init__(self, configuration, now, **unused):
        self.phase = 'READY'
        self.reason = ''
        self.requested = {s: False for s in ('left', 'right')}
        self.samples = {s: 0 for s in self.requested}
        self.last_gui = None

    def request_hand(self, side, enabled, now):
        if side not in self.requested:
            raise ValueError('Unknown hand side')
        if enabled and self.phase == 'FAULT':
            raise ValueError(self.reason)
        self.requested[side] = bool(enabled)

    def side_state(self, side, now):
        if self.phase == 'FAULT':
            return 'FAULT'
        return 'ACTIVE' if self.requested[side] else 'DISABLED'

    def trip(self, reason):
        if self.phase != 'FAULT':
            self.phase = 'FAULT'
            self.reason = str(reason)
            self.requested = {s: False for s in self.requested}


class ManusHandRuntime(FullHandRuntime):
    standalone_manus = True

    def make_raw_gate(self, config):
        from types import SimpleNamespace
        return SimpleNamespace(threshold=None)

    def make_target_gate(self, config):
        return None

    def __init__(self, node, config):
        self.feedback = {}
        self.warning_times = {}
        super().__init__(node, config)
        from sensor_msgs.msg import JointState
        from rclpy.qos import qos_profile_sensor_data
        for side in self.latch.requested:
            self.subscriptions.append(node.create_subscription(
                JointState, f'/sharpa/{side}/joint_states',
                lambda msg, s=side: self.observe_feedback(s, msg), qos_profile_sensor_data))
        node.get_logger().info('Normal MANUS: standalone forwarding with GUI permits; no acquisition, jump, matching or command-timeout latch')

    def warn(self, reason):
        reason = str(reason)
        now = time.monotonic()
        key = reason.split(':', 1)[0]
        if now - self.warning_times.get(key, -float('inf')) >= 2:
            self.warning_times[key] = now
            self.node.get_logger().warning(reason)

    def request(self, side, enabled):
        if side not in self.latch.requested:
            raise ValueError('Unknown hand side')
        if side in self.opening.operations:
            if enabled:
                raise ValueError('Wait for Open hand to finish before Start hand')
            self.opening.cancel(side)
        self.latch.request_hand(side, enabled, time.monotonic())
        if enabled:
            self.opening.results.pop(side, None)
        else:
            self.stop_hardware_side(side)

    def dispatch(self, request):
        command = request.get('command')
        args = request.get('arguments') or {}
        if command == 'heartbeat':
            # Only the bounded, explicit opening action needs a fresh lease.
            # Ordinary following is controlled by the authenticated connection.
            stamp = request.get('monotonic_ns')
            if type(stamp) is int and 0 <= time.monotonic() - stamp / 1e9 <= TIMEOUT_S:
                self.latch.last_gui = stamp / 1e9
        elif command in ('engage_hand', 'disengage_hand'):
            self.request(args.get('side'), command == 'engage_hand')
        elif command == 'disengage_all':
            for side in self.latch.requested:
                self.request(side, False)
        elif command == 'open_hand':
            self.opening.start(args.get('side'))
        elif command != 'status':
            raise ValueError('Unknown hand command')
        return self.status()

    def disconnected(self):
        # Actual GUI loss revokes permits; recovery never resumes motion itself.
        for side in self.latch.requested:
            if self.latch.requested[side] or side in self.opening.operations:
                self.request(side, False)

    def keypoint(self, side, message):
        pass  # No validation-only exact-stamp dependency in normal MANUS.

    def source_fault(self, message):
        self.warn('MANUS source status: ' + message.data)

    def manus(self, side, message):
        if self.latch.phase == 'FAULT':
            return
        try:
            q = np.asarray(message.position, dtype=float)
            if tuple(message.name) != JOINT_NAMES or q.shape != (22,) or not np.isfinite(q).all():
                raise ValueError('MANUS target requires the original 22 finite joints')
            if np.any(q < JOINT_LIMITS[:, 0]) or np.any(q > JOINT_LIMITS[:, 1]):
                raise ValueError('MANUS target rejected by original joint limits')
        except (ValueError, TypeError, AttributeError) as error:
            self.warn(error)
            return
        self.latch.samples[side] += 1
        if not self.latch.requested[side] or side in self.opening.operations:
            return
        from sensor_msgs.msg import JointState
        command = JointState()
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.header.frame_id = f'supervisor_active:manus:{side}:{self.config["token"][:16]}:{self.latch.samples[side]}'
        command.name = list(JOINT_NAMES)
        command.position = q.tolist()
        try:
            self.node.forward_full(self.native_side(side), command)
            self.publishers[side].publish(command)
        except Exception as error:
            self.node.stop_validation('MANUS native sender failed: ' + str(error))

    def native_side(self, side):
        return next(s for s in self.node._hands if s.value == side)

    def observe_feedback(self, side, message):
        try:
            if tuple(message.name) not in (JOINT_NAMES, tuple(side + '_' + n for n in JOINT_NAMES)):
                return
            q = np.asarray(message.position, dtype=float)
            if q.shape == (22,) and np.isfinite(q).all():
                self.feedback[side] = q.copy(), time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.warn('Invalid idle feedback snapshot dropped')

    def idle_snapshot(self, side):
        cached = self.feedback.get(side)
        if cached is None or time.monotonic() - cached[1] > TIMEOUT_S:
            return
        from sensor_msgs.msg import JointState
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = f'supervisor_no_send:{side}:{self.latch.side_state(side, time.monotonic())}'
        message.name = list(JOINT_NAMES)
        message.position = cached[0].tolist()
        self.publishers[side].publish(message)

    def status(self):
        now = time.monotonic()
        return {'mode': self.config['mode'], 'phase': self.latch.phase,
            'reason': self.latch.reason, 'normal_forwarding': True, 'source_ready': True,
            'hands': {s: {'state': 'OPENING' if s in self.opening.operations else self.latch.side_state(s, now),
                'permit': self.latch.requested[s], 'samples': self.latch.samples[s],
                'can_engage': self.latch.phase != 'FAULT' and s not in self.opening.operations,
                'open_status': self.opening.results.get(s, ''), 'reason': self.latch.reason}
                for s in self.latch.requested},
            'hardware_send': 0 if self.config['mode'] == 'fake' else self.node.validation_send_count,
            'mock_command_calls': self.node.validation_send_count if self.config['mode'] == 'fake' else 0}

    def tick(self):
        self.server.drain(self.dispatch, self.disconnected)
        self.opening.tick()
        now = time.monotonic()
        if now - self.last_state_at > .05:
            from std_msgs.msg import String
            message = String()
            message.data = json.dumps(self.status())
            self.state_publisher.publish(message)
            self.last_state_at = now
        if now - self.last_idle_at >= 1 / 30:
            for side in self.latch.requested:
                if not self.latch.requested[side] and side not in self.opening.operations:
                    self.idle_snapshot(side)
            self.last_idle_at = now

    def close(self):
        self.disconnected()
        self.server.close()
