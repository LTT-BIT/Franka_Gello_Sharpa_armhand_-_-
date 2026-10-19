"""Offline coordinator recovery: no FCI connection, actual SDK or robot."""
from types import SimpleNamespace
import time
import numpy as np
import pytest
from pico_bimanual_franka_teleop import hardware
from pico_bimanual_franka_teleop.control_server import OperatorConsole
from pico_bimanual_franka_teleop.types import JointTeleopSample


def test_wait_recover_lose_state_recover_without_replaying_requests(monkeypatch):
    console=OperatorConsole()
    measured=(hardware.LOWER_LIMITS+hardware.UPPER_LIMITS)/2
    sent=[];closed=[];phase={'cycle':0}
    class Robot:
        def __init__(self,**kwargs):pass
        def receive_state(self):
            phase['cycle']+=1
            cycle=phase['cycle']
            if cycle in range(1,20) or cycle in range(23,43):
                return None
            return measured.copy()
        def take_gateway_faults(self):return ()
        def send_command(self,q,sides):
            cycle=phase['cycle']
            sent.append((cycle,tuple(sides)))
            if cycle in (20,43):
                assert not any(console.poll().values())
                assert not any(console.poll_holds().values())
                console.set_active('left',True,target='arm')
            if cycle in (1,23):
                assert not console.snapshot()['arm_available']
                with pytest.raises(ValueError):console.set_active('left',True,target='arm')
            if cycle==44:raise KeyboardInterrupt
        def close(self):closed.append('robot')
    class Source:
        output_kind='joint'
        def sample(self):
            return JointTeleopSample({side:np.zeros(7) for side in ('left','right')},console.poll(),time.monotonic())
        def close(self):closed.append('source')
    class IK:
        def __init__(self,**kwargs):
            self.configuration=SimpleNamespace(q=measured.copy());self.last_diagnostics={}
        def set_posture_reference(self,q):pass
    monkeypatch.setattr(hardware,'UdpRobotBackend',Robot)
    monkeypatch.setattr(hardware,'BimanualPinkIK',IK)
    teleop=hardware.DualFr3HardwareTeleop(command_host='unused',command_port=1,state_host='unused',state_port=2,
        state_timeout=.01,translation_scale=1.,rotation_scale=1.,control_rate=1000.,max_joint_speed=.5,
        robot_state_wait_timeout=.001,arm_source=Source(),operator=console)
    with pytest.raises(KeyboardInterrupt):teleop.run()
    assert (21,('left',)) in sent and (44,('left',)) in sent
    assert all(not sides for cycle,sides in sent if cycle<20 or 23<=cycle<43)
    assert closed==['robot','source']
