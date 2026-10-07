import json

import pytest

from adapters.litchibot.normal_continuity import (
    NormalContinuityGate, full_continuity_gate)
from adapters.litchibot.minimum_interference import MinimumRawGate
from adapters.litchibot.normal_control import BasicNormalTarget
from adapters.litchibot.terminal2_bridge import WorkerInbox
from adapters.litchibot.validation_safety import RawContinuityGate
from adapters.litchibot.test_normal_continuity import row


@pytest.mark.parametrize('delta',[.143064,.179000,.185638,.18,.4])
@pytest.mark.parametrize('mode',['normal','fake'])
@pytest.mark.parametrize('layer,index',[('positions_rad',18),('glove_joint_angles_rad',8)])
def test_reference_delta_no_longer_stops_normal_or_diagnostic(delta,mode,layer,index):
    gate=full_continuity_gate('litchibot',mode)
    assert isinstance(gate,BasicNormalTarget) and not isinstance(gate,RawContinuityGate) and gate.threshold is None
    a=row(1);gate.observe(a,a['source_received_monotonic_ns'])
    b=row(2);b[layer][index]=delta
    assert gate.observe(b,b['source_received_monotonic_ns']) is True


def test_normal_uses_pattern_and_supervised_manus_keep_strict_policy():
    hardware=full_continuity_gate('litchibot','normal')
    assert isinstance(hardware,BasicNormalTarget) and hardware.threshold is None
    a=row(1);hardware.observe(a,a['source_received_monotonic_ns'])
    b=row(2,.143064)
    assert hardware.observe(b,b['source_received_monotonic_ns'])
    assert isinstance(full_continuity_gate('manus','fake'),NormalContinuityGate)
    for gate in (RawContinuityGate(),full_continuity_gate('litchibot','supervised_hardware_validation')):
        assert gate.threshold==.1
        gate.observe(a,a['source_received_monotonic_ns'])
        with pytest.raises(ValueError,match='Raw discontinuity'):gate.observe(b,b['source_received_monotonic_ns'])


@pytest.mark.parametrize('delta',[.143064,.179000,.185638])
def test_standalone_dry_run_inbox(delta,monkeypatch):
    from adapters.litchibot import transport
    monkeypatch.setattr(transport.time,'monotonic_ns',lambda:1_020_000_000)
    inbox=WorkerInbox(diagnostic=True)
    a=row(1);b=row(2);b['positions_rad'][18]=delta
    inbox.feed(json.dumps(a));inbox.feed(json.dumps(b))
    assert inbox.safety_fault is None
    assert not inbox.holds['left']
    assert inbox.take()['left']['sequence']==2


@pytest.mark.parametrize('patch',[
    {'positions_rad':[float('nan')]*22}, {'positions_rad':[float('inf')]*22},
    {'positions_rad':[3]*22}, {'positions_rad':[0]*21},
    {'joint_names':['bad']*22}, {'source_received_monotonic_ns':1},
])
def test_diagnostic_preserves_basic_checks(patch):
    p=row(1);p.update(patch)
    with pytest.raises(ValueError):MinimumRawGate(diagnostic=True).observe(p,1_010_000_000)
