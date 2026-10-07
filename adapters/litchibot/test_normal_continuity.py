import json
import sys
import time
from enum import Enum
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from adapters.litchibot.full_runtime import FullHandRuntime
from adapters.litchibot.normal_continuity import NormalContinuityGate
from adapters.litchibot.retarget import JOINT_NAMES
from adapters.litchibot.test_validation_safety import solved_packet
from adapters.litchibot.transport import TargetGate
from adapters.litchibot.validation_safety import MotionLatch, RawContinuityGate


def row(sequence, value=0, side='left'):
    p=solved_packet(side,sequence)
    p['source_received_monotonic_ns']=1_000_000_000+sequence*10_000_000
    p['positions_rad'][10]=value
    return p


def test_isolated_step_and_return_are_rejected_then_stable():
    gate=NormalContinuityGate()
    for seq,value,accepted in [(1,0,True),(2,.185638,False),(3,0,False),(4,0,True)]:
        p=row(seq,value);assert gate.observe(p,p['source_received_monotonic_ns']) is accepted
    stats=gate.distribution.snapshot()['layers'][0]
    assert stats['samples']==3
    assert stats['per_joint'][10]['max_rad']==pytest.approx(.185638)
    assert sum(stats['per_joint'][10]['histogram'])==3
    # Rejected frame identities still cannot be replayed.
    with pytest.raises(ValueError,match='order'):gate.observe(p,p['source_received_monotonic_ns'])


def test_repeated_extreme_source_and_strict_validation():
    gate=NormalContinuityGate()
    for seq,value in [(1,0),(2,.185),(3,0)]:
        p=row(seq,value);gate.observe(p,p['source_received_monotonic_ns'])
    p=row(4,.185)
    with pytest.raises(ValueError,match='Repeated'):gate.observe(p,p['source_received_monotonic_ns'])
    for layer,index in [('positions_rad',18),('glove_joint_angles_rad',0)]:
        gate=NormalContinuityGate();p=row(1);gate.observe(p,p['source_received_monotonic_ns'])
        p=row(2);p[layer][index]=1.5708
        with pytest.raises(ValueError,match='Extreme'):gate.observe(p,p['source_received_monotonic_ns'])
    gate=RawContinuityGate();p=row(1);gate.observe(p,p['source_received_monotonic_ns'])
    p=row(2,.185)
    with pytest.raises(ValueError,match='discontinuity'):gate.observe(p,p['source_received_monotonic_ns'])


@pytest.mark.parametrize('patch',[
    {'positions_rad':[float('nan')]*22}, {'positions_rad':[float('inf')]*22},
    {'positions_rad':[3]*22}, {'positions_rad':[0]*21}, {'joint_names':list(reversed(JOINT_NAMES))},
    {'side':'bad'}, {'glove_joint_angles_rad':[float('nan')]*20},
    {'source_received_monotonic_ns':1}, {'valid_glove_joint_mask':[False]*20},
])
def test_normal_keeps_basic_rejections(patch):
    p=row(1);p.update(patch)
    with pytest.raises(ValueError):NormalContinuityGate().observe(p,1_010_000_000)


def test_runtime_pauses_recovers_requires_new_start_and_keeps_watchdog(monkeypatch,tmp_path):
    class Side(Enum):
        LEFT='left';RIGHT='right'
    clock=[1.0]
    monkeypatch.setattr(time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(time,'monotonic_ns',lambda:int(clock[0]*1e9))
    class JointState:
        def __init__(self):self.header=SimpleNamespace(stamp=None,frame_id='')
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',SimpleNamespace(JointState=JointState))
    hands={s:SimpleNamespace(stop=Mock(),read_joint_state=lambda:SimpleNamespace(position=[0]*22)) for s in Side}
    latch=MotionLatch({'session_token':'a'*64,'expires_unix_s':float('inf')},1,
                      gui_controlled=True,validation_only=False)
    node=SimpleNamespace(latch=latch,_hands=hands,_has_received_command={s:False for s in Side},
                         _timeout_stopped={s:False for s in Side},needs_enable=set(),
                         forward_full=Mock(),get_logger=lambda:SimpleNamespace(warning=Mock()),
                         get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9),to_msg=lambda:None)))
    node.stop_validation=latch.trip
    runtime=FullHandRuntime.__new__(FullHandRuntime)
    runtime.node=node;runtime.latch=latch;runtime.raw=NormalContinuityGate()
    runtime.config={'source':'litchibot','mode':'normal','token':'a'*64}
    runtime.gate=TargetGate(allow_dry_run=True,max_age_s=.2,max_speed_rad_s=.5)
    runtime.publishers={s:SimpleNamespace(publish=Mock()) for s in ('left','right')}
    runtime.delta_path=tmp_path/'stats.json'
    sequences={'left':0,'right':0}
    def feed(side,value=0):
        sequences[side]+=1;p=row(sequences[side],value,side)
        p['source_received_monotonic_ns']=int(clock[0]*1e9)
        runtime.litchibot(side,SimpleNamespace(data=json.dumps(p)))
    def pair(value=0):
        clock[0]+=.01;feed('left',value);feed('right')
    for _ in range(30):pair()
    assert latch.phase=='READY'
    latch.gui_heartbeat(clock[0]);runtime.request('left',True)
    # Source callbacks can arrive between Start and the next GUI arm heartbeat.
    pair(.08);pair(.08)
    assert latch.phase=='READY' and node.forward_full.call_count==0
    latch.heartbeat({'token':'a'*64,'hold':True,'sequence':1,'monotonic_ns':int(clock[0]*1e9)},clock[0])
    pair(.08);assert node.forward_full.call_count==1
    pair(.185638)
    assert latch.phase=='RECOVERING' and not any(latch.requested.values())
    assert node.forward_full.call_count==1 and all(h.stop.called for h in hands.values())
    with pytest.raises(ValueError):runtime.request('left',True)
    # A persistent changed pose is an observation anchor, never forwarded automatically.
    for _ in range(29):pair(.185638)
    assert latch.phase=='RECOVERING'
    pair(.185638);assert latch.phase=='READY'
    pair(.185638);assert node.forward_full.call_count==1
    latch.gui_heartbeat(clock[0]);runtime.request('left',True)
    latch.heartbeat({'token':'a'*64,'hold':True,'sequence':2,'monotonic_ns':int(clock[0]*1e9)},clock[0])
    pair(.185638);assert node.forward_full.call_count==2
    # Paused acquisition still faults on the original .20s timeout.
    pair();assert latch.phase=='RECOVERING'
    latch.watch(clock[0]+.201,time.time());assert latch.phase=='FAULT'
    runtime.save_delta_stats();assert json.loads(runtime.delta_path.read_text())['layers']


def test_manus_uses_same_soft_policy_and_source_gap_is_hard():
    gate=NormalContinuityGate()
    for seq,value,expected in [(1,0,True),(2,.185,False),(3,.185,True)]:
        p=row(seq,value);stamp=p['source_received_monotonic_ns']
        assert gate.observe_ros_target('left',list(JOINT_NAMES),p['positions_rad'],stamp,stamp,seq,'s',stamp) is expected
    stamp+=201_000_000
    with pytest.raises(ValueError,match='timeout'):
        gate.observe_ros_target('left',list(JOINT_NAMES),p['positions_rad'],stamp,stamp,4,'s',stamp)


def test_source_only_moderate_and_isolated_events_outside_repeat_window():
    gate=NormalContinuityGate();p=row(1);gate.observe(p,p['source_received_monotonic_ns'])
    p=row(2);p['glove_joint_angles_rad'][8]=1.2
    assert gate.observe(p,p['source_received_monotonic_ns']) is False
    p=row(3);p['glove_joint_angles_rad'][8]=1.2
    assert gate.observe(p,p['source_received_monotonic_ns']) is True
    assert gate.distribution.snapshot()['layers'][1]['per_joint'][8]['max_rad']==1.2
    gate=NormalContinuityGate();value=0
    for seq in range(1,335):
        if seq in (2,113,224):value=.185 if value==0 else 0
        p=row(seq,value)
        assert gate.observe(p,p['source_received_monotonic_ns']) is (seq not in (2,113,224))


def test_recovery_before_both_sides_acquired_still_times_out():
    now=time.monotonic()
    latch=MotionLatch({'session_token':'a'*64,'expires_unix_s':float('inf')},now,
                      gui_controlled=True,validation_only=False)
    latch.pause('Early moderate anomaly')
    latch.watch(latch.recovery_started+.01,time.time());assert latch.phase=='RECOVERING'
    latch.watch(latch.recovery_started+.201,time.time());assert latch.phase=='FAULT'
