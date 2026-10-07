import json
import sys
import time
from enum import Enum
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from adapters.litchibot.fault_severity import PerSideMotionLatch, SeverityRawGate, SoftHold
from adapters.litchibot.full_runtime import FullHandRuntime
from adapters.litchibot.normal_continuity import full_continuity_gate
from adapters.litchibot.retarget import JOINT_NAMES
from adapters.litchibot.test_normal_continuity import row
from adapters.litchibot.transport import TargetGate
from adapters.litchibot.validation_safety import RawContinuityGate


@pytest.mark.parametrize('diagnostic',[False,True])
def test_moderate_and_repeated_stay_soft_extreme_and_invalid_stay_hard(diagnostic):
    gate=SeverityRawGate(diagnostic=diagnostic)
    for seq in range(1,9):
        p=row(seq,.185638 if seq%2==0 else 0)
        if seq==1:gate.observe(p,p['source_received_monotonic_ns'])
        else:
            with pytest.raises(SoftHold):gate.observe(p,p['source_received_monotonic_ns'])
    p=row(9);p['positions_rad'][18]=1.5708
    with pytest.raises(ValueError,match='Extreme') as error:gate.observe(p,p['source_received_monotonic_ns'])
    assert not isinstance(error.value,SoftHold)


@pytest.mark.parametrize('diagnostic',[False,True])
def test_partial_then_finite_bad_values_cannot_be_hidden_by_partial(diagnostic):
    gate=SeverityRawGate(diagnostic=diagnostic);p=row(1)
    p.update(valid_glove_joint_count=18,valid_glove_joint_mask=[True]*18+[False]*2,
             valid_target_mask=[True]*20+[False]*2,held_joints=['pinky_DIP'])
    with pytest.raises(SoftHold,match='18/20'):gate.observe(p,p['source_received_monotonic_ns'])
    p=row(2);p.update(valid_glove_joint_count=18,valid_glove_joint_mask=[True]*18+[False]*2)
    p['positions_rad'][18]=float('nan')
    with pytest.raises(ValueError) as error:gate.observe(p,p['source_received_monotonic_ns'])
    assert not isinstance(error.value,SoftHold)
    with pytest.raises(ValueError,match='All source'):RawContinuityGate().observe({**row(1),'valid_glove_joint_mask':[True]*18+[False]*2},1_010_000_000)


@pytest.fixture
def runtime(monkeypatch,tmp_path):
    class Side(Enum):LEFT='left';RIGHT='right'
    clock=[1.0]
    monkeypatch.setattr(time,'monotonic',lambda:clock[0]);monkeypatch.setattr(time,'monotonic_ns',lambda:int(clock[0]*1e9))
    class JointState:
        def __init__(self):self.header=SimpleNamespace(stamp=None,frame_id='')
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',SimpleNamespace(JointState=JointState))
    hands={s:SimpleNamespace(stop=Mock(),read_joint_state=lambda:SimpleNamespace(position=[0.0]*22)) for s in Side}
    latch=PerSideMotionLatch({'session_token':'a'*64,'expires_unix_s':0},1,gui_controlled=True,validation_only=False)
    logs=[];logger=SimpleNamespace(warning=lambda x:logs.append(json.loads(x)),error=lambda x:logs.append(json.loads(x)))
    node=SimpleNamespace(latch=latch,_hands=hands,_has_received_command={s:False for s in Side},
                         _timeout_stopped={s:False for s in Side},needs_enable=set(),forward_full=Mock(),
                         get_logger=lambda:logger,
                         get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9),to_msg=lambda:None)))
    node.stop_validation=latch.trip
    r=FullHandRuntime.__new__(FullHandRuntime)
    r.node=node;r.latch=latch;r.raw=SeverityRawGate();r.config={'source':'litchibot','mode':'normal','token':'a'*64}
    r.gate=TargetGate(allow_dry_run=True,max_age_s=.2,max_speed_rad_s=.5)
    r.publishers={s:SimpleNamespace(publish=Mock()) for s in ('left','right')};r.delta_path=tmp_path/'stats.json'
    sequences={'left':0,'right':0}
    def feed(selected_side,value=0,**patch):
        sequences[selected_side]+=1;p=row(sequences[selected_side],value,selected_side);p.update(patch)
        p['source_received_monotonic_ns']=int(clock[0]*1e9)
        r.litchibot(selected_side,SimpleNamespace(data=json.dumps(p)))
    def pair(value=0,**patch):
        clock[0]+=.01;feed('left');feed('right',value,**patch)
    for _ in range(30):pair()
    def heartbeat(sequence):
        latch.gui_heartbeat(clock[0]);latch.heartbeat({'token':'a'*64,'hold':True,'sequence':sequence,'monotonic_ns':int(clock[0]*1e9)},clock[0])
    latch.gui_heartbeat(clock[0]);r.request('left',True);r.request('right',True);heartbeat(1)
    return SimpleNamespace(r=r,l=latch,node=node,hands=hands,clock=clock,pair=pair,feed=feed,hb=heartbeat,logs=logs)


@pytest.mark.parametrize('mode',['normal','fake'])
@pytest.mark.parametrize('bad',['moderate','partial','slew','acquisition'])
def test_legacy_severity_hold_contract(runtime,mode,bad):
    t=runtime;t.r.config['mode']=mode;t.r.raw=SeverityRawGate(diagnostic=mode=='fake');t.l.diagnostic=mode=='fake'
    t.pair();t.hb(2)
    stops={s:h.stop.call_count for s,h in t.hands.items()}
    if bad=='moderate':t.pair(.185638)
    elif bad=='partial':t.pair(valid_glove_joint_count=18,valid_glove_joint_mask=[True]*18+[False]*2)
    else:
        # Defensive latch rejects an output exceeding slew/acquisition; sender never sees it.
        original=t.r.gate.accept
        t.r.gate.accept=lambda p,**kw:np.array([.2 if i==10 else 0 for i in range(22)]) if p['side']=='right' else original(p,**kw)
        if bad=='acquisition':t.l.phase='READY'  # operator Start before arm heartbeat
        t.pair();t.r.gate.accept=original
        if bad=='acquisition':t.hb(3)
    assert t.l.phase!='FAULT' and t.l.side_state('right',t.clock[0])=='SOFT_HOLD'
    assert t.l.requested['left'] and not t.l.requested['right']
    assert next(h for s,h in t.hands.items() if s.value=='left').stop.call_count==next(n for s,n in stops.items() if s.value=='left')
    assert next(h for s,h in t.hands.items() if s.value=='right').stop.call_count==next(n for s,n in stops.items() if s.value=='right')+1
    with pytest.raises(ValueError):t.r.request('right',True)
    before=t.node.forward_full.call_count
    value=.185638 if bad=='moderate' else 0
    for k in range(29):t.hb(4+k);t.pair(value)
    assert t.l.side_state('right',t.clock[0])=='SOFT_HOLD'
    t.hb(40);t.pair(value)
    assert t.l.side_state('right',t.clock[0])=='READY' and t.l.side_state('left',t.clock[0])=='ACTIVE'
    assert t.node.forward_full.call_count==before+30  # healthy hand only
    t.r.request('right',True);t.hb(41);t.pair(value)
    assert t.l.side_state('right',t.clock[0])=='ACTIVE'
    for e in t.logs:
        assert {'severity','side','joint','source_value','target_value','measured_value','delta','dt','recovery_state'}<=e.keys()
    assert any(e['severity']=='SOFT_HOLD' and e['side']=='right' for e in t.logs)
    assert any(e['recovery_state']=='READY' for e in t.logs)


def test_continuous_partial_is_not_a_disconnect_but_missing_input_is(runtime):
    t=runtime
    for k in range(100):
        t.hb(k+2);t.pair(valid_glove_joint_count=18,valid_glove_joint_mask=[True]*18+[False]*2)
        t.l.watch(t.clock[0],time.time())
        assert t.l.phase!='FAULT' and t.l.side_state('left',t.clock[0])=='ACTIVE'
    t.l.watch(t.clock[0]+.201,time.time());assert t.l.phase=='FAULT'


@pytest.mark.parametrize('patch',[
    {'positions_rad':[float('nan')]*22},{'positions_rad':[float('inf')]*22},
    {'positions_rad':[3.0]*22},{'positions_rad':[0.0]*21},
    {'joint_names':['bad']*22},{'side':'left'},
])
def test_runtime_hard_faults_latch_both_sides(runtime,patch):
    t=runtime;t.pair(**patch)
    assert t.l.phase=='FAULT' and not any(t.l.requested.values())
    assert t.logs[-1]['severity']=='HARD_FAULT'
    if 'positions_rad' in patch and len(patch['positions_rad'])==22:
        assert t.logs[-1]['target_value'] is not None


def test_standalone_partial_holds_one_side_while_other_side_remains_available(monkeypatch):
    from adapters.litchibot.terminal2_bridge import WorkerInbox
    inbox=WorkerInbox(diagnostic=True)
    clock=[1_010_000_000];monkeypatch.setattr(time,'monotonic_ns',lambda:clock[0])
    for seq in range(1,61):
        clock[0]=1_000_000_000+seq*10_000_000
        for side in ('left','right'):
            p=row(seq,side=side)
            if side=='right' and seq<=30:
                p.update(valid_glove_joint_count=18,valid_glove_joint_mask=[True]*18+[False]*2)
            inbox.feed(json.dumps(p))
        rows=inbox.take()
        assert 'left' in rows and inbox.safety_fault is None
        assert ('right' in rows) is (seq==60)
    assert not inbox.holds['right']
    events=inbox.take_events()
    assert any(e['side']=='right' and e['severity']=='SOFT_HOLD' for e in events)
    assert events[-1]['recovery_state']=='READY'


def test_hard_invalid_frame_during_hold_never_recovers(runtime):
    t=runtime;t.pair(.185638);assert t.l.holds['right']
    t.pair(positions_rad=[float('nan')]*22)
    assert t.l.phase=='FAULT' and t.l.side_state('left',t.clock[0])=='HARD_FAULT'
    assert t.logs[-1]['target_value']=='nan'
    for _ in range(35):t.pair()
    assert t.l.phase=='FAULT'


def test_logging_failure_cannot_prevent_authoritative_hard_stop(runtime):
    t=runtime
    def broken_logger(*args):raise RuntimeError('logger unavailable')
    t.node.get_logger=lambda:SimpleNamespace(error=broken_logger,warning=broken_logger)
    with pytest.raises(RuntimeError,match='logger unavailable'):t.pair(positions_rad=[float('nan')]*22)
    assert t.l.phase=='FAULT' and not any(t.l.requested.values())
