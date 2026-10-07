import json
import time

import pytest

from adapters.litchibot.fault_severity import SeverityRawGate, SoftHold, SOURCE_LOSS_HARD_S
from adapters.litchibot.test_normal_continuity import row
from adapters.litchibot.test_fault_severity import runtime
from adapters.litchibot.validation_safety import RawContinuityGate
from adapters.litchibot.minimum_interference import MinimumRawGate


@pytest.mark.parametrize('diagnostic',[False,True])
@pytest.mark.parametrize('gap_s',[.201,14.8,19.9])
@pytest.mark.parametrize('gate_type',[SeverityRawGate,MinimumRawGate])
def test_fresh_post_gap_frame_reanchors_and_next_dt_is_adjacent(diagnostic,gap_s,gate_type):
    gate=gate_type(diagnostic=diagnostic)
    first=row(1);stamp=first['source_received_monotonic_ns']
    assert gate.observe(first,stamp)
    second=row(2);second['source_received_monotonic_ns']=stamp+int(gap_s*1e9)
    # Valid new pose differs grossly from the *old* pose, which is not adjacent.
    second['positions_rad'][18]=1.5708
    with pytest.raises(SoftHold,match='baseline rebuilt') as error:
        gate.observe(second,second['source_received_monotonic_ns'])
    assert error.value.context['delta'] is None
    assert gate.previous['left'][4]==second['source_received_monotonic_ns']
    assert gate.arrival_ns['left']==second['source_received_monotonic_ns']
    assert gate.distribution.snapshot()['layers']==[]  # excludes cross-gap delta
    for seq in range(3,160):
        p=row(seq);p['positions_rad'][18]=1.5708
        p['source_received_monotonic_ns']=second['source_received_monotonic_ns']+(seq-2)*33_333_333
        assert gate.observe(p,p['source_received_monotonic_ns'])
        assert gate.context['left']['dt']==pytest.approx(.033333333)
        assert gate.context['left']['delta']==0
    assert gate.threshold==(None if gate_type is MinimumRawGate else .18 if diagnostic else .1)


@pytest.mark.parametrize('patch',[
    {'positions_rad':[float('nan')]*22},{'positions_rad':[3.0]*22},
    {'positions_rad':[0.0]*21},{'session_id':'changed'}, {'sequence':1},
])
def test_invalid_or_replayed_post_gap_frame_cannot_reanchor(patch):
    gate=SeverityRawGate(diagnostic=True);first=row(1);gate.observe(first,first['source_received_monotonic_ns'])
    second=row(2);second['source_received_monotonic_ns']+=300_000_000;second.update(patch)
    with pytest.raises(ValueError) as error:gate.observe(second,1_320_000_000)
    assert not isinstance(error.value,SoftHold)
    assert gate.previous['left'][3]==1
    assert gate.arrival_ns['left']==first['source_received_monotonic_ns']


def test_supervised_gap_behavior_is_unchanged():
    gate=RawContinuityGate();first=row(1);gate.observe(first,first['source_received_monotonic_ns'])
    second=row(2);second['source_received_monotonic_ns']+=300_000_000
    with pytest.raises(ValueError,match='Source gap watchdog expired'):gate.observe(second,second['source_received_monotonic_ns'])
    assert gate.previous['left'][3]==1 and gate.threshold==.1


@pytest.mark.parametrize('mode',['normal','fake'])
def test_timer_short_gap_stops_one_side_then_rebuilds_and_recovers(runtime,mode):
    t=runtime;t.r.config['mode']=mode;t.r.raw=SeverityRawGate(diagnostic=mode=='fake');t.l.diagnostic=mode=='fake'
    t.pair()
    left_hand=next(h for s,h in t.hands.items() if s.value=='left')
    right_hand=next(h for s,h in t.hands.items() if s.value=='right')
    counts=(left_hand.stop.call_count,right_hand.stop.call_count)
    # Healthy opposite side and GUI keep arriving while right packets briefly stop.
    for seq in range(2,9):
        t.clock[0]+=.033333333;t.hb(seq);t.feed('left')
        t.l.watch(t.clock[0],time.time());t.r.apply_source_holds()
    assert t.l.phase=='ARMED' and t.l.side_state('right',t.clock[0])=='SOFT_HOLD'
    assert left_hand.stop.call_count==counts[0] and right_hand.stop.call_count==counts[1]+1
    sent=t.node.forward_full.call_count
    t.clock[0]+=.033333333;t.hb(9);t.feed('left');t.feed('right',.185638)
    assert t.node.forward_full.call_count==sent+1  # post-gap baseline not forwarded
    baseline=t.r.raw.previous['right'][4]
    assert t.r.raw.context['right']['delta'] is None
    for seq in range(10,40):
        t.clock[0]+=.033333333;t.hb(seq);t.feed('left');t.feed('right',.185638)
        t.l.watch(t.clock[0],time.time());t.r.apply_source_holds()
        assert t.r.raw.context['right']['dt']==pytest.approx(.033333333,abs=1e-9)
    assert t.r.raw.previous['right'][4]>baseline
    assert t.l.side_state('right',t.clock[0])=='READY'
    assert t.l.side_state('left',t.clock[0])=='ACTIVE' and t.l.phase!='FAULT'
    assert not t.l.requested['right']  # operator permit never recreated
    assert right_hand.stop.call_count==counts[1]+1


def test_sustained_packet_loss_still_hard_faults(runtime):
    t=runtime
    # Avoid confusing source loss with the independent GUI lease watchdog.
    t.r.request('left',False);t.r.request('right',False)
    t.l.watch(t.clock[0]+.201,time.time());t.r.apply_source_holds()
    assert t.l.phase!='FAULT' and all(t.l.holds.values())
    t.l.watch(t.clock[0]+SOURCE_LOSS_HARD_S+.001,time.time())
    assert t.l.phase=='FAULT' and 'Sustained source packet loss' in t.l.reason


def test_latched_hard_fault_has_one_log_and_preserves_first_reason(runtime):
    t=runtime;t.pair(positions_rad=[float('nan')]*22)
    reason=t.l.reason
    first=[e for e in t.logs if e['severity']=='HARD_FAULT']
    assert len(first)==1
    # Multiple bad frames, followed by a different fault and a driver retry.
    for _ in range(180):t.pair(positions_rad=[float('nan')]*22)
    t.pair(joint_names=['bad']*22)
    t.r.emit_event('HARD_FAULT','both','Later driver fault',recovery_state='HARD_FAULT')
    assert t.l.reason==reason and t.l.phase=='FAULT'
    assert [e for e in t.logs if e['severity']=='HARD_FAULT']==first
    assert t.r.last_safety_event==first[0]


def test_diagnostic_inbox_does_not_repeat_latched_errors():
    from adapters.litchibot.terminal2_bridge import WorkerInbox
    inbox=WorkerInbox(diagnostic=True)
    inbox.feed('{')
    first=inbox.safety_fault
    for _ in range(180):inbox.feed('{')
    assert inbox.safety_fault==first and inbox.errors==1
    events=inbox.take_events()
    assert len(events)==1 and events[0]['severity']=='HARD_FAULT'
