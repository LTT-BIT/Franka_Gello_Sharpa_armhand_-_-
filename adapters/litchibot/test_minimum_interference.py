import time

import numpy as np
import pytest

from adapters.litchibot.fault_severity import SoftHold
from adapters.litchibot.minimum_interference import MinimumRawGate, MinimumMotionLatch, RejectFrame
from adapters.litchibot.test_normal_continuity import row
from adapters.litchibot.test_fault_severity import runtime as legacy_runtime


@pytest.fixture
def runtime(legacy_runtime):
    t=legacy_runtime
    # Keep the verified input identity/acquisition state while selecting the new policy.
    new=MinimumMotionLatch(t.l.permit,t.l.started,gui_controlled=True,validation_only=False)
    for name in ('phase','requested','samples','measured','last','last_gui','last_hold','armed_at','control_sequence','source_seen','baseline'):
        setattr(new,name,getattr(t.l,name))
    t.l=new;t.r.latch=new;t.node.latch=new;t.node.stop_validation=new.trip
    t.r.raw=MinimumRawGate()
    from adapters.litchibot.transport import TargetGate
    t.r.gate=TargetGate(allow_dry_run=True,max_age_s=.2,max_speed_rad_s=None)
    def heartbeat(sequence):
        new.gui_heartbeat(t.clock[0])
        new.heartbeat({'token':'a'*64,'hold':True,'sequence':sequence,'monotonic_ns':int(t.clock[0]*1e9)},t.clock[0])
    t.hb=heartbeat
    return t


@pytest.mark.parametrize('delta',[.19,.25,.4])
def test_single_and_monotonic_fast_motion_forward_exact_target(runtime,delta):
    t=runtime;t.pair()
    for i in range(1,4):
        t.hb(i+1)
        q=[0.0]*22;q[18]=delta*i  # wide-range MCP flexion: no common single-frame gate
        t.pair(positions_rad=q)
        assert t.l.side_state('right',t.clock[0])=='ACTIVE' and t.l.requested['right']
    sent=[c for c in t.node.forward_full.call_args_list if c.args[0].value=='right']
    values=np.array([c.args[1].position for c in sent])
    np.testing.assert_allclose(values[:,18],[0,delta,2*delta,3*delta])
    assert t.r.gate.max_speed is None
    assert t.r.raw.threshold is None
    assert not any('Target bounded' in e['reason'] for e in t.logs)


def trigger_pattern(t):
    t.pair()
    for seq,value in enumerate((.2,0,.2,0),2):t.hb(seq);t.pair(value)
    assert t.l.side_state('right',t.clock[0])=='SOFT_HOLD'


def test_pattern_hold_preserves_key_opposite_hand_and_auto_resumes(runtime):
    t=runtime;trigger_pattern(t)
    assert t.l.requested=={'left':True,'right':True}
    assert t.l.side_state('left',t.clock[0])=='ACTIVE'
    left=next(h for s,h in t.hands.items() if s.value=='left')
    right=next(h for s,h in t.hands.items() if s.value=='right')
    assert not left.stop.called and right.stop.call_count==1
    before=t.node.forward_full.call_count
    for seq in range(6,36):t.hb(seq);t.pair()
    assert t.l.side_state('right',t.clock[0])=='ACTIVE' and t.l.requested['right']
    assert t.node.forward_full.call_count==before+30  # recovery frame itself isn't executed
    t.hb(36);t.pair()
    assert t.node.forward_full.call_count==before+32  # both send, without another Start


def test_stop_while_held_is_final_until_operator_starts_again(runtime):
    t=runtime;trigger_pattern(t);t.r.request('right',False)
    assert not t.l.requested['right']
    before=t.node.forward_full.call_count
    for seq in range(6,41):t.hb(seq);t.pair()
    assert t.l.side_state('right',t.clock[0])=='READY' and not t.l.requested['right']
    assert t.node.forward_full.call_count==before+35  # left only


def test_partial_isolated_reject_then_persistent_hold_then_automatic_resume(runtime):
    t=runtime;t.pair()
    partial={'valid_glove_joint_count':18,'valid_glove_joint_mask':[True]*18+[False]*2}
    before=t.node.forward_full.call_count
    t.hb(2);t.pair(**partial)
    assert t.l.side_state('right',t.clock[0])=='ACTIVE' and t.l.requested['right']
    assert t.node.forward_full.call_count==before+1 and not any(h.stop.called for h in t.hands.values())
    t.hb(3);t.pair();assert t.node.forward_full.call_count==before+3
    for seq in range(4,7):t.hb(seq);t.pair(**partial)
    assert t.l.holds['right'] and t.l.requested['right'] and not t.l.holds['left']
    for seq in range(7,37):t.hb(seq);t.pair()
    assert t.l.side_state('right',t.clock[0])=='ACTIVE'


def test_no_second_output_clamp_or_shared_slew_rejection(runtime):
    t=runtime;t.pair()
    original=t.r.gate.accept
    t.r.gate.accept=lambda p,**kw:np.full(22,.2) if p['side']=='right' else original(p,**kw)
    # .2 satisfies these schema limits; feedback/structure checks remain active.
    for seq in range(2,6):t.hb(seq);t.pair()
    assert t.l.side_state('right',t.clock[0])=='ACTIVE' and not t.l.holds['right']
    sent=[c.args[1].position for c in t.node.forward_full.call_args_list if c.args[0].value=='right']
    np.testing.assert_array_equal(sent[-1],np.full(22,.2))


def test_log_rate_limited_with_periodic_summary(runtime):
    t=runtime;t.pair()
    reason='Single fast/moderate movement; validated target forwarding continues'
    for seq in range(2,122):
        t.hb(seq);t.pair()
        t.r.emit_event('WARNING','right',reason,joint='pinky_MCP_FE',recovery_state='ACTIVE')
    warnings=[e for e in t.logs if e['reason']==reason]
    assert len(warnings)==1 and t.l.side_state('right',t.clock[0])=='ACTIVE'
    t.clock[0]+=1.01
    t.r.emit_event('WARNING','right',reason,joint='pinky_MCP_FE',recovery_state='ACTIVE')
    assert t.logs[-1]['suppressed_events']>0


def test_persistent_actual_output_feedback_divergence_holds_without_clearing_key(runtime):
    t=runtime;t.pair()
    for seq in range(2,270):
        t.hb(seq);q=[0.0]*22;q[10]=.3;t.pair(positions_rad=q)
        if t.l.holds['right']:break
    assert t.l.holds['right'] and t.l.requested['right']
    assert 'Persistent command' in t.l.side_reasons['right']


def test_extreme_corrupt_source_hard_latches_and_clears_keys(runtime):
    t=runtime;t.pair()
    angles=[0.0]*20;angles[8]=10
    t.pair(glove_joint_angles_rad=angles)
    assert t.l.phase=='FAULT' and not any(t.l.requested.values())
    assert len([e for e in t.logs if e['severity']=='HARD_FAULT'])==1


def test_short_source_gap_reanchors_preserves_key_and_auto_resumes(runtime):
    t=runtime;t.pair()
    for seq in range(2,9):
        t.clock[0]+=.033333333;t.hb(seq);t.feed('left')
        t.l.watch(t.clock[0],time.time());t.r.apply_source_holds()
    assert t.l.holds['right'] and t.l.requested['right'] and not t.l.holds['left']
    t.clock[0]+=.033333333;t.hb(9);t.feed('left');t.feed('right',.2)
    assert t.r.raw.context['right']['delta'] is None
    for seq in range(10,40):
        t.clock[0]+=.033333333;t.hb(seq);t.feed('left');t.feed('right',.2)
        assert t.r.raw.context['right']['dt']==pytest.approx(.033333333,abs=1e-9)
    assert t.l.side_state('right',t.clock[0])=='ACTIVE' and t.l.requested['right']


def test_disengage_all_during_hold_never_restores_permits(runtime):
    t=runtime;trigger_pattern(t)
    for side in ('left','right'):t.r.request(side,False)
    before=t.node.forward_full.call_count
    for _ in range(35):t.l.gui_heartbeat(t.clock[0]);t.pair()
    assert not any(t.l.requested.values()) and t.node.forward_full.call_count==before
    assert not t.l.holds['right']


def test_gui_disconnect_during_hold_is_hard_and_revokes_both(runtime):
    t=runtime;trigger_pattern(t);t.r.disconnected()
    assert t.l.phase=='FAULT' and not any(t.l.requested.values())


def test_sustained_no_packets_during_hold_still_hard_faults(runtime):
    from adapters.litchibot.fault_severity import SOURCE_LOSS_HARD_S
    t=runtime;trigger_pattern(t)
    t.clock[0]+=SOURCE_LOSS_HARD_S+.001;t.hb(6)
    t.l.watch(t.clock[0],time.time())
    assert t.l.phase=='FAULT' and not any(t.l.requested.values())


def test_source_angle_wrap_is_not_extreme():
    gate=MinimumRawGate();first=row(1);first['glove_joint_angles_rad'][8]=np.pi-.01
    gate.observe(first,first['source_received_monotonic_ns'])
    second=row(2);second['glove_joint_angles_rad'][8]=-np.pi+.01
    assert gate.observe(second,second['source_received_monotonic_ns'])


@pytest.mark.parametrize('mode',['normal','fake'])
def test_every_joint_full_range_step_is_not_single_frame_hold(mode):
    from adapters.litchibot.normal_continuity import full_continuity_gate
    from adapters.litchibot.retarget import JOINT_LIMITS
    for j in range(22):
        gate=full_continuity_gate('litchibot',mode)
        first=row(1);first['positions_rad']=JOINT_LIMITS.mean(axis=1).tolist()
        first['positions_rad'][j]=float(JOINT_LIMITS[j,0])
        gate.observe(first,first['source_received_monotonic_ns'])
        second=row(2);second['positions_rad']=first['positions_rad'].copy()
        second['positions_rad'][j]=float(JOINT_LIMITS[j,1])
        assert gate.observe(second,second['source_received_monotonic_ns'])


def test_standalone_diagnostic_pattern_per_side_recovery_and_throttle(monkeypatch):
    from adapters.litchibot.terminal2_bridge import WorkerInbox
    inbox=WorkerInbox(diagnostic=True)
    def feed(seq,value=0):
        p=row(seq,value);monkeypatch.setattr(time,'monotonic_ns',lambda:p['source_received_monotonic_ns'])
        inbox.feed(__import__('json').dumps(p))
    feed(1)
    for seq,value in enumerate((.2,0,.2,0),2):feed(seq,value)
    assert inbox.holds['left'] and not inbox.holds['right'] and inbox.safety_fault is None
    for seq in range(6,36):feed(seq)
    assert not inbox.holds['left'] and inbox.take()['left']['sequence']==35
    events=inbox.take_events()
    assert any(e['severity']=='SOFT_HOLD' for e in events)
    assert any('recovery' in e['reason'] for e in events)
