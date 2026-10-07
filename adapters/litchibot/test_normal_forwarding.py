"""Independent normal path: real runtime methods, synthetic targets, no SDK/ROS."""
import json
import sys
import time
from enum import Enum
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from adapters.litchibot.normal_control import NormalHandAuthority,BasicNormalTarget
from adapters.litchibot.normal_runtime import NormalHandRuntime
from adapters.litchibot.retarget import JOINT_NAMES
from adapters.litchibot.test_normal_continuity import row
from adapters.litchibot.validation_safety import MotionLatch


@pytest.fixture
def normal_runtime(monkeypatch,tmp_path):
    clock=[1.0]
    monkeypatch.setattr(time,'monotonic',lambda:clock[0]);monkeypatch.setattr(time,'monotonic_ns',lambda:int(clock[0]*1e9))
    class Side(Enum):LEFT='left';RIGHT='right'
    class JointState:
        def __init__(self):self.header=SimpleNamespace(stamp=None,frame_id='')
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',SimpleNamespace(JointState=JointState))
    hands={s:SimpleNamespace(stop=Mock(),position=[0.0]*22,is_connected=True) for s in Side}
    for hand in hands.values():hand.read_joint_state=lambda hand=hand:SimpleNamespace(position=hand.position)
    latch=NormalHandAuthority({'session_token':'a'*64},clock[0]);logs=[];sent=[]
    node=SimpleNamespace(_hands=hands,_has_received_command={s:False for s in Side},_timeout_stopped={s:False for s in Side},
                         needs_enable=set(),validation_send_count=0,get_logger=lambda:SimpleNamespace(warning=lambda v:logs.append(json.loads(v)),error=lambda v:logs.append(json.loads(v))),
                         get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9),to_msg=lambda:None)))
    r=NormalHandRuntime.__new__(NormalHandRuntime);r.node=node;r.latch=latch;r._native_paused=set()
    r.config={'source':'litchibot','mode':'fake','token':'a'*64};r.feedback={s:(np.zeros(22),clock[0]) for s in ('left','right')};r.raw=BasicNormalTarget();r.gate=r.raw;r.delta_path=tmp_path/'stats.json'
    r.publishers={s:SimpleNamespace(publish=Mock()) for s in ('left','right')}
    def hard(reason):
        latch.trip(reason)
        for hand in hands.values():hand.stop()
        r.emit_event('HARD_FAULT','both',reason)
    node.stop_validation=hard
    def send(side,msg):
        sent.append((side.value,msg.position.copy()));hands[side].position=msg.position.copy()
        node._has_received_command[side]=True;node.validation_send_count+=1;node.needs_enable.discard(side.value)
        r.observe_feedback(side.value,SimpleNamespace(name=list(JOINT_NAMES),position=msg.position))
    node.forward_full=send;sequences={s:0 for s in ('left','right')};control=[0]
    def feed(side='left',value=0,**patch):
        clock[0]+=.01;sequences[side]+=1;p=row(sequences[side],value,side)
        p['source_received_monotonic_ns']=int(clock[0]*1e9);p.update(patch)
        r.litchibot(side,SimpleNamespace(data=json.dumps(p)));return p
    def heartbeat(age=0,generation=1,sequence=None):
        control[0]+=1
        return r.dispatch({'command':'heartbeat','sequence':control[0] if sequence is None else sequence,
                           'monotonic_ns':int((clock[0]-age)*1e9),'_connection_generation':generation})
    return SimpleNamespace(r=r,l=latch,node=node,hands=hands,clock=clock,logs=logs,sent=sent,feed=feed,hb=heartbeat)


def start(t,side='left'):
    t.hb();t.r.dispatch({'command':'engage_hand','arguments':{'side':side}})


def test_gui_ready_start_stop_is_only_independent_forwarding(normal_runtime):
    t=normal_runtime
    assert not isinstance(t.l,MotionLatch) and t.r.gate is t.r.raw
    assert [t.r.status()['hands'][s]['state'] for s in ('left','right')]==['READY','READY']
    start(t);t.feed(value=.2);assert t.sent[-1][0]=='left' and t.l.side_state('left',1)=='ACTIVE'
    start(t,'right');t.feed('right',.2);assert all(t.l.requested.values())
    before=len(t.sent);t.r.request('left',False);t.feed(value=.3);t.feed('right',.1)
    assert t.l.side_state('left',1)=='READY' and len(t.sent)==before+1
    t.r.request('right',False);assert t.l.side_state('right',1)=='READY'
    assert all(not h.stop.called and h.is_connected for h in t.hands.values())
    assert t.r.publishers['left'].publish.call_count==2  # retarget/recorder continue while OFF


@pytest.mark.parametrize('count',[18,19])
def test_repeated_partial_existing_fallback_never_blocks_or_escalates(normal_runtime,count):
    t=normal_runtime;start(t)
    q=[.1]*22
    for _ in range(40):
        t.hb();t.feed(positions_rad=q,valid_glove_joint_count=count,valid_target_mask=[True]*18+[False]*4,
                      held_joints=['middle_MCP_AA'],initial_fallback_joints=['pinky_CMC'])
    assert len(t.sent)==40 and t.l.side_state('left',t.clock[0])=='ACTIVE'
    np.testing.assert_array_equal(t.sent[-1][1],q)
    assert not any(e['severity']=='HARD_FAULT' for e in t.logs)


@pytest.mark.parametrize('offset',[-.3,.3])
def test_isolated_stale_future_drop_then_next_frame_resumes(normal_runtime,offset):
    t=normal_runtime;start(t);t.feed(value=.2);before=len(t.sent)
    t.feed(source_received_monotonic_ns=int((t.clock[0]+offset)*1e9))
    assert len(t.sent)==before and t.l.phase!='FAULT'
    t.hb();t.feed(value=.3);assert len(t.sent)==before+1


@pytest.mark.parametrize('issue',['stale','replay','delayed'])
def test_heartbeat_diagnostics_never_change_forwarding_or_stop(normal_runtime,issue):
    t=normal_runtime;start(t);start(t,'right');t.feed(value=.2);t.feed('right',.2)
    if issue=='stale':t.hb(age=.3)
    elif issue=='replay':t.hb(sequence=1)
    else:t.l.control_seen=t.clock[0]-19;t.l.watch(t.clock[0])
    assert all(t.l.requested.values())
    assert all(t.l.side_state(s,t.clock[0])=='ACTIVE' for s in t.l.requested)
    before=len(t.sent);t.feed(value=.3);assert len(t.sent)==before+1
    assert all(not h.stop.called for h in t.hands.values())


def test_actual_control_disconnect_recovers_ready_without_native_stop(normal_runtime):
    t=normal_runtime;start(t);start(t,'right');t.feed();t.feed('right');t.r.disconnected()
    assert not any(t.l.requested.values()) and t.l.phase!='FAULT'
    assert all(t.l.side_state(s,t.clock[0])=='OFFLINE' for s in t.l.requested)
    before=len(t.sent);t.feed(value=.2);assert len(t.sent)==before
    t.hb(generation=2,sequence=1)
    assert all(t.l.side_state(s,t.clock[0])=='READY' for s in t.l.requested)
    assert all(not h.stop.called for h in t.hands.values())
    start(t);t.feed(value=.2);assert len(t.sent)==before+1


def test_stop_while_control_paused_is_not_auto_reauthorized(normal_runtime):
    t=normal_runtime;start(t);t.r.disconnected();t.r.request('left',False);before=len(t.sent)
    t.hb(generation=2,sequence=1);t.feed(value=.2)
    assert t.l.side_state('left',t.clock[0])=='READY' and len(t.sent)==before


def test_fast_and_oscillating_motion_pass_exact_no_custom_gate(normal_runtime):
    t=normal_runtime;start(t)
    for value in (.19,.25,.4,0,.4,0,.4,0):t.hb();t.feed(value=value)
    assert [q[10] for _,q in t.sent]==[.19,.25,0,0,0]  # .4 violates original physical limit, so only those frames dropped
    assert t.l.phase!='FAULT' and t.l.requested['left']


def test_wide_joint_fast_oscillation_passes_exact(normal_runtime):
    t=normal_runtime;start(t)
    values=[.19,.25,.4,0,.4,0,.4,0]
    for v in values:
        q=[0.0]*22;q[18]=v;t.hb();t.feed(positions_rad=q)
    assert [q[18] for _,q in t.sent]==values and t.l.phase!='FAULT'


@pytest.mark.parametrize('patch',[{'positions_rad':[float('nan')]*22},{'positions_rad':[float('inf')]*22},
 {'positions_rad':[3]*22},{'positions_rad':[0]*21},{'joint_names':['bad']*22},{'joint_names':None},{'side':'right'}])
def test_bad_final_packet_drops_only_current_and_next_valid_continues(normal_runtime,patch):
    t=normal_runtime;start(t);t.feed(**patch)
    assert t.l.phase!='FAULT' and not t.sent and t.l.requested['left']
    t.feed(value=.2);assert len(t.sent)==1
    assert all(not h.stop.called for h in t.hands.values())


def test_short_source_gap_fresh_baseline_recovers_no_restart(normal_runtime):
    t=normal_runtime;start(t);t.feed(value=.2)
    t.clock[0]+=.25;t.hb();t.l.watch(t.clock[0]);t.r.reconcile_pauses()
    assert t.l.side_state('left',t.clock[0])=='OFFLINE' and t.l.phase!='FAULT'
    t.feed(value=.3)
    assert t.l.side_state('left',t.clock[0])=='READY' and t.r.raw.context['left']['delta'] is None
    assert all(not h.stop.called for h in t.hands.values())


def test_sustained_complete_packet_loss_remains_bottom_line(normal_runtime):
    t=normal_runtime;start(t);t.feed();t.clock[0]+=1.001;t.hb();t.l.watch(t.clock[0])
    assert t.l.phase!='FAULT' and t.l.side_state('left',t.clock[0])=='OFFLINE'
    t.feed(value=.2);assert t.l.side_state('left',t.clock[0])=='READY'
    assert all(not h.stop.called for h in t.hands.values())


def test_missing_source_channels_use_finite_vendor_fallback(normal_runtime):
    t=normal_runtime;start(t)
    q=[.1]*22
    t.feed(positions_rad=q,valid_glove_joint_count=18,glove_joint_angles_rad=[None]*20,held_joints=['thumb_CMC_AA'])
    assert t.l.phase!='FAULT' and t.sent[-1][1]==q


def test_data_issue_on_one_side_keeps_opposite_active(normal_runtime):
    t=normal_runtime;start(t);start(t,'right');t.feed();t.feed('right')
    t.l.pause_data('right','Temporary graph unavailable');t.r.reconcile_pauses()
    before=len(t.sent);t.feed(value=.2)
    assert len(t.sent)==before+1 and t.l.side_state('right',t.clock[0])=='OFFLINE'
    t.feed('right',.2);assert t.l.side_state('right',t.clock[0])=='READY'


def test_bad_metadata_and_source_warning_are_recoverable(normal_runtime):
    t=normal_runtime;start(t);t.feed();before=len(t.sent)
    t.feed(sequence='invalid');assert t.l.phase!='FAULT' and len(t.sent)==before
    t.r.source_fault(SimpleNamespace(data='Temporary source reconnect'))
    assert t.l.side_state('left',t.clock[0])=='ACTIVE'
    t.feed(value=.2);assert t.l.side_state('left',t.clock[0])=='ACTIVE'


def test_warning_telemetry_error_does_not_escalate_or_block_vendor_fallback(normal_runtime):
    t=normal_runtime;start(t)
    t.node.get_logger=lambda:SimpleNamespace(warning=Mock(side_effect=RuntimeError('logger unavailable')),error=Mock())
    t.feed(value=.2,valid_glove_joint_count=18,held_joints=['middle_MCP_AA'])
    assert t.l.phase!='FAULT' and len(t.sent)==1 and t.r.telemetry_errors==1


def test_rejected_bridge_packets_are_not_false_complete_source_loss(normal_runtime):
    t=normal_runtime;start(t);t.feed()
    for _ in range(12):
        t.clock[0]+=.1;t.hb()
        t.r.source_fault(SimpleNamespace(data=json.dumps({'severity':'WARNING','reason':'Stale/future observation',
                             'side':'left','source_arrived':True})))
        t.l.watch(t.clock[0])
    assert t.l.phase!='FAULT' and t.l.side_state('left',t.clock[0])=='OFFLINE'
    t.feed(value=.2);assert t.l.side_state('left',t.clock[0])=='READY'


def test_canonical_diagnostic_overflow_never_blocks_finite_final_target(normal_runtime):
    t=normal_runtime;start(t)
    t.feed(glove_joint_angles_rad=[1e308]*20)
    t.feed(value=.2,glove_joint_angles_rad=[-1e308]*20)
    assert t.l.phase!='FAULT' and len(t.sent)==2


def test_continuous_source_without_heartbeat_keeps_active(normal_runtime):
    t=normal_runtime;start(t)
    for _ in range(600):
        t.clock[0]+=.023;t.feed(value=.2);t.l.watch(t.clock[0])
    assert t.clock[0]>19 and len(t.sent)==600 and t.l.requested['left']
    assert t.l.side_state('left',t.clock[0])=='ACTIVE'
    assert all(not h.stop.called for h in t.hands.values())


@pytest.mark.parametrize('body',['not JSON','null','[]','{"side":"left","schema":"broken"}'])
def test_malformed_envelope_drops_and_next_packet_continues(normal_runtime,body):
    t=normal_runtime;start(t)
    t.r.litchibot('left',SimpleNamespace(data=body))
    assert t.l.phase!='FAULT' and t.l.requested['left'] and not t.sent
    t.feed(value=.2);assert len(t.sent)==1
    assert all(not h.stop.called for h in t.hands.values())


@pytest.mark.parametrize('measurement',[[float('nan')]*22,[float('inf')]*22,[0]*21,[3]*22])
def test_idle_feedback_is_telemetry_never_latches_or_stops(normal_runtime,measurement):
    t=normal_runtime;left=next(h for s,h in t.hands.items() if s.value=='left')
    left.position=measurement
    from adapters.litchibot.retarget import JOINT_NAMES
    t.r.observe_feedback('left',SimpleNamespace(name=JOINT_NAMES,position=measurement))
    t.r.idle_snapshot('left');t.feed('right',.2)
    assert t.l.phase!='FAULT' and all(not h.stop.called for h in t.hands.values())
    start(t);t.feed(value=.2)
    assert t.sent[-1][1][10]==.2 and t.l.side_state('left',t.clock[0])=='ACTIVE'
    assert not any(e['severity']=='HARD_FAULT' for e in t.logs)


def test_temporary_idle_read_error_does_not_block_valid_target(normal_runtime):
    t=normal_runtime;left=next(h for s,h in t.hands.items() if s.value=='left')
    left.read_joint_state=Mock(side_effect=RuntimeError('Temporary SDK snapshot unavailable'))
    t.r.idle_snapshot('left');assert t.l.phase!='FAULT'
    start(t);t.feed(value=.2)
    assert len(t.sent)==1 and not left.stop.called


def test_measurement_outside_command_range_is_not_clipped_into_command(normal_runtime):
    t=normal_runtime;left=next(h for s,h in t.hands.items() if s.value=='left')
    left.position=[3.0]*22
    from adapters.litchibot.retarget import JOINT_NAMES
    t.r.observe_feedback('left',SimpleNamespace(name=JOINT_NAMES,position=left.position))
    t.r.idle_snapshot('left')
    snapshot=t.r.publishers['left'].publish.call_args.args[0]
    assert snapshot.position==[3.0]*22 and snapshot.header.frame_id.startswith('supervisor_no_send:')
    assert not t.sent and t.l.phase!='FAULT'
    start(t);t.feed(positions_rad=[3]*22)
    assert not t.sent  # Final target physical protection still rejects this packet.
    t.feed(value=.2);assert len(t.sent)==1


def test_normal_telemetry_and_partial_warning_never_read_sdk_again(normal_runtime):
    t=normal_runtime
    for hand in t.hands.values():hand.read_joint_state=Mock(side_effect=AssertionError('Duplicate synchronous SDK read'))
    t.r.idle_snapshot('left')
    start(t)
    for _ in range(40):t.feed(value=.2,valid_glove_joint_count=18,held_joints=['thumb_CMC_FE'])
    assert len(t.sent)==40 and t.l.phase!='FAULT'
    assert all(not h.read_joint_state.called for h in t.hands.values())


def test_async_logging_submission_failure_is_not_a_motion_fault(normal_runtime):
    t=normal_runtime;start(t)
    t.r._normal_event_times={}
    t.r.telemetry=SimpleNamespace(errors=0,pending={},submit=Mock(side_effect=RuntimeError('Diagnostic worker unavailable')))
    t.feed(value=.2,valid_glove_joint_count=18,held_joints=['thumb_CMC_FE'])
    assert len(t.sent)==1 and t.l.phase!='FAULT' and t.r.telemetry_errors==1
