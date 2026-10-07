"""GUI transport startup/reconnect failures must not arm or brick unarmed hands."""
import pytest
from adapters.litchibot.test_minimum_interference import runtime, legacy_runtime


def prepare(t):
    t.r.last_gui_sequence=-1;t.r.control_sequence=t.l.control_sequence
    for side in ('left','right'):t.r.request(side,False)


def heartbeat(t,seq,generation=1,age=0):
    t.node.validation_send_count=0
    return t.r.dispatch({'command':'heartbeat','sequence':seq,
        'monotonic_ns':int((t.clock[0]-age)*1e9),'_connection_generation':generation})


@pytest.mark.parametrize('mode',['normal','fake'])
def test_unarmed_late_first_packet_rejected_then_fresh_can_start(runtime,mode):
    t=runtime;prepare(t);t.r.config['mode']=mode
    previous=t.l.last_gui
    with pytest.raises(ValueError,match='age='):heartbeat(t,1,age=.201)
    assert t.l.phase!='FAULT' and not any(t.l.requested.values())
    assert t.l.last_gui==previous and t.r.last_gui_sequence==-1
    assert t.logs[-1]['severity']=='WARNING'
    heartbeat(t,2)
    t.r.request('left',True);heartbeat(t,3)
    assert t.l.side_state('left',t.clock[0])=='ACTIVE'
    assert not t.l.requested['right']


def test_same_connection_replay_rejected_off_and_connection_restart_can_reset(runtime):
    t=runtime;prepare(t)
    heartbeat(t,10,generation=1)
    with pytest.raises(ValueError,match='previous_sequence=10'):heartbeat(t,1,generation=1)
    assert t.r.last_gui_sequence==10 and t.l.phase!='FAULT'
    heartbeat(t,1,generation=2)
    assert t.r.last_gui_sequence==1 and not any(t.l.requested.values())
    t.r.request('right',True);heartbeat(t,2,generation=2)
    assert t.l.side_state('right',t.clock[0])=='ACTIVE'


@pytest.mark.parametrize('bad',['stale','replay','new_connection_replay'])
def test_authorized_bad_heartbeat_still_hard_faults_and_revokes(runtime,bad):
    t=runtime;t.r.last_gui_sequence=10;t.r.control_sequence=t.l.control_sequence
    t.r._gui_connection_generation=1
    with pytest.raises(ValueError,match='GUI stale/replayed'):
        heartbeat(t,11 if bad=='stale' else 1,generation=2 if bad=='new_connection_replay' else 1,age=.201 if bad=='stale' else 0)
    assert t.l.phase=='FAULT' and not any(t.l.requested.values())
    assert 'previous_sequence=10' in t.l.reason


def test_supervised_unarmed_stale_policy_remains_strict(runtime):
    t=runtime;prepare(t);t.r.config['mode']='supervised_hardware_validation'
    with pytest.raises(ValueError,match='GUI stale/replayed'):heartbeat(t,1,age=.201)
    assert t.l.phase=='FAULT'


def test_heartbeat_inside_existing_timeout_still_valid(runtime):
    t=runtime;t.r.last_gui_sequence=-1;t.r.control_sequence=t.l.control_sequence
    heartbeat(t,1,age=.199)
    assert t.l.phase=='ARMED' and all(t.l.requested.values())
