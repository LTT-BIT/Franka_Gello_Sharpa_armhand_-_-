"""Reusable normal-mode scenarios through the unchanged original mock sender."""
import json
import time
import numpy as np
from types import SimpleNamespace
from adapters.litchibot.normal_control import NormalHandAuthority
from adapters.litchibot.test_validation_safety import solved_packet


def exercise_normal_driver(node,fault,hands,mode):
    from std_msgs.msg import String
    r=node.full;l=node.latch
    assert isinstance(l,NormalHandAuthority) and r.normal_path
    assert [l.side_state(s,time.monotonic()) for s in ('left','right')]==['READY','READY']
    seq={'left':0,'right':0};hbseq=0
    def hb(age=0,generation=1):
        nonlocal hbseq
        hbseq+=1;r.dispatch({'command':'heartbeat','sequence':hbseq,'monotonic_ns':time.monotonic_ns()-int(age*1e9),'_connection_generation':generation})
    def feed(side='left',value=0,**patch):
        seq[side]+=1;p=solved_packet(side,seq[side]);q=[0.0]*22;q[18]=value
        p.update(positions_rad=q,source_received_monotonic_ns=time.monotonic_ns());p.update(patch)
        m=String();m.data=json.dumps(p);r.litchibot(side,m)
    hb();r.dispatch({'command':'engage_hand','arguments':{'side':'left'}});feed(value=.08)
    assert hands[0].read_joint_state().position[18]==.08 and node.validation_send_count==1
    if fault=='acquisition':
        assert l.side_state('left',time.monotonic())=='ACTIVE';return
    r.request('right',True);feed('right');assert node.validation_send_count==2
    if fault=='partial':
        for _ in range(40):
            hb();feed(value=.25,valid_glove_joint_count=18,valid_target_mask=[True]*18+[False]*4,held_joints=['pinky_MCP_FE'])
        assert l.phase!='FAULT' and hands[0].read_joint_state().position[18]==.25 and node.validation_send_count==42
        return
    if fault=='stale_future':
        for offset in (-.3,.3):feed(value=.25,source_received_monotonic_ns=time.monotonic_ns()+int(offset*1e9))
        assert l.phase!='FAULT' and node.validation_send_count==2
        hb();feed(value=.25);assert node.validation_send_count==3
        return
    if fault=='heartbeat_paused':
        hb(age=.3);assert l.phase!='FAULT' and all(l.requested.values())
        assert all(l.side_state(s,time.monotonic())=='ACTIVE' for s in ('left','right'))
        before=node.validation_send_count;feed(value=.25);assert node.validation_send_count==before+1
        hb(generation=2);feed(value=.25);assert node.validation_send_count==before+2 and l.phase!='FAULT'
        return
    if fault=='unrecoverable_sender':
        from unittest.mock import Mock
        hands[0].send_action=Mock(side_effect=RuntimeError('Unrecoverable native sender failure'))
        feed(value=.25);assert l.phase=='FAULT' and not any(l.requested.values())
        return
    if fault in ('duplicate_driver','source_disconnect','driver_disconnect'):
        original=node.get_publishers_info_by_topic
        if fault=='duplicate_driver':node.get_node_names_and_namespaces=lambda:[('sharpa_driver','/'),('sharpa_driver','/extra')]
        else:
            missing='/teleop/sharpa/right/source' if fault=='source_disconnect' else '/sharpa/right/joint_states'
            node.get_publishers_info_by_topic=lambda topic:[] if topic==missing else original(topic)
        node.service_guard()
        if fault=='duplicate_driver':
            assert l.phase=='FAULT' and not any(l.requested.values())
        else:
            assert l.phase!='FAULT' and l.side_state('right',time.monotonic())=='OFFLINE' and not l.requested['right']
            assert l.side_state('left',time.monotonic())=='ACTIVE'
            node.get_publishers_info_by_topic=original;hb();feed('right',.25)
            assert l.side_state('right',time.monotonic())=='READY'
        return
    if fault in ('disconnect','per_side_recovery','reanchor'):
        if fault=='disconnect':
            r.disconnected();assert l.phase!='FAULT' and not any(l.requested.values())
            assert l.side_state('left',time.monotonic())=='OFFLINE'
            hb(generation=2);feed(value=.25);feed('right',.25)
            assert all(l.side_state(s,time.monotonic())=='READY' for s in ('left','right'))
            assert not any(l.requested.values());return
        elif fault=='per_side_recovery':
            for value in (.4,0,.4,0):hb();feed('right',value)
            for _ in range(4):
                hb();feed('right',.25,valid_glove_joint_count=18,held_joints=['pinky_MCP_FE'],valid_target_mask=[True]*18+[False]*4)
            assert hands[1].read_joint_state().position[18]==.25
        else:
            old=r.raw.previous['right'];r.raw.previous['right']=(*old[:4],old[4]-14_800_000_000)
            feed('right',.4);assert r.raw.context['right']['delta'] is None
            feed('right',.25);assert 0<r.raw.context['right']['dt']<.2
        assert l.phase!='FAULT' and all(l.side_state(s,time.monotonic())=='ACTIVE' for s in ('left','right'))
        return
    if fault in ('moderate','diagnostic_143','diagnostic_179'):
        value=.185638 if fault=='moderate' else .143064 if fault=='diagnostic_143' else .179
        feed(value=value);assert hands[0].read_joint_state().position[18]==value and l.phase!='FAULT'
        stops=[h.stop.call_count for h in hands];r.request('left',False);feed(value=.4)
        assert l.side_state('left',time.monotonic())=='READY' and [h.stop.call_count for h in hands]==stops
        # Normal restores the standalone driver default: no idle disable.
        enum=next(s for s in node._hands if s.value=='left')
        node._last_command_at[enum]=time.monotonic()-.3
        node._publish_state();assert not node._timeout_stopped[enum]
        node.service_guard();assert not node.needs_enable and not hasattr(node,'native_enable')
        assert [h.stop.call_count for h in hands]==stops
        r.request('left',True);hb();before=node.validation_send_count;feed(value=.25)
        assert node.validation_send_count==before+1
        assert hands[0].read_joint_state().position[18]==.25
        return
    if fault=='timeout':
        l.valid_seen['right']=time.monotonic()-19;node.service_guard()
    else:
        for _ in range(180 if fault=='repeat_hard' else 1):feed(positions_rad=[float('nan')]*22)
    assert l.phase!='FAULT'
    if fault=='timeout':
        assert l.side_state('right',time.monotonic())=='OFFLINE'
        feed('right');assert l.side_state('right',time.monotonic())=='READY'
    else:
        assert all(l.requested.values())
        before=node.validation_send_count;feed(value=.25);assert node.validation_send_count==before+1
