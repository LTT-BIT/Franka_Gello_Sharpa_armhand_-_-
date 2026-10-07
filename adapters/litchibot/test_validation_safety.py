import json
import time
from pathlib import Path

import numpy as np
import pytest

from adapters.litchibot.test_backend import packet
from adapters.litchibot.validation_safety import load_permit,MotionLatch,RawContinuityGate,SOURCE_NAMES
from adapters.litchibot.retarget import JOINT_NAMES


def permit():
    return {'mode':'supervised_hardware_validation','operator':'test only','sop_reference':'test SOP',
        'adult_qualified_operator':True,'sop_reviewed':True,'physical_estop_checked':True,
        'workspace_clear':True,'known_pinky_risk_acknowledged':True,'connect_auto_enable_acknowledged':True,
        'expires_unix_s':time.time()+60,'session_token':'a'*64,'serials':{'left':'testL','right':'testR'},'profile':'test'}


def solved_packet(side='left',sequence=1):
    p=packet();p.update(side=side,sequence=sequence,positions_rad=[0.0]*22,dry_run=True,
        glove_status='solved',valid_glove_joint_mask=[True]*20,glove_joint_angles_rad=[0.0]*20)
    p['glove_joint_names']=list(SOURCE_NAMES)
    return p


def test_permit_defaults_cannot_authorize_hardware(tmp_path):
    example=Path(__file__).resolve().parents[2]/'config/litchibot/validation_permit.example.json'
    p=tmp_path/'permit.json';p.write_text(example.read_text());p.chmod(0o600)
    with pytest.raises(ValueError):load_permit(p)
    data=permit();p.write_text(json.dumps(data));assert load_permit(p)==data
    for patch in ({'adult_qualified_operator':False},{'expires_unix_s':0},
                  {'serials':{'left':'x','right':'x'}},{'session_token':''}):
        p.write_text(json.dumps({**data,**patch}))
        with pytest.raises(ValueError):load_permit(p)
    p.write_text(json.dumps(data));p.chmod(0o644)
    with pytest.raises(ValueError):load_permit(p)


@pytest.mark.parametrize('patch',[
    {'positions_rad':[float('nan')]*22},{'positions_rad':[float('inf')]*22},
    {'positions_rad':[0.0]*21},{'joint_names':list(reversed(JOINT_NAMES))},
    {'side':'other'},{'positions_rad':[10.0]*22},{'glove_status':'partial'},
    {'valid_target_mask':[False]*22},{'valid_glove_joint_mask':[False]*20},
    {'glove_joint_angles_rad':[float('nan')]*20},
])
def test_raw_gate_rejects_contract_and_invalid(patch):
    with pytest.raises(ValueError):RawContinuityGate().observe({**solved_packet(),**patch},now_ns=1_010_000_000)


def test_historical_jump_rejected_before_slew_and_dropped_inbox_frames():
    from adapters.litchibot.terminal2_bridge import WorkerInbox
    inbox=WorkerInbox(supervised=True)
    a=solved_packet();a['source_received_monotonic_ns']=time.monotonic_ns()
    a['positions_rad'][18]=1.5708;a['glove_joint_angles_rad'][17]=2.094395
    b=solved_packet(sequence=2);b['source_received_monotonic_ns']=time.monotonic_ns()
    inbox.feed(json.dumps(a));inbox.feed(json.dumps(b))
    assert 'discontinuity' in inbox.safety_fault and inbox.take()=={}


def warm_latch():
    p=permit();l=MotionLatch(p,1.0)
    for sequence in range(1,31):
        now=1+sequence*.03
        for s in ('left','right'):
            assert not l.command(s,JOINT_NAMES,[0.0]*22,10**9,10**9,f'litchibot:{s}:session:{sequence}',now,[0.0]*22)
    assert l.phase=='READY'
    return l,p,now


def test_deadman_release_timeout_and_no_auto_resume():
    for timeout in (False,True):
        l,p,now=warm_latch()
        heartbeat={'token':p['session_token'],'hold':True,'sequence':1,'monotonic_ns':int(now*1e9)}
        l.heartbeat(heartbeat,now);assert l.phase=='ARMED'
        if timeout:l.watch(now+.201,time.time())
        else:l.heartbeat({**heartbeat,'hold':False,'sequence':2},now)
        assert l.phase=='FAULT'
        l.heartbeat({**heartbeat,'sequence':3},now);assert l.phase=='FAULT'
        with pytest.raises(ValueError):l.command('left',JOINT_NAMES,[0.0]*22,10**9,10**9,'litchibot:left:session:31',now,[0.0]*22)


def test_ros_staleness_side_discontinuity_and_watchdogs():
    cases=[('left',JOINT_NAMES,[0.0]*22,0,'litchibot:left:session:31'),
           ('left',JOINT_NAMES,[0.0]*22,10**9,'litchibot:right:session:31'),
           ('left',JOINT_NAMES,[0.11]*22,10**9,'litchibot:left:session:31')]
    for side,names,q,stamp,frame in cases:
        l,p,now=warm_latch()
        with pytest.raises(ValueError):l.command(side,names,q,stamp,10**9,frame,now,[0.0]*22)
    l,p,now=warm_latch();l.watch(now+.201,time.time());assert l.phase=='FAULT'
    l,p,now=warm_latch();l.watch(now,p['expires_unix_s']);assert l.phase=='FAULT'


def test_worker_crash_and_supervisor_loss_do_not_require_bridge_alive():
    l,p,now=warm_latch()
    l.heartbeat({'token':p['session_token'],'hold':True,'sequence':1,'monotonic_ns':int(now*1e9)},now)
    # Hardware-process timer still trips with neither process producing messages.
    l.watch(now+.21,time.time());assert l.phase=='FAULT'


def test_direct_guarded_entry_cannot_weaken_watchdog_or_binding():
    from adapters.litchibot.validation_driver import validate_driver_arguments
    params=['--ros-args','-p','use_fake_hardware:=false','-p','enable_left:=true',
            '-p','enable_right:=true','-p','left_serial:=testL','-p','right_serial:=testR',
            '-p','stop_on_command_timeout_s:=0.2']
    validate_driver_arguments(params,permit())
    for bad in ([],params[:-2],params+['-p','speed_coefficient:=1.0'],
                params+['-r','__node:=unprotected'],params+['-p','left_command_topic:=other'],
                [v.replace('left_serial:=testL','left_serial:=testR') for v in params]):
        with pytest.raises(ValueError):validate_driver_arguments(bad,permit())


def test_small_range_and_first_hardware_acquisition_are_checked():
    l,p,now=warm_latch()
    l.heartbeat({'token':p['session_token'],'hold':True,'sequence':1,'monotonic_ns':int(now*1e9)},now)
    with pytest.raises(ValueError,match='slew'):
        l.command('left',JOINT_NAMES,[0.05]*22,10**9,10**9,'litchibot:left:session:31',now,[0.0]*22)
    # Independent excursion bound is measured-pose-relative, not preview-relative.
    l,p,now=warm_latch()
    l.heartbeat({'token':p['session_token'],'hold':True,'sequence':1,'monotonic_ns':int(now*1e9)},now)
    l.last['left']=(np.array([0.09]*22),('session',30),now)
    with pytest.raises(ValueError,match='excursion'):
        l.command('left',JOINT_NAMES,[0.16]*22,10**9,10**9,'litchibot:left:session:31',now,[0.0]*22)


def test_validation_launch_selects_one_guarded_process_and_no_plain_driver(tmp_path):
    pytest.importorskip('launch_ros')
    from adapters.litchibot.test_terminal2 import load_launch,context_for,ROOT
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    p=tmp_path/'permit.json';p.write_text(json.dumps(permit()));p.chmod(0o600)
    module=load_launch();description=module.generate_launch_description()
    context=context_for(description,{'hand_source':'litchibot','supervised_hardware_validation':'true',
        'validation_permit':str(p),'litchibot_profile':'test',
        'litchibot_bridge_script':str(ROOT/'adapters/litchibot/terminal2_bridge.py'),
        'validation_driver_script':str(ROOT/'adapters/litchibot/validation_driver.py')})
    module.validate_source(context)
    selected=[e for e in description.entities if isinstance(e,ExecuteProcess)
              and (e.condition is None or e.condition.evaluate(context))]
    assert len(selected)==2
    assert not any(isinstance(e,Node) for e in selected)
    assert context.launch_configurations['validation_left_serial']=='testL'
    context.launch_configurations['dry_run']='true'
    with pytest.raises(RuntimeError):module.validate_source(context)


def test_guarded_driver_calls_original_stop_and_never_loads_sdk(tmp_path,monkeypatch):
    rclpy=pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from unittest.mock import Mock
    import socket
    import secrets
    from sensor_msgs.msg import JointState
    from adapters.litchibot import validation_driver
    from adapters.litchibot.validation_safety import control_paths
    import sharpa_driver.node as original
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    data=permit();data['session_token']=secrets.token_hex(32)
    path=tmp_path/'permit.json';path.write_text(json.dumps(data));path.chmod(0o600)
    sdk_load=Mock(side_effect=AssertionError('Physical SDK must never load in tests'))
    monkeypatch.setattr(SharpaSdkHand,'_load_sdk',sdk_load)
    preflight=Mock();monkeypatch.setattr(validation_driver,'ensure_clear_graph',preflight)
    hands=[]
    def create(config,**kwargs):
        assert preflight.called
        hand=MockSharpaHand(config);hand.stop=Mock(wraps=hand.stop)
        hand._manager=SimpleNamespace(get_all_devices=lambda:[SimpleNamespace(sn=config.serial,
            hand_side=SimpleNamespace(name=config.side.value.upper()))])
        hand.send_action=Mock(wraps=hand.send_action);hands.append(hand);return hand
    monkeypatch.setattr(original,'create_hand',create)
    def exercise(node):
        node.get_publishers_info_by_topic=lambda topic:[SimpleNamespace(node_name='sharpa_driver' if topic.endswith('joint_states') else 'litchibot_command_bridge')]
        node.get_subscriptions_info_by_topic=lambda topic:[SimpleNamespace(node_name='sharpa_driver')]
        node.get_node_names_and_namespaces=lambda:[('sharpa_driver','/')]
        def command(side,sequence,q=None,frame_side=None):
            msg=JointState();msg.name=list(JOINT_NAMES);msg.position=q or [0.0]*22
            msg.header.stamp=node.get_clock().now().to_msg()
            msg.header.frame_id=f'litchibot:{frame_side or side.value}:session:{sequence}'
            node._on_command(side,msg)
        sides=list(node._hands)
        for sequence in range(1,31):
            for side in sides:command(side,sequence)
        assert node.latch.phase=='READY'
        assert all(not hand.send_action.called for hand in hands)
        socket_path,_=control_paths(data)
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sock:
            sock.sendto(json.dumps({'token':data['session_token'],'hold':True,'sequence':1,
                'monotonic_ns':time.monotonic_ns()}).encode(),str(socket_path))
        command(sides[0],31,[0.001]*22)
        assert node.validation_send_count==1
        command(sides[0],32,frame_side='right')
        assert node.latch.phase=='FAULT'
        assert all(hand.stop.called for hand in hands)
        command(sides[0],33)
        assert node.validation_send_count==1
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--permit',str(path),'--ros-args','-p','use_fake_hardware:=false',
        '-p','enable_left:=true','-p','enable_right:=true','-p','left_serial:=testL',
        '-p','right_serial:=testR','-p','stop_on_command_timeout_s:=0.2'])
    sdk_load.assert_not_called()
