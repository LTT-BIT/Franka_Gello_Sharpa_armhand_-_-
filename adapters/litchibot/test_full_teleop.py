"""Full mode tests: synthetic sources, fake hands, stub wrappers, no physical SDK."""
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import threading
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from adapters.litchibot.retarget import JOINT_NAMES
from adapters.litchibot.validation_safety import MotionLatch,RawContinuityGate
from adapters.litchibot.test_validation_safety import solved_packet
from adapters.litchibot.full_control import load_full_config,HandControlServer
from teleop_runtime.full_launcher import FullLauncher,parser,session_config,descendants,alive


def unused_port():
    with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]


@pytest.mark.parametrize('value',[-.005,-.021,float('nan')])
def test_manus_idle_feedback_uses_measurement_allowance_and_bounded_no_send_snapshot(value):
    pytest.importorskip('sensor_msgs.msg')
    from builtin_interfaces.msg import Time
    from litchi_hardware.core.types import HandSide
    from adapters.litchibot.full_runtime import FullHandRuntime
    from adapters.litchibot.validation_safety import ManusNormalMotionLatch
    measured=np.zeros(22);measured[7]=value
    runtime=FullHandRuntime.__new__(FullHandRuntime)
    runtime.latch=ManusNormalMotionLatch({'session_token':'a'*64},time.monotonic(),source='manus',gui_controlled=True,validation_only=False)
    runtime.node=SimpleNamespace(_hands={HandSide.LEFT:SimpleNamespace(read_joint_state=lambda:SimpleNamespace(position=measured))},
        stop_validation=Mock(),get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time())))
    runtime.publishers={'left':Mock()}
    runtime.idle_snapshot('left')
    if np.isfinite(value) and value>=-.02:
        runtime.node.stop_validation.assert_not_called()
        msg=runtime.publishers['left'].publish.call_args.args[0]
        assert msg.position[7]==0 and msg.header.frame_id.startswith('supervisor_no_send:')
        assert measured[7]==value  # Actual feedback is never rewritten.
    else:
        runtime.node.stop_validation.assert_called_once()
        runtime.publishers['left'].publish.assert_not_called()


@pytest.mark.parametrize('state',['stale','missing','current','foreign'])
def test_manus_project_bridge_refresh_before_hardware(tmp_path,monkeypatch,state):
    from teleop_runtime import full_launcher
    root=tmp_path;native=root/'adapters/manus/native';native.mkdir(parents=True)
    library=root/'adapters/manus/build/libmanus_skeleton_bridge.so'
    library.parent.mkdir();(root/'docker').mkdir()
    for p in (native/'manus_skeleton_bridge.cpp',native/'litchi_manus_bridge.h',native.parent/'CMakeLists.txt'):
        p.write_text('fixture');os.utime(p,ns=(200,200))
    if state!='missing':
        library.write_bytes(b'fixture');os.utime(library,ns=(100,100) if state=='stale' else (300,300))
    configured=root/'foreign.so' if state=='foreign' else library
    (root/'docker/.env').write_text('MANUS_NATIVE_LIBRARY='+str(configured)+'\n')
    run=Mock();monkeypatch.setattr(full_launcher.subprocess,'run',run)
    full_launcher.ensure_manus_bridge_current(root)
    if state in ('stale','missing'):
        run.assert_called_once_with([str(root/'adapters/manus/scripts/build.sh')],cwd=root,check=True)
    else:run.assert_not_called()


def config_file(tmp_path,source='litchibot',mode='fake'):
    data=session_config(parser().parse_args(['--hand-source',source,'--control-port',str(unused_port())]))
    data['mode']=mode
    if mode=='normal':
        devices=tmp_path/'devices.json';devices.write_text(json.dumps({'serials':{'left':'testL','right':'testR'}}))
        data['devices']=str(devices)
    p=tmp_path/'session.json';p.write_text(json.dumps(data));p.chmod(0o600)
    return p,data


def test_normal_has_no_validation_prerequisites(tmp_path):
    p,_=config_file(tmp_path,mode='normal');cfg=load_full_config(p)
    assert 'permit' not in cfg and 'adult_qualified_operator' not in cfg['policy']
    assert cfg['policy']['expires_unix_s']==float('inf')
    p.chmod(0o644)
    with pytest.raises(ValueError):load_full_config(p)


def normal_latch():
    l=MotionLatch({'session_token':'a'*64,'expires_unix_s':0},1,gui_controlled=True,validation_only=False)
    for seq in range(1,31):
        t=1+seq*.03
        for s in ('left','right'):l.command(s,JOINT_NAMES,[0.0]*22,1,1,f'litchibot:{s}:test:{seq}',t,[0.0]*22)
    return l,t


def hb(l,t,seq):
    l.gui_heartbeat(t);l.heartbeat({'token':'a'*64,'hold':True,'sequence':seq,'monotonic_ns':int(t*1e9)},t)


def test_normal_per_side_start_stop_restart_and_fault_priority():
    l,t=normal_latch()
    assert l.side_state('left',t)=='DISABLED'
    l.gui_heartbeat(t);l.request_hand('left',True,t);hb(l,t,1)
    assert l.side_state('left',t)=='ACTIVE' and l.side_state('right',t)=='DISABLED'
    l.request_hand('right',True,t);assert l.side_state('right',t)=='ACTIVE'
    l.request_hand('left',False,t);assert l.side_state('left',t)=='DISABLED'
    assert l.side_state('right',t)=='ACTIVE'
    l.request_hand('right',False,t);assert l.phase=='READY'
    l.watch(t+.01,time.time());assert l.phase=='READY'
    l.request_hand('left',True,t);hb(l,t,2);assert l.side_state('left',t)=='ACTIVE'
    l.trip('raw pinky discontinuity');assert not any(l.requested.values())
    with pytest.raises(ValueError,match='pinky'):l.request_hand('left',True,t)


def test_normal_retains_timeout_nan_limits_and_historical_discontinuity():
    l,t=normal_latch();l.gui_heartbeat(t);l.request_hand('left',True,t);hb(l,t,1)
    l.watch(t+.201,time.time());assert l.phase=='FAULT'


def test_cold_acquisition_pause_keeps_startup_deadline_and_no_permits():
    l=MotionLatch({'session_token':'a'*64,'expires_unix_s':float('inf')},time.monotonic(),
        gui_controlled=True,source='manus',validation_only=False)
    l.samples['left']=14
    l.pause('Cold V4 discontinuity')
    l.watch(l.started+.3,time.time())
    assert l.phase=='WARMUP' and l.samples=={'left':0,'right':0}
    assert not any(l.requested.values())
    with pytest.raises(ValueError):l.request_hand('left',True,l.started+.3)
    l.watch(l.started+30.01,time.time())
    assert l.phase=='FAULT' and l.reason=='Acquisition timeout'
    ready,t=normal_latch()
    assert ready.phase=='READY'
    ready.pause('Runtime anomaly')
    assert ready.phase=='RECOVERING'
    ready.watch(max(t,ready.recovery_started)+.201,time.time())
    assert ready.phase=='FAULT'
    for values in ([float('nan')]*22,[float('inf')]*22,[3]*22,[0]*21):
        l,t=normal_latch()
        with pytest.raises(ValueError):l.command('left',JOINT_NAMES,values,1,1,'litchibot:left:test:31',t+.01,[0]*22)
    g=RawContinuityGate();a=solved_packet();a['source_received_monotonic_ns']=time.monotonic_ns();a['positions_rad'][18]=1.5708
    g.observe(a)
    b=solved_packet(sequence=2);b['source_received_monotonic_ns']=time.monotonic_ns()
    with pytest.raises(ValueError,match='pinky_MCP_FE'):g.observe(b)


def test_normal_no_validation_expiry_window_or_hold_prerequisite():
    l,t=normal_latch();l.gui_heartbeat(t);l.request_hand('left',True,t);hb(l,t,1)
    # Runtime leases and source remain fresh while elapsed run exceeds acceptance cap.
    later=t+61
    l.last={s:(v[0],v[1],later) for s,v in l.last.items()};hb(l,later,2)
    l.watch(later,time.time());assert l.phase=='ARMED'
    l.baseline['left'][:]=-.2
    assert l.command('left',JOINT_NAMES,[0]*22,1,1,'litchibot:left:test:31',later,[0]*22)


@pytest.mark.parametrize('source',['manus','litchibot'])
@pytest.mark.parametrize('mode',['fake','normal'])
def test_full_launch_selection_single_guard_and_original_retarget_remap(tmp_path,source,mode):
    from adapters.litchibot.test_terminal2 import load_launch,context_for,ROOT
    pytest.importorskip('launch_ros')
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    from launch.utilities import perform_substitutions
    module=load_launch();description=module.generate_launch_description();p,cfg=config_file(tmp_path,source,mode)
    context=context_for(description,{'hand_source':source,'full_teleop':'true','full_config':str(p),
        'validation_driver_script':str(ROOT/'adapters/litchibot/validation_driver.py'),
        'litchibot_bridge_script':str(ROOT/'adapters/litchibot/terminal2_bridge.py')})
    module.validate_source(context)
    selected=[e for e in description.entities if isinstance(e,ExecuteProcess) and (e.condition is None or e.condition.evaluate(context))]
    nodes=[e for e in selected if isinstance(e,Node)]
    assert not any(e.node_package=='sharpa_driver' for e in nodes)
    guards=[e for e in selected if not isinstance(e,Node) and 'validation_driver.py' in ' '.join(perform_substitutions(context,arg) for arg in e.cmd)]
    assert len(guards)==1
    cmd=' '.join(perform_substitutions(context,arg) for arg in guards[0].cmd)
    assert 'use_fake_hardware:='+('true' if mode=='fake' else 'false') in cmd
    assert ('manus_driver' in [e.node_package for e in nodes])==(source=='manus')
    if source=='manus':
        rt=next(e for e in nodes if e.node_package=='manus_sharpa_teleop')
        pairs=rt._Node__remappings
        values=[(perform_substitutions(context,a),perform_substitutions(context,b)) for a,b in pairs]
        assert values==[(f'/sharpa/{s}/command',f'/teleop/sharpa/{s}/target') for s in ('left','right')]
    else:
        bridge=next(e for e in selected if not isinstance(e,Node) and e not in guards)
        assert '--managed-full' in ' '.join(perform_substitutions(context,arg) for arg in bridge.cmd)


def test_full_dry_run_cannot_select_normal_hardware(tmp_path):
    from adapters.litchibot.test_terminal2 import load_launch,context_for,ROOT
    module=load_launch();p,_=config_file(tmp_path,mode='normal')
    ctx=context_for(module.generate_launch_description(),{'hand_source':'litchibot','dry_run':'true','full_teleop':'true',
        'full_config':str(p),'validation_driver_script':str(ROOT/'adapters/litchibot/validation_driver.py')})
    with pytest.raises(RuntimeError,match='conflict'):module.validate_source(ctx)


@pytest.mark.parametrize('source',['manus','litchibot'])
def test_real_host_wrapper_lifecycle_with_stub_subsystems(tmp_path,source):
    root=tmp_path/'repo';scripts=root/'ops/run';scripts.mkdir(parents=True)
    # Actual subprocesses emulate old wrappers, including a GUI in a nested session.
    stub='''#!/usr/bin/env python3
import os,signal,subprocess,sys,time,json
from pathlib import Path
p=Path(os.environ['TELEOP_FULL_CONFIG']).parent
(p/(Path(sys.argv[0]).stem+'.json')).write_text(json.dumps({'argv':sys.argv,'config':os.environ['TELEOP_FULL_CONFIG'],'pid':os.getpid()}))
child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(120)'],start_new_session=True)
def stop(*args):
 child.terminate();child.wait();raise SystemExit(0)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
while True:time.sleep(.1)
'''
    for name in ('run_gello_arms_only.sh','run_sharpa_hands_cyclonedds.sh'):
        f=scripts/name;f.write_text(stub);f.chmod(0o755)
    p,cfg=config_file(tmp_path,source)
    cfg['profile']='deployment-profile'
    p.write_text(json.dumps(cfg))
    l=FullLauncher(root,p,cfg)
    try:
        l.start();deadline=time.monotonic()+3
        while time.monotonic()<deadline and not all((tmp_path/(name+'.json')).exists() for name in ('run_gello_arms_only','run_sharpa_hands_cyclonedds')):time.sleep(.02)
        records=[json.loads((tmp_path/(name+'.json')).read_text()) for name in ('run_gello_arms_only','run_sharpa_hands_cyclonedds')]
        assert '--hand-source' not in records[0]['argv']
        assert source in records[1]['argv'] and '--dry-run' in records[1]['argv']
        assert ('litchibot_profile:=deployment-profile' in records[1]['argv'])==(source=='litchibot')
        owned={pid:identity for child in l.children for pid,identity in descendants(child.pid).items()}
        assert len(owned)>=4
    finally:l.close()
    assert all(c.poll() is not None for c in l.children)
    assert not any(alive(pid,identity) for pid,identity in owned.items())


@pytest.fixture
def isolated_guard_paths(monkeypatch):
    # Mock-only host tests use their own lock, away from a live user session.
    from adapters.litchibot import validation_driver
    from adapters.litchibot import terminal2_bridge
    with tempfile.TemporaryDirectory(prefix='hand-gate-test-') as directory:
        root=Path(directory)
        monkeypatch.setattr(validation_driver,'control_paths',
                            lambda permit:(root/'control.sock',root/'status.json'))
        monkeypatch.setattr(terminal2_bridge,'control_paths',
                            lambda permit:(root/'control.sock',root/'status.json'))
        yield


@pytest.mark.parametrize('spike',[.456202,1.0])
def test_manus_standby_does_not_latch_and_legacy_valid_jumps_continue(tmp_path,monkeypatch,isolated_guard_paths,spike):
    rclpy=pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseArray,Pose
    from sensor_msgs.msg import JointState
    import sharpa_driver.node as original
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    from adapters.litchibot import validation_driver
    p,cfg=config_file(tmp_path,'manus');hands=[]
    forbidden=Mock(side_effect=AssertionError('Physical SDK forbidden'))
    monkeypatch.setattr(SharpaSdkHand,'_load_sdk',forbidden)
    monkeypatch.setattr(validation_driver,'ensure_clear_graph',Mock())
    def create(config,**kw):
        assert kw['use_fake_hardware'] is True
        hand=MockSharpaHand(config);hand.stop=Mock(wraps=hand.stop)
        hand.send_action=Mock(wraps=hand.send_action);hands.append(hand);return hand
    monkeypatch.setattr(original,'create_hand',create)
    def exercise(node):
        runtime=node.full
        def feed(side,value=0):
            target=JointState();target.header.stamp=node.get_clock().now().to_msg()
            target.name=list(JOINT_NAMES);q=[0.0]*22
            q[JOINT_NAMES.index('thumb_IP')]=float(value);target.position=q
            runtime.manus(side,target)
        for _ in range(6):feed('right',spike);feed('left');feed('right',0)
        assert node.latch.phase=='READY' and node.validation_send_count==0
        with patch('adapters.manus.normal_hand_runtime.time.monotonic',return_value=time.monotonic()+600):
            runtime.tick()
        assert node.latch.phase=='READY'
        runtime.request('right',True)
        for _ in range(6):feed('right',spike);feed('right',0)
        assert node.latch.phase=='READY' and node.validation_send_count==12
        assert node.latch.requested['right'] and not node.latch.requested['left']
        runtime.request('right',False);feed('right',spike)
        assert node.validation_send_count==12
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--full-config',str(p),'--ros-args','-p','use_fake_hardware:=true',
        '-p','enable_left:=true','-p','enable_right:=true','-p','left_serial:=fake-left',
        '-p','right_serial:=fake-right','-p','stop_on_command_timeout_s:=0.0'])
    forbidden.assert_not_called()


@pytest.mark.parametrize('source',['litchibot'])
@pytest.mark.parametrize('mode',['fake','normal'])
@pytest.mark.parametrize('fault',['disconnect','timeout','jump','moderate','acquisition','diagnostic_143','diagnostic_179','per_side_recovery','duplicate_driver','source_disconnect','driver_disconnect','reanchor','repeat_hard','partial','stale_future','heartbeat_paused','unrecoverable_sender'])
def test_full_guard_original_fake_driver_no_sdk_and_gui_dispatch(tmp_path,monkeypatch,source,mode,fault,isolated_guard_paths,capfd):
    rclpy=pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseArray,Pose
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from adapters.litchibot import validation_driver
    import sharpa_driver.node as original
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    p,cfg=config_file(tmp_path,source,mode)
    sdk=Mock(side_effect=AssertionError('Never load physical SDK'));monkeypatch.setattr(SharpaSdkHand,'_load_sdk',sdk)
    monkeypatch.setattr(validation_driver,'ensure_clear_graph',Mock())
    hands=[]
    def create(config,**kwargs):
        assert kwargs['use_fake_hardware']==(mode=='fake');hand=MockSharpaHand(config);hand.send_action=Mock(wraps=hand.send_action)
        if mode=='normal':
            # Exercise normal binding/enable branches with an in-memory SDK boundary.
            hand._manager=SimpleNamespace(get_all_devices=lambda:[SimpleNamespace(sn=config.serial,hand_side=config.side)])
            hand._ensure_enabled=Mock(side_effect=lambda:setattr(hand,'_stopped',False))
        hand.stop=Mock(wraps=hand.stop);hands.append(hand);return hand
    monkeypatch.setattr(original,'create_hand',create)
    def exercise(node):
        assert len(node._hands)==2 and node._command_subscriptions==[]
        node.get_publishers_info_by_topic=lambda topic:[SimpleNamespace(node_name=('litchibot_command_bridge' if source=='litchibot' else 'manus_sharpa_retarget') if topic.startswith('/teleop/') else 'sharpa_driver')]
        node.get_subscriptions_info_by_topic=lambda topic:[]
        node.get_node_names_and_namespaces=lambda:[('sharpa_driver','/')]
        runtime=node.full
        from adapters.litchibot.test_normal_driver import exercise_normal_driver
        exercise_normal_driver(node,fault,hands,mode)
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--full-config',str(p),'--ros-args','-p',f'use_fake_hardware:={str(mode=="fake").lower()}',
        '-p','enable_left:=true','-p','enable_right:=true','-p','left_serial:='+('testL' if mode=='normal' else 'fake-left'),
        '-p','right_serial:='+('testR' if mode=='normal' else 'fake-right'),'-p','stop_on_command_timeout_s:='+('0.0' if source=='litchibot' else '0.2')])
    sdk.assert_not_called()
    expected=.18 if source=='litchibot' else .1
    stderr=capfd.readouterr().err
    if source=='litchibot':assert 'independent permits; no validation latch' in stderr
    else:assert f'RawContinuityGate threshold = {expected:.6f} rad' in stderr
    if fault=='repeat_hard' and source=='litchibot':assert stderr.count('"severity": "HARD_FAULT"')==0


def test_full_launcher_ctrl_c_and_orphan_after_wrapper_crash(tmp_path):
    root=tmp_path/'repo';scripts=root/'ops/run';scripts.mkdir(parents=True)
    # Both wrappers create a separate session. The hand wrapper crashes WITHOUT
    # cleanup; the Linux subreaper must still reclaim its orphan worker.
    for crash in (False,True):
        case=tmp_path/str(crash);case.mkdir();p,cfg=config_file(case)
        for name in ('run_gello_arms_only.sh','run_sharpa_hands_cyclonedds.sh'):
            is_crash=crash and name.startswith('run_sharpa')
            code=f'''#!/usr/bin/env python3
import os,signal,subprocess,sys,time
from pathlib import Path
child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(120)'],start_new_session=True)
Path(os.environ['TELEOP_FULL_CONFIG']).with_name('{name}.pid').write_text(str(child.pid))
if {is_crash!r}:
 time.sleep(.1);os._exit(7)
def stop(*args):child.terminate();child.wait();raise SystemExit(0)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
while True:time.sleep(.1)
'''
            f=scripts/name;f.write_text(code);f.chmod(0o755)
        harness='from teleop_runtime.full_launcher import FullLauncher;import json,sys;from pathlib import Path;p=Path(sys.argv[2]);raise SystemExit(FullLauncher(sys.argv[1],p,json.loads(p.read_text())).run())'
        process=subprocess.Popen([sys.executable,'-c',harness,str(root),str(p)],cwd=Path(__file__).resolve().parents[2],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            deadline=time.monotonic()+4
            while time.monotonic()<deadline and len(list(case.glob('*.pid')))<2:time.sleep(.01)
            ids=[int(f.read_text()) for f in case.glob('*.pid')];assert len(ids)==2
            if not crash:process.send_signal(__import__('signal').SIGINT)
            output,error=process.communicate(timeout=28)
            assert process.returncode==(7 if crash else 0),(output,error)
            assert all(not Path(f'/proc/{pid}').exists() for pid in ids)
        finally:
            if process.poll() is None:process.kill();process.wait()


def test_ros_control_service_cannot_create_operator_permit():
    pytest.importorskip('rclpy')
    from adapters.litchibot.full_runtime import FullHandRuntime
    l,t=normal_latch();l.gui_heartbeat(time.monotonic())
    runtime=FullHandRuntime.__new__(FullHandRuntime);runtime.latch=l;runtime.request=Mock()
    response=SimpleNamespace(success=None,message='')
    runtime.enable_service('left',SimpleNamespace(data=True),response)
    assert response.success is False and 'GUI Start hand' in response.message
    runtime.request.assert_not_called()


def test_full_fifo_never_coalesces_historical_jump():
    from adapters.litchibot.terminal2_bridge import WorkerInbox
    inbox=WorkerInbox(fifo=True)
    for seq,value in ((1,1.5708),(2,0),(3,1.5708)):
        p=solved_packet(sequence=seq);p['positions_rad'][18]=value;p['source_received_monotonic_ns']=time.monotonic_ns()
        inbox.feed(json.dumps(p))
    frames=inbox.take_all();assert len(frames)==3
    g=RawContinuityGate();g.observe(frames[0])
    with pytest.raises(ValueError,match='pinky_MCP_FE'):g.observe(frames[1])


def test_real_dds_full_pipeline_state_tactile_and_raw_fault(tmp_path,monkeypatch,isolated_guard_paths):
    rclpy=pytest.importorskip('rclpy')
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    import sharpa_driver.node as original
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    from adapters.litchibot import validation_driver
    p,cfg=config_file(tmp_path)
    spy=Mock(side_effect=AssertionError('No physical SDK'));monkeypatch.setattr(SharpaSdkHand,'_load_sdk',spy)
    def exercise(node):
        source=Node('litchibot_command_bridge');recorder=Node('teleop_data_collector')
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(source);executor.add_node(recorder)
        pubs={s:source.create_publisher(String,f'/teleop/sharpa/{s}/source',1000) for s in ('left','right')}
        received={s:[] for s in ('left','right')}
        subs=[recorder.create_subscription(JointState,f'/sharpa/{s}/command',lambda m,s=s:received[s].append(m),10) for s in ('left','right')]
        try:
            # DDS discovery only: no physical SDK, no external domain or launcher.
            end=time.monotonic()+2
            while time.monotonic()<end and not all(pub.get_subscription_count()==1 for pub in pubs.values()):executor.spin_once(timeout_sec=.005)
            assert all(pub.get_subscription_count()==1 for pub in pubs.values())
            sequence=0
            def frames(count,jump=False):
                nonlocal sequence
                for _ in range(count):
                    sequence+=1
                    for side in ('left','right'):
                        row=solved_packet(side,sequence);row['source_received_monotonic_ns']=time.monotonic_ns()
                        if jump and side=='left':row['positions_rad'][18]=float('nan')
                        m=String();m.data=json.dumps(row);pubs[side].publish(m)
                    end=time.monotonic()+.02
                    while time.monotonic()<end:executor.spin_once(timeout_sec=.001)
            frames(35)
            assert node.latch.phase=='READY',node.latch.reason
            # Slow consumer / burst: old observations must not form a replay
            # backlog ahead of a fresh target. No age limit is relaxed.
            seen=[];receive=node.full.litchibot
            def capture(side,msg):
                seen.append((side,json.loads(msg.data)['sequence']))
                receive(side,msg)
            node.full.litchibot=capture
            for index in range(100):
                sequence+=1
                for side in ('left','right'):
                    packet=solved_packet(side,sequence)
                    packet['source_received_monotonic_ns']=time.monotonic_ns()-(300_000_000 if index<99 else 0)
                    msg=String();msg.data=json.dumps(packet);pubs[side].publish(msg)
            time.sleep(.02)  # DDS receives the burst while executor is busy.
            end=time.monotonic()+.15
            while time.monotonic()<end:executor.spin_once(timeout_sec=.001)
            assert seen==[('left',sequence),('right',sequence)] or seen==[('right',sequence),('left',sequence)]
            assert all(node.full.raw.previous[s][3]==sequence for s in ('left','right'))
            assert all(node.latch.data_ready.values())
            node.full.litchibot=receive
            assert all(received[s] for s in received)
            assert node.validation_send_count==0
            assert len(node.get_publishers_info_by_topic('/sharpa/left/command'))==1
            assert sum(n=='sharpa_driver' for n,ns in node.get_node_names_and_namespaces())==1
            topics=dict(node.get_topic_names_and_types())
            for s in ('left','right'):
                assert topics[f'/sharpa/{s}/joint_states']==['sensor_msgs/msg/JointState']
                assert topics[f'/sharpa/{s}/command']==['sensor_msgs/msg/JointState']
                tactile=[t for t in topics if t.startswith(f'/sharpa/{s}/tactile/') and (t.endswith('/raw') or t.endswith('/deformation'))]
                assert len(tactile)==10
            frames(1,jump=True)
            assert node.latch.phase!='FAULT'
            frames(1);assert node.latch.phase!='FAULT'
            assert node.validation_send_count==0 and node.full.status()['hardware_send']==0
        finally:
            executor.remove_node(node);executor.remove_node(source);executor.remove_node(recorder)
            source.destroy_node();recorder.destroy_node();executor.shutdown()
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--full-config',str(p),'--ros-args','-p','use_fake_hardware:=true',
        '-p','enable_left:=true','-p','enable_right:=true','-p','left_serial:=fake-left',
        '-p','right_serial:=fake-right','-p','stop_on_command_timeout_s:=0.0'])
    spy.assert_not_called()


def test_full_bridge_sdk_worker_stub_guard_proof_and_fault_keeps_input_running(tmp_path,monkeypatch,isolated_guard_paths):
    rclpy=pytest.importorskip('rclpy')
    from adapters.litchibot import validation_driver
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    p,cfg=config_file(tmp_path)
    template=solved_packet();template['positions_rad']=[0.0]*22
    worker=tmp_path/'synthetic_worker_python'
    worker.write_text(f'''#!{sys.executable}
import json,time
row={template!r}
for seq in range(1,95):
 if seq==65:print('malformed diagnostic test frame',flush=True)
 for side in ('left','right'):
  row.update(side=side,sequence=seq,source_received_monotonic_ns=time.monotonic_ns())
  print(json.dumps(row),flush=True)
 time.sleep(.03)
''');worker.chmod(0o755)
    sdk=Mock(side_effect=AssertionError('Do not load hardware SDK'));monkeypatch.setattr(SharpaSdkHand,'_load_sdk',sdk)
    def exercise(node):
        bridge=Path(__file__).with_name('terminal2_bridge.py')
        process=subprocess.Popen([sys.executable,str(bridge),'--managed-full','--full-config',str(p),
            '--worker-python',str(worker),'--profile',cfg['profile'],'--sdk-root','unused','--data-root','unused',
            '--log-dir',str(tmp_path/'logs')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        ready=False
        try:
            end=time.monotonic()+6
            while time.monotonic()<end and process.poll() is None:
                rclpy.spin_once(node,timeout_sec=.003)
                if node.latch.phase=='READY':ready=True
            process.send_signal(__import__('signal').SIGINT)
            out,error=process.communicate(timeout=3)
            assert process.returncode==0,(out,error)
            assert ready,node.latch.reason
            assert node.latch.phase!='FAULT'
            assert node.validation_send_count==0 and node.full.status()['hardware_send']==0
            summary=json.loads((tmp_path/'logs/bridge.jsonl').read_text().splitlines()[-1])
            assert min(summary['published'].values())>70  # Continues after frame 65 fault.
            assert summary['hardware_send_count']==0 and summary['guard_confirmed'] is True
        finally:
            if process.poll() is None:
                process.send_signal(__import__('signal').SIGINT)
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:process.kill();process.wait()
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--full-config',str(p),'--ros-args','-p','use_fake_hardware:=true',
        '-p','enable_left:=true','-p','enable_right:=true','-p','left_serial:=fake-left',
        '-p','right_serial:=fake-right','-p','stop_on_command_timeout_s:=0.0'])
    sdk.assert_not_called()


def test_full_validation_cannot_rebase_excursion_by_repeated_start():
    l,t=normal_latch();l.validation_only=True;l.permit['expires_unix_s']=time.time()+300
    l.gui_heartbeat(t);l.request_hand('left',True,t);hb(l,t,1)
    original=l.baseline['left'].copy()
    l.measured['left'][:]=.1
    l.request_hand('left',True,t)  # Duplicate Start is idempotent.
    np.testing.assert_array_equal(l.baseline['left'],original)
    l.request_hand('left',False,t);l.request_hand('left',True,t)
    np.testing.assert_array_equal(l.baseline['left'],original)
    assert l.phase=='ARMED'


def test_gui_exit_stops_hands_before_slow_arm_cleanup(tmp_path,monkeypatch):
    from teleop_runtime.full_launcher import notify_gui_session_exit
    root=tmp_path/'repo';scripts=root/'ops/run';scripts.mkdir(parents=True)
    p,cfg=config_file(tmp_path)
    for name,side in [('run_sharpa_hands_cyclonedds.sh','hands'),('run_gello_arms_only.sh','arms')]:
        code=f'''#!/usr/bin/env python3
import os,signal,time
from pathlib import Path
p=Path(os.environ['TELEOP_FULL_CONFIG']).parent
(p/'{side}.ready').write_text('ready')
def stop(*args):
 (p/'{side}.stop').write_text(str(time.monotonic()))
 if '{side}'=='arms':time.sleep(1.5)
 (p/'{side}.done').write_text(str(time.monotonic()))
 raise SystemExit(0)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
while True:time.sleep(.01)
'''
        f=scripts/name;f.write_text(code);f.chmod(0o755)
    harness='from teleop_runtime.full_launcher import FullLauncher;import json,sys;from pathlib import Path;p=Path(sys.argv[2]);raise SystemExit(FullLauncher(sys.argv[1],p,json.loads(p.read_text())).run())'
    process=subprocess.Popen([sys.executable,'-c',harness,str(root),str(p)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    monkeypatch.setenv('TELEOP_FULL_EXIT_SOCKET',str(p.with_name('launcher-exit.sock')))
    try:
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and len(list(tmp_path.glob('*.ready')))<2:time.sleep(.01)
        assert len(list(tmp_path.glob('*.ready')))==2
        assert notify_gui_session_exit('wrong-token')
        time.sleep(.15);assert process.poll() is None and not (tmp_path/'hands.stop').exists()
        start=time.monotonic();assert notify_gui_session_exit(cfg['token'])
        deadline=start+1
        while time.monotonic()<deadline and not (tmp_path/'hands.done').exists():time.sleep(.01)
        assert (tmp_path/'hands.done').exists()  # No wait for arm/backend exit.
        assert float((tmp_path/'hands.stop').read_text())-start<1
        output,error=process.communicate(timeout=8)
        assert process.returncode==0,(output,error)
        assert float((tmp_path/'hands.done').read_text())<=float((tmp_path/'arms.stop').read_text())
        assert (tmp_path/'arms.done').exists() and not p.with_name('launcher-exit.sock').exists()
    finally:
        if process.poll() is None:process.send_signal(__import__('signal').SIGINT);process.communicate(timeout=28)


@pytest.mark.parametrize('shutdown',['SIGHUP','SIGTERM','terminal_close'])
def test_full_launcher_terminal_exit_reclaims_independent_hand_session(tmp_path,shutdown):
    import pty
    import signal
    root=tmp_path/'repo';scripts=root/'ops/run';scripts.mkdir(parents=True)
    p,cfg=config_file(tmp_path)
    for name,side in [('run_sharpa_hands_cyclonedds.sh','hands'),('run_gello_arms_only.sh','arms')]:
        code=f'''#!/usr/bin/env python3
import os,signal,time,subprocess,sys
from pathlib import Path
p=Path(os.environ['TELEOP_FULL_CONFIG']).parent
child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(120)'],start_new_session=True)
(p/'{side}.pid').write_text(str(child.pid))
(p/'{side}.ready').write_text(str(os.getsid(0)))
def stop(*args):
 (p/'{side}.stop').write_text(str(time.monotonic()))
 child.terminate();child.wait()
 (p/'{side}.done').write_text('stop/disable/disconnect complete (stub)')
 raise SystemExit(0)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
while True:time.sleep(.01)
'''
        f=scripts/name;f.write_text(code);f.chmod(0o755)
    # Acquire an actual controlling PTY. Closing its master delivers kernel
    # SIGHUP, unlike merely closing a subprocess stdout pipe.
    harness='''import os,fcntl,termios,json,sys
from pathlib import Path
from teleop_runtime.full_launcher import FullLauncher
os.setsid()
if sys.argv[3]=='terminal_close':fcntl.ioctl(0,termios.TIOCSCTTY,0)
p=Path(sys.argv[2])
raise SystemExit(FullLauncher(sys.argv[1],p,json.loads(p.read_text())).run())
'''
    master,slave=pty.openpty()
    process=subprocess.Popen([sys.executable,'-c',harness,str(root),str(p),shutdown],stdin=slave,stdout=slave,stderr=slave)
    os.close(slave)
    try:
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and len(list(tmp_path.glob('*.ready')))<2:time.sleep(.01)
        assert len(list(tmp_path.glob('*.ready')))==2
        assert all(int(f.read_text())!=process.pid for f in tmp_path.glob('*.ready'))
        worker_pids=[int(f.read_text()) for f in tmp_path.glob('*.pid')]
        if shutdown=='terminal_close':os.close(master);master=None
        else:process.send_signal(getattr(signal,shutdown))
        assert process.wait(timeout=10)==0
        assert all((tmp_path/f'{s}.done').exists() for s in ('hands','arms'))
        assert float((tmp_path/'hands.stop').read_text())<=float((tmp_path/'arms.stop').read_text())
        assert all(not Path(f'/proc/{pid}').exists() for pid in worker_pids)
        assert not p.with_name('launcher-exit.sock').exists()
    finally:
        if process.poll() is None:process.send_signal(signal.SIGINT);process.wait(timeout=28)
        if master is not None:os.close(master)


@pytest.mark.parametrize('selection,active_other', [('left',False),('both',False),('left',True)])
def test_manus_explicit_open_uses_original_sender_and_keeps_permits_off(
        tmp_path, monkeypatch, isolated_guard_paths, selection, active_other):
    rclpy = pytest.importorskip('rclpy')
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Pose, PoseArray
    from adapters.litchibot import validation_driver
    from adapters.litchibot.test_open_hand import START
    import sharpa_driver.node as original
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    path, cfg = config_file(tmp_path, 'manus', 'fake')
    sdk = Mock(side_effect=AssertionError('Physical SDK must not be loaded'))
    monkeypatch.setattr(SharpaSdkHand, '_load_sdk', sdk)
    monkeypatch.setattr(validation_driver, 'ensure_clear_graph', Mock())
    hands = []
    def create(configuration, **kwargs):
        assert kwargs['use_fake_hardware'] is True
        hand = MockSharpaHand(configuration)
        hand.send_action = Mock(wraps=hand.send_action)
        hands.append(hand)
        return hand
    monkeypatch.setattr(original, 'create_hand', create)
    def exercise(node):
        runtime = node.full
        def observe():
            for side in ('left', 'right'):
                key = PoseArray();key.header.stamp = node.get_clock().now().to_msg()
                key.poses = [Pose() for _ in range(25)]
                runtime.keypoint(side, key)
                msg = JointState();msg.header = key.header
                msg.name = list(JOINT_NAMES);msg.position = [0.0]*22
                runtime.manus(side, msg)
        for _ in range(30):observe()
        assert node.latch.phase == 'READY', node.latch.reason
        sequence = 0
        def heartbeat():
            nonlocal sequence
            sequence += 1
            runtime.dispatch(dict(command='heartbeat', sequence=sequence, monotonic_ns=time.monotonic_ns()))
        heartbeat()
        # Starting from a nonzero measured pose must use the same original SDK
        # send_action path as teleop; publishing telemetry alone cannot pass.
        if active_other:
            runtime.dispatch(dict(command='engage_hand', arguments=dict(side='right')))
            heartbeat();observe()
        for hand in hands:
            if hand.config.side.value == 'left' or selection == 'both' or not active_other:
                hand._position = tuple(START)
        result = runtime.dispatch(dict(command='open_hand', arguments=dict(side=selection)))
        selected = ('left','right') if selection == 'both' else ('left',)
        assert all(result['hands'][s]['state'] == 'OPENING' and not result['hands'][s]['permit'] for s in selected)
        with pytest.raises(ValueError, match='finish'):
            runtime.dispatch(dict(command='engage_hand', arguments=dict(side='left')))
        end = time.monotonic() + 4
        while runtime.opening.operations and time.monotonic() < end:
            heartbeat();observe();runtime.tick();time.sleep(.02)
        assert not runtime.opening.operations
        assert node.latch.phase != 'FAULT', node.latch.reason
        assert not node.latch.requested['left']
        assert node.latch.requested['right'] is active_other
        for hand in hands:
            if hand.config.side.value in selected:
                assert hand.send_action.call_count > 20
                np.testing.assert_array_equal(hand.read_joint_state().position, np.zeros(22))
            elif active_other:
                assert hand.send_action.call_count > 20
                np.testing.assert_array_equal(hand.read_joint_state().position, np.zeros(22))
            else:
                hand.send_action.assert_not_called()
                np.testing.assert_array_equal(hand.read_joint_state().position, START)
        sent = node.validation_send_count
        left_sent = hands[0].send_action.call_count
        # Continued glove traffic after completion must not re-enable or send.
        for _ in range(3):heartbeat();observe();runtime.tick()
        assert hands[0].send_action.call_count == left_sent
        if not active_other:assert node.validation_send_count == sent
        heartbeat();runtime.dispatch(dict(command='engage_hand', arguments=dict(side='left')))
        heartbeat();observe()
        assert node.latch.requested['left'] and node.latch.requested['right'] is active_other
        assert hands[0].send_action.call_count > left_sent
    monkeypatch.setattr(rclpy, 'spin', exercise)
    validation_driver.main(['--full-config', str(path), '--ros-args',
        '-p', 'use_fake_hardware:=true', '-p', 'enable_left:=true', '-p', 'enable_right:=true',
        '-p', 'left_serial:=fake-left', '-p', 'right_serial:=fake-right',
        '-p', 'stop_on_command_timeout_s:=0.0'])
    sdk.assert_not_called()
