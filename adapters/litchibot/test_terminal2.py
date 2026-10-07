"""Production Terminal 2 regression. ROS tests require the existing ROS Python."""
import ast
import hashlib
import importlib.util
from importlib.machinery import SourceFileLoader
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
import threading
from unittest.mock import Mock

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
LAUNCH=ROOT/'portable_deps/litchi_hardware/ros2/src/manus_sharpa_teleop/manus_sharpa_teleop/launch/teleop.launch.py'


def test_original_terminal1_terminal3_sender_calibration_and_v4_unchanged():
    protected=json.loads((ROOT/'docs/litchibot/baselines/protected_files.json').read_text())
    for name, expected in protected.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name


def test_wrapper_manus_default_explicit_and_original_have_same_exec(tmp_path):
    root=tmp_path/'repo'
    scripts=root/'ops/run';scripts.mkdir(parents=True)
    (root/'ops/lib').mkdir()
    shutil.copy(ROOT/'ops/lib/deployment_env.sh',root/'ops/lib/deployment_env.sh')
    (root/'config').mkdir();(root/'config/cyclonedds.xml').write_text('<CycloneDDS/>')
    litchi=root/'portable';(litchi/'ros2/install').mkdir(parents=True)
    (litchi/'ros2/install/setup.bash').write_text('')
    (root/'docker').mkdir()
    (root/'docker/.env').write_text(f'LITCHI_REPO={litchi}\nHOUSEKEEPING_CPUSET=0\nTELEOP_ROS_DOMAIN_ID=7\n')
    for name, source in [('original.sh',ROOT/'docs/litchibot/baselines/run_sharpa_hands_cyclonedds.sh.txt'),
                         ('updated.sh',ROOT/'ops/run/run_sharpa_hands_cyclonedds.sh')]:
        shutil.copy(source,scripts/name)
    bins=tmp_path/'bin';bins.mkdir()
    (bins/'taskset').write_text('#!/bin/bash\nshift 2\nexec "$@"\n')
    (bins/'pixi').write_text('#!/usr/bin/python3\nimport json,os,sys\nprint(json.dumps({"argv":sys.argv[1:],"env":{k:os.environ[k] for k in ("RMW_IMPLEMENTATION","ROS_DOMAIN_ID","CYCLONEDDS_URI","ROS_LOCALHOST_ONLY")}}))\n')
    for path in bins.iterdir():path.chmod(0o755)
    env=os.environ.copy();env['PATH']=str(bins)+':'+env['PATH'];env.pop('LITCHI_REPO',None);env.pop('HOUSEKEEPING_CPUSET',None)
    def run(name,args):
        completed=subprocess.run(['bash',str(scripts/name),*args],env=env,capture_output=True,text=True)
        assert completed.returncode==0,completed.stderr
        return json.loads(completed.stdout.splitlines()[-1])
    extra=['publish_tactile:=true','retarget_backend:=v4']
    original=run('original.sh',extra)
    assert run('updated.sh',extra)==original
    assert run('updated.sh',['--hand-source','manus',*extra])==original
    assert run('updated.sh',['hand_source:=manus',*extra])==original
    litchi_result=run('updated.sh',['--hand-source','litchibot','--dry-run'])
    assert 'hand_source:=litchibot' in litchi_result['argv']
    assert 'dry_run:=true' in litchi_result['argv']
    assert litchi_result['env']==original['env']
    blocked=subprocess.run(['bash',str(scripts/'updated.sh'),'--hand-source','litchibot'],env=env,capture_output=True,text=True)
    assert blocked.returncode==2 and 'BLOCKER FOR REAL HARDWARE MOTION' in blocked.stderr


def load_launch(path=LAUNCH):
    pytest.importorskip('launch_ros')
    spec=importlib.util.spec_from_loader('production_launch_test',SourceFileLoader('production_launch_test',str(path)))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def context_for(description, overrides=None):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    context=LaunchContext()
    for entity in description.entities:
        if isinstance(entity,DeclareLaunchArgument):entity.execute(context)
    context.launch_configurations.update(overrides or {})
    return context


def test_manus_launch_arguments_and_algorithm_nodes_preserved():
    current=load_launch()
    old=load_launch(ROOT/'docs/litchibot/baselines/teleop.launch.py.txt')
    # .txt does not have an import loader; compile into a module namespace directly.
    from launch.actions import DeclareLaunchArgument
    old_description=old.generate_launch_description()
    new_description=current.generate_launch_description()
    a=context_for(old_description);b=context_for(new_description)
    for k,value in a.launch_configurations.items():assert b.launch_configurations[k]==value,k
    assert b.launch_configurations['hand_source']=='manus'
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    old_nodes=[n for n in old_description.entities if isinstance(n,Node)]
    new_nodes=[n for n in new_description.entities if isinstance(n,Node)]
    assert len(old_nodes)==len(new_nodes)==3
    for overrides in ({},{'use_fake_hardware':'True'},
                      {'enable_left':'false','publish_tactile':'false','left_sharpa_serial':'TEST'}):
        old_description=old.generate_launch_description();new_description=current.generate_launch_description()
        old_nodes=[n for n in old_description.entities if isinstance(n,Node)]
        new_nodes=[n for n in new_description.entities if isinstance(n,Node)]
        a=context_for(old_description,overrides);b=context_for(new_description,overrides)
        for original,updated in zip(old_nodes,new_nodes):
            assert original.node_package==updated.node_package
            assert evaluate_parameters(a,original._Node__parameters)==evaluate_parameters(b,updated._Node__parameters)


@pytest.mark.parametrize('source,dry,expected',[('manus','false',('manus_driver','manus_sharpa_teleop','sharpa_driver')),
    ('litchibot','true',('sharpa_driver',))])
def test_launch_graph_one_driver_and_no_manus_in_litchibot(source,dry,expected):
    module=load_launch()
    from launch_ros.actions import Node
    from launch.actions import ExecuteProcess
    from launch.utilities import perform_substitutions
    description=module.generate_launch_description()
    context=context_for(description,{'hand_source':source,'dry_run':dry,'litchibot_bridge_script':'bridge.py'})
    module.validate_source(context)
    selected=[n for n in description.entities if isinstance(n,Node) and (n.condition is None or n.condition.evaluate(context))]
    packages=tuple(n.node_package if isinstance(n.node_package,str)
                   else perform_substitutions(context,n.node_package) for n in selected)
    assert packages==expected
    assert packages.count('sharpa_driver')==1
    processes=[n for n in description.entities if isinstance(n,ExecuteProcess) and not isinstance(n,Node)
               and (n.condition is None or n.condition.evaluate(context))]
    assert len(processes)==(source=='litchibot')
    from launch_ros.utilities import evaluate_parameters
    driver=selected[-1]
    parameters=evaluate_parameters(context,driver._Node__parameters)
    assert parameters[0]['use_fake_hardware']==(dry=='true')
    assert parameters[0]['publish_tactile'] is True


def test_launch_blocks_direct_non_dry_litchibot():
    module=load_launch();description=module.generate_launch_description()
    context=context_for(description,{'hand_source':'litchibot','dry_run':'false'})
    with pytest.raises(RuntimeError,match='BLOCKER'):module.validate_source(context)


def test_worker_bad_stream_is_contained_and_latest_per_side_bounded():
    from adapters.litchibot.terminal2_bridge import WorkerInbox
    inbox=WorkerInbox()
    for bad in ('not json','[]','{}',json.dumps({'schema':'litchibot.sharpa_target.v1','side':'left','dry_run':False})):
        inbox.feed(bad)
    assert inbox.errors==4
    for i in range(10):
        inbox.feed(json.dumps({'schema':'litchibot.sharpa_target.v1','side':'left','dry_run':True,'sequence':i}))
    assert inbox.take()['left']['sequence']==9
    assert inbox.take()=={}


@pytest.mark.parametrize('code,exit_code,malformed',[
    ("print('not json'); raise SystemExit(3)",3,1),
    ("pass",0,0),
    ("import sys; print('USBError: [Errno 13] Access denied (insufficient permissions)',file=sys.stderr); raise SystemExit(1)",1,0),
])
def test_bridge_worker_error_or_disconnect_exits_cleanly_without_commands(tmp_path,monkeypatch,code,exit_code,malformed,capfd):
    pytest.importorskip('rclpy')
    from adapters.litchibot import terminal2_bridge
    original_popen=subprocess.Popen
    # An actual child process emulates EOF/error; no glove or Sharpa SDK is loaded.
    monkeypatch.setattr(terminal2_bridge.subprocess,'Popen',
        lambda command,**kwargs:original_popen([sys.executable,'-c',code],**kwargs))
    args=['--dry-run','--worker-python',sys.executable,
        '--profile','test','--sdk-root','unused','--data-root','unused','--log-dir',str(tmp_path)]
    if malformed:
        with pytest.raises(RuntimeError,match='DIAGNOSTIC LATCHED STOP'):terminal2_bridge.main(args)
    else:assert terminal2_bridge.main(args)==exit_code
    summary=json.loads((tmp_path/'bridge.jsonl').read_text().splitlines()[-1])
    assert summary['published']=={'left':0,'right':0}
    assert summary['worker_exit_code']==exit_code
    assert summary['malformed_worker_lines']==malformed
    assert summary['hardware_send_count']==0
    if 'USBError' in code:
        output=capfd.readouterr().err
        assert '[Errno 13] Access denied' in output
        assert 'worker_stderr.log' in output
        assert output.count('"severity": "HARD_FAULT"')==1


def test_ros_command_matches_v4_final_publish_contract():
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseArray,Pose
    from litchi_hardware.core.types import HandSide
    from manus_sharpa_teleop.retarget_node import RetargetNode
    from manus_sharpa_teleop.v4_retargeter import SharpaV4Retargeter
    from litchi_hardware.hardware.sharpa.schema import SHARPA_WAVE_JOINT_SCHEMA
    from adapters.litchibot.terminal2_bridge import to_joint_state
    from adapters.litchibot.retarget import JOINT_NAMES,JOINT_LIMITS
    assert JOINT_NAMES==SHARPA_WAVE_JOINT_SCHEMA.names
    np.testing.assert_allclose(JOINT_LIMITS,SHARPA_WAVE_JOINT_SCHEMA.limits)
    assert SHARPA_WAVE_JOINT_SCHEMA.unit=='rad'
    retargeter=SharpaV4Retargeter.__new__(SharpaV4Retargeter)
    retargeter._lock=threading.Lock()
    retargeter._request=lambda request:{'joint_position':[0.1]*22}
    for side in HandSide:
        publisher=Mock()
        node=SimpleNamespace(_max_input_age_s=.25,_retargeter=retargeter,
            _side_publishers={side:publisher},get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10**10)),
            _warn_throttled=Mock())
        msg=PoseArray();msg.header.stamp.sec=10
        msg.poses=[Pose() for _ in range(25)]
        RetargetNode._on_keypoints(node,side,msg)
        original=publisher.publish.call_args.args[0]
        packet={'side':side.value,'source_received_monotonic_ns':10**9,'session_id':'test','sequence':1}
        new=to_joint_state(packet,[0.1]*22,10**10,10**9)
        assert list(new.name)==list(original.name)
        assert list(new.position)==list(original.position)
        assert new.header.stamp==original.header.stamp
        assert ':'.join(('litchibot',side.value)) in new.header.frame_id


def test_shared_fake_driver_never_initializes_physical_sdk(monkeypatch):
    rclpy=pytest.importorskip('rclpy')
    from sharpa_driver.node import SharpaDriverNode
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from sensor_msgs.msg import JointState
    load=Mock(side_effect=AssertionError('Physical SDK was initialized'))
    monkeypatch.setattr(SharpaSdkHand,'_load_sdk',load)
    rclpy.init(args=['--ros-args','-p','use_fake_hardware:=true','-p','enable_left:=true','-p','enable_right:=true'])
    node=None
    try:
        node=SharpaDriverNode()
        for side,hand in node._hands.items():
            assert isinstance(hand,MockSharpaHand)
            msg=JointState();msg.name=list(hand.joint_names) if hasattr(hand,'joint_names') else []
            msg.position=[0.1]*22
            node._on_command(side,msg)
            assert hand.read_joint_state().position==(0.1,)*22
        assert len(node._tactile_publishers)==10
        node._publish_state();node._publish_tactile()
        load.assert_not_called()
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


def test_terminal3_topic_contract_unchanged():
    import yaml
    config=yaml.safe_load((ROOT/'data_collection/config/record_gello_sharpa_tactile.yaml').read_text())
    topics=config['teleop_data_collector']['ros__parameters']['topics']
    hand={v['topic']:v['type'] for v in topics.values() if v['topic'].startswith('/sharpa/')}
    for side in ('left','right'):
        for suffix in ('command','joint_states'):assert hand[f'/sharpa/{side}/{suffix}']=='sensor_msgs/msg/JointState'
        for finger in ('thumb','index','middle','ring','pinky'):
            assert hand[f'/sharpa/{side}/tactile/{finger}/wrench']=='geometry_msgs/msg/WrenchStamped'
            assert hand[f'/sharpa/{side}/tactile/{finger}/deformation']=='sensor_msgs/msg/Image'
    assert len(hand)==24


def test_deprecated_parallel_hardware_entrance_refuses_even_enable_flag():
    done=subprocess.run([str(ROOT/'ops/run/run_litchibot_sharpa_output.sh'),'--enable-output'],capture_output=True,text=True)
    assert done.returncode==2 and 'DEPRECATED AND DISABLED' in done.stderr


def test_normal_bridge_restarts_acquisition_and_never_requires_validation_proof(tmp_path,monkeypatch,capfd):
    import time
    pytest.importorskip('rclpy')
    import rclpy
    from adapters.litchibot import terminal2_bridge as bridge
    from adapters.litchibot.test_full_teleop import config_file
    from adapters.litchibot.test_validation_safety import solved_packet
    cfg,data=config_file(tmp_path)
    from adapters.litchibot.full_control import load_full_config
    from adapters.litchibot.validation_safety import control_paths
    ignored_status=control_paths(load_full_config(cfg)['policy'])[1]
    ignored_status.write_text(json.dumps({'session':data['token'][:16],'unix_s':0,'phase':'FAULT','reason':'old validation status'}))
    original_popen=subprocess.Popen;starts=[];targets=[]
    template=solved_packet();template['positions_rad']=[0.0]*22
    worker_code=f'''import json,time
row={template!r}
for seq in range(1,100):
 for side in ('left','right'):
  row.update(side=side,sequence=seq,source_received_monotonic_ns=time.monotonic_ns())
  print(json.dumps(row),flush=True)
 time.sleep(.02)
'''
    def popen(command,**kw):
        starts.append(command)
        code="import sys; print('temporary USB unavailable',file=sys.stderr); sys.exit(1)" if len(starts)==1 else worker_code
        return original_popen([sys.executable,'-c',code],**kw)
    monkeypatch.setattr(bridge.subprocess,'Popen',popen)
    original_joint=bridge.to_joint_state
    def target(packet,*a,**kw):targets.append(packet);return original_joint(packet,*a,**kw)
    monkeypatch.setattr(bridge,'to_joint_state',target)
    original_spin=rclpy.spin_once
    deadline=time.monotonic()+5
    def spin(*a,**kw):
        if len(targets)>=2:raise KeyboardInterrupt
        assert time.monotonic()<deadline,'acquisition did not recover'
        return original_spin(*a,**kw)
    monkeypatch.setattr(rclpy,'spin_once',spin)
    try:
        assert bridge.main(['--managed-full','--full-config',str(cfg),'--worker-python',sys.executable,
                            '--profile',data['profile'],'--sdk-root','unused','--data-root','unused','--log-dir',str(tmp_path/'logs')])==0
    finally:ignored_status.unlink(missing_ok=True)
    assert len(starts)==2 and min(p['valid_glove_joint_count'] for p in targets)==20
    summary=json.loads((tmp_path/'logs/bridge.jsonl').read_text().splitlines()[-1])
    assert sum(summary['published'].values())>=2 and summary['guard_confirmed'] is False
    assert summary['hardware_send_count']==0
    assert '"severity": "HARD_FAULT"' not in capfd.readouterr().err
