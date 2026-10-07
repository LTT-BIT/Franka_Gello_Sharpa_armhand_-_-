"""Normal session lifecycle through original mock driver; no physical SDK."""
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from adapters.litchibot.test_full_teleop import config_file,isolated_guard_paths
from adapters.litchibot.test_validation_safety import solved_packet


@pytest.mark.parametrize('terminal_hangup',[False,True])
def test_normal_stop_idle_restart_and_shutdown(tmp_path,monkeypatch,isolated_guard_paths,terminal_hangup):
    rclpy=pytest.importorskip('rclpy')
    import sharpa_driver.node as original
    from std_msgs.msg import String
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    from adapters.litchibot import validation_driver
    p,cfg=config_file(tmp_path);hands=[]
    forbidden=Mock(side_effect=AssertionError('Physical SDK forbidden'))
    monkeypatch.setattr(SharpaSdkHand,'_load_sdk',forbidden)
    def create(config,**kwargs):
        hand=MockSharpaHand(config)
        for method in ('connect','stop','disconnect','send_action'):
            setattr(hand,method,Mock(wraps=getattr(hand,method)))
        hand._ensure_enabled=Mock(side_effect=AssertionError('Normal Start must not re-enable'))
        hands.append(hand);return hand
    monkeypatch.setattr(original,'create_hand',create)
    def exercise(node):
        node.get_publishers_info_by_topic=lambda topic:[SimpleNamespace(node_name='litchibot_command_bridge' if topic.startswith('/teleop/') else 'sharpa_driver')]
        node.get_node_names_and_namespaces=lambda:[('sharpa_driver','/')]
        r=node.full;seq=dict(left=0,right=0)
        assert node.get_parameter('stop_on_command_timeout_s').value==0.0
        assert not hasattr(node,'native_enable') and not node.needs_enable
        assert all(h.is_connected and not h._stopped and h.connect.call_count==1 for h in hands)
        assert all(r.status()['hands'][s]['state']=='READY' for s in seq)
        def feed(side,value):
            seq[side]+=1;packet=solved_packet(side,seq[side]);packet['positions_rad']=[0.0]*22
            packet['positions_rad'][18]=value;packet['source_received_monotonic_ns']=time.monotonic_ns()
            msg=String();msg.data=json.dumps(packet);r.litchibot(side,msg)
        for side in seq:r.request(side,True);feed(side,.1)
        assert all(r.status()['hands'][s]['state']=='ACTIVE' for s in seq)
        r.request('left',False);before=hands[0].send_action.call_count
        start=time.monotonic()
        while time.monotonic()-start<.35:
            feed('left',.3);feed('right',.2);node._publish_state();node.service_guard();time.sleep(1/30)
        assert time.monotonic()-start>.20
        assert hands[0].send_action.call_count==before
        assert r.status()['hands']['left']['state']=='READY'
        assert r.status()['hands']['right']['state']=='ACTIVE'
        assert all(h.is_connected and not h._stopped and h.stop.call_count==0 and h.disconnect.call_count==0 for h in hands)
        assert not any(node._timeout_stopped.values()) and not node.needs_enable
        r.request('left',True);feed('left',.4)
        assert hands[0].send_action.call_count==before+1
        assert hands[0].read_joint_state().position[18]==.4
        assert all(h._ensure_enabled.call_count==0 and h.connect.call_count==1 for h in hands)
        if terminal_hangup:
            import signal
            signal.raise_signal(signal.SIGHUP)
            from rclpy.executors import ExternalShutdownException
            with pytest.raises(ExternalShutdownException):node.service_guard()
            assert not any(node.latch.requested.values())
            # Context shutdown belongs to main's cleanup after callback unwind.
            assert rclpy.ok()
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--full-config',str(p),'--ros-args','-p','use_fake_hardware:=true','-p','enable_left:=true','-p','enable_right:=true',
                            '-p','left_serial:=fake-left','-p','right_serial:=fake-right','-p','stop_on_command_timeout_s:=0.0'])
    assert len(hands)==2
    for hand in hands:
        assert hand.stop.call_count==1 and hand.disconnect.call_count==1
        assert not hand.is_connected and hand._stopped
        hand._ensure_enabled.assert_not_called()
    forbidden.assert_not_called()


@pytest.mark.parametrize('source,mode,expected',[
    ('litchibot','normal','0.0'),('litchibot','fake','0.0'),
    ('litchibot','supervised_hardware_validation','0.2'),('manus','normal','0.2'),('manus','fake','0.2')])
def test_real_launch_selects_only_normal_litchibot_timeout(monkeypatch,source,mode,expected):
    pytest.importorskip('launch_ros')
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from adapters.litchibot import full_control
    root=Path(__file__).resolve().parents[2]
    path=root/'portable_deps/litchi_hardware/ros2/src/manus_sharpa_teleop/manus_sharpa_teleop/launch/teleop.launch.py'
    spec=importlib.util.spec_from_file_location('audited_teleop_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(full_control,'load_full_config',lambda p:dict(source=source,mode=mode,profile='test',policy={'serials':dict(left='L',right='R')}))
    context=LaunchContext();context.launch_configurations.update(
        hand_source=source,dry_run='false',full_teleop='true',full_config='unused',
        validation_driver_script=str(root/'adapters/litchibot/validation_driver.py'),
        litchibot_bridge_script=str(root/'adapters/litchibot/terminal2_bridge.py'),
        enable_left='true',enable_right='true',litchibot_profile='test',speed_coefficient='0.3',current_coefficient='0.6')
    module.validate_source(context)
    assert context.launch_configurations['full_command_timeout']==expected
