"""Native cleanup evidence, including a blocked ROS feedback callback."""
import json
import socket
import threading
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from adapters.litchibot.normal_shutdown import NormalShutdown
from adapters.litchibot.test_full_teleop import config_file,isolated_guard_paths


def test_both_disables_finish_before_either_disconnect(tmp_path):
    disabled={};barrier=threading.Barrier(2);calls=[]
    class Hand:
        is_connected=True;_stopped=False
        def __init__(self,s):self.s=s
        def stop(self):
            barrier.wait(timeout=1);self._stopped=True;disabled[self.s]=True;calls.append(('stop',self.s))
        def disconnect(self):
            assert len(disabled)==2
            calls.append(('disconnect',self.s));self.is_connected=False
    # Hashable original side enum is replaced by tiny enum in this pure test.
    from enum import Enum
    Side=Enum('Side',{'left':'left','right':'right'})
    c=NormalShutdown({s:Hand(s.name) for s in Side},tmp_path/'session.json','a'*64,lambda:None)
    c.finish()
    report=json.loads(c.report_path.read_text())
    assert report['state']=='COMPLETE'
    assert all(v['disabled_verified'] and v['disconnected_verified'] for v in report['hands'].values())
    assert [k for k,s in calls][:2]==['stop','stop']


def test_native_disable_failure_is_not_reported_as_clean_exit(tmp_path):
    from enum import Enum
    Side=Enum('Side',{'left':'left','right':'right'})
    class Hand:
        is_connected=True;_stopped=False
        def stop(self):self._stopped=True
        def _read_setting(self,k):return True  # Device still enabled despite API returning.
        def disconnect(self):self.is_connected=False
    c=NormalShutdown({s:Hand() for s in Side},tmp_path/'session.json','b'*64,lambda:None)
    with pytest.raises(RuntimeError,match='shutdown failed'):c.finish()
    report=json.loads(c.report_path.read_text());assert report['state']=='FAILED'
    assert all(v['disabled_verified'] is False and 'stop_error' in v for v in report['hands'].values())


def test_shutdown_reaches_original_mock_while_ros_feedback_callback_is_blocked(tmp_path,monkeypatch,isolated_guard_paths):
    rclpy=pytest.importorskip('rclpy')
    import sharpa_driver.node as original
    from litchi_hardware.hardware.sharpa.mock import MockSharpaHand
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    from adapters.litchibot import validation_driver
    p,cfg=config_file(tmp_path);hands=[];blocked=threading.Event();released=threading.Event();errors=[]
    forbidden=Mock(side_effect=AssertionError('Physical SDK forbidden'));monkeypatch.setattr(SharpaSdkHand,'_load_sdk',forbidden)
    def create(config,**kwargs):
        h=MockSharpaHand(config)
        h.stop=Mock(wraps=h.stop)
        disconnect=h.disconnect
        def close():disconnect();released.set()
        h.disconnect=Mock(side_effect=close);hands.append(h);return h
    monkeypatch.setattr(original,'create_hand',create)
    def exercise(node):
        original_read=hands[0].read_joint_state
        def stalled():blocked.set();assert released.wait(2);return original_read()
        hands[0].read_joint_state=stalled
        def request():
            try:
                assert blocked.wait(1)
                with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
                    s.sendto(json.dumps(dict(command='shutdown',token=cfg['token'])).encode(),str(tmp_path/'hand-shutdown.sock'))
            except Exception as e:errors.append(str(e));released.set()
        sender=threading.Thread(target=request,daemon=True);sender.start()
        node._publish_state()  # Main ROS thread stalls here until native cleanup.
        sender.join(1);assert not errors and released.is_set()
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--full-config',str(p),'--ros-args','-p','use_fake_hardware:=true','-p','enable_left:=true','-p','enable_right:=true',
                            '-p','left_serial:=fake-left','-p','right_serial:=fake-right','-p','stop_on_command_timeout_s:=0.0'])
    assert all(h.stop.call_count==1 and h.disconnect.call_count==1 and not h.is_connected and h._stopped for h in hands)
    report=json.loads((tmp_path/'hand-shutdown.json').read_text());assert report['state']=='COMPLETE'
    forbidden.assert_not_called()


@pytest.mark.parametrize('failure',[False,True])
def test_launcher_waits_for_native_evidence_before_wrapper_teardown(tmp_path,failure):
    from enum import Enum
    from teleop_runtime.full_launcher import FullLauncher
    Side=Enum('Side',{'left':'left','right':'right'})
    p,cfg=config_file(tmp_path)
    class Hand:
        is_connected=True;_stopped=False
        def stop(self):
            if failure:raise RuntimeError('Mock disable rejected')
            self._stopped=True
        def disconnect(self):self.is_connected=False
    c=NormalShutdown({s:Hand() for s in Side},p,cfg['token'],lambda:None)
    launcher=FullLauncher(tmp_path,p,cfg)
    try:
        error=launcher.shutdown_normal_driver()
        assert (error is not None)==failure and c.done.wait(1)
        assert json.loads(c.report_path.read_text())['state']==('FAILED' if failure else 'COMPLETE')
        if failure:
            with pytest.raises(RuntimeError):c.finish()
        else:c.finish()
    finally:
        if not c.closed.is_set():
            try:c.finish()
            except RuntimeError:pass


def test_blocked_disconnect_does_not_prevent_opposite_disable(tmp_path):
    from enum import Enum
    import time
    Side=Enum('Side',{'left':'left','right':'right'});release=threading.Event();entered=threading.Event()
    class Hand:
        is_connected=True;_stopped=False
        def __init__(self,s):self.s=s
        def stop(self):self._stopped=True
        def disconnect(self):
            if self.s==Side.left:entered.set();assert release.wait(2)
            self.is_connected=False
    c=NormalShutdown({s:Hand(s) for s in Side},tmp_path/'session.json','c'*64,lambda:None)
    try:
        c.request();assert entered.wait(1)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            report=json.loads(c.report_path.read_text())
            if report['hands']['right'].get('disconnected_verified'):break
            time.sleep(.01)
        assert report['state']=='STOPPING' and not c.done.is_set()
        assert all(v['disabled_verified'] for v in report['hands'].values())
        assert report['hands']['right']['disconnected_verified']
    finally:release.set();c.finish()
