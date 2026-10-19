"""Real Qt/JSON-TCP integration. No ROS or hardware; authority is MotionLatch."""
import json
import pytest
import os
import socket
import threading
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QSettings
from adapters.litchibot.full_control import HandControlServer
from adapters.litchibot.validation_safety import MotionLatch
from adapters.litchibot.normal_control import NormalHandAuthority
from adapters.litchibot.retarget import JOINT_NAMES
from apps.operator_gui.operator_gui import OperatorWindow
from pico_bimanual_franka_teleop.control_server import OperatorConsole,OperatorControlServer


def port():
    with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]


def test_manus_acquisition_disables_start_without_changing_legacy_status(tmp_path,monkeypatch):
    app=QApplication.instance() or QApplication([])
    monkeypatch.delenv('TELEOP_FULL_CONFIG',raising=False)
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path))
    window=OperatorWindow('127.0.0.1',port(),collection_port=port())
    try:
        def status(allowed):
            return {'hands':{s:{'state':'DISABLED','permit':False,'can_engage':allowed}
                for s in ('left','right')}}
        window._apply_hand_status(status(False))
        assert all(not b.isEnabled() for b in window.hand_engage_buttons.values())
        window._apply_hand_status(status(True))
        assert all(b.isEnabled() for b in window.hand_engage_buttons.values())
        window._apply_hand_status({'hands':{s:{'state':'DISABLED','permit':False} for s in ('left','right')}})
        assert all(b.isEnabled() for b in window.hand_engage_buttons.values())
    finally:window.close();app.processEvents()


def test_gui_authority_independent_arms_and_disengage_all(tmp_path,monkeypatch):
    app=QApplication.instance() or QApplication([])
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path))
    control_port=port();token='c'*64
    cfg=tmp_path/'session.json';cfg.write_text(json.dumps({'control_port':control_port,'token':token,'mode':'normal'}))
    monkeypatch.setenv('TELEOP_FULL_CONFIG',str(cfg))
    latch=NormalHandAuthority({'session_token':token,'expires_unix_s':0},time.monotonic(),gui_controlled=True,validation_only=False)
    server=HandControlServer(control_port,token);stop=threading.Event();commands=[];seq=0;source_pause={'left':False,'right':False}
    def status():return {'hands':{s:{'state':latch.side_state(s,time.monotonic()),'permit':latch.requested[s],'reason':latch.side_reasons[s]} for s in ('left','right')},'reason':latch.reason}
    def dispatch(req):
        commands.append(req['command'])
        now=time.monotonic()
        if req['command']=='heartbeat':
            latch.gui_packet(req,now)
        elif req['command'] in ('engage_hand','disengage_hand'):
            latch.request_hand(req['arguments']['side'],req['command']=='engage_hand',now)
        elif req['command']=='disengage_all':
            for s in ('left','right'):latch.request_hand(s,False,now)
        elif req['command']=='open_hand':raise ValueError('Open hand no validated policy')
        return status()
    def loop():
        nonlocal seq
        while not stop.is_set():
            seq+=1;now=time.monotonic()
            for s in ('left','right'):
                if latch.phase!='FAULT':
                    latch.note_source(s,now)
                    if not source_pause[s]:latch.valid_data(s,now)
            server.drain(dispatch,lambda:latch.pause_control('GUI connection lost'))
            latch.watch(time.monotonic(),time.time());time.sleep(.005)
    thread=threading.Thread(target=loop,daemon=True);thread.start()
    console=OperatorConsole();arm_port=port();arms=OperatorControlServer(('127.0.0.1',arm_port),console);arms.start()
    window=OperatorWindow('127.0.0.1',arm_port)
    def wait(predicate):
        end=time.monotonic()+3
        while time.monotonic()<end:
            app.processEvents()
            if predicate():return
            time.sleep(.005)
        raise AssertionError((window.hand_status,latch.reason,commands))
    try:
        wait(lambda:latch.phase=='READY' and window.connection_state=='connected' and window.hand_status.get('hands',{}).get('left',{}).get('state')=='READY')
        assert not latch.requested['left'] and not window.hand_client.held
        window.hand_engage_buttons['left'].click()
        wait(lambda:window.hand_status['hands']['left']['state']=='ACTIVE')
        assert not latch.requested['right']
        window.hand_engage_buttons['right'].click();wait(lambda:window.hand_status['hands']['right']['state']=='ACTIVE')
        window.hand_engage_buttons['left'].click();wait(lambda:window.hand_status['hands']['left']['state']=='READY')
        assert latch.requested['right']
        window._send('engage_arm',{'side':'left'});wait(lambda:console.snapshot()['active']['left'])
        window._send('disengage_all');wait(lambda:not any(latch.requested.values()) and not console.snapshot()['active']['left'])
        assert latch.phase=='READY'
        # Old arm status cannot overwrite hand authority.
        window._apply_status({'hand_active':{'left':True,'right':True}})
        assert not window.hand_engage_buttons['left'].isChecked()
        window._send('open_hand',{'side':'left'});wait(lambda:'no validated policy' in window.feedback.toPlainText())
        window.hand_engage_buttons['left'].click();wait(lambda:window.hand_status['hands']['left']['state']=='ACTIVE')
        window.hand_engage_buttons['right'].click();wait(lambda:window.hand_status['hands']['right']['state']=='ACTIVE')
        source_pause['left']=True;latch.pause_data('left','Temporary source gap')
        window._apply_hand_status(status())
        assert not window.hand_engage_buttons['left'].isEnabled()
        assert not window.hand_engage_buttons['left'].isChecked()
        assert 'OFFLINE' in window.hand_engage_buttons['left'].text()
        assert 'Temporary source gap' in window.feedback.toPlainText()
        assert window.hand_engage_buttons['right'].isChecked()
        assert window.hand_engage_buttons['right'].isEnabled()
        source_pause['left']=False
        wait(lambda:window.hand_status['hands']['left']['state']=='READY')
        window.hand_engage_buttons['left'].click()
        wait(lambda:window.hand_status['hands']['left']['state']=='ACTIVE')
        assert window.hand_status['hands']['right']['state']=='ACTIVE'
        assert window.hand_engage_buttons['left'].isEnabled()
        assert latch.requested['left']
        window._send('engage_arm',{'side':'left'});wait(lambda:console.snapshot()['active']['left'])
        window._send('hold_arm',{'side':'left','enabled':True})
        wait(lambda:console.poll_holds()['left'])
        assert all(latch.requested.values()) and latch.side_state('left',time.monotonic())=='ACTIVE'
        window._send('disengage_arm',{'side':'left'});wait(lambda:not console.snapshot()['active']['left'])
        latch.trip('Historical left pinky discontinuity')
        wait(lambda:window.hand_status['hands']['left']['state']=='OFFLINE')
        assert not window.hand_engage_buttons['left'].isEnabled()
        assert 'pinky' in window.feedback.toPlainText()
        assert console.snapshot()['active']['left'] is False
    finally:
        window.close();app.processEvents();stop.set();thread.join(1);server.close();arms.close()


def test_real_runtime_late_startup_heartbeat_and_new_gui_connection(tmp_path):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from enum import Enum
    from apps.operator_gui.hand_client import HandClient
    from adapters.litchibot.normal_runtime import NormalHandRuntime
    from adapters.litchibot.normal_control import BasicNormalTarget
    from adapters.litchibot.transport import TargetGate
    app=QApplication.instance() or QApplication([])
    token='d'*64;control_port=port();latch=NormalHandAuthority({'session_token':token},time.monotonic(),gui_controlled=True,validation_only=False)
    class Side(Enum):LEFT='left';RIGHT='right'
    hands={s:SimpleNamespace(stop=Mock(),read_joint_state=lambda:SimpleNamespace(position=[0]*22)) for s in Side}
    logs=[]
    node=SimpleNamespace(validation_send_count=0,_hands=hands,_has_received_command={s:False for s in Side},
                         _timeout_stopped={s:False for s in Side},needs_enable=set(),stop_validation=latch.trip,
                         get_logger=lambda:SimpleNamespace(warning=lambda line:logs.append(json.loads(line)),error=lambda line:logs.append(json.loads(line))))
    r=NormalHandRuntime.__new__(NormalHandRuntime);r.node=node;r.latch=latch;r.config={'mode':'fake','token':token};r._native_paused=set()
    r.raw=BasicNormalTarget();r.gate=r.raw;r.delta_path=tmp_path/'stats.json'
    r.last_gui_sequence=-1;r.control_sequence=0
    server=HandControlServer(control_port,token);stop=threading.Event();requests=[];first=True
    def dispatch(req):
        nonlocal first
        requests.append(req.copy())
        if first and req['command']=='heartbeat':
            first=False;req={**req,'monotonic_ns':req['monotonic_ns']-1_000_000_000}
        return r.dispatch(req)
    def loop():
        seq=0
        while not stop.is_set():
            seq+=1;now=time.monotonic()
            for side in ('left','right'):
                if latch.phase!='FAULT':
                    latch.note_source(side,now)
                    latch.valid_data(side,now)
            server.drain(dispatch,r.disconnected)
            latch.watch(time.monotonic(),time.time());time.sleep(.005)
    thread=threading.Thread(target=loop,daemon=True);thread.start();clients=[];statuses=[]
    def client():
        c=HandClient(control_port,token);c.status.connect(statuses.append);clients.append(c);return c
    def wait(predicate):
        end=time.monotonic()+3
        while time.monotonic()<end:
            app.processEvents()
            if predicate():return
            time.sleep(.003)
        raise AssertionError((latch.phase,latch.reason,requests))
    try:
        c=client();wait(lambda:latch.phase=='READY' and latch.sequence is not None and latch.sequence>=2)
        assert any(e['severity']=='WARNING' and 'heartbeat' in e['reason'] for e in logs)
        c.request('engage_hand',{'side':'left'});wait(lambda:latch.side_state('left',time.monotonic())=='ACTIVE')
        c.request('disengage_all');wait(lambda:not any(latch.requested.values()))
        old_generation=latch.connection;c.close()
        wait(lambda:server.owner is None)
        newer=client()
        wait(lambda:latch.connection!=old_generation and latch.sequence is not None)
        assert latch.phase!='FAULT' and not any(latch.requested.values())
        newer.request('engage_hand',{'side':'right'});wait(lambda:latch.side_state('right',time.monotonic())=='ACTIVE')
        assert not latch.requested['left']
        newer.request('disengage_all');wait(lambda:not any(latch.requested.values()))
    finally:
        for c in clients:c.close()
        stop.set();thread.join(1);server.close();app.processEvents()


@pytest.mark.parametrize('delay_first',[False,True])
def test_normal_delayed_heartbeat_response_does_not_disconnect_or_revoke(tmp_path,delay_first):
    from apps.operator_gui.hand_client import HandClient
    app=QApplication.instance() or QApplication([])
    token='e'*64;server=HandControlServer(port(),token);server.reply_timeout=None
    stop=threading.Event();delay=threading.Event();done=threading.Event();requests=[]
    if delay_first:delay.set()
    def dispatch(req):
        requests.append(req)
        if delay.is_set() and req['command']=='heartbeat':
            delay.clear();time.sleep(.35);done.set()
        return {'normal_forwarding':True,'hands':{'left':{'state':'ACTIVE','permit':True},'right':{'state':'READY','permit':False}}}
    def loop():
        while not stop.is_set():server.drain(dispatch,lambda:None);time.sleep(.003)
    thread=threading.Thread(target=loop,daemon=True);thread.start()
    client=HandClient(server.server_address[1],token,normal_forwarding=True);statuses=[];client.status.connect(statuses.append)
    def wait(predicate):
        end=time.monotonic()+3
        while time.monotonic()<end:
            app.processEvents()
            if predicate():return
            time.sleep(.003)
        raise AssertionError((statuses,requests))
    try:
        wait(lambda:client.last_status.get('normal_forwarding'))
        generation=server.generation;delay.set();wait(done.is_set)
        wait(lambda:client.last_response is not None and time.monotonic()-client.last_response<.1)
        assert server.generation==generation and server.owner is not None
        assert all(s['hands']['left']['state']=='ACTIVE' and s['hands']['left']['permit'] for s in statuses)
    finally:
        client.close();app.processEvents();stop.set();thread.join(1);server.close()


def test_gui_window_close_notifies_launcher_without_supervisor_connection(tmp_path,monkeypatch):
    app=QApplication.instance() or QApplication([])
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path))
    cfg=tmp_path/'session.json';token='e'*64
    cfg.write_text(json.dumps(dict(source='litchibot',mode='normal',token=token,control_port=port())))
    monkeypatch.setenv('TELEOP_FULL_CONFIG',str(cfg))
    path=tmp_path/'exit.sock';monkeypatch.setenv('TELEOP_FULL_EXIT_SOCKET',str(path))
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as receiver:
        receiver.bind(str(path));receiver.settimeout(1)
        window=OperatorWindow('127.0.0.1',port(),collection_port=port())
        window.show();app.processEvents()
        try:
            window.close()
            assert json.loads(receiver.recv(4096))==dict(command='gui_closed',token=token)
        finally:window.close();window.deleteLater();app.processEvents()


def test_manus_unified_gui_collector_gap_stop_and_pedal_restart(tmp_path, monkeypatch):
    """Exercise the shared full-entry GUI through the real collector TCP API."""
    from types import SimpleNamespace
    root = Path(__file__).resolve().parents[2]
    for package in ('teleop_core', 'teleop_data_collector'):
        monkeypatch.syspath_prepend(str(root / 'ros_ws' / 'src' / package))
    from teleop_data_collector.continuity import ContinuityMonitor
    from teleop_data_collector.collector_control import CollectorController, CollectorControlServer

    app = QApplication.instance() or QApplication([])
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    cfg = tmp_path / 'session.json'
    cfg.write_text(json.dumps(dict(source='manus', mode='normal', token='f'*64, control_port=port())))
    monkeypatch.setenv('TELEOP_FULL_CONFIG', str(cfg))
    monkeypatch.delenv('TELEOP_FULL_EXIT_SOCKET', raising=False)
    gate = threading.Event()
    topic = '/cam1/color/image_raw'
    monitor = ContinuityMonitor([topic])

    class Recorder:
        active = False
        current_bag_dir = None
        last_bag_dir = None
        milestones = []
        count = 0
        report = {}

        def start(self):
            self.current_bag_dir = tmp_path / f'episode{self.count}'
            self.count += 1
            self.current_bag_dir.mkdir()
            self.last_bag_dir = self.current_bag_dir
            self.active = True
            return self.current_bag_dir

        def set_live_continuity(self, report):
            self.report = report

        def stop(self, *, interrupted=False):
            assert gate.wait(3), 'finalization gate timed out'
            bag = self.current_bag_dir
            failed = bool(self.report.get('auto_stop_reason'))
            (bag / 'collection_state.json').write_text(json.dumps(dict(
                state='incomplete' if failed else 'finalized', finalized=not failed,
                failures=[self.report['auto_stop_reason']] if failed else [],
                live_continuity=self.report)))
            self.active = False
            self.current_bag_dir = None
            self.on_recording_stopped()
            return bag

    recorder = Recorder()
    logger = SimpleNamespace(info=lambda _: None, warn=lambda _: None, error=lambda _: None)
    controller = CollectorController(SimpleNamespace(continuity_monitor=monitor, get_logger=lambda: logger),
                                     recorder, quality_dir=tmp_path / 'quality')
    controller.set_ready()
    server = CollectorControlServer(('127.0.0.1', 0), controller)
    thread = server.start_in_thread()
    from teleop_runtime.episode_home import EpisodeHomeBridge
    console = OperatorConsole()
    arms = OperatorControlServer(('127.0.0.1', 0), console)
    arms.start()
    window = OperatorWindow('127.0.0.1', arms.server_address[1], collection_port=server.server_address[1])
    bridge = EpisodeHomeBridge(server.server_address, arms.server_address, owner='f'*16)
    bridge.start()
    window.collection_poll_timer.setInterval(20)

    def wait(predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            app.processEvents()
            if predicate():
                return
            time.sleep(.003)
        raise AssertionError(window.collection_status)

    pedal = next(s for s in window.collection_shortcuts if s.key().toString() == 'L')
    try:
        wait(lambda: window.collection_state == 'READY')
        assert window.full_config['source'] == 'manus'
        assert '实时缺帧监控：开启' in window.collection_status_label.text()
        pedal.activated.emit()
        wait(lambda: window.collection_state == 'RECORDING')
        # Three separate one-frame gaps must only warn, never sum into failure.
        stamp = 1_000_000_000
        monitor.observe(topic, stamp, time.monotonic())
        for _ in range(3):
            stamp += 66_666_667
            monitor.observe(topic, stamp, time.monotonic())
        controller.check_continuity()
        wait(lambda: bool(window.collection_status.get('alerts')))
        assert window.collection_state == 'RECORDING'
        assert window.continuity_dialog is None
        assert '预警' in window.collection_status_label.text()
        # A single gap missing three frames triggers autonomous finalization.
        stamp += 133_333_333
        monitor.observe(topic, stamp, time.monotonic())
        controller.check_continuity()
        wait(lambda: window.collection_state == 'STOPPING')
        assert window.continuity_dialog is None
        assert not window.collection_record_button.isEnabled()
        assert not window.collection_milestone_button.isEnabled()
        gate.set()
        wait(lambda: window.collection_status.get('quality') == '报错')
        assert window.collection_state == 'INCOMPLETE'
        assert window.continuity_dialog is not None
        assert not window.continuity_dialog.isModal()
        assert window.quality_dialog is None
        assert not window.collection_record_button.isEnabled()
        assert 'Home' in window.collection_status_label.text()
        home = controller.status()['after_home']
        wait(lambda: console.snapshot()['episode_home'].get('state') == 'queued')
        job_id = console.snapshot()['episode_home']['id']
        console.update_episode_home(job_id, 'running')
        wait(lambda: window.collection_status['after_home']['state'] == 'running')
        assert not controller.status()['can_start']
        console.update_episode_home(job_id, 'done', 'offline simulated settling')
        wait(lambda: window.collection_record_button.isEnabled())
        pedal.activated.emit()
        wait(lambda: window.collection_status.get('current_episode') == 'episode1')
        assert window.collection_state == 'RECORDING'
        assert window.continuity_dialog is None
        assert not window.collection_status['alerts']
        assert window.collection_status['auto_stop_reason'] is None
        assert monitor.snapshot()['auto_stop_reason'] is None
    finally:
        bridge.stopping.set()
        bridge.thread.join(2)
        gate.set()
        controller.wait_for_finalization()
        controller.stop_if_present(interrupted=True)
        window.close()
        window.deleteLater()
        app.processEvents()
        arms.close()
        server.shutdown()
        server.server_close()
        thread.join(1)


def test_manus_unified_gui_reports_missing_continuity_monitor(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    cfg = tmp_path / 'session.json'
    cfg.write_text(json.dumps(dict(source='manus', mode='normal', token='f'*64, control_port=port())))
    monkeypatch.setenv('TELEOP_FULL_CONFIG', str(cfg))
    monkeypatch.delenv('TELEOP_FULL_EXIT_SOCKET', raising=False)
    window = OperatorWindow('127.0.0.1', port(), collection_port=port())
    try:
        assert '触觉采集终端' in window.collection_status_label.text()
        window._apply_collection_status(dict(state='READY', continuity_enabled=False, can_start=True))
        assert '实时缺帧监控：关闭' in window.collection_status_label.text()
        assert '触觉采集器' in window.collection_status_label.text()
    finally:
        window.close()
        window.deleteLater()
        app.processEvents()


def test_unified_manus_home_and_capture_only_send_arm_commands(tmp_path,monkeypatch):
    from unittest.mock import Mock
    app=QApplication.instance() or QApplication([])
    cfg=tmp_path/'session.json'
    cfg.write_text(json.dumps(dict(source='manus',mode='fake',token='f'*64,control_port=port())))
    monkeypatch.setenv('TELEOP_FULL_CONFIG',str(cfg))
    monkeypatch.delenv('TELEOP_FULL_EXIT_SOCKET',raising=False)
    window=OperatorWindow('127.0.0.1',port(),collection_port=port())
    window._send=Mock()
    window.hand_client.request=Mock()
    window.connection_state='connected'
    try:
        assert all(b.text()=='Record arm as Home' for b in window.capture_home_buttons.values())
        pedal=next(s for s in window.shortcuts if s.key().toString()=='B')
        pedal.activated.emit()
        assert window._send.call_args.args==('home_arm',{'side':'both','task':'powder_weighing'})
        window._record_current_home('left','assembly')
        assert window._send.call_args.args==('capture_home',{'side':'left','task':'assembly'})
        window.hand_client.request.assert_not_called()
    finally:
        window.close();window.deleteLater();app.processEvents()


def test_unified_episode_home_gates_recording_and_pedal_retries(tmp_path, monkeypatch):
    from unittest.mock import Mock
    app = QApplication.instance() or QApplication([])
    cfg = tmp_path / 'session.json'
    cfg.write_text(json.dumps(dict(source='manus', mode='fake', token='f'*64, control_port=port())))
    monkeypatch.setenv('TELEOP_FULL_CONFIG', str(cfg))
    monkeypatch.delenv('TELEOP_FULL_EXIT_SOCKET', raising=False)
    window = OperatorWindow('127.0.0.1', port(), collection_port=port())
    window._send_collection = Mock()
    window._send = Mock()
    window.hand_client.request = Mock()
    window.connection_state = 'connected'
    try:
        window._apply_collection_status(dict(state='READY', can_start=True))
        window._toggle_collection_recording()
        assert window._send_collection.call_args.args == ('start', {
            'return_arm_home': {'task': 'powder_weighing', 'owner': 'f'*16}})
        home = dict(id='episode1', state='running')
        status = dict(state='FINALIZED', last_episode='episode1', quality='一般',
                      can_start=False, after_home=home)
        window._apply_collection_status(status)
        assert not window.collection_record_button.isEnabled()
        assert '机械臂 Home 中' in window.collection_status_label.text()
        pedal = next(s for s in window.shortcuts if s.key().toString() == 'B')
        pedal.activated.emit()
        assert window._send_collection.call_args.args == ('retry_home', {'id': 'episode1', 'owner': 'f'*16})
        window._send.assert_not_called()
        home['state'] = 'failed'
        window._apply_collection_status(status)
        assert '中间踏板重试' in window.collection_status_label.text()
        home['state'] = 'done'
        status['can_start'] = True
        window._apply_collection_status(status)
        assert window.collection_record_button.isEnabled()
        pedal.activated.emit()
        assert window._send.call_args.args[0] == 'home_arm'
        window.hand_client.request.assert_not_called()
    finally:
        window.close(); window.deleteLater(); app.processEvents()


def test_storage_sidebar_sends_selected_parent_and_reports_final_location(tmp_path, monkeypatch):
    from unittest.mock import Mock
    app = QApplication.instance() or QApplication([])
    monkeypatch.delenv('TELEOP_FULL_CONFIG', raising=False)
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path / 'settings'))
    window = OperatorWindow('127.0.0.1', port(), collection_port=port())
    window._send_collection = Mock()
    try:
        parent = tmp_path / 'saved'
        status = dict(state='READY', can_start=True,
                      storage_settings=dict(supported=True, default_parent=str(parent)))
        window._apply_collection_status(status)
        assert window.storage_parent_edit.text() == str(parent)
        assert all(f'{letter}  {quality}' in window.storage_preview.text()
                   for quality, letter in [('优等', 'A'), ('一般', 'B'), ('报错', 'C'), ('放弃', 'D')])
        chosen = tmp_path / 'selected'
        monkeypatch.setattr('apps.operator_gui.operator_gui.QFileDialog.getExistingDirectory', lambda *args: str(chosen))
        window.storage_browse_button.click()
        window._toggle_collection_recording()
        assert window._send_collection.call_args.args == ('start', {'storage_parent': str(chosen)})
        assert window.settings.value('collection_storage_parent') == str(chosen)
        # While recording, changing the next parent never changes this episode.
        frozen = dict(parent=str(chosen), episode='episode52', state='recording',
                      path=str(chosen / 'pending' / 'episode52'))
        window._apply_collection_status(dict(status, state='RECORDING', active=True,
                                            can_start=False, can_stop=True, storage=frozen))
        window.storage_parent_edit.setText(str(tmp_path / 'next'))
        assert window.storage_episode_label.text() == 'episode52 · 录制中 · 暂存'
        assert str(chosen) in window.storage_episode_label.toolTip()
        final = dict(frozen, state='saved', path=str(chosen / 'A' / 'episode52'))
        window._apply_collection_status(dict(status, state='FINALIZED', quality='优等', storage=final))
        assert '已归档' in window.storage_episode_label.text()
        assert window.storage_episode_label.text() == 'episode52 · 已归档 A'
        assert final['path'] in window.storage_episode_label.toolTip()
        assert window.storage_open_button.isEnabled()
        failed = dict(frozen, state='failed', message='target already exists')
        window._apply_collection_status(dict(status, state='INCOMPLETE', can_start=False, storage=failed))
        assert not window.collection_record_button.isEnabled()
        assert '重新选择质量重试' in window.storage_episode_label.text()
    finally:
        window.close(); window.deleteLater(); app.processEvents()


def test_finish_path_edit_restores_pedal_shortcuts(tmp_path, monkeypatch):
    from unittest.mock import Mock
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    app = QApplication.instance() or QApplication([])
    monkeypatch.delenv('TELEOP_FULL_CONFIG', raising=False)
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path / 'settings'))
    window = OperatorWindow('127.0.0.1', port(), collection_port=port())
    window._send_collection = Mock()
    window._send = Mock()
    try:
        window.show()
        window.activateWindow()
        app.processEvents()
        window.socket.blockSignals(True)
        window.collection_socket.blockSignals(True)
        window.connection_state = 'connected'
        parent = str(tmp_path / 'saved')
        window._apply_collection_status(dict(state='READY', can_start=True,
            storage_settings=dict(supported=True, default_parent=parent)))
        window.storage_parent_edit.setFocus()
        QTest.keyClick(window.storage_parent_edit, Qt.Key_Return)
        app.processEvents()
        assert not window.storage_parent_edit.hasFocus()
        QTest.keyClick(window, Qt.Key_L)
        app.processEvents()
        assert window._send_collection.call_args.args == ('start', {'storage_parent': parent})
        QTest.keyClick(window, Qt.Key_B)
        app.processEvents()
        assert window._send.call_args.args == ('home_arm', {'side': 'both', 'task': 'powder_weighing'})
    finally:
        window.close(); window.deleteLater(); app.processEvents()


def test_sidebar_defaults_restore_local_remote_and_all_switches(tmp_path,monkeypatch):
    from teleop_runtime.transfer_settings import read_settings
    app=QApplication.instance() or QApplication([])
    monkeypatch.delenv('TELEOP_FULL_CONFIG',raising=False)
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path/'qt'))
    first=OperatorWindow('127.0.0.1',port(),collection_port=port())
    local=str(tmp_path/'saved');remote=str(tmp_path/'remote')
    first.storage_parent_edit.setText(local)
    first.storage_parent_edit.textEdited.emit(local)
    panel=first.transfer_panel
    panel.remote_edit.setText(remote);panel.remote_edit.textEdited.emit(remote)
    panel.upload_checkbox.setChecked(True)
    panel.upload_d_checkbox.setChecked(True)
    panel.delete_checkbox.setChecked(True)
    # Persist while still editing, without needing a recording or focus change.
    assert read_settings()['local_parent']==local
    assert read_settings()['remote_parent']==remote
    first.close();app.processEvents()
    second=OperatorWindow('127.0.0.1',port(),collection_port=port())
    try:
        assert second.storage_parent_edit.text()==local
        assert second.transfer_panel.remote_edit.text()==remote
        assert all(widget.isChecked() for widget in (second.transfer_panel.upload_checkbox,
            second.transfer_panel.upload_d_checkbox,second.transfer_panel.delete_checkbox))
        assert not Path(local).exists() and not Path(remote).exists()
    finally:
        second.close();app.processEvents()


def test_offline_arms_disable_motion_and_task_management_preserves_layout(tmp_path,monkeypatch):
    from PySide6.QtWidgets import QInputDialog,QMessageBox
    from operator_tasks import get_operator_tasks
    from unittest.mock import Mock
    app=QApplication.instance() or QApplication([])
    monkeypatch.delenv('TELEOP_FULL_CONFIG',raising=False)
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path/'qt'))
    window=OperatorWindow('127.0.0.1',port(),collection_port=port())
    try:
        window._apply_status(dict(arm_available=False,status_line='ARMS UNAVAILABLE',
            active=dict(left=False,right=False),arm_hold=dict(left=False,right=False)))
        assert all(not button.isEnabled() for button in window.arm_engage_buttons.values())
        assert not window.home_both_button.isEnabled() and not window.capture_ready_button.isEnabled()
        assert window.task_selector.isEnabled()
        assert 'unavailable' in window.arm_engage_buttons['left'].text()
        monkeypatch.setattr(QInputDialog,'getText',lambda *a:('离线新任务',True))
        window._task_action_selected(window.task_selector.findData('__add_task__'))
        _wait_task_edit(window,app)
        selected=window._selected_task()
        assert get_operator_tasks()[selected]=='离线新任务'
        window._apply_status(dict(arm_available=True,active={},arm_hold={}))
        window._send=Mock()
        window._record_current_home('left')
        assert window._send.call_args.args==('capture_home',dict(side='left',task=selected))
        # Neither arm-state refresh nor returning online changes the selected task.
        window._apply_status(dict(arm_available=True,active={},arm_hold={}))
        assert window._selected_task()==selected and window.home_both_button.isEnabled()
        window._shortcut_home('both')
        assert window._send.call_args.args==('home_arm',dict(side='both',task=selected))
        window._delete_task_confirmed(selected,QMessageBox.Yes)
        _wait_task_edit(window,app)
        assert selected not in get_operator_tasks()
        assert window._selected_task() in get_operator_tasks()
    finally:window.close();app.processEvents()


def test_gui_delete_removes_pose_and_trajectory_using_yaml_helper(tmp_path,monkeypatch):
    from operator_tasks import add_operator_task,task_storage_paths,get_operator_tasks
    from PySide6.QtWidgets import QMessageBox
    app=QApplication.instance() or QApplication([])
    monkeypatch.delenv('TELEOP_FULL_CONFIG',raising=False)
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path/'qt'))
    selected=add_operator_task('永久清理测试')
    poses,trajectories=task_storage_paths()
    # JSON is also valid YAML; the GUI Python need not import PyYAML.
    poses.write_text(json.dumps(dict(version=1,tasks={selected:dict(home=dict(left=[1]*7,right=[2]*7)),
        'assembly':dict(home=dict(left=[3]*7,right=[4]*7))},ready='shared-ready')))
    trajectories.mkdir(parents=True)
    target=trajectories/(selected+'.yaml');target.write_text('selected trajectory')
    other=trajectories/'assembly.yaml';other.write_text('keep')
    window=OperatorWindow('127.0.0.1',port(),collection_port=port())
    try:
        window._reload_task_selector(selected)
        window._delete_task_confirmed(selected,QMessageBox.Yes)
        _wait_task_edit(window,app)
        assert selected not in get_operator_tasks()
        assert selected not in poses.read_text()
        assert 'shared-ready' in poses.read_text() and 'assembly' in poses.read_text()
        assert not target.exists() and other.read_text()=='keep'
        assert '永久删除' in window.feedback.toPlainText()
    finally:window.close();app.processEvents()


def _wait_task_edit(window,app):
    from PySide6.QtTest import QTest
    deadline=time.monotonic()+8
    while window.task_edit_pending and time.monotonic()<deadline:
        app.processEvents();QTest.qWait(10)
    assert not window.task_edit_pending, window.feedback.toPlainText()


def test_task_cleanup_does_not_block_gui_timers(tmp_path,monkeypatch):
    from PySide6.QtWidgets import QMessageBox
    from PySide6.QtCore import QTimer
    from PySide6.QtTest import QTest
    from operator_tasks import add_operator_task,delete_operator_task,get_operator_tasks
    import apps.operator_gui.operator_gui as module
    app=QApplication.instance() or QApplication([])
    monkeypatch.delenv('TELEOP_FULL_CONFIG',raising=False)
    QSettings.setPath(QSettings.NativeFormat,QSettings.UserScope,str(tmp_path/'qt'))
    selected=add_operator_task('后台清理')
    release=threading.Event();ticks=[]
    def slow_delete(task):
        assert release.wait(5)
        delete_operator_task(task)
    monkeypatch.setattr(module,'delete_operator_task',slow_delete)
    window=OperatorWindow('127.0.0.1',port(),collection_port=port())
    try:
        window._reload_task_selector(selected)
        window._delete_task_confirmed(selected,QMessageBox.Yes)
        assert window.task_edit_pending and not window.task_selector.isEnabled()
        QTimer.singleShot(5,lambda:ticks.append('GUI timer alive'))
        QTest.qWait(30);app.processEvents()
        assert ticks==['GUI timer alive']
        window._set_connection_state('connected')
        assert not window.home_both_button.isEnabled()
        assert window.disengage_all_button.isEnabled()
        assert all(button.isEnabled() for button in window.hand_engage_buttons.values())
        window._apply_status(dict(arm_available=True,active={},arm_hold={}))
        assert not window.home_both_button.isEnabled()
        release.set();_wait_task_edit(window,app)
        assert selected not in get_operator_tasks() and window.task_selector.isEnabled()
    finally:
        release.set()
        if window.task_edit_pending:_wait_task_edit(window,app)
        window.close();app.processEvents()
