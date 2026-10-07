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
