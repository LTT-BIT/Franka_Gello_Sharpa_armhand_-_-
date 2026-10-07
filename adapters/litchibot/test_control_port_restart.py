"""Fast restarts reclaim TIME_WAIT while a live control server stays exclusive."""
import json
import socket
from types import SimpleNamespace
import pytest
from adapters.litchibot.full_control import HandControlServer
from teleop_runtime import full_launcher


def unused_port():
    with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]


def test_hand_server_immediate_restart_and_active_listener_exclusion():
    port=unused_port();server=HandControlServer(port,'a'*64)
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=1) as client:
            client.sendall((json.dumps({'token':'invalid'})+'\n').encode())
            assert client.recv(1)==b''  # server closes first: its port enters TIME_WAIT
    finally:server.close()
    restarted=HandControlServer(port,'a'*64)
    try:
        with pytest.raises(OSError):HandControlServer(port,'b'*64)
    finally:restarted.close()


def test_preflight_accepts_closed_tcp_state_but_rejects_live_listener(monkeypatch):
    arm_port=unused_port();port=unused_port()
    monkeypatch.setattr(full_launcher,'deployment_port',lambda root,key,default:arm_port if key=='TELEOP_CONTROL_PORT' else default)
    monkeypatch.setattr(full_launcher.subprocess,'run',lambda *a,**kw:SimpleNamespace(stdout=''))
    server=HandControlServer(port,'a'*64)
    try:
        with pytest.raises(RuntimeError,match=f'Control port {port} unavailable'):full_launcher.preflight(port)
        with socket.create_connection(('127.0.0.1',port),timeout=1) as client:
            client.sendall((json.dumps({'token':'invalid'})+'\n').encode())
            assert client.recv(1)==b''
    finally:server.close()
    full_launcher.preflight(port)
