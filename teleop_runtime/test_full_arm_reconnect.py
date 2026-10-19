"""Explicit hardware-service retry never restarts running controllers or hands."""
from io import BytesIO
from pathlib import Path
from unittest.mock import Mock
import json
import pytest
from teleop_runtime.full_launcher import FullLauncher


@pytest.mark.parametrize('available',[False,True])
def test_retry_requires_unavailable_backend_and_only_starts_franka(tmp_path,monkeypatch,available):
    (tmp_path/'docker').mkdir()
    config=dict(token='t'*64,control_port=5593,source='manus',mode='normal')
    launcher=FullLauncher(tmp_path,tmp_path/'session.json',config)
    class Connection:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def sendall(self,packet):assert json.loads(packet)['command']=='status'
        def makefile(self,*args):
            return BytesIO(json.dumps(dict(ok=True,result=dict(arm_available=available))).encode()+b'\n')
    monkeypatch.setattr('teleop_runtime.full_launcher.socket.create_connection',lambda *a,**k:Connection())
    run=Mock()
    monkeypatch.setattr('teleop_runtime.full_launcher.subprocess.run',run)
    launcher.reconnect_arms()
    if available:run.assert_not_called()
    else:
        assert run.call_args.args[0]==['docker','compose','up','-d','--no-deps','franka-control']
        assert run.call_args.kwargs['timeout']==30


def test_exit_in_progress_never_starts_hardware(tmp_path,monkeypatch):
    launcher=FullLauncher(tmp_path,tmp_path/'session.json',dict(token='t',control_port=5593))
    launcher.stopping=True
    # Even if a request arrived just before shutdown, retry cannot resurrect it.
    class Connection:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def sendall(self,packet):pass
        def makefile(self,*args):return BytesIO(b'{"ok":true,"result":{"arm_available":false}}\n')
    monkeypatch.setattr('teleop_runtime.full_launcher.socket.create_connection',lambda *a,**k:Connection())
    run=Mock();monkeypatch.setattr('teleop_runtime.full_launcher.subprocess.run',run)
    launcher.reconnect_arms()
    run.assert_not_called()
