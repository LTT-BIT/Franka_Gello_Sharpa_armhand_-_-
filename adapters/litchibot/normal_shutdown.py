"""Session teardown only: independent of the ROS forwarding executor."""
import json
import os
from pathlib import Path
import socket
import threading
import time


class NormalShutdown:
    def __init__(self,hands,config_path,token,begin,*,evidence_path=None):
        self.hands=dict(hands);self.token=token;self.begin=begin
        directory=Path(config_path).parent
        self.path=directory/'hand-shutdown.sock';self.report_path=directory/'hand-shutdown.json'
        self.evidence_path=Path(evidence_path) if evidence_path else None
        self.requested=threading.Event();self.done=threading.Event();self.closed=threading.Event()
        self.lock=threading.Lock();self.results={s.value:{} for s in hands};self.error=None
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.socket.bind(str(self.path));os.chmod(self.path,0o600);self.socket.settimeout(.05)
        self.write('READY')
        self.thread=threading.Thread(target=self.run,name='normal-session-shutdown',daemon=True);self.thread.start()

    def write(self,state):
        # Atomic evidence, separate from coalesced runtime telemetry.
        with self.lock:
            data=dict(pid=os.getpid(),session=self.token[:16],state=state,hands=self.results)
            for path in (self.report_path,self.evidence_path):
                if path is None:continue
                tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data));os.chmod(tmp,0o600);tmp.replace(path)

    def note(self,side,key,value):
        with self.lock:self.results[side.value][key]=value
        self.write('STOPPING')
        try:os.write(2,(f'Hand shutdown {side.value}: {key}={value}\n').encode())
        except OSError:pass

    def request(self):self.requested.set()

    def run(self):
        try:
            while not self.requested.is_set() and not self.closed.is_set():
                try:packet=json.loads(self.socket.recv(4097))
                except (socket.timeout,ValueError,UnicodeDecodeError):continue
                if isinstance(packet,dict) and packet.get('command')=='shutdown' and packet.get('token')==self.token:self.request()
            if not self.requested.is_set():return
            self.begin()
            self.write('STOPPING')
            # Attempt both disables independently; one blocked stop cannot keep
            # the opposite hand enabled. No target is ever sent here.
            def disable(side,hand):
                try:
                    self.note(side,'stop_started',True);hand.stop();self.note(side,'stop_returned',True)
                    if hasattr(hand,'_read_setting'):
                        # Readback only; reuse original SDK enable settle timing.
                        try:
                            from litchi_hardware.hardware.sharpa.sdk_driver import _ENABLE_SETTLE_TIMEOUT_S,_ENABLE_POLL_INTERVAL_S
                        except ImportError:
                            _ENABLE_SETTLE_TIMEOUT_S=_ENABLE_POLL_INTERVAL_S=0.0  # Pure software test double.
                        deadline=time.monotonic()+_ENABLE_SETTLE_TIMEOUT_S
                        off=not bool(hand._read_setting('get_enable_state'))
                        while not off and time.monotonic()<deadline:
                            time.sleep(_ENABLE_POLL_INTERVAL_S)
                            off=not bool(hand._read_setting('get_enable_state'))
                    else:off=bool(hand._stopped)  # Original mock actuator.
                    self.note(side,'disabled_verified',off)
                    if not off:raise RuntimeError('Native enable state still True after stop')
                except Exception as e:self.note(side,'stop_error',str(e))
            threads=[threading.Thread(target=disable,args=(s,h),daemon=True) for s,h in self.hands.items()]
            for t in threads:t.start()
            for t in threads:t.join()
            # All stop attempts finish before potentially blocking disconnect.
            def disconnect(side,hand):
                try:
                    self.note(side,'disconnect_started',True);hand.disconnect()
                    self.note(side,'disconnected_verified',not hand.is_connected)
                    if hand.is_connected:raise RuntimeError('Device still connected after disconnect')
                except Exception as e:self.note(side,'disconnect_error',str(e))
            threads=[threading.Thread(target=disconnect,args=(s,h),daemon=True) for s,h in self.hands.items()]
            for t in threads:t.start()
            for t in threads:t.join()
            success=all(r.get('disabled_verified') and r.get('disconnected_verified') and not r.get('stop_error') and not r.get('disconnect_error') for r in self.results.values())
            if not success:self.error=RuntimeError('Hand shutdown failed: '+json.dumps(self.results))
            self.write('COMPLETE' if success else 'FAILED')
        except Exception as e:
            self.error=e
            self.write('FAILED')
        finally:self.done.set()

    def finish(self):
        self.request();self.done.wait()
        self.closed.set();self.socket.close();self.path.unlink(missing_ok=True);self.thread.join()
        if self.error:raise self.error
