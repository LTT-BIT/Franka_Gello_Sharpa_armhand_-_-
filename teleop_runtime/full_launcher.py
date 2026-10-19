"""Host lifecycle owner. Reuses production wrappers and coordinates post-recording arm Home."""
from __future__ import annotations
import argparse
import ctypes
import fcntl
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import tempfile
import time
import threading

ROOT=Path(__file__).resolve().parents[1]
SERVICES={'franka-control','teleop-control','moveit-ik','preset-ik','gello-bridge','hand-control'}


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hand-source',choices=('manus','litchibot'),default='manus')
    modes=p.add_mutually_exclusive_group()
    modes.add_argument('--dry-run',action='store_true',help='Default: fake Sharpa, no hardware sending')
    modes.add_argument('--hardware',action='store_true',help='Normal GUI-controlled mode, initially DISABLED')
    modes.add_argument('--supervised-hardware-validation',action='store_true')
    p.add_argument('--validation-permit',default='')
    p.add_argument('--devices',default=str(ROOT/'config/litchibot/hand_devices.json'),help='One-time deployment serial mapping for normal hardware')
    p.add_argument('--profile',default='LYG226360006')
    p.add_argument('--control-port',type=int,default=5593)
    p.add_argument('--fake-input',action='store_true',help='Manus synthetic input, allowed only with fake Sharpa')
    return p


def session_config(args):
    if args.fake_input and (args.hardware or args.supervised_hardware_validation or args.hand_source!='manus'):
        raise ValueError('Synthetic input requires Manus + fake hardware')
    data={'source':args.hand_source,'mode':'normal' if args.hardware else 'fake',
          'token':secrets.token_hex(32),'profile':args.profile,'control_port':args.control_port,
          'devices':str(Path(args.devices).resolve()),'fake_input':args.fake_input,
          'expires_unix_s':time.time()+86400}
    if args.supervised_hardware_validation:
        if not args.validation_permit:raise ValueError('Validation permit required')
        from adapters.litchibot.validation_safety import load_permit
        permit=load_permit(args.validation_permit)
        data.update(mode='supervised_hardware_validation',permit=str(Path(args.validation_permit).resolve()),token=permit['session_token'])
    elif args.validation_permit:raise ValueError('Validation permit only belongs to supervised validation')
    return data


def deployment_port(root,key,default):
    value=os.environ.get(key)
    if not value:
        path=Path(root)/'docker/.env'
        if path.exists():
            for line in path.read_text().splitlines():
                if line.startswith(key+'='):
                    value=line.split('=',1)[1].strip().strip('"\'');break
    return int(value or default)


def ensure_manus_bridge_current(root=ROOT):
    """Rebuild only the explicitly configured project bridge when stale.

    Portable copies can retain an old ignored .so after updating tracked C++.
    Never rebuild a foreign library or initialize a device here.
    """
    root=Path(root)
    env_file=root/'docker/.env'
    if not env_file.is_file():return
    configured=''
    for line in env_file.read_text().splitlines():
        if line.startswith('MANUS_NATIVE_LIBRARY='):
            configured=line.split('=',1)[1].strip();break
    library=root/'adapters/manus/build/libmanus_skeleton_bridge.so'
    if not configured or Path(configured).resolve()!=library.resolve():return
    sources=[root/'adapters/manus/CMakeLists.txt',
             root/'adapters/manus/native/manus_skeleton_bridge.cpp',
             root/'adapters/manus/native/litchi_manus_bridge.h']
    if not all(p.is_file() for p in sources):
        raise RuntimeError('Configured project MANUS bridge source is missing')
    if not library.is_file() or max(p.stat().st_mtime_ns for p in sources)>library.stat().st_mtime_ns:
        print('MANUS project bridge missing or older than source; rebuilding before device startup',flush=True)
        subprocess.run([str(root/'adapters/manus/scripts/build.sh')],cwd=root,check=True)


def preflight(port):
    from teleop_runtime.dds_buffers import check_receive_buffer
    check_receive_buffer()
    arm_port=deployment_port(ROOT,'TELEOP_CONTROL_PORT',5590)
    recorder_port=deployment_port(ROOT,'COLLECTION_CONTROL_PORT',5592)
    preset_port=deployment_port(ROOT,'PRESET_IK_PORT',5591)
    if port in (arm_port,recorder_port,preset_port):raise ValueError('Hand control port conflicts with existing operator/recorder/preset interface')
    for value in (arm_port,port):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            try:
                probe.bind(('127.0.0.1',value))
                probe.listen(1)
            except OSError as error:
                raise RuntimeError(f'Control port {value} unavailable: {error}') from error
    result=subprocess.run(['docker','compose','ps','--services','--status','running'],cwd=ROOT/'docker',
                          text=True,capture_output=True,check=True)
    existing=SERVICES.intersection(result.stdout.splitlines())
    if existing:raise RuntimeError('Full launcher refuses to take over existing services: '+', '.join(sorted(existing)))


def descendants(root_pid):
    """Record Linux process identity, including nested setsid workers/GUI groups."""
    entries={}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            fields=(p/'stat').read_text().rsplit(')',1)[1].split()
            entries[int(p.name)]=(int(fields[1]),int(fields[2]),fields[19])
        except (OSError,ValueError,IndexError):continue
    owned={root_pid};old=set()
    while old!=owned:
        old=owned.copy();owned.update(pid for pid,(parent,_,_) in entries.items() if parent in owned)
    return {pid:entries[pid] for pid in owned if pid in entries}


def alive(pid,identity):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return fields[19]==identity[2] and fields[0]!='Z'
    except (OSError,IndexError):return False


def notify_full_session(token, command):
    """Send an authenticated lifecycle request to this GUI's launcher."""
    path=os.environ.get('TELEOP_FULL_EXIT_SOCKET')
    if not path:return False
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sender:
            sender.settimeout(.1)
            sender.sendto(json.dumps({'command':command,'token':token}).encode(),path)
        return True
    except OSError:return False


def notify_gui_session_exit(token):
    """Explicit window close ends its owned full session, independently of TCP."""
    return notify_full_session(token, "gui_closed")


class FullLauncher:
    def __init__(self,root,config_path,config):
        self.root=Path(root);self.path=Path(config_path);self.config=config
        self.children=[];self.stopping=False;self.owned={};self.subreaper=None
        self.exit_socket=None;self.exit_path=self.path.with_name('launcher-exit.sock')
        self.episode_home_bridge=None
        self.arm_reconnect_thread=None
        self.owns_docker=(self.root/'docker/compose.yaml').is_file()

    def start(self):
        # Nested setsid GUI/SDK workers remain our descendants even if a wrapper
        # crashes before its cleanup trap. Linux reparents them to this owner.
        libc=ctypes.CDLL(None,use_errno=True);old=ctypes.c_int()
        if libc.prctl(37,ctypes.byref(old),0,0,0)!=0 or libc.prctl(36,1,0,0,0)!=0:
            raise OSError(ctypes.get_errno(),'Cannot establish subprocess ownership')
        self.subreaper=old.value
        self.exit_socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.exit_socket.bind(str(self.exit_path));os.chmod(self.exit_path,0o600)
        self.exit_socket.setblocking(False)
        env={**os.environ,'TELEOP_FULL_CONFIG':str(self.path),
             'TELEOP_FULL_EXIT_SOCKET':str(self.exit_path)}
        hands=[str(self.root/'ops/run/run_sharpa_hands_cyclonedds.sh'),
               '--hand-source',self.config['source'],'--full-config',str(self.path)]
        if self.config['source']=='litchibot':
            hands.append('litchibot_profile:='+self.config['profile'])
        if self.config['mode']=='fake':hands.append('--dry-run')
        self.children.append(subprocess.Popen(hands,cwd=self.root,env=env,start_new_session=True))
        self.children.append(subprocess.Popen([str(self.root/'ops/run/run_gello_arms_only.sh')],
                                             cwd=self.root,env=env,start_new_session=True))
        if self.owns_docker:
            from teleop_runtime.episode_home import EpisodeHomeBridge
            self.episode_home_bridge=EpisodeHomeBridge(
                ('127.0.0.1',deployment_port(self.root,'COLLECTION_CONTROL_PORT',5592)),
                (os.environ.get('TELEOP_CONTROL_HOST','127.0.0.1'),
                 deployment_port(self.root,'TELEOP_CONTROL_PORT',5590)), owner=self.config['token'][:16])
            self.episode_home_bridge.start()

    def gui_closed(self):
        if self.exit_socket is None:return False
        for _ in range(32):
            try:data=self.exit_socket.recv(4097)
            except BlockingIOError:return False
            try:request=json.loads(data)
            except (ValueError,UnicodeDecodeError):continue
            if (isinstance(request,dict) and request.get('command')=='gui_closed'
                    and request.get('token')==self.config['token']):return True
            if (isinstance(request,dict) and request.get('command')=='reconnect_arms'
                    and request.get('token')==self.config['token'] and self.owns_docker):
                if self.arm_reconnect_thread is None or not self.arm_reconnect_thread.is_alive():
                    self.arm_reconnect_thread=threading.Thread(target=self.reconnect_arms,daemon=True)
                    self.arm_reconnect_thread.start()
        return False

    def reconnect_arms(self):
        """Explicit retry only. Never stop a running controller or restart hands."""
        try:
            port=deployment_port(self.root,'TELEOP_CONTROL_PORT',5590)
            with socket.create_connection(('127.0.0.1',port),timeout=1) as connection:
                connection.sendall(b'{"id":0,"command":"status"}\n')
                with connection.makefile('rb') as stream:
                    response=json.loads(stream.readline(65536))
            status=response.get('result',{})
            if not response.get('ok') or status.get('arm_available',True):
                print('Arm reconnect refused: arm backend unavailable or arms already ready',flush=True)
                return
            if self.stopping:return
            # up only starts an exited hardware service. An active controller is
            # never torn down; GUI requests cannot restart a moving robot.
            subprocess.run(['docker','compose','up','-d','--no-deps','franka-control'],
                cwd=self.root/'docker',check=True,timeout=30)
            print('Arm hardware connection retried; fresh state and manual engagement are still required',flush=True)
        except (OSError,ValueError,subprocess.SubprocessError) as error:
            print('Arm reconnect failed: '+str(error),flush=True)

    def remember(self):
        for child in self.children:self.owned.update(descendants(child.pid))
        if self.subreaper is not None:
            for pid,identity in descendants(os.getpid()).items():
                if pid!=os.getpid():self.owned.setdefault(pid,identity)

    def revoke(self):
        # Best effort; independent GUI lease + in-node watchdog do not depend on this.
        try:
            with socket.create_connection(('127.0.0.1',self.config['control_port']),timeout=.15) as s:
                s.sendall((json.dumps({'id':0,'command':'disengage_all','token':self.config['token']})+'\n').encode())
                s.recv(65536)
        except OSError:pass
        if self.owns_docker:
            # Revoke the existing arm authority immediately, before waiting for
            # hand teardown. Original server dispatch remains unchanged.
            try:
                port=deployment_port(self.root,'TELEOP_CONTROL_PORT',5590)
                host=os.environ.get('TELEOP_CONTROL_HOST','127.0.0.1')
                with socket.create_connection((host,port),timeout=.15) as s:
                    s.sendall((json.dumps({'id':0,'command':'disengage_all'})+'\n').encode())
                    s.recv(65536)
            except OSError:pass

    def shutdown_normal_driver(self):
        if self.config.get('source')!='litchibot' or self.config.get('mode') not in ('normal','fake'):return None
        endpoint=self.path.with_name('hand-shutdown.sock')
        report_path=self.path.with_name('hand-shutdown.json')
        if not endpoint.exists():
            if report_path.exists():
                report=json.loads(report_path.read_text())
                if report.get('session')==self.config['token'][:16] and report.get('state')!='COMPLETE':
                    return 'Native hand shutdown not confirmed: '+json.dumps(report)
            return None
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sender:
            sender.settimeout(.15)
            sender.sendto(json.dumps(dict(command='shutdown',token=self.config['token'])).encode(),str(endpoint))
        deadline=time.monotonic()+18
        while time.monotonic()<deadline:
            try:report=json.loads(report_path.read_text())
            except (OSError,ValueError):report={}
            if report.get('session')==self.config['token'][:16] and report.get('state') in ('COMPLETE','FAILED'):
                print('Hand driver shutdown: '+json.dumps(report),flush=True)
                hands=report.get('hands',{})
                verified=set(hands)=={'left','right'} and all(r.get('disabled_verified') is True and r.get('disconnected_verified') is True for r in hands.values())
                return None if report['state']=='COMPLETE' and verified else 'Native hand shutdown failed: '+json.dumps(report)
            time.sleep(.05)
        return 'Native hand shutdown not confirmed before wrapper teardown: '+json.dumps(report)

    def close(self):
        if self.stopping:return
        self.stopping=True
        if self.episode_home_bridge is not None:
            self.episode_home_bridge.stopping.set()
        if self.exit_socket is not None:
            self.exit_socket.close();self.exit_path.unlink(missing_ok=True);self.exit_socket=None
        self.remember();self.revoke();errors=[]
        if self.arm_reconnect_thread is not None:
            self.arm_reconnect_thread.join(timeout=32)
        if self.episode_home_bridge is not None and self.episode_home_bridge.thread is not None:
            self.episode_home_bridge.thread.join(timeout=3)

        # Let the driver disable devices before ROS launch begins its 5s/10s
        # escalation. This channel does not depend on the ROS executor.
        try:
            error=self.shutdown_normal_driver()
            if error:errors.append(error)
        except OSError as error:errors.append('Hand shutdown request failed: '+str(error))
        # Stop hands first, then let the original arm wrapper stop its Docker services
        # and run_operator reclaim its own backend/GUI sessions. Never touch recorder.
        for child in self.children:
            owned={p:i for p,i in self.owned.items() if i[1]==child.pid}
            owned.update(descendants(child.pid))
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
            deadline=time.monotonic()+18
            while time.monotonic()<deadline and (child.poll() is None or any(alive(p,i) for p,i in owned.items())):
                owned.update(descendants(child.pid));time.sleep(.05)
            for sig,wait_s in ((signal.SIGTERM,5),(signal.SIGKILL,1)):
                survivors={p:i for p,i in owned.items() if alive(p,i)}
                if not survivors:break
                for pid,identity in survivors.items():
                    try:os.kill(pid,sig)
                    except ProcessLookupError:pass
                end=time.monotonic()+wait_s
                while time.monotonic()<end and any(alive(p,i) for p,i in survivors.items()):time.sleep(.05)
            try:child.wait(timeout=1)
            except subprocess.TimeoutExpired:errors.append('Owned subprocess could not be reclaimed')
            if any(alive(p,i) for p,i in owned.items()):errors.append('Owned descendants remain after cleanup')
        # Adopted orphan sessions may no longer retain their old wrapper ancestry.
        self.remember()
        for sig,wait_s in ((signal.SIGINT,1),(signal.SIGTERM,2),(signal.SIGKILL,1)):
            survivors={p:i for p,i in self.owned.items() if alive(p,i)}
            if not survivors:break
            for pid in survivors:
                try:os.kill(pid,sig)
                except ProcessLookupError:pass
            end=time.monotonic()+wait_s
            while time.monotonic()<end and any(alive(p,i) for p,i in survivors.items()):time.sleep(.05)
        for pid,identity in self.owned.items():
            try:os.waitpid(pid,os.WNOHANG)
            except ChildProcessError:pass
        if self.subreaper is not None:
            ctypes.CDLL(None).prctl(36,self.subreaper,0,0,0)
        if any(alive(p,i) for p,i in self.owned.items()):errors.append('Owned orphan worker remains')
        if self.owns_docker:
            try:
                # Covers interruption during compose up before the old shell marks
                # stack_started=true. Preflight proved these services were absent.
                result=subprocess.run(['docker','compose','ps','--services','--status','running'],
                    cwd=self.root/'docker',text=True,capture_output=True,timeout=15,check=True)
                remaining=sorted(SERVICES.intersection(result.stdout.splitlines()))
                if remaining:
                    subprocess.run(['docker','compose','stop','--timeout','10',*remaining],
                        cwd=self.root/'docker',timeout=25,check=True)

            except (OSError,subprocess.SubprocessError) as error:errors.append('Docker cleanup: '+str(error))
        if errors:raise RuntimeError('; '.join(errors))

    def run(self):
        stopped=False
        def stop(*unused):
            nonlocal stopped
            stopped=True
        # Terminal close sends SIGHUP; our setsid children would otherwise
        # outlive this owner and leave the native hands enabled.
        old={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
        for sig in old:signal.signal(sig,stop)
        try:
            self.start()
            while not stopped:
                if self.gui_closed():
                    print('Operator GUI closed; shutting down hands before remaining stack',flush=True)
                    return 0
                self.remember()
                failed=next((p for p in self.children if p.poll() is not None),None)
                if failed is not None:
                    print(f'Full subsystem exited ({failed.returncode}); stopping owned stack',flush=True)
                    return failed.returncode or 1
                time.sleep(.05)
            return 0
        finally:
            try:self.close()
            finally:
                for sig,handler in old.items():signal.signal(sig,handler)


def main(argv=None):
    args=parser().parse_args(argv)
    # Locked before any subprocess or Docker startup.
    lock_path=Path('/tmp')/f'gello-full-{os.getuid()}.lock'
    fd=os.open(lock_path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        data=session_config(args)
        with tempfile.TemporaryDirectory(prefix='gello-full-') as directory:
            path=Path(directory)/'session.json';path.write_text(json.dumps(data));path.chmod(0o600)
            from adapters.litchibot.full_control import load_full_config
            load_full_config(path)
            if args.hand_source=='manus' and not args.fake_input:
                ensure_manus_bridge_current()
            preflight(args.control_port)
            print(f"FULL TELEOP source={data['source']} mode={data['mode']}; arms/hands initially DISABLED",flush=True)
            return FullLauncher(ROOT,path,data).run()
    except (ValueError,OSError,RuntimeError,subprocess.CalledProcessError) as error:
        print('Full startup refused: '+str(error),flush=True);return 2
    finally:os.close(fd)


if __name__=='__main__':raise SystemExit(main())
