"""Full-mode transport/configuration only; MotionLatch remains motion authority."""
from collections import deque
import json
import os
from pathlib import Path
import socketserver
import socket
import threading
import time



def load_full_config(path):
    p=Path(path);st=p.stat()
    if st.st_uid!=os.getuid() or st.st_mode&0o077:
        raise ValueError('Private full-teleop configuration required')
    data=json.loads(p.read_text())
    if data.get('source') not in ('manus','litchibot') or data.get('mode') not in ('fake','normal','supervised_hardware_validation'):
        raise ValueError('Invalid full-teleop source/mode')
    if not isinstance(data.get('control_port'),int) or not 1024<=data['control_port']<=65535:
        raise ValueError('Valid local hand control port required')
    if type(data.get('fake_input',False)) is not bool or (data.get('fake_input') and (data['mode']!='fake' or data['source']!='manus')):
        raise ValueError('Synthetic Manus input is restricted to software mode')
    token=data.get('token','')
    if len(token)!=64 or any(c not in '0123456789abcdef' for c in token):
        raise ValueError('Full session token required')
    if data['mode']=='supervised_hardware_validation':
        from .validation_safety import load_permit
        policy=load_permit(data['permit'])
        if token!=policy['session_token']:raise ValueError('Full/permit session mismatch')
    elif data['mode']=='normal':
        devices=json.loads(Path(data['devices']).read_text())
        serials=devices.get('serials',{})
        if set(serials)!={'left','right'} or any(not isinstance(v,str) or not v.strip() for v in serials.values()) or serials['left']==serials['right']:
            raise ValueError('Deploy distinct physical hand serials once before normal hardware use')
        policy={'session_token':token,'expires_unix_s':float('inf'),'serials':serials,'profile':data['profile']}
    else:
        policy={'session_token':token,'expires_unix_s':data['expires_unix_s'],
                'serials':{'left':'fake-left','right':'fake-right'},'profile':data['profile']}
    if data['profile']!=policy['profile']:raise ValueError('Profile mismatch')
    data['policy']=policy
    return data


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        owner=object();authenticated=False
        generation=None
        self.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        try:
            while True:
                line=self.rfile.readline(65537)
                if not line or len(line)>65536:break
                request=json.loads(line)
                if request.get('token')!=self.server.token:break
                revoke_only=request.get('command')=='disengage_all'
                if not authenticated and not revoke_only:
                    with self.server.lock:
                        if self.server.owner is not None:break
                        self.server.owner=owner
                        self.server.generation+=1
                        generation=self.server.generation
                    authenticated=True
                # Connection identity is transport-owned, never supplied by the GUI.
                request['_connection_generation']=generation
                event=threading.Event();reply={}
                with self.server.lock:
                    if len(self.server.jobs)>=100:break
                    self.server.jobs.append((None if revoke_only else owner,request,event,reply))
                if not event.wait(self.server.reply_timeout):break
                self.wfile.write((json.dumps({'id':request.get('id'),**reply})+'\n').encode());self.wfile.flush()
        except (OSError,ValueError):
            pass
        finally:
            if authenticated:
                with self.server.lock:
                    if self.server.owner is owner:
                        self.server.owner=None
                        self.server.disconnected=True


class HandControlServer(socketserver.ThreadingTCPServer):
    # Reclaim our own closed TCP connections, never share an active listener.
    allow_reuse_address=True
    daemon_threads=True
    def __init__(self,port,token):
        self.reply_timeout=.15
        self.token=token;self.owner=None;self.jobs=deque();self.lock=threading.Lock();self.disconnected=False;self.generation=0
        super().__init__(('127.0.0.1',port),_Handler)
        self.thread=threading.Thread(target=self.serve_forever,kwargs={'poll_interval':.05},daemon=True)
        self.thread.start()

    def drain(self,callback,lost):
        with self.lock:
            jobs=list(self.jobs);self.jobs.clear();disconnected=self.disconnected;self.disconnected=False
            owner=self.owner
        if disconnected:lost()
        for source,request,event,reply in jobs:
            try:
                if source is not owner and not (source is None and request.get('command')=='disengage_all'):
                    raise ValueError('GUI session disconnected')
                reply.update(ok=True,result=callback(request))
            except Exception as error:reply.update(ok=False,error=str(error))
            finally:event.set()

    def close(self):
        self.shutdown();self.server_close();self.thread.join(timeout=1)
