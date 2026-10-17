"""Independent recorder-owned file queue. Never opens a robot/control socket."""
from __future__ import annotations
import argparse
from contextlib import ExitStack
from types import SimpleNamespace
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

from .transfer_settings import atomic_json, read_settings, state_root
from .transfer_worker import ready_state, mount_for


class Watcher:
    """Directory-generation baseline: startup data is observed, never queued."""
    def __init__(self, settle=1.5):
        self.root=None
        self.root_identity=None
        self.seen=set()
        self.candidates={}
        self.settle=settle
        self.entry_cache={}

    def entries(self, root):
        result={}
        for folder in 'ABCD':
            directory=root/folder
            if directory.is_symlink():
                continue
            try:info=directory.stat()
            except FileNotFoundError:continue
            if not directory.is_dir():continue
            signature=(info.st_dev,info.st_ino,info.st_mtime_ns)
            cached=self.entry_cache.get(str(directory))
            if cached and cached[0]==signature:
                result.update(cached[1])
                continue
            values={}
            with os.scandir(directory) as entries:
                for entry in entries:
                    if entry.is_dir(follow_symlinks=False) and entry.name.startswith('episode'):
                        info=entry.stat(follow_symlinks=False)
                        values[(str(directory/entry.name),info.st_ino)]=Path(directory/entry.name)
            self.entry_cache[str(directory)]=(signature,values)
            result.update(values)
        return result

    def scan(self, config, now):
        value=config.get('local_parent','')
        if not value or not Path(value).is_absolute():
            return []
        root=Path(os.path.abspath(value))
        try:
            mount_for(root)
        except RuntimeError:
            pass
        else:
            raise ValueError('监控源必须是本机存储目录，不能是远端挂载目录')
        for component in [*reversed(root.parents),root]:
            if component.is_symlink():
                raise ValueError('监控源请使用实际本机路径，不使用符号链接')
        entries=self.entries(root)
        try:
            info=root.stat(); identity=(info.st_dev,info.st_ino)
        except FileNotFoundError:
            identity=None
        if root!=self.root or identity!=self.root_identity:
            self.root=root
            self.root_identity=identity
            self.seen=set(entries)
            self.candidates={}
            return []
        for key,path in entries.items():
            if key not in self.seen:
                self.seen.add(key)
                self.candidates[key]=(path,None,now)
        jobs=[]
        for key,(path,previous,changed) in list(self.candidates.items()):
            if key not in entries:
                del self.candidates[key]
                continue
            try:
                fingerprint=(path/'collection_state.json').stat().st_mtime_ns
                if fingerprint!=previous:
                    self.candidates[key]=(path,fingerprint,now)
                    continue
                if now-changed<self.settle:
                    continue
                state=ready_state(path)
            except (OSError,ValueError,RuntimeError):
                continue
            folder=path.parent.name
            del self.candidates[key]
            upload_required = folder in 'AB' or (folder == 'D' and config.get('upload_d_enabled', False))
            action='upload' if upload_required and config['upload_enabled'] else (
                'delete' if not upload_required and folder in 'CD' and config['delete_enabled'] else None)
            if action:
                jobs.append(dict(id=uuid.uuid4().hex,source=str(path),folder=folder,
                    recording_id=state['source_recording_id'],action=action,upload_required=upload_required,
                    remote_parent=config['remote_parent'],rate_mib=config.get('rate_mib',20),
                    state='queued',attempts=0,next_try=0))
        return jobs


class Service:
    def __init__(self, owner_pid, directory=None, *, idle_timeout=90):
        self.owner_pid=owner_pid
        try:self.owner_identity=Path(f'/proc/{owner_pid}/stat').read_text().rsplit(')',1)[1].split()[19]
        except OSError:self.owner_identity=None
        self.directory=Path(directory or state_root()/'sessions'/uuid.uuid4().hex).absolute()
        self.directory.mkdir(parents=True,exist_ok=True)
        self.watcher=Watcher()
        self.jobs=[]
        self.verifier=SimpleNamespace(child=None,current=None,global_lock=None,
                                      child_started=0,timed_out=False)
        self.child=None
        self.current=None
        self.global_lock=None
        self.draining=False
        self.drain_started=None
        self.idle_timeout=idle_timeout
        self.message='已启动，建立现有目录基线；不处理旧数据'
        self.last_result=''
        self.error=False
        self.timed_out=False
        self.last_write=0
        self.child_started=time.time()

    def save_job(self, job):
        atomic_json(self.directory/job['id']/'job.json',job)

    def owner_alive(self):
        try:
            os.kill(self.owner_pid,0)
            identity=Path(f'/proc/{self.owner_pid}/stat').read_text().rsplit(')',1)[1].split()[19]
            return identity==self.owner_identity
        except (ProcessLookupError,FileNotFoundError):
            return False

    def drain(self, *_args):
        if not self.draining:
            self.draining=True
            self.drain_started=time.monotonic()
            self.message='采集器已停止，后台队列正在收尾'
            for job in self.jobs:
                if job['state']=='retry':
                    job['state']='interrupted'
                    job['error']='采集结束，网络失败任务保留本地数据，不再自动重试'
                    self.save_job(job)

    def status(self, force=False):
        if not force and time.monotonic()-self.last_write<.5:
            return
        self.last_write=time.monotonic()
        queued=sum(j['state'] in {'queued','retry','running'} for j in self.jobs)
        active=[]
        for slot in (self,self.verifier):
            if not slot.current:continue
            try:progress=json.loads((self.directory/slot.current['id']/'progress.json').read_text())
            except (OSError,ValueError):progress={}
            active.append(dict(episode=Path(slot.current['source']).name,action=slot.current['action'],
                               source=slot.current['source'],progress=progress.get('message',''),
                               bytes=progress.get('bytes',0)))
        descriptions=[f"{item['episode']} · {dict(upload='上传中',verify='读回远端校验',delete='删除本地 bag')[item['action']]}"
                      for item in active]
        atomic_json(self.directory/'status.json',dict(pid=os.getpid(),state='draining' if self.draining else 'running',
            updated=time.time(),queued=queued,message='；'.join(descriptions) if active else self.message,
            last_result=self.last_result,current='\n'.join(item['source'] for item in active),
            watching=str(self.watcher.root or ''),active=active,
            progress='；'.join(f"{item['episode']}：{item['progress']}" for item in active),
            bytes=sum(item['bytes'] for item in active),error=self.error))

    def start_job(self, config, slot=None):
        slot=slot or self
        if slot.child is not None:return
        for job in self.jobs:
            if job['state'] not in {'queued','retry'} or job['next_try']>time.time():
                continue
            if (job['action']=='verify') != (slot is self.verifier):
                continue
            if not Path(job['source']).is_dir():
                job.update(state='superseded',error='源数据已改评或移走，本任务不再操作')
                self.save_job(job)
                continue
            # A queued D deletion must respect a newly enabled upload requirement.
            if job['action']=='delete' and job['folder']=='D' and config.get('upload_d_enabled', False) and not job.get('proof'):
                job.update(action='upload', upload_required=True)
                self.save_job(job)
            upload_allowed = config['upload_enabled'] and (
                job['folder'] != 'D' or config.get('upload_d_enabled', False))
            if job['action']=='upload' and not upload_allowed:
                if self.draining:
                    job.update(state='interrupted',error='自动上传已关闭，保留本地数据')
                    self.save_job(job)
                else:
                    self.message='自动上传已关闭，已入队数据暂停并保留本地'
                continue
            if job['action']=='delete' and not config['delete_enabled']:
                job['state']='kept';self.save_job(job)
                self.message=f"{Path(job['source']).name} · 自动删除已关闭，本地保留"
                self.last_result=self.message
                continue
            # Old serial workers take this guard exclusively. New workers share
            # it, with one exclusive lock per stage across all recorder sessions.
            state_root().mkdir(parents=True,exist_ok=True)
            stack=ExitStack()
            try:
                guard=stack.enter_context((state_root()/'operation.lock').open('a'))
                fcntl.flock(guard,fcntl.LOCK_SH|fcntl.LOCK_NB)
                lane='verify' if slot is self.verifier else 'upload'
                stream=stack.enter_context((state_root()/f'{lane}.lock').open('a'))
                fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:
                stack.close()
                self.message='排队等待对应上传／校验通道'
                return
            slot.global_lock=stack
            try:
                directory=self.directory/job['id']
                directory.mkdir(exist_ok=True)
                (directory/'result.json').unlink(missing_ok=True)
                job.update(state='running',attempts=job['attempts']+1)
                if job['action']=='upload':
                    job['verify_upload_enabled']=config.get('verify_upload_enabled',True)
                # Share the existing IO budget between the two concurrent lanes.
                attempt=dict(job,pipeline_budget=True)
                atomic_json(directory/'attempt.json',attempt)
                atomic_json(directory/'progress.json',dict(updated=time.time(),message='准备文件操作',bytes=0))
                self.save_job(job)
                command=[sys.executable,'-m','teleop_runtime.transfer_worker',str(directory)]
                if shutil.which('ionice'):
                    command=['ionice','-c','3',*command]
                slot.child=subprocess.Popen(command,stdin=subprocess.DEVNULL,start_new_session=True)
            except (OSError,ValueError) as error:
                slot.global_lock.close();slot.global_lock=None
                job.update(state='interrupted' if self.draining else 'retry',
                           error=str(error),next_try=time.time()+10)
                self.save_job(job)
                self.message=f'后台文件子进程启动失败：{error}'
                return
            slot.child_started=time.time()
            slot.current=job
            slot.timed_out=False
            self.error=False
            self.message=f"{Path(job['source']).name} · {dict(upload='上传中',verify='读回远端校验',delete='删除本地 bag')[job['action']]} {job['folder']}"
            return

    def poll_job(self, config, slot=None):
        slot=slot or self
        if slot.child is None:return
        job=slot.current
        directory=self.directory/job['id']
        try:progress=json.loads((directory/'progress.json').read_text())
        except (OSError,ValueError):progress={}
        if slot.child.poll() is None:
            if time.time()-progress.get('updated',slot.child_started)>self.idle_timeout and not slot.timed_out:
                # Kill only the disposable file-operation process, never recorder.
                slot.timed_out=True
                os.killpg(slot.child.pid,signal.SIGKILL)
                self.message=('网络/文件操作超时，等待操作中断；本地源数据保留'
                    if job['action'] in {'upload','verify'} else '本地删除操作超时，剩余文件保留在 .deleting')
            return
        slot.child.wait()
        slot.child=None
        slot.global_lock.close();slot.global_lock=None
        slot.current=None
        try:result=json.loads((directory/'result.json').read_text())
        except (OSError,ValueError):
            result=dict(ok=False,error=('文件操作超时或异常退出；本地源数据保留'
                if job['action'] in {'upload','verify'} else '删除操作中断；剩余文件见本机父目录下 .deleting'))
        self.error=not result.get('ok')
        if result.get('ok'):
            if job['action']=='upload':
                if result['result'].get('completion'):
                    job['proof']=result['result']['completion']
                    if config['delete_enabled']:
                        job.update(action='delete',state='queued',next_try=0)
                        self.message=f"{Path(job['source']).name} · 已上传（未读回校验），等待删除本地 bag"
                    else:
                        job['state']='done'
                        self.message=f"{Path(job['source']).name} · 已上传 {job['folder']}（未读回校验），本地保留"
                else:
                    job.update(manifest=result['result'],action='verify',state='queued',next_try=0)
                    self.message=f"{Path(job['source']).name} · 传输完成，等待读回远端校验"
            elif job['action']=='verify':
                job['proof']=result['result']
                if config['delete_enabled']:
                    job.update(action='delete',state='queued',next_try=0)
                    self.message=f"{Path(job['source']).name} · 上传校验完成，等待删除本地 bag"
                else:
                    job['state']='done'
                    self.message=f"{Path(job['source']).name} · 已上传 {job['folder']}，本地保留"
            else:
                job['state']='done'
                skipped="（未读回校验）" if (job.get("proof") or {}).get("readback_verified") is False else ""
                self.message=f"{Path(job['source']).name} · {result['result']['message']}（{job['folder']}）{skipped}"
            self.last_result=self.message
        else:
            job['error']=result.get('error','文件操作失败')
            job.update(state='interrupted' if self.draining or job['action']=='delete' else 'retry',
                       next_try=time.time()+min(60,10*job['attempts']))
            self.message=f"{Path(job['source']).name} · {job['error']}"
            self.last_result=self.message
        self.save_job(job)

    def run(self):
        try:os.nice(10)
        except OSError:pass
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGHUP,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,self.drain)
        self.status(True)
        while True:
            try:
                try:
                    config=read_settings()
                except (OSError,ValueError) as error:
                    config=dict(local_parent='',remote_parent='',upload_enabled=False,upload_d_enabled=False,delete_enabled=False)
                    self.message=f'后台设置无效，保留本地数据：{error}'
                if not self.owner_alive():self.drain()
                # Allow one bounded final discovery/settling period after recorder
                # exit, so its final classification isn't missed.
                if not self.draining or time.monotonic()-self.drain_started<4:
                    try:
                        previous_root=self.watcher.root
                        for job in self.watcher.scan(config,time.monotonic()):
                            self.jobs.append(job);self.save_job(job)
                        if self.watcher.root!=previous_root and self.current is None:
                            self.message='监控中；启动或切换目录时的已有数据已排除'
                    except (OSError,ValueError,RuntimeError) as error:
                        self.message=f'监控目录等待处理：{error}'
                        self.error=True
                self.poll_job(config)
                self.poll_job(config,self.verifier)
                self.start_job(config,self.verifier)
                self.start_job(config)
                self.status()
                pending=any(j['state'] in {'queued','retry','running'} for j in self.jobs)
                if self.draining and self.child is None and self.verifier.child is None and not pending and time.monotonic()-self.drain_started>=4:
                    break
            except (OSError,ValueError,RuntimeError) as error:
                self.message=f'后台文件任务等待处理：{error}'
                self.status()
            time.sleep(.5)
        interrupted=sum(job['state']=='interrupted' for job in self.jobs)
        final_message='采集器已关闭，当前文件操作已收尾'
        if interrupted:
            final_message+=f'；{interrupted} 个中断任务详见操作记录'
        atomic_json(self.directory/'status.json',dict(pid=os.getpid(),state='stopped',updated=time.time(),
            queued=0,message=final_message,last_result=self.last_result,error=self.error or bool(interrupted)))
        return 0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--owner-pid',type=int,required=True)
    args=parser.parse_args()
    return Service(args.owner_pid).run()

if __name__=='__main__':raise SystemExit(main())
