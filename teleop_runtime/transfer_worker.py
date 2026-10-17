"""One bounded file-operation attempt in a disposable host child process."""
from __future__ import annotations
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import time

from .transfer_settings import atomic_json, read_settings, state_root

CHUNK = 1024 * 1024


def mount_for(path):
    path = Path(os.path.abspath(path))
    matches = []
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        before, after = line.split(' - ',1)
        fields = before.split()
        # mountinfo escapes spaces, tabs, newlines and backslashes as octal.
        import re
        mount = Path(re.sub(r'\\([0-7]{3})', lambda m:chr(int(m[1],8)), fields[4]))
        filesystem, source, *_ = after.split()
        if path == mount or mount in path.parents:
            matches.append((len(mount.parts), int(fields[0]), str(mount), filesystem, source))
    if not matches:
        raise RuntimeError('目标不在已挂载的远端文件系统内')
    _, mount_id, mount, filesystem, source = max(matches)
    if filesystem not in {'fuse.sshfs','nfs','nfs4','cifs'}:
        raise RuntimeError('远端目录尚未挂载；保留本地数据，未写入本机空目录')
    return (str(mount_id), mount, filesystem, source)


def ready_state(source):
    state = json.loads((source/'collection_state.json').read_text())
    storage = state.get('storage') or {}
    home = state.get('after_home') or {}
    if state.get('transfer_ready') is False:
        raise RuntimeError('本段文件操作尚未完成')
    if storage.get('state') != 'saved' or not state.get('quality'):
        raise RuntimeError('本段分类尚未保存完成')
    if home and home.get('state') != 'done':
        raise RuntimeError('本段 Home 与状态写入尚未完成')
    if storage.get('folder') != source.parent.name:
        raise RuntimeError('本段分类路径已经变化')
    if not state.get('source_recording_id'):
        raise RuntimeError('缺少录制唯一编号，保留本地数据')
    return state


def source_inventory(source):
    result = {}
    for base, directories, files in os.walk(source, followlinks=False):
        for name in directories + files:
            path = Path(base)/name
            info = path.lstat()
            if stat.S_ISLNK(info.st_mode) or not (stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode)):
                raise RuntimeError(f'不处理符号链接或特殊文件：{path}')
        for name in files:
            path = Path(base)/name
            info = path.stat()
            result[str(path.relative_to(source))] = [info.st_size,info.st_mtime_ns,info.st_ino]
    return result


class Operation:
    def __init__(self, job, directory):
        self.job = job
        self.directory = Path(directory)
        self.rate = max(1,float(job.get('rate_mib',20))) * CHUNK
        self.total = 0
        self.started = time.monotonic()
        self.next_io = self.started
        self.last_progress = 0

    def progress(self, message, count=0):
        self.total += count
        if time.monotonic()-self.last_progress >= .5:
            atomic_json(self.directory/'progress.json', dict(updated=time.time(),message=message,bytes=self.total))
            self.last_progress=time.monotonic()
        rate=self.rate
        if self.job.get('pipeline_budget'):
            other='upload' if self.job['action']=='verify' else 'verify'
            with (state_root()/f'{other}.lock').open('a') as lock:
                try:
                    fcntl.flock(lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
                except BlockingIOError:
                    rate/=2
        now=time.monotonic()
        self.next_io=max(self.next_io,now)+count/rate
        delay=self.next_io-now
        if delay>0:
            time.sleep(delay)

    def digest(self, path, label):
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            while chunk := stream.read(CHUNK):
                digest.update(chunk)
                self.progress(label,len(chunk))
        return digest.hexdigest()

    def remote_context(self):
        """Pin the mounted filesystem separately in each disposable stage worker."""
        source=Path(self.job['source'])
        if source.is_symlink():
            raise RuntimeError('源 episode 不能是符号链接')
        source=source.resolve(strict=True)
        remote=Path(self.job['remote_parent']).expanduser().resolve()
        if source == remote or source in remote.parents or remote in source.parents:
            raise RuntimeError('本地与远端目录不能相同或相互包含')
        mounted=mount_for(remote)
        descriptor=os.open(remote,os.O_RDONLY|os.O_DIRECTORY)
        try:
            details=Path(f'/proc/self/fdinfo/{descriptor}').read_text().splitlines()
            actual=next(line.split(':',1)[1].strip() for line in details if line.startswith('mnt_id:'))
            if actual!=mounted[0]:
                raise RuntimeError('远端挂载已变化，未进行写入')
            os.fchdir(descriptor)
        finally:
            os.close(descriptor)
        state=ready_state(source)
        if state['source_recording_id']!=self.job['recording_id']:
            raise RuntimeError('录制编号已变化')
        return source,remote,mounted

    def copy_upload(self):
        source,remote,mounted=self.remote_context()
        inventory=source_inventory(source)
        hashes={}
        destination=Path('.')/self.job['folder']/source.name
        reused=destination.exists()
        if reused:
            hashes={name:self.digest(source/name,'计算本地校验') for name in inventory}
            old=json.loads((destination/'.upload-receipt.json').read_text())
            if old.get('recording_id')!=self.job['recording_id'] or old.get('hashes')!=hashes:
                raise RuntimeError(f'远端同名数据冲突，不覆盖：{destination}')
        else:
            staging=Path('.uploading')/self.job['id']
            staging.mkdir(parents=True,exist_ok=True)
            for name in inventory:
                target=staging/name
                target.parent.mkdir(parents=True,exist_ok=True)
                partial=target.with_name(target.name+'.part')
                digest=hashlib.sha256()
                with (source/name).open('rb') as reader, partial.open('wb') as writer:
                    while chunk:=reader.read(CHUNK):
                        digest.update(chunk)
                        writer.write(chunk)
                        self.progress(f'上传 {name}',len(chunk))
                    writer.flush()
                    os.fsync(writer.fileno())
                hashes[name]=digest.hexdigest()
                partial.replace(target)
        if source_inventory(source)!=inventory or mount_for(remote)!=mounted:
            raise RuntimeError('上传过程中源数据或远端挂载发生变化，本地数据保留')
        # This is only a copy manifest, NEVER a deletion authorization.
        return dict(inventory=inventory,hashes=hashes,recording_id=self.job['recording_id'],
                    mount=list(mounted),reused=reused)

    def verify_upload(self, manifest=None, *, read_back=True):
        manifest=manifest or self.job.get('manifest')
        if not manifest:
            raise RuntimeError('缺少上传清单，本地数据保留')
        source,remote,mounted=self.remote_context()
        inventory=manifest['inventory']; hashes=manifest['hashes']
        if (manifest['recording_id']!=self.job['recording_id'] or
                list(mounted)!=manifest['mount'] or source_inventory(source)!=inventory):
            raise RuntimeError('校验前源数据或挂载变化，本地数据保留')
        destination=Path('.')/self.job['folder']/source.name
        staging=Path('.uploading')/self.job['id']
        reused=manifest['reused'] or destination.exists()
        target=destination if reused else staging
        if reused:
            receipt=json.loads((destination/'.upload-receipt.json').read_text())
            if receipt.get('recording_id')!=self.job['recording_id'] or receipt.get('hashes')!=hashes:
                raise RuntimeError('远端同名数据冲突，不覆盖')
        for name in inventory:
            if read_back:
                if self.digest(target/name,'读回远端校验')!=hashes[name]:
                    raise RuntimeError(f'远端读回校验失败：{name}')
            elif (target/name).stat().st_size!=inventory[name][0]:
                raise RuntimeError(f'远端文件大小不一致：{name}')
        if source_inventory(source)!=inventory or mount_for(remote)!=mounted:
            raise RuntimeError('校验期间源或挂载变化，本地数据保留')
        if not reused:
            receipt=dict(recording_id=self.job['recording_id'],hashes=hashes,inventory=inventory,
                         readback_verified=read_back)
            atomic_json(staging/'.upload-receipt.json',receipt)
            destination.parent.mkdir(parents=True,exist_ok=True)
            reservation=Path('.upload-locks')/self.job['folder']/source.name
            reservation.parent.mkdir(parents=True,exist_ok=True)
            try:
                reservation.mkdir()
                atomic_json(reservation/'owner.json',dict(id=self.job['id']))
            except FileExistsError:
                if json.loads((reservation/'owner.json').read_text()).get('id')!=self.job['id']:
                    raise RuntimeError('远端同名上传正在进行或上次任务未收尾，不覆盖')
            if destination.exists():
                raise RuntimeError(f'远端同名目录已存在，不覆盖：{destination}')
            staging.rename(destination)
            (reservation/'owner.json').unlink()
            reservation.rmdir()
        if source_inventory(source)!=inventory or mount_for(remote)!=mounted:
            raise RuntimeError('发布期间源或挂载变化，本地数据保留')
        return dict(destination=str(remote/self.job['folder']/source.name),inventory=inventory,
                    hashes=hashes,recording_id=self.job['recording_id'],mount=list(mounted),readback_verified=read_back,transfer_completed=True)

    def upload(self):
        """Synchronous helper; the daemon schedules these stages independently."""
        return self.verify_upload(self.copy_upload())

    def delete(self):
        if not read_settings()['delete_enabled']:
            return dict(deleted=False,message='自动删除已关闭，保留本地数据')
        source=Path(self.job['source'])
        if source.is_symlink():
            raise RuntimeError('源 episode 不能是符号链接')
        source=source.resolve(strict=True)
        parent=source.parent.parent
        # Shared with the collector only for tiny local metadata/rename operations;
        # no network operation ever holds this lock.
        with (parent/'.episode-files.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            state=ready_state(source)
            if state['source_recording_id'] != self.job['recording_id']:
                raise RuntimeError('录制编号已变化，不删除')
            config = read_settings()
            if not config['delete_enabled']:
                return dict(deleted=False,message='自动删除已关闭')
            requires_proof = self.job['folder'] in {'A','B'} or (
                self.job['folder'] == 'D' and (config['upload_d_enabled'] or self.job.get('upload_required', False)))
            if requires_proof:
                proof=self.job.get('proof') or {}
                if proof.get('readback_verified') is False and not (
                        self.job.get('verify_upload_enabled') is False and proof.get('transfer_completed') is True):
                    raise RuntimeError('缺少跳过读回校验的传输完成记录，本地数据保留')
                if proof.get('recording_id') != self.job['recording_id'] or source_inventory(source) != proof.get('inventory'):
                    raise RuntimeError('没有匹配当前本地数据的远端校验证明，不删除')
            elif self.job['folder'] not in {'C','D'}:
                raise RuntimeError('无效分类，不删除')
            atomic_json(self.directory/'source-state.json',state)
            quarantine=parent/'.deleting'/self.job['id']
            quarantine.parent.mkdir(exist_ok=True)
            source.rename(quarantine)
        # Never hold up the collector while unlinking large numbers of local files.
        shutil.rmtree(quarantine)
        return dict(deleted=True,message='本地 bag 已删除')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    directory=args.directory
    # A crashed supervisor cannot leave an unowned deletion worker behind.
    import ctypes
    import signal
    parent=os.getppid()
    ctypes.CDLL(None).prctl(1,signal.SIGKILL,0,0,0)
    if os.getppid()!=parent:
        return 2
    job=json.loads((directory/'attempt.json').read_text())
    try:
        operation=Operation(job,directory)
        methods={'upload':operation.copy_upload,'verify':operation.verify_upload,'delete':operation.delete}
        result=methods[job['action']]()
        if job['action']=='upload' and not job.get('verify_upload_enabled',True):
            result=dict(manifest=result,completion=operation.verify_upload(result,read_back=False))
        atomic_json(directory/'result.json',dict(ok=True,result=result))
    except Exception as error:
        atomic_json(directory/'result.json',dict(ok=False,error=str(error)))
        return 1
    return 0

if __name__=='__main__':
    raise SystemExit(main())
