"""Offline tests only. Local temporary directories simulate a mounted remote."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from teleop_runtime.transfer_settings import update_settings, read_settings, atomic_json
from teleop_runtime.transfer_worker import Operation, mount_for
from teleop_runtime.collection_transfer import Watcher, Service


@pytest.fixture(autouse=True)
def isolated(tmp_path,monkeypatch):
    monkeypatch.setenv('COLLECTION_TRANSFER_CONFIG',str(tmp_path/'config.json'))
    monkeypatch.setenv('COLLECTION_TRANSFER_STATE',str(tmp_path/'state'))
    monkeypatch.chdir(tmp_path)


def episode(root, folder, number=1, home='done'):
    path=root/folder/f'episode{number}'
    path.mkdir(parents=True)
    atomic_json(path/'collection_state.json',dict(source_recording_id=f'id-{number}',quality={'A':'优等','B':'一般','C':'报错','D':'放弃'}[folder],
        storage=dict(state='saved',folder=folder),after_home=dict(state=home),transfer_ready=home=='done'))
    (path/'metadata.yaml').write_text('synthetic offline test')
    (path/'bag.mcap').write_bytes(b'offline test data'*100)
    return path


def fake_mount_identity(remote):
    descriptor=os.open(remote,os.O_RDONLY|os.O_DIRECTORY)
    try:
        lines=Path(f'/proc/self/fdinfo/{descriptor}').read_text().splitlines()
        identity=next(line.split(':',1)[1].strip() for line in lines if line.startswith('mnt_id:'))
        return (identity,str(remote),'fuse.sshfs','test')
    finally:
        os.close(descriptor)


def job_for(path,tmp_path,action='upload'):
    directory=tmp_path/'job'
    directory.mkdir(exist_ok=True)
    job=dict(id='job1',source=str(path),folder=path.parent.name,recording_id='id-1',
        remote_parent=str(tmp_path/'remote'),rate_mib=1000,action=action)
    return job,directory


def test_startup_and_switching_roots_never_queue_existing_files(tmp_path):
    local=tmp_path/'local'
    old=[episode(local,folder,index) for index,folder in enumerate('ABCD')]
    config=dict(local_parent=str(local),remote_parent=str(tmp_path/'remote'),upload_enabled=True,delete_enabled=True)
    watch=Watcher(settle=0)
    assert not watch.scan(config,0)
    assert not watch.scan(config,1)
    assert all(path.exists() for path in old)
    new=episode(local,'A',99)
    assert not watch.scan(config,2)
    queued=watch.scan(config,3)
    assert len(queued)==1 and queued[0]['source']==str(new) and queued[0]['action']=='upload'
    other=tmp_path/'other';episode(other,'D',100)
    assert not watch.scan(dict(config,local_parent=str(other)),4)
    assert not watch.scan(dict(config,local_parent=str(other)),5)


def test_no_deletion_or_backfill_while_switches_are_off(tmp_path):
    root=tmp_path/'local';root.mkdir()
    config=dict(local_parent=str(root),remote_parent='',upload_enabled=False,delete_enabled=False)
    watcher=Watcher(settle=0)
    watcher.scan(config,0)
    for index,folder in enumerate('ABCD'):episode(root,folder,index)
    watcher.scan(config,1)
    assert not watcher.scan(config,2)
    assert not watcher.scan(dict(config,upload_enabled=True,delete_enabled=True),3)
    episode(root,'C',10)
    watcher.scan(dict(config,delete_enabled=True),4)
    result=watcher.scan(dict(config,delete_enabled=True),5)
    assert len(result)==1 and result[0]['action']=='delete'


def test_home_and_metadata_must_finish_before_queueing(tmp_path):
    root=tmp_path/'local';root.mkdir()
    config=dict(local_parent=str(root),remote_parent='',upload_enabled=True,delete_enabled=True)
    watcher=Watcher(settle=0);watcher.scan(config,0)
    path=episode(root,'A',home='running')
    watcher.scan(config,1)
    assert not watcher.scan(config,2)
    state=json.loads((path/'collection_state.json').read_text())
    state['after_home']['state']='done';state['transfer_ready']=True
    atomic_json(path/'collection_state.json',state)
    watcher.scan(config,3)
    assert len(watcher.scan(config,4))==1


def test_unmounted_remote_never_receives_writes_or_authorizes_deletion(tmp_path):
    source=episode(tmp_path/'local','A')
    job,directory=job_for(source,tmp_path)
    Path(job['remote_parent']).mkdir()
    with pytest.raises(RuntimeError,match='挂载'):
        Operation(job,directory).upload()
    assert not list(Path(job['remote_parent']).iterdir())
    assert (source/'bag.mcap').is_file()


def test_upload_readback_then_delete_and_idempotent_retry(tmp_path,monkeypatch):
    source=episode(tmp_path/'local','A')
    job,directory=job_for(source,tmp_path)
    remote=Path(job['remote_parent']);remote.mkdir()
    monkeypatch.setattr('teleop_runtime.transfer_worker.mount_for',lambda path:fake_mount_identity(remote))
    proof=Operation(job,directory).upload()
    destination=Path(proof['destination'])
    assert (destination/'bag.mcap').read_bytes()==(source/'bag.mcap').read_bytes()
    assert Operation(job,directory).upload()['hashes']==proof['hashes']
    update_settings(delete_enabled=True)
    result=Operation(dict(job,action='delete',proof=proof),directory).delete()
    assert result['deleted'] and not source.exists()
    assert (destination/'bag.mcap').is_file()


def test_corrupt_remote_and_same_name_conflict_keep_source(tmp_path,monkeypatch):
    source=episode(tmp_path/'local','A')
    job,directory=job_for(source,tmp_path)
    remote=Path(job['remote_parent']);remote.mkdir()
    monkeypatch.setattr('teleop_runtime.transfer_worker.mount_for',lambda path:fake_mount_identity(remote))
    proof=Operation(job,directory).upload()
    destination=Path(proof['destination'])
    (destination/'bag.mcap').write_bytes(b'corrupt')
    with pytest.raises(RuntimeError,match='校验失败'):
        Operation(job,directory).upload()
    assert source.exists()
    receipt=json.loads((destination/'.upload-receipt.json').read_text())
    receipt['recording_id']='someone-else'
    atomic_json(destination/'.upload-receipt.json',receipt)
    with pytest.raises(RuntimeError,match='冲突'):
        Operation(job,directory).upload()
    assert source.exists()


def test_delete_switch_and_changed_source_prevent_delete(tmp_path):
    source=episode(tmp_path/'local','A')
    job,directory=job_for(source,tmp_path,action='delete')
    assert not Operation(job,directory).delete()['deleted']
    update_settings(delete_enabled=True)
    with pytest.raises(RuntimeError,match='校验证明'):
        Operation(job,directory).delete()
    assert source.exists()
    cd=episode(tmp_path/'local','D',2)
    result=Operation(dict(job,source=str(cd),recording_id='id-2',folder='D'),directory).delete()
    assert result['deleted'] and not cd.exists()


def test_interrupted_copy_keeps_source_and_hidden_partial(tmp_path,monkeypatch):
    source=episode(tmp_path/'local','B')
    job,directory=job_for(source,tmp_path)
    remote=Path(job['remote_parent']);remote.mkdir()
    monkeypatch.setattr('teleop_runtime.transfer_worker.mount_for',lambda path:fake_mount_identity(remote))
    operation=Operation(job,directory)
    normal=operation.progress
    def fail(message,count=0):
        if message.startswith('上传 '):raise OSError('network interrupted')
        normal(message,count)
    monkeypatch.setattr(operation,'progress',fail)
    with pytest.raises(OSError,match='network interrupted'):operation.upload()
    assert source.exists() and not (remote/'B'/source.name).exists()
    assert list((remote/'.uploading'/'job1').rglob('*.part'))


def test_shutdown_finishes_current_process_and_cancels_network_retry(tmp_path):
    service=Service(os.getpid(),tmp_path/'session',idle_timeout=.01)
    source=episode(tmp_path/'local','A')
    job,directory=job_for(source,tmp_path)
    job.update(state='running',attempts=1,next_try=0)
    directory=service.directory/job['id'];directory.mkdir()
    atomic_json(directory/'progress.json',dict(updated=time.time()-10))
    service.jobs=[job];service.current=job
    service.global_lock=(tmp_path/'lock').open('a')
    service.child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True)
    service.drain()
    service.poll_job(dict(delete_enabled=True))
    service.child.wait(timeout=3)
    service.poll_job(dict(delete_enabled=True))
    assert job['state']=='interrupted' and source.exists()
    assert service.child is None


def test_real_daemon_ignores_old_data_queues_and_drains_on_term(tmp_path):
    root=tmp_path/'local';root.mkdir()
    old=episode(root,'D',0)
    update_settings(local_parent=str(root),remote_parent=str(tmp_path/'not-mounted'),
                    upload_enabled=True,delete_enabled=True)
    repository=Path(__file__).resolve().parents[1]
    environment=dict(os.environ,PYTHONPATH=str(repository))
    child=subprocess.Popen([sys.executable,'-m','teleop_runtime.collection_transfer',
                            '--owner-pid',str(os.getpid())],env=environment)
    def wait(predicate,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            assert child.poll() is None
            time.sleep(.05)
        raise AssertionError('daemon did not reach expected state')
    def status():
        paths=list((tmp_path/'state/sessions').glob('*/status.json'))
        if not paths:return {}
        try:return json.loads(paths[0].read_text())
        except ValueError:return {}
    try:
        wait(lambda:status().get('watching')==str(root))
        assert old.exists()
        new=episode(root,'D',1)
        upload=episode(root,'A',2)
        wait(lambda:not new.exists())
        wait(lambda:any('挂载' in path.read_text() for path in (tmp_path/'state/sessions').glob('*/*/job.json')))
        assert upload.exists() and old.exists()
        child.terminate()
        assert child.wait(timeout=8)==0
        assert status()['state']=='stopped'
        assert old.exists() and upload.exists()
    finally:
        if child.poll() is None:
            child.kill();child.wait()


@pytest.mark.parametrize('upload,d_upload,delete,expected', [
    (True,True,True,'upload'), (True,True,False,'upload'),
    (False,True,True,None), (True,False,True,'delete'),
    (True,False,False,None), (False,False,True,'delete')])
def test_d_policy_only_new_data(tmp_path, upload, d_upload, delete, expected):
    root=tmp_path/'local';root.mkdir()
    config=dict(local_parent=str(root),remote_parent=str(tmp_path/'remote'),
                upload_enabled=upload,upload_d_enabled=d_upload,delete_enabled=delete)
    old=episode(root,'D',0)
    watcher=Watcher(settle=0);assert not watcher.scan(config,0)
    new=episode(root,'D',1)
    watcher.scan(config,1)
    jobs=watcher.scan(config,2)
    assert [job['action'] for job in jobs]==([expected] if expected else [])
    assert old.exists() and new.exists()


def test_d_deletion_checks_latest_policy_and_latched_upload_requirement(tmp_path,monkeypatch):
    source=episode(tmp_path/'local','D')
    job,directory=job_for(source,tmp_path,action='delete')
    update_settings(delete_enabled=True,upload_d_enabled=True)
    with pytest.raises(RuntimeError,match='校验证明'):
        Operation(job,directory).delete()
    update_settings(upload_d_enabled=False)
    with pytest.raises(RuntimeError,match='校验证明'):
        Operation(dict(job,upload_required=True),directory).delete()
    update_settings(upload_d_enabled=True)
    remote=Path(job['remote_parent']);remote.mkdir()
    monkeypatch.setattr('teleop_runtime.transfer_worker.mount_for',lambda path:fake_mount_identity(remote))
    proof=Operation(job,directory).upload()
    assert Operation(dict(job,proof=proof,upload_required=True),directory).delete()['deleted']
    assert not source.exists() and (remote/'D/episode1/bag.mcap').is_file()


def test_queued_d_delete_replaced_by_upload_requirement(tmp_path):
    source=episode(tmp_path/'local','D')
    service=Service(os.getpid())
    job,_=job_for(source,tmp_path,action='delete')
    job.update(state='queued',next_try=0,attempts=0)
    service.jobs=[job]
    service.start_job(dict(upload_enabled=False,upload_d_enabled=True,delete_enabled=True))
    assert job['action']=='upload' and job['upload_required']
    assert service.child is None and source.exists()


def test_pipeline_copy_finishes_while_previous_verify_waits(tmp_path,monkeypatch):
    """Real child workers, local fake mount, gated verification: no network/robot."""
    root=tmp_path/'local'
    sources=[episode(root,'A',number) for number in (1,2)]
    remote=tmp_path/'remote';remote.mkdir()
    update_settings(upload_enabled=True,delete_enabled=True)
    config=read_settings()
    service=Service(os.getpid(),tmp_path/'pipeline')
    for index,source in enumerate(sources,1):
        service.jobs.append(dict(id=f'job{index}',source=str(source),folder='A',
            recording_id=f'id-{index}',remote_parent=str(remote),rate_mib=1000,
            action='upload',state='queued',attempts=0,next_try=0))
    entered=tmp_path/'verifying';release=tmp_path/'release'
    repository=Path(__file__).resolve().parents[1]
    bootstrap='''
import os,sys,time
from pathlib import Path
from teleop_runtime import transfer_worker as worker
def fake_mount(path):
    descriptor=os.open(path,os.O_RDONLY|os.O_DIRECTORY)
    try:
        identity=next(line.split(':',1)[1].strip() for line in Path(f'/proc/self/fdinfo/{descriptor}').read_text().splitlines() if line.startswith('mnt_id:'))
        return (identity,str(path),'fuse.sshfs','offline-test')
    finally:os.close(descriptor)
worker.mount_for=fake_mount
original=worker.Operation.digest
def gated(self,path,label):
    if self.job['action']=='verify' and self.job['recording_id']=='id-1':
        Path(ENTERED).write_text('verifying')
        while not Path(RELEASE).exists():time.sleep(.01)
    return original(self,path,label)
worker.Operation.digest=gated
sys.argv=[sys.argv[0],sys.argv[1]]
raise SystemExit(worker.main())
'''
    bootstrap=bootstrap.replace('ENTERED',repr(str(entered))).replace('RELEASE',repr(str(release)))
    actual_popen=subprocess.Popen
    def spawn(command,**kwargs):
        return actual_popen([sys.executable,'-c',bootstrap,command[-1]],
            env=dict(os.environ,PYTHONPATH=str(repository)),**kwargs)
    monkeypatch.setattr('teleop_runtime.collection_transfer.subprocess.Popen',spawn)
    observed_parallel=False
    def pump_until(predicate):
        nonlocal observed_parallel
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            service.poll_job(config);service.poll_job(config,service.verifier)
            service.start_job(config,service.verifier);service.start_job(config)
            if service.child and service.verifier.child:
                if service.current['action']=='upload':observed_parallel=True
                service.status(True)
                status=json.loads((service.directory/'status.json').read_text())
                assert 'verify' in {item['action'] for item in status['active']}
                assert len(status['active'])==2
            if predicate():return
            time.sleep(.02)
        raise AssertionError(str(service.jobs))
    try:
        pump_until(lambda:entered.exists() and service.jobs[1]['action']=='verify')
        assert observed_parallel
        assert service.jobs[0]['state']=='running'
        assert service.jobs[1]['state']=='queued'
        assert all(path.exists() for path in sources)
        assert not (remote/'A/episode1').exists()
        assert (remote/'.uploading/job2/bag.mcap').is_file()
        # Recorder shutdown drains BOTH stages, including copied-but-unverified data.
        service.drain();release.touch()
        pump_until(lambda:all(job['state']=='done' for job in service.jobs))
        assert all(not path.exists() for path in sources)
        assert all((remote/'A'/f'episode{number}/bag.mcap').is_file() for number in (1,2))
    finally:
        release.touch()
        for slot in (service,service.verifier):
            if slot.child:
                if slot.child.poll() is None:slot.child.kill()
                slot.child.wait(timeout=3)
            if slot.global_lock:slot.global_lock.close()


def test_copied_manifest_never_authorizes_local_deletion(tmp_path,monkeypatch):
    source=episode(tmp_path/'local','D')
    job,directory=job_for(source,tmp_path)
    remote=Path(job['remote_parent']);remote.mkdir()
    monkeypatch.setattr('teleop_runtime.transfer_worker.mount_for',lambda path:fake_mount_identity(remote))
    manifest=Operation(job,directory).copy_upload()
    assert not (remote/'D/episode1').exists()
    update_settings(delete_enabled=True,upload_d_enabled=True)
    with pytest.raises(RuntimeError,match='校验证明'):
        Operation(dict(job,manifest=manifest),directory).delete()
    (remote/'.uploading/job1/bag.mcap').write_bytes(b'corrupt copy')
    with pytest.raises(RuntimeError,match='校验失败'):
        Operation(dict(job,manifest=manifest),directory).verify_upload()
    assert source.exists() and not (remote/'D/episode1').exists()


def test_no_readback_mode_publishes_and_deletes_a_b_d(tmp_path,monkeypatch):
    root=tmp_path/'local'
    sources=[episode(root,folder,number) for number,folder in enumerate('ABD',1)]
    remote=tmp_path/'remote';remote.mkdir()
    update_settings(upload_enabled=True,upload_d_enabled=True,verify_upload_enabled=False,delete_enabled=True)
    config=read_settings();service=Service(os.getpid(),tmp_path/'no-readback')
    for number,source in enumerate(sources,1):
        service.jobs.append(dict(id=f'job{number}',source=str(source),folder=source.parent.name,
            recording_id=f'id-{number}',remote_parent=str(remote),rate_mib=1000,
            action='upload',state='queued',attempts=0,next_try=0,upload_required=True))
    repository=Path(__file__).resolve().parents[1]
    bootstrap='''
import sys
from pathlib import Path
from teleop_runtime import transfer_worker as worker
from teleop_runtime.test_collection_transfer import fake_mount_identity
worker.mount_for=lambda path:fake_mount_identity(path)
def forbidden(*args,**kwargs):raise AssertionError('unexpected readback')
worker.Operation.digest=forbidden
sys.argv=[sys.argv[0],sys.argv[1]]
raise SystemExit(worker.main())
'''
    actual_popen=subprocess.Popen
    def spawn(command,**kwargs):
        return actual_popen([sys.executable,'-c',bootstrap,command[-1]],
                           env=dict(os.environ,PYTHONPATH=str(repository)),**kwargs)
    monkeypatch.setattr('teleop_runtime.collection_transfer.subprocess.Popen',spawn)
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            service.poll_job(config);service.poll_job(config,service.verifier)
            service.start_job(config,service.verifier);service.start_job(config)
            assert service.verifier.child is None
            if all(job['state']=='done' for job in service.jobs):break
            time.sleep(.02)
        assert all(job['state']=='done' for job in service.jobs),service.jobs
        assert all(not source.exists() for source in sources)
        for number,folder in enumerate('ABD',1):
            destination=remote/folder/f'episode{number}'
            assert (destination/'bag.mcap').is_file()
            receipt=json.loads((destination/'.upload-receipt.json').read_text())
            assert receipt['readback_verified'] is False
            assert service.jobs[number-1]['verify_upload_enabled'] is False
    finally:
        for slot in (service,service.verifier):
            if slot.child:
                if slot.child.poll() is None:slot.child.kill()
                slot.child.wait(timeout=3)
            if slot.global_lock:slot.global_lock.close()


def test_no_readback_still_rejects_incomplete_copy(tmp_path,monkeypatch):
    source=episode(tmp_path/'local','A')
    job,directory=job_for(source,tmp_path)
    remote=Path(job['remote_parent']);remote.mkdir()
    monkeypatch.setattr('teleop_runtime.transfer_worker.mount_for',lambda path:fake_mount_identity(remote))
    manifest=Operation(job,directory).copy_upload()
    (remote/'.uploading/job1/bag.mcap').write_bytes(b'truncated')
    with pytest.raises(RuntimeError,match='大小不一致'):
        Operation(dict(job,manifest=manifest),directory).verify_upload(read_back=False)
    assert source.exists() and not (remote/'A/episode1').exists()


def test_verifier_timeout_does_not_kill_concurrent_upload(tmp_path):
    service=Service(os.getpid(),tmp_path/'session',idle_timeout=.01)
    children=[]
    try:
        for index,slot in enumerate((service,service.verifier),1):
            source=episode(tmp_path/'local','A',index)
            job=dict(id=f'job{index}',source=str(source),folder='A',recording_id=f'id-{index}',
                     state='running',action='verify' if index==2 else 'upload',attempts=1,next_try=0)
            directory=service.directory/job['id'];directory.mkdir()
            atomic_json(directory/'progress.json',dict(updated=time.time()-10 if index==2 else time.time()))
            slot.current=job;service.jobs.append(job)
            slot.global_lock=(tmp_path/f'lock{index}').open('a')
            slot.child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True)
            children.append(slot.child)
        service.drain()
        service.poll_job(dict(delete_enabled=True),service.verifier)
        children[1].wait(timeout=3)
        service.poll_job(dict(delete_enabled=True),service.verifier)
        assert service.verifier.child is None
        assert service.jobs[1]['state']=='interrupted'
        assert service.child.poll() is None
        assert all(Path(job['source']).exists() for job in service.jobs)
    finally:
        for child in children:
            if child.poll() is None:child.kill()
            child.wait(timeout=3)
        for slot in (service,service.verifier):
            if slot.global_lock:slot.global_lock.close()
