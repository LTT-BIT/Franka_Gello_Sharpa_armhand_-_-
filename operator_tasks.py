"""Persistent task IDs shared by the ROS-free operator and ROS Home server."""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import tempfile
import subprocess
import sys
import uuid

OPERATOR_TASKS = {
    "powder_weighing": "粉末称量",
    "assembly": "装配",
    "bean_picking": "夹豆",
}
DEFAULT_OPERATOR_TASK = "powder_weighing"


def task_registry_path() -> Path:
    if os.environ.get('OPERATOR_TASKS_FILE'):
        return Path(os.environ['OPERATOR_TASKS_FILE']).expanduser()
    # Docker mounts the same host data root at /data.
    if os.path.ismount('/data'):
        return Path('/data/operator_gui/tasks.json')
    root=os.environ.get('TELEOP_DATA_ROOT','')
    if not root:
        deployment=Path(__file__).resolve().parent/'docker/.env'
        if deployment.is_file():
            for line in deployment.read_text().splitlines():
                if line.startswith('TELEOP_DATA_ROOT='):
                    root=line.split('=',1)[1].strip().strip('\"\'');break
    return (Path(root).expanduser()/'operator_gui/tasks.json' if root else
            Path.home()/'.config/gello-teleop/tasks.json')


def valid_task_id(value: object) -> bool:
    return isinstance(value,str) and (value in OPERATOR_TASKS or
        re.fullmatch(r'task_[0-9a-f]{32}',value) is not None)


def get_operator_tasks(path=None) -> dict[str,str]:
    path=Path(path) if path is not None else task_registry_path()
    try:
        document=json.loads(path.read_text())
    except FileNotFoundError:
        return dict(OPERATOR_TASKS)
    tasks=document.get('tasks') if isinstance(document,dict) and document.get('version')==1 else None
    if (not isinstance(tasks,dict) or not tasks or
            any(not valid_task_id(key) or not isinstance(label,str) or not label.strip() or
                len(label)>40 for key,label in tasks.items()) or len(set(tasks.values()))!=len(tasks)):
        raise ValueError('任务配置无效；请检查 '+str(path))
    return dict(tasks)


def require_operator_task(value: object) -> str:
    task=str(value)
    if task not in get_operator_tasks():
        raise ValueError(f"unknown operator task: {task!r}")
    return task


def task_label(task: str) -> str:
    return get_operator_tasks().get(task, OPERATOR_TASKS.get(task,task))


@contextmanager
def task_registry_lock(path=None):
    path=Path(path) if path is not None else task_registry_path()
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        yield path


def deleted_operator_tasks(path=None) -> set[str]:
    path=Path(path) if path is not None else task_registry_path()
    try:document=json.loads(path.read_text())
    except FileNotFoundError:return set()
    deleted=document.get('deleted_tasks',[])
    if not isinstance(deleted,list) or any(not valid_task_id(task) for task in deleted):
        raise ValueError('永久删除任务记录无效')
    return set(deleted)


def _write_registry(path,tasks,deleted):
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,
                prefix='.tasks-',delete=False) as stream:
            temporary=Path(stream.name)
            json.dump(dict(version=1,tasks=tasks,deleted_tasks=sorted(deleted)),stream,
                      ensure_ascii=False,indent=2)
            # Container-created task lists must remain readable by the host GUI.
            os.fchmod(stream.fileno(),0o644)
            stream.flush();os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)


def _delegate_task_edit(action,value):
    root=Path(__file__).resolve().parent
    env={**os.environ,'PYTHONPATH':str(root)+os.pathsep+os.environ.get('PYTHONPATH','')}
    if os.environ.get('OPERATOR_TASKS_FILE'):
        # Explicit custom/test roots stay on the host; never mount arbitrary paths.
        python=Path(sys.base_prefix)/'envs'/os.environ.get('TELEOP_CONDA_ENV','gello-upper-body-teleop')/'bin/python'
        if not python.is_file():raise OSError('任务清理需要安装 PyYAML 的遥操 Python 环境')
        command=[str(python),'-m','teleop_runtime.task_cleanup',action,value]
        cwd=root
    else:
        # Metadata only: no ROS node, device initialization or robot commands.
        command=['docker','compose','run','--rm','--no-deps','-T',
                 '-e','PYTHONPATH=/workspace/franka_upper_body_teleop',
                 'tools','python3','-m','teleop_runtime.task_cleanup',action,value]
        cwd=root/'docker'
    try:
        completed=subprocess.run(command,cwd=cwd,env=env,text=True,capture_output=True,timeout=20)
    except subprocess.TimeoutExpired as error:
        raise OSError('任务文件操作超时；请确认结果后重试删除') from error
    if completed.returncode:
        raise OSError((completed.stderr or completed.stdout).strip()[-700:])
    # The helper prints exactly one JSON result; Docker messages are on stderr.
    return json.loads(completed.stdout.strip().splitlines()[-1])['result']


def _edit_tasks(change):
    with task_registry_lock() as path:
        tasks=get_operator_tasks(path)
        deleted=deleted_operator_tasks(path)
        result=change(tasks)
        _write_registry(path,tasks,deleted)
    return result


def add_operator_task(label: str) -> str:
    label=label.strip()
    if not label or len(label)>40 or any(ord(char)<32 for char in label):
        raise ValueError('任务名称须为 1～40 个字符，不能包含换行或控制字符')
    def change(tasks):
        if label in tasks.values():raise ValueError('已有同名任务')
        key='task_'+uuid.uuid4().hex
        tasks[key]=label
        return key
    try:return _edit_tasks(change)
    except PermissionError:
        if os.environ.get('TELEOP_TASK_HELPER'):raise
        return _delegate_task_edit('add',label)


def task_storage_paths(path=None):
    path=Path(path) if path is not None else task_registry_path()
    data_root=path.parent.parent if path.parent.name=='operator_gui' else path.parent
    return path.with_name('poses.yaml'),data_root/'arm_ui/absolute_trajectories'


def _purge_task_data(task,path):
    poses,trajectories=task_storage_paths(path)
    trajectory=trajectories/(task+'.yaml')
    data_root=path.parent.parent if path.parent.name=='operator_gui' else path.parent
    for target in (poses,trajectory):
        for entry in (target,*target.parents):
            if entry==data_root:break
            if entry.is_symlink():raise ValueError('拒绝清理符号链接：'+str(entry))
        if target.exists() and not target.is_file():raise ValueError('任务数据路径不是文件：'+str(target))
    if poses.exists():
        import yaml
        try:
            with poses.open(encoding='utf-8') as stream:document=yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError('Home 姿态文件无法解析，未清理：'+str(poses)) from error
        if not isinstance(document,dict) or document.get('version')!=1 or not isinstance(document.get('tasks'),dict):
            raise ValueError('Home 姿态文件无效，未清理：'+str(poses))
        if task in document['tasks']:
            del document['tasks'][task]
            temporary=None
            try:
                with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=poses.parent,
                        prefix='.operator_poses.',suffix='.tmp',delete=False) as stream:
                    temporary=Path(stream.name)
                    yaml.safe_dump(document,stream,allow_unicode=True,sort_keys=False)
                    stream.flush();os.fsync(stream.fileno())
                temporary.replace(poses)
            finally:
                if temporary is not None:temporary.unlink(missing_ok=True)
    trajectory.unlink(missing_ok=True)


def delete_operator_task(task: str) -> None:
    if not valid_task_id(task):raise ValueError('无效任务编号')
    try:
        # GUI Python intentionally has no ROS/YAML dependencies. Use its existing
        # tools environment if a pose file needs YAML editing.
        if task_storage_paths()[0].exists():
            import yaml  # noqa: F401
        with task_registry_lock() as path:
            tasks=get_operator_tasks(path)
            deleted=deleted_operator_tasks(path)
            if task not in tasks and task not in deleted:raise ValueError('任务已不存在')
            if task in tasks and len(tasks)==1:raise ValueError('至少保留一个任务')
            tasks.pop(task,None)
            deleted.add(task)
            # Revoke first. If cleanup fails, no service may use cached poses;
            # repeating delete retries the remaining cleanup under the same lock.
            _write_registry(path,tasks,deleted)
            _purge_task_data(task,path)
    except (PermissionError,ModuleNotFoundError):
        if os.environ.get('TELEOP_TASK_HELPER'):raise
        return _delegate_task_edit('delete',task)
