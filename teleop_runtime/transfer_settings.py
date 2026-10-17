"""Local-only transfer preferences and bounded status snapshots; no device IO."""
from __future__ import annotations
import fcntl
import json
import os
from pathlib import Path
import tempfile
import time


def config_path():
    return Path(os.environ.get('COLLECTION_TRANSFER_CONFIG',
        str(Path(os.environ.get('XDG_CONFIG_HOME', str(Path.home()/'.config'))) / 'gello-teleop/collection-transfer.json')))


def state_root():
    return Path(os.environ.get('COLLECTION_TRANSFER_STATE',
        str(Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state'))) / 'gello-teleop/collection-transfer')))


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.transfer-', delete=False) as stream:
            name = Path(stream.name)
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        name.replace(path)
    finally:
        if name is not None:
            name.unlink(missing_ok=True)


def read_settings():
    data = dict(local_parent='', remote_parent=str(Path.home()/'company-remote-2'),
                upload_enabled=False, upload_d_enabled=False, verify_upload_enabled=True, delete_enabled=False, rate_mib=20)
    try:
        value = json.loads(config_path().read_text())
        if isinstance(value, dict):
            data.update({key: value[key] for key in data if key in value})
    except FileNotFoundError:
        pass
    if not all(isinstance(data[key], bool) for key in ('upload_enabled','upload_d_enabled','verify_upload_enabled','delete_enabled')):
        raise ValueError('自动传输开关配置无效')
    return data


def update_settings(**changes):
    path = config_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = read_settings()
        data.update(changes)
        atomic_json(path, data)
    return data


def read_status():
    root = state_root()/'sessions'
    if not root.exists():
        return dict(state='stopped', message='采集器未启动；设置已保存', queued=0)
    paths = sorted(root.glob('*/status.json'), key=lambda p:p.stat().st_mtime, reverse=True)[:8]
    reports = []
    for path in paths:
        try:
            report=json.loads(path.read_text())
            if report.get('state')!='stopped' and not Path(f"/proc/{report.get('pid',0)}").exists():
                report.update(state='stopped',message='后台已中断；未完成数据保留',error=True)
            reports.append(report)
        except (OSError, ValueError):
            continue
    if not reports:
        return dict(state='stopped', message='等待采集器启动', queued=0)
    active = [r for r in reports if r.get('state') != 'stopped']
    selected = max(active or reports, key=lambda r:r.get('updated',0))
    selected = dict(selected)
    selected['queued'] = sum(r.get('queued',0) for r in active)
    return selected
