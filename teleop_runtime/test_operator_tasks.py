"""Task management edits temporary metadata only, never hardware poses."""
import json
import pytest
from operator_tasks import (OPERATOR_TASKS,get_operator_tasks,add_operator_task,
    delete_operator_task,require_operator_task,task_registry_path)


@pytest.fixture(autouse=True)
def isolated_registry(tmp_path,monkeypatch):
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(tmp_path/'operator_gui/tasks.json'))


def test_add_remove_and_restore_registry_without_reusing_pose_identity():
    assert get_operator_tasks()==OPERATOR_TASKS
    first=add_operator_task('新任务')
    assert require_operator_task(first)==first
    assert get_operator_tasks()[first]=='新任务'
    delete_operator_task(first)
    with pytest.raises(ValueError):require_operator_task(first)
    second=add_operator_task('新任务')
    assert second!=first
    assert json.loads(task_registry_path().read_text())['tasks'][second]=='新任务'


def test_duplicate_invalid_name_and_last_task_are_rejected():
    for label in ('', ' ', 'a'*41,'bad\nname','装配'):
        with pytest.raises(ValueError):add_operator_task(label)
    delete_operator_task('assembly');delete_operator_task('bean_picking')
    with pytest.raises(ValueError,match='至少'):delete_operator_task('powder_weighing')
    assert get_operator_tasks()=={'powder_weighing':'粉末称量'}


def test_invalid_registry_never_falls_back_to_moving_another_task():
    task_registry_path().parent.mkdir(parents=True)
    task_registry_path().write_text('{"version":1,"tasks":{"../../unsafe":"任务"}}')
    with pytest.raises(ValueError):get_operator_tasks()


def test_permanent_delete_removes_only_selected_home_and_trajectory(tmp_path):
    import yaml
    from operator_tasks import task_storage_paths,deleted_operator_tasks
    selected=add_operator_task('彻底删除')
    poses,trajectories=task_storage_paths()
    document=dict(version=1,tasks={selected:dict(label='彻底删除',home=dict(left=[1]*7,right=[2]*7)),
                                  'assembly':dict(label='装配',home=dict(left=[3]*7,right=[4]*7))},
                  ready=dict(left=[5]*7,right=[6]*7),extra='preserve')
    poses.write_text(yaml.safe_dump(document,allow_unicode=True))
    trajectories.mkdir(parents=True)
    (trajectories/(selected+'.yaml')).write_text('selected trajectory')
    keep=trajectories/'assembly.yaml';keep.write_text('other trajectory')
    episode=tmp_path/'A/episode1/bag.mcap';episode.parent.mkdir(parents=True);episode.write_bytes(b'bag')
    delete_operator_task(selected)
    actual=yaml.safe_load(poses.read_text())
    assert selected not in get_operator_tasks() and selected in deleted_operator_tasks()
    assert selected not in actual['tasks']
    assert actual['tasks']['assembly']==document['tasks']['assembly']
    assert actual['ready']==document['ready'] and actual['extra']=='preserve'
    assert not (trajectories/(selected+'.yaml')).exists()
    assert keep.read_text()=='other trajectory' and episode.read_bytes()==b'bag'
    # Cleanup retries are idempotent and additions preserve deletion records.
    delete_operator_task(selected)
    add_operator_task('后来添加')
    assert selected in deleted_operator_tasks()


def test_partial_delete_revokes_home_and_retry_completes_cleanup(tmp_path,monkeypatch):
    from operator_tasks import task_storage_paths,deleted_operator_tasks
    import operator_tasks
    selected=add_operator_task('重试清理')
    _,trajectories=task_storage_paths()
    trajectories.mkdir(parents=True)
    target=trajectories/(selected+'.yaml');target.write_text('recording')
    real=operator_tasks._purge_task_data
    monkeypatch.setattr(operator_tasks,'_purge_task_data',lambda *args: (_ for _ in ()).throw(OSError('disk busy')))
    with pytest.raises(OSError,match='disk busy'):delete_operator_task(selected)
    assert selected not in get_operator_tasks() and selected in deleted_operator_tasks()
    assert target.exists()
    monkeypatch.setattr(operator_tasks,'_purge_task_data',real)
    delete_operator_task(selected)
    assert not target.exists()


def test_last_task_rejection_does_not_touch_pose_or_trajectory(tmp_path):
    from operator_tasks import task_storage_paths
    delete_operator_task('assembly');delete_operator_task('bean_picking')
    poses,trajectories=task_storage_paths()
    poses.write_text('{version: 1, tasks: {powder_weighing: {home: keep}}}')
    trajectories.mkdir(parents=True)
    target=trajectories/'powder_weighing.yaml';target.write_text('keep')
    before=poses.read_bytes()
    with pytest.raises(ValueError,match='至少'):delete_operator_task('powder_weighing')
    assert poses.read_bytes()==before and target.read_text()=='keep'


def test_symlink_cleanup_refused_without_following_external_file(tmp_path):
    from operator_tasks import task_storage_paths
    selected=add_operator_task('链接保护')
    _,trajectories=task_storage_paths();trajectories.mkdir(parents=True)
    outside=tmp_path/'outside.yaml';outside.write_text('untouched')
    (trajectories/(selected+'.yaml')).symlink_to(outside)
    with pytest.raises(ValueError,match='符号链接'):delete_operator_task(selected)
    assert outside.read_text()=='untouched'


def test_container_owned_registry_uses_existing_metadata_only_tools_helper(tmp_path,monkeypatch):
    import operator_tasks
    from types import SimpleNamespace
    monkeypatch.delenv('OPERATOR_TASKS_FILE')
    monkeypatch.setenv('TELEOP_DATA_ROOT',str(tmp_path/'deployment'))
    monkeypatch.setattr(operator_tasks,'_edit_tasks',lambda change: (_ for _ in ()).throw(PermissionError('root-owned')))
    calls=[]
    def run(command,**kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0,stdout=json.dumps(dict(result='task_'+'a'*32)),stderr='')
    monkeypatch.setattr(operator_tasks.subprocess,'run',run)
    assert add_operator_task('容器权限')=='task_'+'a'*32
    command=calls[0]
    assert command[:7]==['docker','compose','run','--rm','--no-deps','-T','-e']
    assert command[-6:]==['tools','python3','-m','teleop_runtime.task_cleanup','add','容器权限']
    assert not (tmp_path/'deployment').exists()
