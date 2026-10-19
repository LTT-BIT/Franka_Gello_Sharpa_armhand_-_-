import importlib.util
import math
from pathlib import Path

import pytest
import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "reset_to_initial_pose.py"
)
SPEC = importlib.util.spec_from_file_location("reset_to_initial_pose", SCRIPT_PATH)
RESET = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RESET)


@pytest.mark.parametrize("distance", [0.001, 0.01, 0.1, 1.0, 2.0, 5.0])
def test_reset_duration_respects_peak_limits(distance: float) -> None:
    duration = RESET.reset_duration(distance)

    peak_speed = distance * RESET.SMOOTHERSTEP_PEAK_SPEED / duration
    peak_acceleration = (
        distance * RESET.SMOOTHERSTEP_PEAK_ACCELERATION / duration**2
    )
    assert duration >= RESET.RESET_MIN_DURATION
    assert peak_speed <= RESET.RESET_MAX_SPEED
    assert peak_acceleration <= RESET.RESET_MAX_ACCELERATION


def test_reset_duration_scales_with_large_move() -> None:
    assert math.isclose(RESET.reset_duration(1.0), 9.375)
    assert math.isclose(RESET.reset_duration(2.0), 18.75)


def test_main_does_not_publish_after_context_shutdown(monkeypatch) -> None:
    calls = []

    class FakeNode:
        def _set_active(self, active):
            calls.append(("active", active))

        def destroy_node(self):
            calls.append(("destroy",))

    class FakeExecutor:
        def __init__(self, num_threads):
            calls.append(("executor", num_threads))

        def add_node(self, node):
            calls.append(("add", node))

        def spin(self):
            calls.append(("spin",))

        def shutdown(self):
            calls.append(("executor_shutdown",))

    node = FakeNode()
    monkeypatch.setattr(RESET, "get_package_share_directory", lambda _package: "/tmp")
    monkeypatch.setattr(RESET, "load_targets", lambda _path: {})
    monkeypatch.setattr(RESET.rclpy, "init", lambda: calls.append(("init",)))
    monkeypatch.setattr(RESET.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        RESET.rclpy,
        "shutdown",
        lambda: calls.append(("rclpy_shutdown",)),
    )
    monkeypatch.setattr(RESET, "InitialPoseReset", lambda _path, _targets: node)
    monkeypatch.setattr(RESET, "MultiThreadedExecutor", FakeExecutor)

    RESET.main()

    # The initial state publication is allowed while the context is valid;
    # cleanup must not try a second publication after SIGINT invalidates it.
    assert calls.count(("active", False)) == 1
    assert ("executor_shutdown",) in calls
    assert ("destroy",) in calls
    assert ("rclpy_shutdown",) not in calls


def test_retime_trajectory_preserves_path_and_only_extends_fast_intervals() -> None:
    positions = lambda value: {"left": [value] * 7, "right": [-value] * 7}
    samples = [
        (0.0, positions(0.0)),
        (0.02, positions(0.01)),
        (0.12, positions(0.02)),
    ]

    retimed = RESET.retime_trajectory(samples, max_speed=0.25)

    assert [sample[1] for sample in retimed] == [sample[1] for sample in samples]
    assert math.isclose(retimed[0][0], 0.0)
    assert math.isclose(retimed[1][0], 0.04)
    assert math.isclose(retimed[2][0], 0.14)
    assert RESET.maximum_trajectory_speed(retimed) <= 0.25 + 1e-9


@pytest.mark.parametrize(
    "arguments",
    [
        (-1.0,),
        (1.0, 0.0),
        (1.0, 0.2, 0.0),
        (math.inf,),
        (math.nan,),
    ],
)
def test_reset_duration_rejects_invalid_limits(arguments: tuple[float, ...]) -> None:
    with pytest.raises(ValueError):
        RESET.reset_duration(*arguments)


def test_write_targets_round_trips_atomically(tmp_path: Path) -> None:
    path = tmp_path / "initial_pose.yaml"
    targets = {
        "left": [float(index) for index in range(7)],
        "right": [float(-index) for index in range(7)],
    }

    RESET.write_targets(path, targets)

    assert RESET.load_targets(path) == targets
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document["captured_utc"]
    assert not tuple(tmp_path.glob(".initial_pose.*.tmp"))


def test_operator_pose_store_seeds_three_homes_and_requires_ready_capture(
    tmp_path: Path,
) -> None:
    path = tmp_path / "operator_gui" / "poses.yaml"
    legacy = {
        "left": [0.1] * 7,
        "right": [-0.1] * 7,
    }

    tasks, ready = RESET.load_operator_poses(path, legacy)

    assert set(tasks) == {"powder_weighing", "assembly", "bean_picking"}
    assert tasks["powder_weighing"] == legacy
    assert tasks["assembly"] == {"left": None, "right": None}
    assert tasks["bean_picking"] == {"left": None, "right": None}
    assert ready is None
    assert path.exists()

    recorded_ready = {"left": [0.2] * 7, "right": [-0.2] * 7}
    RESET.write_operator_poses(path, tasks, recorded_ready)
    loaded_tasks, loaded_ready = RESET.load_operator_poses(path, legacy)
    assert loaded_tasks == tasks
    assert loaded_ready == recorded_ready


def test_load_absolute_trajectory_validates_task_and_dual_arm_shape(
    tmp_path: Path,
) -> None:
    path = tmp_path / "assembly.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "kind": "dual_arm_absolute_joint_trajectory",
                "task": "assembly",
                "joint_names": {
                    side: [f"{side}_fr3_joint{index}" for index in range(1, 8)]
                    for side in ("left", "right")
                },
                "samples": [
                    {
                        "time_sec": timestamp,
                        "positions": {
                            "left": [timestamp] * 7,
                            "right": [-timestamp] * 7,
                        },
                    }
                    for timestamp in (0.0, 0.1, 0.3)
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    samples = RESET.load_absolute_trajectory(path, "assembly")

    assert [sample[0] for sample in samples] == [0.0, 0.1, 0.3]


def test_dynamic_tasks_load_independent_homes_and_permanently_delete(tmp_path,monkeypatch):
    from operator_tasks import add_operator_task,delete_operator_task,get_operator_tasks
    registry=tmp_path/'operator_gui/tasks.json'
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(registry))
    pose_path=registry.with_name('poses.yaml')
    legacy=dict(left=[.1]*7,right=[-.1]*7)
    old,ready=RESET.load_operator_poses(pose_path,legacy)
    original={task:dict(poses) for task,poses in old.items()}
    selected=add_operator_task('自定义 Home')
    monkeypatch.setattr(RESET,'TASKS',get_operator_tasks())
    tasks,ready=RESET.load_operator_poses(pose_path,legacy)
    assert tasks[selected]==dict(left=None,right=None)
    tasks[selected]['left']=[.2]*7
    RESET.write_operator_poses(pose_path,tasks,ready)
    tasks,ready=RESET.load_operator_poses(pose_path,legacy)
    assert tasks[selected]==dict(left=[.2]*7,right=None)
    assert all(tasks[key]==value for key,value in original.items())
    delete_operator_task(selected)
    monkeypatch.setattr(RESET,'TASKS',get_operator_tasks())
    RESET.write_operator_poses(pose_path,tasks,ready)
    loaded,_=RESET.load_operator_poses(pose_path,legacy)
    assert selected not in loaded
    assert selected not in yaml.safe_load(pose_path.read_text())['tasks']
    from types import SimpleNamespace
    node=SimpleNamespace(task_registry_path=registry,task_targets=loaded)
    response=SimpleNamespace(success=True,message='')
    assert not RESET.InitialPoseReset._task_available(node,selected,response)
    assert not response.success and '删除' in response.message


def test_new_task_services_register_and_removed_tasks_are_not_reused(tmp_path,monkeypatch):
    import threading
    from types import SimpleNamespace
    from operator_tasks import add_operator_task,get_operator_tasks
    registry=tmp_path/'tasks.json'
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(registry))
    selected=add_operator_task('服务新增任务')
    calls=[]
    node=SimpleNamespace(lock=threading.Lock(),task_registry_path=registry,task_targets={},
        registered_tasks=set(),task_callbacks=None,
        create_service=lambda service,path,callback,**kw:calls.append(path),
        get_logger=lambda:SimpleNamespace(error=lambda message:pytest.fail(message)))
    node._register_task_services=lambda tasks:RESET.InitialPoseReset._register_task_services(node,tasks)
    monkeypatch.setattr(RESET,'TASKS',dict(RESET.OPERATOR_TASKS))
    RESET.InitialPoseReset._refresh_tasks(node)
    assert node.task_targets[selected]==dict(left=None,right=None)
    assert f'/capture_home/{selected}/left' in calls
    assert f'/reset_to_home/{selected}' in calls
    count=len(calls)
    RESET.InitialPoseReset._refresh_tasks(node)
    assert len(calls)==count


def test_new_unrecorded_home_fails_without_starting_any_motion(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from operator_tasks import add_operator_task
    registry=tmp_path/'tasks.json'
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(registry))
    selected=add_operator_task('尚未记录')
    monkeypatch.setattr(RESET,'TASKS',{**RESET.OPERATOR_TASKS,selected:'尚未记录'})
    node=SimpleNamespace(task_registry_path=registry,task_targets={selected:dict(left=None,right=None)})
    node._task_available=lambda task,response:RESET.InitialPoseReset._task_available(node,task,response)
    node._run_target_reset=lambda *a:pytest.fail('Unrecorded task must not send any motion')
    response=SimpleNamespace(success=True,message='')
    result=RESET.InitialPoseReset._reset_home(node,None,response,selected)
    assert not result.success and 'not been recorded' in result.message


def test_deleted_home_cache_is_pruned_even_when_task_labels_unchanged(tmp_path,monkeypatch):
    import threading
    from types import SimpleNamespace
    from operator_tasks import add_operator_task,delete_operator_task,get_operator_tasks
    registry=tmp_path/'operator_gui/tasks.json'
    monkeypatch.setenv('OPERATOR_TASKS_FILE',str(registry))
    selected=add_operator_task('缓存删除')
    stale={selected:dict(left=[.1]*7,right=[.2]*7)}
    delete_operator_task(selected)
    monkeypatch.setattr(RESET,'TASKS',get_operator_tasks())
    node=SimpleNamespace(lock=threading.Lock(),task_registry_path=registry,task_targets=dict(stale))
    RESET.InitialPoseReset._refresh_tasks(node)
    assert selected not in node.task_targets
    # Even a late Ready/capture save using the old dictionary cannot restore it.
    poses=registry.with_name('poses.yaml')
    RESET.write_operator_poses(poses,stale,None)
    assert selected not in yaml.safe_load(poses.read_text())['tasks']
