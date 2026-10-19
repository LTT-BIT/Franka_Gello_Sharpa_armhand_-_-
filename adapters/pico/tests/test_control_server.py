import json
import socket
import time

import pytest

from pico_bimanual_franka_teleop.control_server import (
    OperatorConsole,
    OperatorControlServer,
)


def test_console_matches_the_input_source_contract():
    console = OperatorConsole()
    assert console.poll() == {"left": False, "right": False}
    assert console.poll_hands() == {"left": False, "right": False}
    assert console.poll_holds() == {"left": False, "right": False}
    console.active["left"] = True
    console.hand_active["left"] = True
    assert console.poll()["left"] is True
    console.deny("left", "tracker missing")
    assert console.poll()["left"] is False
    assert console.poll_hands()["left"] is True
    console.disable_all("robot state missing")
    console.set_status("STATE | trackers: ok")
    snapshot = console.snapshot()
    assert snapshot["status_line"] == "STATE | trackers: ok"
    assert snapshot["hand_active"] == {"left": False, "right": False}
    assert any("tracker missing" in line for line in snapshot["feedback"])
    assert any("robot state missing" in line for line in snapshot["feedback"])


def test_console_requests_are_one_shot():
    console = OperatorConsole()
    console.requests["open_hands"] = True
    taken = console.take_requests()
    assert taken["open_hands"] is True
    assert console.take_requests()["open_hands"] is False


@pytest.mark.parametrize('succeeded',[True,False])
def test_home_preserves_only_existing_permits_and_locks_pedal_until_success(succeeded):
    console=OperatorConsole()
    console.set_active('left',True,target='arm')
    console.set_hold('left',True)  # Already held is still authorized.
    console.set_active('right',True,target='hand')
    console.begin_arm_home()
    assert console.snapshot()['arm_authorized']=={'left':True,'right':False}
    assert console.poll()=={'left':False,'right':False}
    assert all(console.poll_holds().values())
    assert console.poll_hands()=={'left':False,'right':True}
    with pytest.raises(ValueError,match='locked'):console.set_hold('left',False)
    with pytest.raises(ValueError,match='locked'):console.set_active('right',True,target='arm')
    console.finish_arm_home(succeeded)
    assert all(console.poll_holds().values()) and not any(console.poll().values())
    if succeeded:
        console.set_hold('left',False);console.set_hold('right',False)
        assert console.poll()=={'left':True,'right':False}
    else:
        with pytest.raises(ValueError,match='locked'):console.set_hold('left',False)


def test_disengage_during_home_never_restores_permits_or_hold():
    console=OperatorConsole()
    console.set_active('left',True)
    console.begin_arm_home();console.disable_all('operator STOP')
    console.finish_arm_home(True)
    assert not any(console.snapshot()['arm_authorized'].values())
    assert not any(console.poll_holds().values())
    console.set_hold('left',False)
    assert not any(console.poll().values())


def make_server():
    console = OperatorConsole()
    server = OperatorControlServer(("127.0.0.1", 0), console)
    return server, console


def test_dispatch_maps_commands_onto_the_console():
    server, console = make_server()
    try:
        server.dispatch("engage", {"side": "left"})
        assert console.active["left"] is True
        assert console.hand_active["left"] is True
        server.dispatch("disengage", {"side": "left"})
        assert console.active["left"] is False
        assert console.hand_active["left"] is False
        server.dispatch("engage_arm", {"side": "left"})
        assert console.active["left"] is True
        assert console.hand_active["left"] is False
        server.dispatch("engage_hand", {"side": "left"})
        assert console.active["left"] is True
        assert console.hand_active["left"] is True
        server.dispatch("disengage_arm", {"side": "left"})
        assert console.active["left"] is False
        assert console.hand_active["left"] is True
        server.dispatch("hold_arm", {"side": "left", "enabled": True})
        assert console.active["left"] is False
        assert console.arm_hold["left"] is True
        server.dispatch("engage_arm", {"side": "left"})
        assert console.active["left"] is True
        assert console.arm_hold["left"] is False
        server.dispatch("hold_arm", {"side": "left", "enabled": True})
        server.dispatch("hold_arm", {"side": "left", "enabled": False})
        assert console.arm_hold["left"] is False
        assert console.active["left"] is True
        server.dispatch("disengage_hand", {"side": "left"})
        assert console.hand_active["left"] is False
        server.dispatch("engage", {"side": "right"})
        server.dispatch("disengage_all", {})
        assert console.active == {"left": False, "right": False}
        assert console.hand_active == {"left": False, "right": False}
        server.dispatch("open_hand", {"side": "right"})
        server.dispatch("open_hand", {})
        server.dispatch("home_arm", {"side": "left"})
        server.dispatch("home_arm", {})
        server.dispatch("capture_home", {"side": "right"})
        server.dispatch("capture_ready", {})
        server.dispatch("move_ready", {})
        server.dispatch("ready_to_home", {"task": "assembly"})
        server.dispatch("run_preset", {"key": "q"})
        server.dispatch("stop_action", {})
        requests = console.take_requests()
        assert requests["open_right_hand"] and requests["open_hands"]
        assert requests["reset_left"] and requests["reset"]
        assert requests["capture_home_right"]
        assert requests["capture_ready"] and requests["move_ready"]
        assert requests["ready_to_home"] == "assembly"
        assert requests["preset_q"] and requests["stop_action"]
        assert requests["abort_action"]
        with pytest.raises(ValueError):
            server.dispatch("open_hand", {"side": "middle"})
        with pytest.raises(ValueError):
            server.dispatch("engage", {"side": "middle"})
        with pytest.raises(ValueError):
            server.dispatch("run_preset", {"key": "r"})
        with pytest.raises(ValueError):
            server.dispatch("hold_arm", {"side": "left", "enabled": "yes"})
        with pytest.raises(ValueError):
            server.dispatch("warp", {})
    finally:
        server.server_close()


def test_socket_round_trip_speaks_line_json():
    server, console = make_server()
    server.start()
    try:
        with socket.create_connection(server.server_address, timeout=2.0) as client:
            with client.makefile("rw", encoding="utf-8") as stream:
                for request_id, command, arguments in (
                    (1, "engage", {"side": "right"}),
                    (2, "status", {}),
                    (3, "nonsense", {}),
                ):
                    stream.write(
                        json.dumps(
                            {
                                "id": request_id,
                                "command": command,
                                "arguments": arguments,
                            }
                        )
                        + "\n"
                    )
                    stream.flush()
                replies = [json.loads(stream.readline()) for _ in range(3)]
        for _ in range(100):
            if not console.poll()["right"]:
                break
            time.sleep(0.01)
    finally:
        server.close()
    assert replies[0] == {"id": 1, "ok": True, "result": {}}
    assert replies[1]["ok"] and replies[1]["result"]["active"]["right"] is True
    assert not replies[2]["ok"] and "Unknown command" in replies[2]["error"]
    assert console.active["right"] is False
    assert any(
        "operator frontend disconnected" in line
        for line in console.snapshot()["feedback"]
    )


def test_only_the_last_frontend_disconnect_disengages():
    server, console = make_server()
    server.start()
    first = socket.create_connection(server.server_address, timeout=2.0)
    second = socket.create_connection(server.server_address, timeout=2.0)
    try:
        first_stream = first.makefile("rw", encoding="utf-8")
        second_stream = second.makefile("rw", encoding="utf-8")
        second_stream.write(
            json.dumps({"id": 2, "command": "status", "arguments": {}})
            + "\n"
        )
        second_stream.flush()
        assert json.loads(second_stream.readline())["ok"]
        first_stream.write(
            json.dumps(
                {
                    "id": 1,
                    "command": "engage",
                    "arguments": {"side": "left"},
                }
            )
            + "\n"
        )
        first_stream.flush()
        assert json.loads(first_stream.readline())["ok"]
        first_stream.close()
        first.close()
        assert console.poll()["left"] is True
        second_stream.close()
        second.close()
        for _ in range(100):
            if not console.poll()["left"]:
                break
            time.sleep(0.01)
        assert console.poll()["left"] is False
    finally:
        first.close()
        second.close()
        server.close()


def test_episode_home_deduplicates_and_disconnect_result_cannot_be_overwritten():
    console = OperatorConsole()
    job = dict(id='episode-1:0', task='assembly')
    assert console.queue_episode_home(job)['state'] == 'queued'
    assert console.take_requests()['episode_home']['id'] == job['id']
    console.queue_episode_home(job)
    assert not console.take_requests()['episode_home']
    console.update_episode_home(job['id'], 'running')
    with pytest.raises(ValueError, match='in progress'):
        console.queue_episode_home(dict(id='other', task='assembly'))
    console.disable_all('operator frontend disconnected')
    console.update_episode_home(job['id'], 'done', 'late completion')
    assert console.snapshot()['episode_home']['state'] == 'failed'
    console.queue_episode_home(dict(id='episode-1:1', task='assembly'))
    assert console.take_requests()['episode_home']['id'] == 'episode-1:1'


def test_background_home_connection_cannot_replace_operator_frontend():
    console = OperatorConsole()
    server = OperatorControlServer(('127.0.0.1', 0), console)
    try:
        server.client_connected()  # background coordinator alone
        with pytest.raises(ValueError, match='GUI'):
            server.dispatch('episode_home', dict(id='episode-1:0', task='assembly'))
        assert not console.snapshot()['episode_home']
        server.client_connected()  # persistent operator frontend
        assert server.dispatch('episode_home', dict(id='episode-1:0', task='assembly'))['state'] == 'queued'
    finally:
        server.server_close()


def test_offline_arm_commands_rejected_but_hand_authority_survives():
    console=OperatorConsole()
    server=OperatorControlServer(('127.0.0.1',0),console)
    try:
        console.set_active('left',True,target='arm')
        console.set_active('right',True,target='hand')
        console.request('reset','assembly')
        console.set_arm_available(False,'FCI unavailable')
        assert not any(console.snapshot()['arm_authorized'].values())
        assert console.poll_hands()['right']
        assert not console.take_requests()['reset']
        for command,arguments in [('engage_arm',dict(side='left')),
            ('hold_arm',dict(side='left',enabled=True)),('home_arm',{}),('capture_home',dict(side='left')),
            ('capture_ready',{}),('move_ready',{}),('ready_to_home',{}),('run_preset',dict(key='q'))]:
            with pytest.raises(ValueError,match='FCI unavailable'):
                server.dispatch(command,arguments)
        server.dispatch('engage_hand',dict(side='left'))
        console.set_arm_available(True)
        assert not any(console.poll().values())
        assert not any(console.poll_holds().values())
        assert all(console.poll_hands().values())
        server.dispatch('engage_arm',dict(side='left'))
        assert console.poll()['left']
    finally:server.server_close()
