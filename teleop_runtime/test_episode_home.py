"""Offline orchestration tests. No robot connections."""
import pytest

from teleop_runtime.episode_home import EpisodeHomeBridge


class Services:
    def __init__(self):
        self.home = dict(id='episode-id', attempt=0, task='assembly', state='armed')
        self.job = {}
        self.starts = 0
        self.lost_reply = False

    def call(self, endpoint, command, arguments=None):
        arguments = arguments or {}
        if endpoint == 'recorder':
            if command == 'status':
                return {'after_home': dict(self.home)}
            assert command == 'report_home'
            self.home.update(state=arguments['state'], message=arguments['message'])
        elif command == 'status':
            return {'episode_home': dict(self.job)}
        else:
            assert command == 'episode_home'
            self.starts += 1
            self.job = dict(arguments, state='queued')
            if self.lost_reply:
                raise TimeoutError('accepted but reply lost')
        return {}


def test_writer_must_exit_before_home_and_no_duplicate_motion():
    services = Services()
    bridge = EpisodeHomeBridge('recorder', 'arms', call=services.call)
    bridge.tick()
    assert services.starts == 0
    services.home['state'] = 'pending'
    bridge.tick()
    bridge.tick()
    bridge.tick()
    assert services.starts == 1
    assert services.home['state'] == 'running'
    services.job.update(state='done', message='measured joints settled')
    bridge.tick()
    assert services.home['state'] == 'done'


def test_lost_reply_and_bridge_restart_reconcile_existing_job():
    services = Services()
    services.home['state'] = 'pending'
    services.lost_reply = True
    bridge = EpisodeHomeBridge('recorder', 'arms', call=services.call)
    with pytest.raises(TimeoutError):
        bridge.tick()
    bridge = EpisodeHomeBridge('recorder', 'arms', call=services.call)
    bridge.tick()
    assert services.starts == 1
    assert services.home['state'] == 'running'
    services.job['state'] = 'failed'
    bridge.tick()
    bridge.tick()
    assert services.home['state'] == 'failed'
    assert services.starts == 1
    services.home.update(state='pending', attempt=1)
    services.lost_reply = False
    bridge.tick()
    assert services.starts == 2


def test_lost_arm_job_blocks_instead_of_replaying_motion():
    services = Services()
    services.home['state'] = 'running'
    bridge = EpisodeHomeBridge('recorder', 'arms', call=services.call)
    bridge.tick()
    assert services.home['state'] == 'failed'
    assert services.starts == 0


def test_shutdown_and_other_session_cannot_start_home():
    services = Services()
    services.home.update(state='pending', owner='other')
    bridge = EpisodeHomeBridge('recorder', 'arms', call=services.call, owner='this')
    bridge.tick()
    assert services.starts == 0
    services.home['owner'] = 'this'
    bridge.stopping.set()
    bridge.tick()
    assert services.starts == 0
