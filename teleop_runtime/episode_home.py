"""Coordinate completed recorder episodes with the existing arm Home service.

No ROS subscriptions, joint commands, hand commands, or recording ownership.
The recorder owns the start gate; the arm service owns motion and settling.
"""
from __future__ import annotations

import json
import socket
import threading


def rpc(endpoint, command, arguments=None):
    # Short connections must not keep the arm frontend-disconnect guard alive.
    with socket.create_connection(endpoint, timeout=.7) as connection:
        connection.settimeout(.7)
        connection.sendall((json.dumps({"id": 1, "command": command,
                                       "arguments": arguments or {}}) + "\n").encode())
        with connection.makefile("rb") as stream:
            line = stream.readline(1_048_577)
        if not line or len(line) > 1_048_576:
            raise RuntimeError("invalid control response")
        response = json.loads(line)
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "control request failed"))
        return response.get("result") or {}


class EpisodeHomeBridge:
    def __init__(self, collector, arms, *, call=rpc, owner=""):
        self.collector = collector
        self.arms = arms
        self.call = call
        self.owner = owner
        self.stopping = threading.Event()
        self.thread = None
        self.attempted = None

    def start(self):
        self.thread = threading.Thread(target=self.run, name="episode-arm-home", daemon=True)
        self.thread.start()

    def report(self, home, state, message):
        self.call(self.collector, "report_home", {
            "id": home["id"], "attempt": home["attempt"], "state": state, "message": message})

    def tick(self):
        home = self.call(self.collector, "status").get("after_home") or {}
        if home.get("owner", "") != self.owner or home.get("state") not in {"pending", "running"} or self.stopping.is_set():
            return
        job_id = f"{home['id']}:{home['attempt']}"
        try:
            job = self.call(self.arms, "status").get("episode_home") or {}
        except (OSError, ValueError, RuntimeError) as error:
            self.report(home, "failed", f"无法确认机械臂 Home 状态，请按 B 重试：{error}")
            return
        if job.get("id") == job_id:
            state = job.get("state")
            self.report(home, "running" if state == "queued" else state, job.get("message", ""))
        elif home["state"] == "running" or self.attempted == job_id:
            # Uncertain/lost result must never silently submit motion again.
            self.report(home, "failed", "Home 任务状态丢失，请按 B 重试")
        elif not self.stopping.is_set():
            self.attempted = job_id
            # An accepted request whose response times out is reconciled by ID
            # on the next tick, rather than automatically submitting it again.
            self.call(self.arms, "episode_home", {"id": job_id, "task": home["task"]})

    def run(self):
        while not self.stopping.is_set():
            try:
                self.tick()
            except (OSError, ValueError, RuntimeError):
                # Recorder can be absent or restarting independently of teleop.
                pass
            self.stopping.wait(.15)
