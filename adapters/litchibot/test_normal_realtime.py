"""Actual DDS + original mock sender + TCP; never loads the physical SDK."""
import json
import os
import socket
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from adapters.litchibot.test_full_teleop import config_file,isolated_guard_paths
from adapters.litchibot.test_validation_safety import solved_packet


def summary(values):
    a=np.asarray(values,dtype=float)*1000
    return dict(mean_ms=float(a.mean()),p95_ms=float(np.percentile(a,95)),max_ms=float(a.max()),samples=len(a))


def test_continuous_latest_forwarding_with_slow_diagnostics(tmp_path,monkeypatch,isolated_guard_paths):
    rclpy=pytest.importorskip('rclpy')
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from litchi_hardware.hardware.sharpa.sdk_driver import SharpaSdkHand
    from adapters.litchibot import validation_driver
    from adapters.litchibot.normal_runtime import NormalHandRuntime
    p,cfg=config_file(tmp_path)
    forbidden=Mock(side_effect=AssertionError('Physical SDK forbidden'))
    monkeypatch.setattr(SharpaSdkHand,'_load_sdk',forbidden)
    duration=float(os.environ.get('LITCHIBOT_RT_SECONDS','6'))
    def exercise(node):
        source=Node('litchibot_command_bridge');executor=SingleThreadedExecutor()
        executor.add_node(node);executor.add_node(source)
        pubs={s:source.create_publisher(String,f'/teleop/sharpa/{s}/source',1) for s in ('left','right')}
        runtime=node.full;stop=threading.Event();started=threading.Event();errors=[]
        sent={s:[] for s in pubs};produced={s:[] for s in pubs};callback=[];ages=[];rtts=[];timeouts=[]
        extra_reads=Mock(side_effect=AssertionError('Normal adapter must not read hardware feedback'))
        # Cache path may read original ROS state, never this legacy helper.
        monkeypatch.setattr(runtime,'native_side',runtime.native_side)
        original_receive=runtime.litchibot;original_send=node.forward_full
        def receive(side,msg):
            t=time.monotonic();packet=json.loads(msg.data)
            ages.append(t-packet['source_received_monotonic_ns']/1e9)
            original_receive(side,msg);callback.append(time.monotonic()-t)
        runtime.litchibot=receive
        def send(side,msg):
            result=original_send(side,msg)
            if result is not False:sent[side.value].append(time.monotonic())
            return result
        node.forward_full=send
        # Assert no adapter feedback reads even though the original driver still
        # publishes its real mock joint states and retains its native watchdog.
        original_idle=runtime.idle_snapshot
        def idle(side):
            hand=node._hands[runtime.native_side(side)];read=hand.read_joint_state
            hand.read_joint_state=extra_reads
            try:original_idle(side)
            finally:hand.read_joint_state=read
        runtime.idle_snapshot=idle
        original_log=node.get_logger()
        def warning(message):
            if 'stopped after command timeout' in str(message):timeouts.append(str(message))
            time.sleep(.15)
        node.get_logger=lambda:SimpleNamespace(warning=warning,error=original_log.error,info=original_log.info,fatal=original_log.fatal)
        write=runtime.write_delta_snapshot
        def slow_write(snapshot):time.sleep(.30);write(snapshot)
        runtime.write_delta_snapshot=slow_write
        status_write=runtime.defer_status_file
        def slow_status(path,payload):
            status_write(path,payload)
            runtime.telemetry.submit('injected-slow-disk',lambda:time.sleep(.25))
        runtime.defer_status_file=slow_status
        disconnects=[];disconnected=runtime.disconnected
        def lost():disconnects.append(time.monotonic());disconnected()
        runtime.disconnected=lost
        def producer():
            sequence=0;deadline=time.monotonic()
            try:
                while not stop.is_set():
                    sequence+=1
                    for s,pub in pubs.items():
                        packet=solved_packet(s,sequence);packet['positions_rad']=[0.0]*22
                        packet['positions_rad'][18]=.2+.15*np.sin(sequence/12)
                        packet.update(source_received_monotonic_ns=time.monotonic_ns(),valid_glove_joint_count=18,held_joints=['thumb_CMC_FE'])
                        msg=String();msg.data=json.dumps(packet);pub.publish(msg);produced[s].append(time.monotonic())
                    deadline+=1/30;stop.wait(max(0,deadline-time.monotonic()))
            except Exception as e:errors.append(str(e))
        def controller():
            try:
                with socket.create_connection(('127.0.0.1',cfg['control_port']),timeout=2) as sock:
                    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);reader=sock.makefile('rb');seq=0
                    def request(command,args=None):
                        nonlocal seq
                        seq+=1;start=time.monotonic()
                        row=dict(id=seq,command=command,arguments=args or {},token=cfg['token'],sequence=seq,monotonic_ns=time.monotonic_ns())
                        sock.sendall((json.dumps(row)+'\n').encode());reply=json.loads(reader.readline())
                        assert reply['ok'],reply
                        if command=='heartbeat':rtts.append(time.monotonic()-start)
                        return reply['result']
                    for s in pubs:request('engage_hand',{'side':s})
                    started.set()
                    while not stop.is_set():
                        status=request('heartbeat')
                        assert all(status['hands'][s]['state']=='ACTIVE' for s in pubs),status
                        stop.wait(.05)
            except Exception as e:errors.append(str(e));started.set()
        producer_thread=threading.Thread(target=producer,daemon=True);control_thread=threading.Thread(target=controller,daemon=True)
        try:
            end=time.monotonic()+2
            while time.monotonic()<end and not all(pub.get_subscription_count()==1 for pub in pubs.values()):executor.spin_once(timeout_sec=.003)
            producer_thread.start()
            end=time.monotonic()+2
            while time.monotonic()<end and not all(s in runtime.raw.previous for s in pubs):executor.spin_once(timeout_sec=.003)
            control_thread.start()
            end=time.monotonic()+2
            while not started.is_set() and time.monotonic()<end:executor.spin_once(timeout_sec=.003)
            start=time.monotonic();end=start+duration
            while time.monotonic()<end and not errors:
                executor.spin_once(timeout_sec=.003)
            finish=time.monotonic()
            assert not errors,errors
            assert node.latch.phase!='FAULT' and all(node.latch.requested.values())
            assert not timeouts and not disconnects and not extra_reads.called
            assert all(not value for value in node._timeout_stopped.values())
            intervals={s:np.diff([t for t in sent[s] if start<=t<=finish]) for s in pubs}
            source_hz={s:sum(start<=t<=finish for t in produced[s])/(finish-start) for s in pubs}
            forward_hz={s:sum(start<=t<=finish for t in sent[s])/(finish-start) for s in pubs}
            for s in pubs:
                assert forward_hz[s]>28 and max(intervals[s])<.2
                assert abs(source_hz[s]-forward_hz[s])<1
            assert max(ages)<.2 and max(rtts)<.2
            report=dict(duration_s=finish-start,source_hz=source_hz,forwarding_hz=forward_hz,
                        command_interval={s:summary(v) for s,v in intervals.items()},callback_processing=summary(callback),
                        heartbeat_rtt=summary(rtts),target_packet_age=summary(ages),native_command_timeouts=len(timeouts),
                        supervisor_disconnects=len(disconnects),diagnostic_queue_depth=len(runtime.telemetry.pending),
                        sdk_feedback_reads_by_normal_adapter=extra_reads.call_count,physical_sdk_loads=forbidden.call_count,
                        slow_diagnostics=dict(logger_ms=150,statistics_write_ms=300,status_disk_ms=250),
                        sdk='original mock sender; physical SDK prohibited',ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID'))
            assert report['diagnostic_queue_depth']<=32
            dest=os.environ.get('LITCHIBOT_RT_REPORT')
            if dest:Path(dest).write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2))
        finally:
            stop.set();producer_thread.join(2)
            if control_thread.ident is not None:control_thread.join(2)
            executor.remove_node(source);executor.remove_node(node);source.destroy_node();executor.shutdown()
    monkeypatch.setattr(rclpy,'spin',exercise)
    validation_driver.main(['--full-config',str(p),'--ros-args','-p','use_fake_hardware:=true','-p','enable_left:=true','-p','enable_right:=true',
                            '-p','left_serial:=fake-left','-p','right_serial:=fake-right','-p','stop_on_command_timeout_s:=0.0'])
    forbidden.assert_not_called()
