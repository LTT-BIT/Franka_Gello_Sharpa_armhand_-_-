"""Production Terminal 2 bridge: fake dry-run or explicitly guarded validation."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from contextlib import nullcontext

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from adapters.litchibot.retarget import JOINT_NAMES
from adapters.litchibot.transport import TargetGate, normalize_feedback
from adapters.litchibot.fault_severity import SoftHold, event, frame_context
from adapters.litchibot.minimum_interference import MinimumRawGate, RejectFrame
from adapters.litchibot.normal_control import RecoverableCondition
from adapters.litchibot.validation_safety import RawContinuityGate, load_permit, control_paths, TIMEOUT_S, SPEED_RAD_S


class WorkerInbox:
    """Bounded latest-per-side inbox; malformed stdout cannot crash ROS."""
    def __init__(self, supervised=False, fifo=False, diagnostic=False,normal_full=False):
        self.lock = threading.Lock()
        self.latest = {}
        self.errors = 0
        if supervised and diagnostic:raise ValueError('Diagnostic and supervised modes are exclusive')
        self.raw_gate = MinimumRawGate(diagnostic=True) if diagnostic else RawContinuityGate() if supervised else None
        self.diagnostic=diagnostic
        self.holds={'left':False,'right':False};self.stable={'left':0,'right':0}
        self.events=deque(maxlen=128)
        self.safety_fault = None
        self.fifo=fifo;self.queue=deque()
        self.normal_full=normal_full
        self.event_times={}

    def record_event(self,severity,side,reason,state,**context):
        key=(severity,side,str(reason),state);now=time.monotonic()
        last,count=self.event_times.get(key,(-float('inf'),0))
        if now-last<2:
            self.event_times[key]=(last,count+1);return
        data=event(severity,side,reason,recovery_state=state,**context)
        data['suppressed_events']=count
        self.event_times[key]=(now,0);self.events.append(data)

    def feed(self, line):
        try:
            row = json.loads(line)
            if not isinstance(row, dict) or row.get('schema') != 'litchibot.sharpa_target.v1':
                raise ValueError('Invalid worker packet schema')
            if row.get('side') not in ('left', 'right') or row.get('dry_run') is not True:
                raise ValueError('Worker output must be side-specific dry-run')
            with self.lock:
                if self.safety_fault and not self.fifo:
                    return
                if self.raw_gate is not None:
                    self.raw_gate.observe(row)
                    if self.diagnostic and row['side'] in self.raw_gate.warnings:
                        reason,context=self.raw_gate.warnings[row['side']]
                        self.record_event('WARNING',row['side'],reason,'SOFT_HOLD' if self.holds[row['side']] else 'READY',**context)
                    if self.diagnostic and self.holds[row['side']]:
                        side=row['side'];self.stable[side]+=1
                        if self.stable[side]<30:return
                        self.holds[side]=False
                        self.record_event('WARNING',side,'Stable diagnostic recovery','READY',**self.raw_gate.context[side])
                if self.fifo:
                    if len(self.queue)>=128:
                        if self.normal_full:
                            self.queue.popleft()
                            self.record_event('WARNING',row['side'],'Source backlog: oldest observation dropped','READY')
                        else:raise ValueError('Full source queue overflow')
                    self.queue.append(row)
                else:self.latest[row['side']] = row
        except RejectFrame as error:
            with self.lock:
                side=row['side'];self.latest.pop(side,None);self.stable[side]=0
                self.record_event('WARNING',side,error,'SOFT_HOLD' if self.holds[side] else 'READY',**error.context)
        except SoftHold as error:
            self.hold_side(row['side'],error)
        except Exception as error:
            with self.lock:
                if self.normal_full:
                    self.errors+=1
                    side=row.get('side') if isinstance(locals().get('row'),dict) else None
                    self.record_event('WARNING',side,'Worker packet dropped: '+str(error),'READY')
                    return
                if self.safety_fault:return
                if self.diagnostic:
                    side=row.get('side') if isinstance(locals().get('row'),dict) else None
                    self.events.append(event('HARD_FAULT',side,error,recovery_state='HARD_FAULT',**self.raw_gate.context.get(side,{})))
                self.errors += 1
                if self.raw_gate is not None or self.fifo:
                    self.safety_fault = str(error)
                    self.latest.clear()

    def hold_side(self,side,error):
        with self.lock:
            self.holds[side]=True;self.stable[side]=0
            self.latest.pop(side,None)
            self.record_event('SOFT_HOLD',side,error,'SOFT_HOLD',**error.context)

    def take_events(self):
        with self.lock:
            events=list(self.events);self.events.clear();return events

    def take(self):
        with self.lock:
            rows = self.latest
            self.latest = {}
            return rows

    def take_all(self):
        with self.lock:
            rows=list(self.queue);self.queue.clear();return rows


def to_joint_state(packet, positions, ros_now_ns, monotonic_now_ns):
    from sensor_msgs.msg import JointState
    age_ns = monotonic_now_ns-int(packet['source_received_monotonic_ns'])
    stamp_ns = ros_now_ns-age_ns
    if age_ns < 0 or stamp_ns < 0:
        raise ValueError('Invalid source timestamp')
    msg = JointState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    msg.header.frame_id = f"litchibot:{packet['side']}:{packet['session_id']}:{packet['sequence']}"
    msg.name = list(JOINT_NAMES)  # Same generic names/order as V4 RetargetNode.
    msg.position = [float(v) for v in positions]
    return msg


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--supervised-hardware-validation', action='store_true')
    parser.add_argument('--validation-permit', default='')
    parser.add_argument('--full-config', default='')
    parser.add_argument('--managed-full', action='store_true')
    parser.add_argument('--worker-python', required=True)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--sdk-root', required=True)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--receiver-id', default='')
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--log-dir', default='')
    args = parser.parse_args(argv)
    full=None
    if args.full_config:
        from adapters.litchibot.full_control import load_full_config
        full=load_full_config(args.full_config)
        if full['source']!='litchibot':parser.error('LitchiBot full source required')
    if args.managed_full and not full:parser.error('Managed source requires full config')
    hardware = full['mode']!='fake' if full else args.supervised_hardware_validation
    normal_full=bool(full and full['source']=='litchibot' and full['mode'] in ('normal','fake'))
    if not full and args.dry_run == hardware:
        parser.error('BLOCKER FOR REAL HARDWARE MOTION: choose dry-run or explicit supervised validation')
    if full and args.dry_run and hardware:parser.error('Full hardware mode conflicts with dry-run')
    permit = full['policy'] if full else (load_permit(args.validation_permit) if hardware else None)
    status_path = control_paths(permit)[1] if hardware or full else None
    hardware_status = {}
    if hardware and args.profile != permit['profile']:
        parser.error('Glove profile differs from supervised permit')
    def request_stop(reason):
        if hardware:
            import socket
            path,_ = control_paths(permit)
            with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as stop:
                stop.sendto(json.dumps({'token':permit['session_token'],'hold':False,
                    'sequence':time.monotonic_ns(),'monotonic_ns':time.monotonic_ns(),
                    'reason':reason}).encode(),str(path))
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and non-negative')
    log_dir = Path(args.log_dir) if args.log_dir else Path(tempfile.mkdtemp(prefix='litchibot-terminal2-'))
    log_dir.mkdir(parents=True, exist_ok=True)
    print('LITCHIBOT_TERMINAL2_LOG_DIR='+str(log_dir), flush=True)
    log = (log_dir/'bridge.jsonl').open('x', encoding='utf-8')
    stderr = (log_dir/'worker_stderr.log').open('x', encoding='utf-8')
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    env.pop('PYTHONHOME', None)
    env['PYTHONNOUSERSITE'] = '1'
    command = [args.worker_python, str(Path(__file__).with_name('terminal2_worker.py')),
        '--profile', args.profile, '--sdk-root', args.sdk_root, '--data-root', args.data_root,
        '--receiver-id', args.receiver_id, '--duration', str(args.duration),
        '--log', str(log_dir/'worker_targets.jsonl')]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=stderr, text=True,
                               env=env, start_new_session=True)
    inbox = WorkerInbox(supervised=hardware and not full,fifo=bool(full),diagnostic=not hardware and not full,normal_full=normal_full)
    if inbox.raw_gate is not None:
        if inbox.raw_gate.threshold is None:
            print('RawContinuityGate: pattern policy; single-frame cutoff disabled; 0.180000 rad historical diagnostic reference only',flush=True)
        else:print(f'RawContinuityGate threshold = {inbox.raw_gate.threshold:.6f} rad',flush=True)
    def read_stdout():
        for line in process.stdout:
            inbox.feed(line)
    thread = threading.Thread(target=read_stdout, daemon=True)
    thread.start()
    worker_restart_at=0;worker_restart_count=0

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = Node('litchibot_command_bridge')
    feedback = {}
    gate = TargetGate(allow_dry_run=True, **({'max_age_s':TIMEOUT_S,'max_speed_rad_s':SPEED_RAD_S} if hardware else {}))
    prefix='/teleop/sharpa' if full else '/sharpa'
    suffix='target' if full else 'command'
    publishers = {s: node.create_publisher(JointState, f'{prefix}/{s}/{suffix}', 10)
                  for s in ('left', 'right')}
    source_publishers={s:node.create_publisher(String,f'/teleop/sharpa/{s}/source',1000) for s in ('left','right')} if full else {}
    fault_publisher=node.create_publisher(String,'/teleop/sharpa/source_fault',10) if full else None
    def state(side, msg):
        try:
            if hardware and list(msg.name)!=[side+'_'+name for name in JOINT_NAMES]:
                raise ValueError('Hardware feedback side/order mismatch')
            values = normalize_feedback(side, msg.name, msg.position)
            stamp_ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            age_ns = node.get_clock().now().nanoseconds-stamp_ns
            if 0 <= age_ns <= 250_000_000:
                feedback[side] = (values, time.monotonic_ns()-age_ns)
        except Exception as error:
            if hardware and not normal_full:
                inbox.safety_fault = 'Hardware feedback rejected: '+str(error)
    subscriptions = [node.create_subscription(JointState, f'/sharpa/{s}/joint_states',
        lambda msg, s=s: state(s, msg), qos_profile_sensor_data) for s in ('left','right')]
    # Require proof of fake mode OR the separately authorized guarded host.
    client = node.create_client(GetParameters, '/sharpa_driver/get_parameters')
    future = None
    next_check = 0
    fake_confirmed = False
    guard_confirmed = False
    proof_at = 0
    counts = {'left':0, 'right':0}
    last_packets = {}
    report_counts = counts.copy()
    report_time = time.monotonic()
    rejected = 0
    warned_at = 0
    rc = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            for safety_event in inbox.take_events():
                side=safety_event['side'];joint=safety_event['joint']
                if side in feedback and joint in JOINT_NAMES:
                    safety_event['measured_value']=feedback[side][0][JOINT_NAMES.index(joint)]
                line=json.dumps(safety_event,allow_nan=False)
                if safety_event['severity']=='HARD_FAULT':node.get_logger().error(line)
                else:node.get_logger().warning(line)
                log.write(line+'\n')
            if not full and inbox.safety_fault:
                label='VALIDATION' if hardware else 'DIAGNOSTIC'
                raise RuntimeError(label+' LATCHED STOP: '+inbox.safety_fault)
            if not normal_full and (hardware or full) and status_path.exists():
                hardware_status = json.loads(status_path.read_text())
                if hardware_status.get('session')!=permit['session_token'][:16] or not 0<=time.time()-hardware_status['unix_s']<=TIMEOUT_S:
                    raise RuntimeError('Guard status/session watchdog expired')
                if hardware_status['phase']=='FAULT' and not full:
                    raise RuntimeError('VALIDATION LATCHED STOP: '+hardware_status['reason'])
            elif not normal_full and hardware and proof_at and time.monotonic()-proof_at>TIMEOUT_S:
                raise RuntimeError('Guard status unavailable')
            if future is not None and future.done():
                try:
                    values = future.result().values
                    fake_confirmed = len(values)==1 and values[0].type==1 and values[0].bool_value
                    if hardware or full:
                        guard_confirmed = (len(values)==6 and values[0].type==1 and values[0].bool_value==(not hardware)
                            and values[1].type==1 and values[1].bool_value
                            and values[2].string_value==permit['session_token'][:16]
                            and values[3].double_value==(0.0 if normal_full else TIMEOUT_S)
                            and values[4].string_value==permit['serials']['left']
                            and values[5].string_value==permit['serials']['right'])
                    proof_at = time.monotonic()
                except Exception:
                    fake_confirmed = False
                    guard_confirmed = False
                future = None
            if time.monotonic() >= next_check and future is None and client.service_is_ready():
                req = GetParameters.Request()
                req.names = ['use_fake_hardware']
                if hardware or full:
                    req.names += ['supervised_validation_guard','validation_session','stop_on_command_timeout_s','left_serial','right_serial']
                future = client.call_async(req)
                next_check = time.monotonic()+(0.05 if hardware or full else 1)
            if full and inbox.safety_fault:
                failure=String();failure.data=json.dumps({'severity':'HARD_FAULT','reason':inbox.safety_fault}) if normal_full else inbox.safety_fault;fault_publisher.publish(failure)
            packets=[(p['side'],p) for p in inbox.take_all()] if full else inbox.take().items()
            for side, packet in packets:
                last_packets[side] = packet
                try:
                    endpoints = node.get_subscriptions_info_by_topic(f'/sharpa/{side}/command')
                    driver_endpoints = [e for e in endpoints if 'sharpa' in e.node_name.lower()]
                    # In normal full mode this bridge publishes observations only;
                    # the owning host, never this service proof, decides SDK forwarding.
                    confirmed = True if normal_full else (guard_confirmed and time.monotonic()-proof_at<=TIMEOUT_S) if hardware or full else fake_confirmed
                    if not confirmed or (not full and (len(driver_endpoints)!=1 or driver_endpoints[0].node_name!='sharpa_driver')):
                        if proof_at==0:
                            continue  # No hardware proof yet: no publication.
                        raise ValueError('Expected exactly one confirmed mode-specific SharpaDriverNode sink')
                    now_ns = time.monotonic_ns()
                    if normal_full and (type(packet.get('source_received_monotonic_ns')) is not int or not 0<=(now_ns-packet['source_received_monotonic_ns'])/1e9<=TIMEOUT_S):
                        raise RecoverableCondition('Stale/future observation dropped before source publication')
                    if full:
                        source=String();source.data=json.dumps(packet,allow_nan=False);source_publishers[side].publish(source)
                        publishers[side].publish(to_joint_state(packet,packet['positions_rad'],node.get_clock().now().nanoseconds,now_ns))
                        counts[side]+=1
                        continue
                    with inbox.lock if inbox.diagnostic else nullcontext():
                        if inbox.diagnostic and inbox.holds[side]:
                            gate.clear(side);continue
                        positions = gate.accept(packet, feedback=feedback.get(side), now_ns=now_ns)
                        msg = to_joint_state(packet, positions, node.get_clock().now().nanoseconds, now_ns)
                        publishers[side].publish(msg)
                        counts[side] += 1
                    log.write(json.dumps({'type':'ros_command', 'side':side,
                        'sequence':packet['sequence'], 'source_received_monotonic_ns':packet['source_received_monotonic_ns'],
                        'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                        'published_monotonic_ns':now_ns, 'joint_names':msg.name,
                        'positions_rad':list(msg.position), 'raw_target_rad':packet['positions_rad'],
                        'valid_target_mask':packet['valid_target_mask'], 'unit':'rad',
                        'fake_driver_confirmed':fake_confirmed, 'guard_confirmed':guard_confirmed,
                        'mode':'supervised_hardware_validation' if hardware else 'dry_run',
                        'hardware_send_count':None if hardware else 0,
                        'validation_state':hardware_status.get('phase')}, allow_nan=False)+'\n')
                except Exception as error:
                    if normal_full:
                        # Report packet arrival independently from its freshness;
                        # otherwise repeated rejected packets masquerade as source loss.
                        failure=String();failure.data=json.dumps({'severity':'WARNING','reason':str(error),
                            'side':side,'source_arrived':True});fault_publisher.publish(failure)
                        rejected+=1;continue
                    if full:
                        if full['mode'] in ('normal','fake') and inbox.safety_fault is None:
                            data=event('HARD_FAULT',side,error,recovery_state='HARD_FAULT',**frame_context(packet))
                            node.get_logger().error(json.dumps(data,allow_nan=False))
                        if inbox.safety_fault is None:inbox.safety_fault='Full source frame rejected: '+str(error)
                        failure=String();failure.data=json.dumps({'severity':'HARD_FAULT','reason':inbox.safety_fault}) if normal_full else inbox.safety_fault;fault_publisher.publish(failure)
                    if hardware and not full:
                        raise RuntimeError('VALIDATION LATCHED STOP: '+str(error)) from error
                    if inbox.diagnostic:
                        context=dict(inbox.raw_gate.context.get(side,{}))
                        if str(error)=='No Sharpa feedback for acquisition':
                            inbox.hold_side(side,SoftHold(error,**context));gate.clear(side)
                        else:
                            data=event('HARD_FAULT',side,error,recovery_state='HARD_FAULT',**context)
                            node.get_logger().error(json.dumps(data,allow_nan=False))
                            raise RuntimeError('HARD_FAULT: '+str(error)) from error
                    rejected += 1
                    if not inbox.diagnostic and not (full and full['mode'] in ('normal','fake')) and time.monotonic()-warned_at > 1:
                        node.get_logger().warning('Target skipped: '+str(error))
                        warned_at = time.monotonic()
            current_time = time.monotonic()
            if current_time-report_time >= 1:
                statuses = []
                for side in ('left','right'):
                    packet = last_packets.get(side)
                    hz = (counts[side]-report_counts[side])/(current_time-report_time)
                    if packet is None:
                        status = 'waiting for glove data'
                    else:
                        source_ns = packet.get('source_received_monotonic_ns')
                        age_ms = ((time.monotonic_ns()-source_ns)/1e6
                                  if isinstance(source_ns,int) else None)
                        age = f'{age_ms:.0f}ms' if age_ms is not None else 'unknown'
                        status = (f"{packet.get('glove_status','unknown')} "
                                  f"valid={packet.get('valid_glove_joint_count','?')}/20 age={age}")
                    statuses.append(f'{side}: {status} | command={counts[side]} {hz:.1f}Hz')
                mode = 'FULL TELEOP SOURCE' if full else ('SUPERVISED VALIDATION' if hardware else 'DRY-RUN')
                sends = 'see guarded driver status' if hardware else '0'
                node.get_logger().info(f'LitchiBot {mode} | '+' | '.join(statuses)+
                    f' | 22 joints rad | fake_confirmed={fake_confirmed} | guard_confirmed={guard_confirmed} | hardware_send={sends} | rejected={rejected}')
                report_counts = counts.copy()
                report_time = current_time
            if process.poll() is not None and not thread.is_alive():
                if normal_full and args.duration==0:
                    # A transient acquisition process failure cannot tear down the
                    # hand/arm launcher. Keep the source publisher and retry capture.
                    if time.monotonic()-worker_restart_at>=.25:
                        worker_restart_at=time.monotonic();worker_restart_count+=1
                        failure=String();failure.data=json.dumps({'severity':'WARNING','reason':'Acquisition worker unavailable; reconnecting'})
                        fault_publisher.publish(failure)
                        command[-1]=str(log_dir/f'worker_targets-restart-{worker_restart_count}.jsonl')
                        process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=stderr,text=True,env=env,start_new_session=True)
                        thread=threading.Thread(target=read_stdout,daemon=True);thread.start()
                    continue
                rc = process.returncode
                break
    except KeyboardInterrupt:
        pass
    finally:
        if hardware:
            try:
                request_stop('Bridge exited / fault / operator interrupt')
            except OSError as error:
                node.get_logger().error('Stop channel unavailable; guarded watchdog remains active: '+str(error))
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        thread.join(timeout=1)
        if process.returncode and process.returncode>0 and inbox.safety_fault is None:
            # A startup failure may produce no stdout packet at all. Surface its
            # actual SDK/USB error instead of only launch's secondary shutdown.
            stderr.flush()
            error_path=log_dir/'worker_stderr.log'
            with error_path.open('rb') as failure_log:
                failure_log.seek(0,2)
                failure_log.seek(max(0,failure_log.tell()-8192))
                detail=failure_log.read().decode('utf-8',errors='replace').strip()
            reason=f'LitchiBot worker exited with code {process.returncode}; stderr: {error_path}'
            if detail:reason+='\n'+detail
            node.get_logger().error(json.dumps(event('WARNING' if normal_full else 'HARD_FAULT','both',reason,recovery_state='OFFLINE' if normal_full else 'HARD_FAULT'),allow_nan=False))
        log.write(json.dumps({'type':'summary', 'published':counts, 'rejected':rejected,
            'malformed_worker_lines':inbox.errors, 'worker_exit_code':process.returncode,
            'hardware_send_count':None if hardware else 0, 'fake_driver_confirmed':fake_confirmed,
            'guard_confirmed':guard_confirmed})+'\n')
        log.close()
        stderr.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
