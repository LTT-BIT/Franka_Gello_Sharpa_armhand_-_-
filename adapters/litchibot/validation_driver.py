"""Optional safety wrapper around the unchanged original SharpaDriverNode.

Only the production validation launch may create this single hardware process.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import socket
import signal
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from adapters.litchibot.validation_safety import MotionLatch,load_permit,control_paths,TIMEOUT_S


def validate_driver_arguments(arguments,permit, *, software=False):
    """Validate actual ROS parameters BEFORE any SDK connection, including direct use."""
    values={};args=list(arguments)
    if not args or args.pop(0)!='--ros-args':
        raise ValueError('Explicit canonical guarded-driver ROS parameters required')
    allowed={'use_fake_hardware','enable_left','enable_right','left_serial','right_serial',
             'stop_on_command_timeout_s','speed_coefficient','current_coefficient',
             'state_rate_hz','publish_tactile','tactile_poll_rate_hz'}
    while args:
        flag=args.pop(0)
        if not args:
            raise ValueError('Incomplete ROS argument')
        item=args.pop(0)
        if flag=='-r' and item=='__node:=sharpa_driver':
            continue
        if flag!='-p' or ':=' not in item:
            raise ValueError('Unexpected guarded driver argument / remap')
        name,value=item.split(':=',1)
        if name not in allowed or name in values:
            raise ValueError('Unexpected/duplicate guarded parameter')
        values[name]=value
    for name,expected in {'use_fake_hardware':'true' if software else 'false','enable_left':'true','enable_right':'true',
                          'left_serial':permit['serials']['left'],'right_serial':permit['serials']['right']}.items():
        if values.get(name)!=expected:
            raise ValueError('Guarded hardware parameter mismatch: '+name)
    if float(values.get('stop_on_command_timeout_s','0'))!=TIMEOUT_S:
        raise ValueError('Guarded watchdog may not be disabled')
    for name,default,maximum in (('speed_coefficient',0.3,0.3),('current_coefficient',0.6,0.6),
                                 ('state_rate_hz',30,60),('tactile_poll_rate_hz',60,120)):
        value=float(values.get(name,default))
        if not 0<value<=maximum or (name=='state_rate_hz' and value<30):
            raise ValueError('Guarded coefficient/rate outside bounds: '+name)


def ensure_clear_graph(node, *, full=False):
    # Called BEFORE constructing the original driver / connecting SDK devices.
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:
        import rclpy
        rclpy.spin_once(node,timeout_sec=0.05)
    if any(name=='sharpa_driver' for name,ns in node.get_node_names_and_namespaces()):
        raise RuntimeError('Another SharpaDriverNode already exists; validation refused')
    for side in ('left','right'):
        publishers=node.get_publishers_info_by_topic(f'/sharpa/{side}/command')
        if (publishers if full else (len(publishers)>1 or any(e.node_name!='litchibot_command_bridge' for e in publishers))) or (any(e.node_name!='teleop_data_collector' for e in node.get_subscriptions_info_by_topic(f'/sharpa/{side}/command')) if full else node.get_subscriptions_info_by_topic(f'/sharpa/{side}/command')) or node.get_publishers_info_by_topic(f'/sharpa/{side}/joint_states'):
            raise RuntimeError('Existing Sharpa command/state graph; validation refused')

def validate_normal_arguments(arguments,configuration,software):
    """Ownership/mode/watchdog binding only; original driver validates its settings."""
    values={};args=list(arguments)
    if not args or args.pop(0)!='--ros-args':raise ValueError('Explicit driver parameters required')
    while args:
        flag=args.pop(0)
        if not args:raise ValueError('Incomplete driver argument')
        item=args.pop(0)
        if flag=='-r' and item=='__node:=sharpa_driver':continue
        if flag!='-p' or ':=' not in item:raise ValueError('Unexpected driver remap')
        name,value=item.split(':=',1)
        if name in values:raise ValueError('Duplicate driver parameter')
        values[name]=value
    required={'use_fake_hardware':'true' if software else 'false','enable_left':'true','enable_right':'true',
              'left_serial':configuration['serials']['left'],'right_serial':configuration['serials']['right']}
    if any(values.get(k)!=v for k,v in required.items()):raise ValueError('Physical device ownership/mode mismatch')
    if float(values.get('stop_on_command_timeout_s','0'))!=0.0:raise ValueError('Normal session lifecycle requires command timeout 0.0')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--permit',default='')
    parser.add_argument('--full-config',default='')
    args,ros_args=parser.parse_known_args(argv)
    config=None
    if args.full_config:
        from adapters.litchibot.full_control import load_full_config
        config=load_full_config(args.full_config)
    permit=config['policy'] if config else load_permit(args.permit)
    software=config is not None and config['mode']=='fake'
    normal=config is not None and config['source']=='litchibot' and config['mode'] in ('normal','fake')
    manus_normal=config is not None and config['source']=='manus' and config['mode'] in ('normal','fake')
    if normal or manus_normal:validate_normal_arguments(ros_args,permit,software)
    else:validate_driver_arguments(ros_args,permit,software=software)
    sock_path,status_path=control_paths(permit)
    lock_path=sock_path.parent/'hardware.lock'
    lock_fd=os.open(lock_path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    fcntl.flock(lock_fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    control=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    control.bind(str(sock_path));os.chmod(sock_path,0o600);control.setblocking(False)
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    from sharpa_driver.node import SharpaDriverNode
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    stop_requested=False
    shutdown=None
    def request_shutdown(*unused):
        nonlocal stop_requested
        stop_requested=True
        if shutdown is not None:shutdown.request()
    # Direct terminal hangup must reach the same original SDK cleanup as Ctrl+C.
    old_handlers={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    for s in old_handlers:signal.signal(s,request_shutdown)
    probe=Node('litchibot_validation_preflight',use_global_arguments=False)
    try:
        if config:ensure_clear_graph(probe,full=True)
        else:ensure_clear_graph(probe)
        if stop_requested:
            raise RuntimeError('Validation cancelled before SDK connection')
    except Exception:
        control.close();sock_path.unlink(missing_ok=True);rclpy.shutdown();os.close(lock_fd)
        for s,handler in old_handlers.items():signal.signal(s,handler)
        raise
    finally:
        probe.destroy_node()

    class GuardedSharpaDriverNode(SharpaDriverNode):
        def __init__(self):
            latch_type=MotionLatch
            if config and config['source']=='manus' and config['mode'] in ('normal','fake'):
                from adapters.manus.normal_hand_runtime import ManusForwardingAuthority
                latch_type=ManusForwardingAuthority
            if config and config['source']=='litchibot' and config['mode'] in ('normal','fake'):
                from adapters.litchibot.normal_control import NormalHandAuthority
                latch_type=NormalHandAuthority
            self.latch=latch_type(permit,time.monotonic(),gui_controlled=config is not None,
                source=config['source'] if config else 'litchibot',
                validation_only=config is None or config['mode']=='supervised_hardware_validation')
            self.full=None;self.needs_enable=set()
            self.validation_send_count=0
            self.stop_applied=False
            self.stopped_sides=set()
            try:
                super().__init__()
            except BaseException:
                self._disconnect_all()
                raise
            try:
                if self.get_parameter('use_fake_hardware').value is not software:
                    raise ValueError('Validation hardware wrapper requires explicit real hardware')
                for side,hand in self._hands.items():
                    info=hand.info
                    if info.serial!=permit['serials'][side.value] or info.metadata['side']!=side.value:
                        raise ValueError('Actual device serial/side mismatch')
                    if not software:
                        devices=list(hand._manager.get_all_devices())
                        device=next((d for d in devices if str(d.sn)==info.serial),None)
                        if device is None or getattr(getattr(device,'hand_side',None),'name','').lower()!=side.value:
                            raise ValueError('SDK must report the physical device hand side explicitly')
                if len(self._hands)!=2:
                    raise ValueError('Both explicitly bound hands required')
                expected_timeout=0.0 if normal or manus_normal else TIMEOUT_S
                if float(self.get_parameter('stop_on_command_timeout_s').value)!=expected_timeout:
                    raise ValueError(f'Configured command timeout must be {expected_timeout:.2f} seconds')
                self.get_logger().info(f'Sharpa stop_on_command_timeout_s = {expected_timeout:.2f}; normal_session={normal or manus_normal}')
                self.declare_parameter('supervised_validation_guard',True)
                self.declare_parameter('validation_session',permit['session_token'][:16])
                if config:
                    for subscription in self._command_subscriptions:self.destroy_subscription(subscription)
                    self._command_subscriptions=[]
                    if manus_normal:
                        from adapters.manus.normal_hand_runtime import ManusHandRuntime
                        self.full=ManusHandRuntime(self,config)
                        for side,hand in self._hands.items():
                            hand.stop();self.needs_enable.add(side.value)
                    elif normal:
                        from adapters.litchibot.normal_runtime import NormalHandRuntime
                        self.full=NormalHandRuntime(self,config)
                    else:
                        from adapters.litchibot.full_runtime import FullHandRuntime
                        self.full=FullHandRuntime(self,config)
                        for side,hand in self._hands.items():
                            hand.stop();self.needs_enable.add(side.value)
                self.guard_timer=self.create_timer(0.01,self.service_guard)
                if config and config['source']=='manus':
                    from adapters.litchibot.full_tactile import FullTactilePublisher
                    self.full_tactile=FullTactilePublisher(self)
            except Exception:
                self.destroy_node();raise

        def _publish_tactile(self):
            worker=getattr(self,'full_tactile',None)
            if worker is None:return super()._publish_tactile()
            worker.poll()

        def stop_validation(self,reason):
            self.latch.trip(reason)
            if not self.stop_applied:
                for side,hand in self._hands.items():
                    if side in self.stopped_sides:
                        continue
                    try:
                        hand.stop()
                        self.stopped_sides.add(side)
                    except Exception as error:
                        self.get_logger().fatal(f'{side.value} STOP FAILED: {error}; use physical E-stop per SOP')
                self.stop_applied=len(self.stopped_sides)==len(self._hands)
                if self.full and self.full.severity_policy:
                    self.full.emit_event('HARD_FAULT','both',reason,recovery_state='HARD_FAULT')
                else:self.get_logger().error('VALIDATION LATCHED STOP: '+self.latch.reason)

        def _disconnect_all(self):
            if normal and shutdown is not None:
                try:shutdown.finish()
                finally:self._hands.clear()
                return
            # Attempt BOTH hands even if one stop/disconnect call raises.
            for side,hand in getattr(self,'_hands',{}).items():
                try:
                    hand.stop()
                except Exception as error:
                    self.get_logger().fatal(f'{side.value} shutdown stop failed: {error}; physical E-stop per SOP')
                try:
                    hand.disconnect()
                except Exception as error:
                    self.get_logger().error(f'{side.value} disconnect failed: {error}')
            if hasattr(self,'_hands'):
                self._hands.clear()

        def service_guard(self):
            if normal:return self.service_normal()
            if manus_normal:return self.service_manus()
            if stop_requested:
                self.stop_validation('Operator/process shutdown')
                # Unwind the executor before shutting its context down. Calling
                # rclpy.shutdown() inside an active callback can wait on itself.
                raise ExternalShutdownException()
            try:
                for _ in range(100):
                    try:
                        data=control.recv(4097)
                    except BlockingIOError:
                        break
                    if len(data)>4096:
                        raise ValueError('Oversized supervisor packet')
                    self.latch.heartbeat(json.loads(data),time.monotonic())
                # Full tick drains fresh GUI traffic before checking the same lease.
                # Checking first could expire an already-queued timely heartbeat.
                if self.full:self.full.tick()
                else:self.latch.watch(time.monotonic(),time.time())
                if any(self._timeout_stopped.values()):
                    raise ValueError('Original driver command timeout stopped a hand')
                for side in ('left','right'):
                    pubs=self.get_publishers_info_by_topic(f'/sharpa/{side}/command')
                    sinks=self.get_subscriptions_info_by_topic(f'/sharpa/{side}/command')
                    driver_sinks=[e for e in sinks if e.node_name=='sharpa_driver']
                    states=self.get_publishers_info_by_topic(f'/sharpa/{side}/joint_states')
                    if self.full:
                        valid=len(pubs)==1 and pubs[0].node_name=='sharpa_driver' and not driver_sinks
                    else:
                        valid=len(driver_sinks)==1 and len(pubs)<=1 and all(e.node_name=='litchibot_command_bridge' for e in pubs)
                    if not valid or len(states)!=1 or states[0].node_name!='sharpa_driver':
                        raise ValueError('Non-unique driver or unexpected command publisher')
                    if self.latch.phase=='ARMED' and len(pubs)!=1:
                        raise ValueError('Source bridge disappeared')
                    if self.full:
                        input_topic=f'/teleop/sharpa/{side}/'+('source' if config['source']=='litchibot' else 'target')
                        expected='litchibot_command_bridge' if config['source']=='litchibot' else 'manus_sharpa_retarget'
                        sources=self.get_publishers_info_by_topic(input_topic)
                        if len(sources)>1 or any(e.node_name!=expected for e in sources):
                            raise ValueError('Non-unique/unexpected hand target publisher')
                        if self.latch.phase in ('READY','ARMED','RECOVERING') and len(sources)!=1:
                            raise ValueError('Hand source publisher disappeared')

                if sum(n=='sharpa_driver' for n,ns in self.get_node_names_and_namespaces())!=1:
                    raise ValueError('Multiple or missing SharpaDriverNode')
            except Exception as error:
                self.latch.trip(error)
            if self.latch.phase=='FAULT':
                self.stop_validation(self.latch.reason)
            payload={'phase':self.latch.phase,'reason':self.latch.reason,
                     'hardware_command_calls':0 if software else self.validation_send_count,'unix_s':time.time(),
                     'session':permit['session_token'][:16]}
            temporary=status_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(payload));os.chmod(temporary,0o600);temporary.replace(status_path)

        def service_manus(self):
            if stop_requested:
                self.stop_validation('Operator/process shutdown')
                raise ExternalShutdownException()
            try:
                # Legacy datagrams are not an extra heartbeat lease in normal
                # MANUS. The authenticated GUI connection owns its permits.
                for _ in range(100):
                    try:control.recv(4097)
                    except BlockingIOError:break
                self.full.tick()
                if sum(n=='sharpa_driver' for n,ns in self.get_node_names_and_namespaces())>1:
                    raise RuntimeError('Duplicate driver ownership')
                for side,hand in self._hands.items():
                    if not hand.is_connected:raise RuntimeError('Native device disconnected: '+side.value)
                    sources=self.get_publishers_info_by_topic(f'/teleop/sharpa/{side.value}/target')
                    if len(sources)>1 or any(p.node_name!='manus_sharpa_retarget' for p in sources):
                        raise RuntimeError('Duplicate/unexpected MANUS target owner')
                    for topic in (f'/sharpa/{side.value}/command',f'/sharpa/{side.value}/joint_states'):
                        pubs=self.get_publishers_info_by_topic(topic)
                        if len(pubs)>1 or any(p.node_name!='sharpa_driver' for p in pubs):
                            raise RuntimeError('Duplicate/unexpected driver ownership')
            except Exception as error:self.stop_validation(error)
            now=time.monotonic()
            if now-getattr(self,'_manus_status_at',0)>1:
                payload={'phase':self.latch.phase,'reason':self.latch.reason,
                         'hardware_command_calls':0 if software else self.validation_send_count,
                         'unix_s':time.time(),'session':permit['session_token'][:16]}
                temporary=status_path.with_suffix('.tmp')
                try:
                    temporary.write_text(json.dumps(payload));os.chmod(temporary,0o600);temporary.replace(status_path)
                except OSError as error:self.full.warn('Hand status storage unavailable: '+str(error))
                self._manus_status_at=now

        def service_normal(self):
            if stop_requested:
                for s in self.latch.requested:self.full.request(s,False)
                raise ExternalShutdownException()
            try:
                for _ in range(100):
                    try:data=control.recv(4097)
                    except BlockingIOError:break
                    self.full.legacy_control(data)
                self.full.tick()
                nodes=sum(n=='sharpa_driver' for n,ns in self.get_node_names_and_namespaces())
                if nodes>1:raise RuntimeError('Duplicate driver ownership')
                for side,hand in self._hands.items():
                    s=side.value
                    if not hand.is_connected:raise RuntimeError('Unrecoverable driver/device disconnect: '+s)
                    pubs=self.get_publishers_info_by_topic(f'/sharpa/{s}/command')
                    states=self.get_publishers_info_by_topic(f'/sharpa/{s}/joint_states')
                    sources=self.get_publishers_info_by_topic(f'/teleop/sharpa/{s}/source')
                    if len(pubs)>1 or any(e.node_name!='sharpa_driver' for e in pubs) or len(states)>1 or any(e.node_name!='sharpa_driver' for e in states):
                        raise RuntimeError('Duplicate/unexpected driver ownership')
                    if len(sources)>1 or any(e.node_name!='litchibot_command_bridge' for e in sources):
                        raise RuntimeError('Duplicate/unexpected source ownership')
                    if nodes==0 or not states or not sources:
                        self.latch.pause_data(s,'Temporary source/driver graph unavailable')
                self.full.reconcile_pauses()
            except Exception as error:self.stop_validation(error)
            payload={'phase':self.latch.phase,'reason':self.latch.reason,'hardware_command_calls':0 if software else self.validation_send_count,
                     'unix_s':time.time(),'session':permit['session_token'][:16]}
            self.full.defer_status_file(status_path,payload)

        def _on_command(self,side,message):
            try:
                self.service_guard()
                if self.latch.phase=='FAULT':
                    return
                measured=self._hands[side].read_joint_state().position
                stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                send=self.latch.command(side.value,message.name,message.position,stamp,
                    self.get_clock().now().nanoseconds,message.header.frame_id,time.monotonic(),measured)
                if send:
                    previous_command_time=self._last_command_at[side]
                    super()._on_command(side,message)
                    # Original callback catches SDK errors; verify it actually advanced.
                    if self._last_command_at[side]<=previous_command_time or self._timeout_stopped[side]:
                        raise ValueError('Original driver did not accept command')
                    self.validation_send_count+=1
            except Exception as error:
                self.stop_validation(error)

        def forward_full(self,side,message):
            if not self.full or self.latch.side_state(side.value,time.monotonic())!='ACTIVE':
                raise ValueError('No authoritative hand permit')
            return self._forward_managed(side,message)

        def forward_open(self,side,message):
            if not self.full:raise ValueError('Managed session required')
            self.full.opening.authorize(side.value,message)
            return self._forward_managed(side,message,authorize=lambda:self.full.opening.authorize(side.value,message))


        def _forward_managed(self,side,message,authorize=None):
            # Normal hands stay enabled for the session; only validation/Manus
            # use the existing re-enable path after their native watchdog stop.
            if not normal and side.value in self.needs_enable:
                if software:self._hands[side]._stopped=False
                else:self._hands[side]._ensure_enabled()
                self.needs_enable.discard(side.value)
            if authorize is not None:authorize()
            previous=self._last_command_at[side]
            super()._on_command(side,message)
            if self._last_command_at[side]<=previous:raise ValueError('Original sender rejected command')
            self.validation_send_count+=1
            return True

    node=None
    try:
        node=GuardedSharpaDriverNode()
        if normal:
            from adapters.litchibot.normal_shutdown import NormalShutdown
            def begin_shutdown():
                nonlocal stop_requested
                stop_requested=True
                for side in node.latch.requested:node.full.request(side,False)
            shutdown=NormalShutdown(node._hands,args.full_config,permit['session_token'],begin_shutdown,
                evidence_path=status_path.with_name(status_path.stem+'.shutdown.json'))
        if stop_requested:
            node.stop_validation('Cancelled during SDK connection')
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        try:
            if node:
                # Stop devices before closing GUI transport; a transport cleanup
                # error must not prevent the original SDK stop/disconnect path.
                try:
                    if normal:
                        node._disconnect_all()  # Original cleanup, not a runtime fault.
                    else:node.stop_validation('Validation process shutdown')
                finally:
                    try:
                        if getattr(node,'full_tactile',None):node.full_tactile.close()
                        if node.full:node.full.close()
                    finally:
                        if normal and shutdown is not None:Node.destroy_node(node)
                        else:node.destroy_node()
        finally:
            try:
                control.close();sock_path.unlink(missing_ok=True)
                if rclpy.ok():rclpy.shutdown()
            finally:
                os.close(lock_fd)
                for s,handler in old_handlers.items():signal.signal(s,handler)


if __name__=='__main__':
    main()
