"""Normal LitchiBot: vendor target -> per-side permit -> basic check -> original sender.
Shared FullHandRuntime utilities here are transport/telemetry, not validation state.
"""
import json
import time
import numpy as np
from .full_runtime import FullHandRuntime
from .normal_control import BasicNormalTarget, RecoverableCondition, UnrecoverableCommand
from .retarget import JOINT_NAMES,JOINT_LIMITS
from .validation_safety import TIMEOUT_S


class NormalHandRuntime(FullHandRuntime):
    normal_path=True
    @property
    def severity_policy(self):return True
    def make_raw_gate(self,config):return BasicNormalTarget()
    def make_target_gate(self,config):return self.raw

    def __init__(self,node,config):
        self._native_paused=set()
        self.feedback={};self._normal_event_times={}
        super().__init__(node,config)
        from .normal_telemetry import DeferredTelemetry
        self.telemetry=DeferredTelemetry()
        from sensor_msgs.msg import JointState
        from rclpy.qos import qos_profile_sensor_data
        for side in self.latch.requested:
            self.subscriptions.append(node.create_subscription(JointState,f'/sharpa/{side}/joint_states',
                lambda msg,s=side:self.observe_feedback(s,msg),qos_profile_sensor_data))
        self.server.reply_timeout=None  # Queued response delay is not connection loss.
        node.get_logger().info('LitchiBot normal forwarding: independent permits; no validation latch, warmup, jump/slew/acquisition gate')

    def emit_event(self,*args,**kwargs):
        try:return self._emit_event(*args,**kwargs)
        except Exception:self.telemetry_errors=getattr(self,'telemetry_errors',0)+1

    def _emit_event(self,*args,**kwargs):
        # Telemetry is not a command gate. Actual hard-fault callers still stop
        # devices in their finally path even if logging/serialization fails.
        if len(args)>1 and kwargs.get('joint') in JOINT_NAMES and 'measured_value' not in kwargs:
            cached=getattr(self,'feedback',{}).get(args[1])
            kwargs['measured_value']=None if cached is None else float(cached[0][JOINT_NAMES.index(kwargs['joint'])])
        if hasattr(self,'telemetry'):
            from .fault_severity import event
            severity,side,reason=args[:3];reason=str(reason)
            kwargs.setdefault('recovery_state',self.latch.side_state(side,time.monotonic()) if side in self.latch.requested else self.latch.phase)
            key=(severity,side,reason,kwargs['recovery_state']);now=time.monotonic()
            if severity=='HARD_FAULT':
                if getattr(self,'_normal_hard_logged',False):return
                self._normal_hard_logged=True
            elif now-self._normal_event_times.get(key,-float('inf'))<2:return
            self._normal_event_times[key]=now
            # Copy context in owner thread; serialization/log I/O run off-thread.
            data=event(severity,side,reason,**kwargs)
            logger=self.node.get_logger()
            self.telemetry.submit(('event',key),lambda:(logger.error if severity=='HARD_FAULT' else logger.warning)(json.dumps(data,allow_nan=False)))
            return data
        try:return super().emit_event(*args,**kwargs)
        except Exception:
            self.telemetry_errors=getattr(self,'telemetry_errors',0)+1

    def native_side(self,side):return next(s for s in self.node._hands if s.value==side)

    def request(self,side,enabled):
        self.latch.request_hand(side,enabled,time.monotonic())
        # Stop is only permit OFF: no hand.stop/disconnect/process change.

    def pause_native(self,side):
        # Compatibility hook only. Original driver/native watchdog owns stopping.
        pass

    def reconcile_pauses(self):
        pass

    def status(self):
        now=time.monotonic()
        result={'mode':self.config['mode'],'phase':self.latch.phase,'reason':self.latch.reason,
            'hands':{s:{'state':self.latch.side_state(s,now),'permit':self.latch.requested[s],
                       'samples':self.latch.samples[s],'reason':self.latch.reason if self.latch.phase=='FAULT' or not self.latch.control_ready else self.latch.side_reasons[s]} for s in self.latch.requested},
            'hardware_send':0 if self.config['mode']=='fake' else self.node.validation_send_count,
            'mock_command_calls':self.node.validation_send_count if self.config['mode']=='fake' else 0,
            'delta_statistics_file':str(self.delta_path),'normal_forwarding':True,
            'telemetry_errors':getattr(self,'telemetry_errors',0)+getattr(getattr(self,'telemetry',None),'errors',0),
            'telemetry_pending':len(self.telemetry.pending) if hasattr(self,'telemetry') else 0}
        return result

    def dispatch(self,request):
        self.latch.control_connected()  # Authenticated transport, not heartbeat lease.
        command=request.get('command');args=request.get('arguments') or {}
        if command=='heartbeat':
            try:self.latch.gui_packet(request,time.monotonic())
            except RecoverableCondition as e:
                stamp=request.get('monotonic_ns')
                age=time.monotonic()-stamp/1e9 if type(stamp) is int else None
                self.emit_event('WARNING','both',e,dt=age,recovery_state=self.latch.side_state('left',time.monotonic()))
        elif command in ('engage_hand','disengage_hand'):
            self.request(args.get('side'),command=='engage_hand')
        elif command=='disengage_all':
            for s in self.latch.requested:self.request(s,False)
        elif command=='open_hand':raise RecoverableCondition('Open hand has no target policy')
        elif command!='status':raise RecoverableCondition('Unknown hand forwarding command')
        return self.status()

    def disconnected(self):
        self.latch.pause_control('Temporary hand supervisor disconnect')
        self.reconcile_pauses()
        self.emit_event('WARNING','both',self.latch.reason,recovery_state='OFFLINE')

    def legacy_control(self,data):
        try:
            message=json.loads(data)
            if message.get('token')==self.config['token']:
                self.emit_event('WARNING','both','Legacy control status: '+str(message.get('reason','')))
        except (ValueError,TypeError,AttributeError):
            self.emit_event('WARNING','both','Malformed control message rejected')

    def source_fault(self,message):
        try:p=json.loads(message.data)
        except (ValueError,TypeError):p={'severity':'WARNING','reason':message.data}
        if not isinstance(p,dict):p={'severity':'WARNING','reason':'Malformed source status message'}
        sides=[p['side']] if p.get('side') in self.latch.requested else list(self.latch.requested)
        for s in sides:
            if p.get('source_arrived') is True:self.latch.note_source(s,time.monotonic())
        self.emit_event('WARNING',p.get('side','both'),p.get('reason','Source packet/status rejected'))

    def tick(self):
        self.server.drain(self.dispatch,self.disconnected)
        self.latch.watch(time.monotonic())
        if self.latch.phase=='FAULT':self.node.stop_validation(self.latch.reason)
        else:self.reconcile_pauses()
        now=time.monotonic()
        if now-self.last_delta_at>=1:
            try:
                if hasattr(self,'telemetry'):
                    from .normal_continuity import DeltaDistribution
                    snapshot=DeltaDistribution()
                    snapshot.layers={k:[v[0],v[1].copy(),v[2].copy(),v[3].copy()] for k,v in self.raw.distribution.layers.items()}
                    self.telemetry.submit('delta',lambda:self.write_delta_snapshot(snapshot))
                else:self.save_delta_stats()
            except (OSError,ValueError,TypeError):self.emit_event('WARNING','both','Delta statistics storage unavailable')
            self.last_delta_at=now
        if now-self.last_state_at>.05:
            from std_msgs.msg import String
            state=self.status()
            def publish_state():
                msg=String();msg.data=json.dumps(state);self.state_publisher.publish(msg)
            if hasattr(self,'telemetry'):self.telemetry.submit('state',publish_state)
            else:publish_state()
            self.last_state_at=now
        if now-self.last_idle_at>=1/30:
            for side in self.latch.requested:
                if self.latch.side_state(side,now)!='ACTIVE':self.idle_snapshot(side)
            self.last_idle_at=now

    def litchibot(self,side,message):
        if self.latch.phase=='FAULT':return
        self.latch.note_source(side,time.monotonic())
        try:
            try:packet=json.loads(message.data)
            except (ValueError,TypeError) as e:raise RecoverableCondition('Malformed source JSON') from e
            if not isinstance(packet,dict) or packet.get('side')!=side:raise UnrecoverableCommand('Malformed/wrong-side final target')
            try:q=self.raw.accept(packet,time.monotonic_ns())
            except Exception as e:raise RecoverableCondition('Source packet dropped: '+str(e)) from e
            self.latch.valid_data(side,time.monotonic())
            if packet.get('valid_glove_joint_count')!=20 or packet.get('held_joints') or packet.get('initial_fallback_joints'):
                self.emit_event('WARNING',side,'Partial/fallback vendor target accepted unchanged',**self.raw.context[side])
            self.process_normal(side,packet,q)
        except RecoverableCondition as e:
            self.emit_event('WARNING',side,e,**self.raw.context.get(side,{}))
        except Exception as e:
            # Only this final-data/device path escalates. Recoverable packet/control
            # exceptions are handled above and never reach this boundary.
            self.hard_fault(side,e,**self.raw.context.get(side,{}))

    def process_normal(self,side,packet,q):
        from sensor_msgs.msg import JointState
        enum=self.native_side(side)
        self.latch.watch(time.monotonic())
        if self.latch.phase=='FAULT':self.node.stop_validation(self.latch.reason);return
        forward=self.latch.side_state(side,time.monotonic())=='ACTIVE'
        if not forward:
            self.idle_snapshot(side)
            return
        msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.name=list(JOINT_NAMES);msg.position=[float(v) for v in q]
        msg.header.frame_id=f"supervisor_active:litchibot:{side}:{packet['session_id']}:{packet['sequence']}"
        if self.node.forward_full(enum,msg) is False:return
        self._native_paused.discard(side)
        self.publishers[side].publish(msg)

    @property
    def latest_target(self):
        return self.raw.previous  # One accepted adjacent anchor/target per side.

    def idle_snapshot(self,side):
        # Measurements are telemetry, not targets. Do not apply a command-range
        # gate to idle measurements or turn one unavailable snapshot into a latch.
        # Real disconnect/ownership/sender errors remain the host's responsibility.
        from sensor_msgs.msg import JointState
        cached=getattr(self,'feedback',{}).get(side)
        if cached is None or time.monotonic()-cached[1]>TIMEOUT_S:return
        measured=cached[0]
        if measured.shape!=(22,) or not np.isfinite(measured).all():
            self.emit_event('WARNING',side,'Invalid idle feedback snapshot dropped')
            return
        bad=np.flatnonzero((measured<JOINT_LIMITS[:,0]) | (measured>JOINT_LIMITS[:,1]))
        if len(bad):
            j=int(bad[0])
            self.emit_event('WARNING',side,'Measured feedback outside command range (telemetry only)',
                            joint=JOINT_NAMES[j],measured_value=float(measured[j]))
        msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.header.frame_id=f'supervisor_no_send:{side}:{self.latch.side_state(side,time.monotonic())}'
        msg.name=list(JOINT_NAMES);msg.position=[float(v) for v in measured]
        self.publishers[side].publish(msg)

    def observe_feedback(self,side,msg):
        try:
            names=tuple(msg.name)
            if names not in (JOINT_NAMES,tuple(side+'_'+n for n in JOINT_NAMES)):raise ValueError('Unexpected telemetry joint order')
            values=np.asarray(msg.position,dtype=float)
            if values.shape!=(22,) or not np.isfinite(values).all():raise ValueError('Invalid telemetry snapshot')
        except (ValueError,TypeError,AttributeError) as e:
            self.emit_event('WARNING',side,str(e));return
        self.feedback[side]=(values.copy(),time.monotonic())

    def write_delta_snapshot(self,snapshot):
        import os
        with self.delta_path.open('w',encoding='utf-8') as stream:
            os.fchmod(stream.fileno(),0o600)
            json.dump(snapshot.snapshot(),stream,allow_nan=False)

    def defer_status_file(self,path,payload):
        def write():
            import os
            temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(payload))
            os.chmod(temporary,0o600);temporary.replace(path)
        self.telemetry.submit('status-file',write)

    def close(self):
        for s in self.latch.requested:self.latch.requested[s]=False
        self.server.close()
        self.telemetry.close()
