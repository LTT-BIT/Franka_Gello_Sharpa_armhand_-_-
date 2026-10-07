"""Source routing and GUI transport for the existing guarded hardware host."""
import json
import time
from pathlib import Path

import numpy as np

from .full_control import HandControlServer
from .retarget import JOINT_NAMES
from .transport import TargetGate
from .validation_safety import RawContinuityGate,TIMEOUT_S,SPEED_RAD_S
from .fault_severity import PerSideMotionLatch, SoftHold, event, frame_context
from .minimum_interference import MinimumMotionLatch, RejectFrame


class FullHandRuntime:
    def __init__(self,node,config):
        from std_msgs.msg import String
        from std_srvs.srv import SetBool,Trigger
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import PoseArray
        from rclpy.qos import qos_profile_sensor_data
        self.node=node;self.config=config;self.latch=node.latch
        from .normal_continuity import full_continuity_gate
        self.raw=self.make_raw_gate(config)
        if getattr(self,'normal_path',False):
            node.get_logger().info('Normal LitchiBot: final-command integrity/freshness only; native limits unchanged')
        elif self.raw.threshold is None:
            node.get_logger().info('RawContinuityGate: minimum-interference pattern policy; single-frame cutoff disabled; 0.180000 rad is historical diagnostic reference only')
        else:node.get_logger().info(f'RawContinuityGate threshold = {self.raw.threshold:.6f} rad')
        self.delta_path=Path('/tmp')/('hand-deltas-'+config['token'][:16]+'.json')
        self.last_delta_at=0
        self.gate=self.make_target_gate(config)
        if isinstance(self.latch,MinimumMotionLatch):
            node.get_logger().info('Normal command path: added shared slew/acquisition clamp disabled; native driver/SDK limits unchanged')
        self.keypoints={s:{} for s in ('left','right')};self.pending={s:[] for s in ('left','right')}
        self.sequence={'left':0,'right':0};self.control_sequence=0;self.last_gui_sequence=-1
        self.server=HandControlServer(config['control_port'],config['token'])
        self.publishers={s:node.create_publisher(JointState,f'/sharpa/{s}/command',10) for s in ('left','right')}
        self.state_publisher=node.create_publisher(String,'/teleop/sharpa/supervisor/state',10)
        self.subscriptions=[];self.services=[]
        for side in ('left','right'):
            self.services.append(node.create_service(SetBool,f'/teleop/sharpa/{side}/enable',
                lambda req,resp,s=side:self.enable_service(s,req,resp)))
            if config['source']=='litchibot':
                self.subscriptions.append(node.create_subscription(String,f'/teleop/sharpa/{side}/source',
                    # Normal teleop executes the latest target, not a playback FIFO.
                    # Retain strict/history paths unchanged for other modes.
                    lambda msg,s=side:self.litchibot(s,msg),1 if getattr(self,'normal_path',False) else 1000))
            else:
                self.subscriptions.append(node.create_subscription(PoseArray,f'/manus/{side}/keypoints',
                    lambda msg,s=side:self.keypoint(s,msg),qos_profile_sensor_data))
                self.subscriptions.append(node.create_subscription(JointState,f'/teleop/sharpa/{side}/target',
                    lambda msg,s=side:self.manus(s,msg),1000))
        self.subscriptions.append(node.create_subscription(String,'/teleop/sharpa/source_fault',
            self.source_fault,10))
        self.services.append(node.create_service(Trigger,'/teleop/sharpa/disengage_all',self.all_service))
        self.last_state_at=0
        self.last_idle_at=0

    def make_raw_gate(self,config):
        from .normal_continuity import full_continuity_gate
        return full_continuity_gate(config['source'],config['mode'])

    def make_target_gate(self,config):
        return TargetGate(allow_dry_run=True,max_age_s=TIMEOUT_S,
            max_speed_rad_s=None if isinstance(self.latch,MinimumMotionLatch) else SPEED_RAD_S)

    def source_fault(self,msg):
        self.node.stop_validation('Source fault: '+msg.data)

    def enable_service(self,side,req,resp):
        try:
            if req.data and not self.latch.requested[side]:
                raise ValueError('GUI Start hand must grant the operator permit; ROS cannot create it')
            if not req.data:self.request(side,False)
            resp.success=True;resp.message='Supervisor request accepted'
        except Exception as error:resp.success=False;resp.message=str(error)
        return resp

    def all_service(self,req,resp):
        for side in ('left','right'):self.request(side,False)
        resp.success=True;resp.message='Both hand permits revoked';return resp

    def request(self,side,enabled):
        if enabled and self.latch.phase!='FAULT' and self.latch.requested.get(side):return
        self.latch.request_hand(side,enabled,time.monotonic());self.gate.clear(side)
        if not enabled:self.stop_hardware_side(side)

    def stop_hardware_side(self,side):
        enum=next(s for s in self.node._hands if s.value==side)
        try:self.node._hands[enum].stop()
        except Exception as error:
            self.node.stop_validation('Hand stop failed: '+str(error));raise
        self.node._has_received_command[enum]=False;self.node._timeout_stopped[enum]=False
        self.node.needs_enable.add(side)

    def status(self):
        now=time.monotonic()
        return {'mode':self.config['mode'],'phase':self.latch.phase,'reason':self.latch.reason,
            'hands':{s:{'state':self.latch.side_state(s,now),'permit':self.latch.requested[s],
                        'samples':self.latch.samples[s],
                        'reason':self.latch.side_reasons[s] if self.severity_policy else self.latch.reason} for s in ('left','right')},
            'hardware_send':0 if self.config['mode']=='fake' else self.node.validation_send_count,
            'severity':('HARD_FAULT' if self.latch.phase=='FAULT' else 'SOFT_HOLD' if self.severity_policy and any(self.latch.holds.values()) else 'WARNING'),
            'delta_statistics_file':str(self.delta_path),
            'mock_command_calls':self.node.validation_send_count if self.config['mode']=='fake' else 0}

    def dispatch(self,request):
        command=request.get('command');args=request.get('arguments') or {}
        if command=='heartbeat':
            now=time.monotonic();stamp=request.get('monotonic_ns');sequence=request.get('sequence')
            generation=request.get('_connection_generation')
            normal=self.config['mode'] in ('normal','fake')
            permitted=any(self.latch.requested.values())
            if generation is not None and generation!=getattr(self,'_gui_connection_generation',None):
                # Reconnect cannot authorize motion or clear a hard latch. Sequence
                # baselines are per authenticated TCP connection while permits are OFF.
                if normal and not permitted:self.last_gui_sequence=-1
                self._gui_connection_generation=generation
            if not isinstance(stamp,int) or not 0<=now-stamp/1e9<=TIMEOUT_S or not isinstance(sequence,int) or sequence<=self.last_gui_sequence:
                age=now-stamp/1e9 if isinstance(stamp,int) else None
                detail=f'GUI stale/replayed heartbeat: age={age}, sequence={sequence}, previous_sequence={self.last_gui_sequence}'
                if normal and not permitted and self.latch.phase!='FAULT':
                    self.emit_event('WARNING','both','GUI stale/replayed heartbeat rejected while hand permits OFF',dt=age,recovery_state=self.latch.phase)
                    raise ValueError(detail)
                self.node.stop_validation(detail);raise ValueError(self.latch.reason)
            self.last_gui_sequence=sequence;self.latch.gui_heartbeat(now)
            hold=args.get('hold') is True
            if self.config['mode'] in ('fake','normal'):hold=any(self.latch.requested.values())
            if hold and self.latch.phase in ('READY','ARMED'):
                self.control_sequence+=1
                self.latch.heartbeat({'token':self.config['token'],'hold':True,'sequence':self.control_sequence,
                    'monotonic_ns':stamp},now)
            elif self.config['mode']=='supervised_hardware_validation' and self.latch.phase=='ARMED':
                self.node.stop_validation('Operator released hold-to-run')
        elif command in ('engage_hand','disengage_hand'):
            self.request(args.get('side'),command=='engage_hand')
        elif command=='disengage_all':
            for side in ('left','right'):self.request(side,False)
        elif command=='open_hand':
            raise ValueError('Open hand has no validated Sharpa policy; no command sent')
        elif command!='status':raise ValueError('Unknown hand command')
        return self.status()

    def disconnected(self):
        if any(self.latch.requested.values()):self.node.stop_validation('GUI control connection lost')

    def tick(self):
        self.server.drain(self.dispatch,self.disconnected)
        self.latch.watch(time.monotonic(),time.time())
        self.apply_source_holds()
        for side in ('left','right'):
            if self.pending[side] and time.monotonic()-self.pending[side][0][0]>TIMEOUT_S:
                self.node.stop_validation('Matching Manus validity input unavailable')
        if hasattr(self.raw,'distribution') and time.monotonic()-self.last_delta_at>=1:
            self.save_delta_stats();self.last_delta_at=time.monotonic()
        if time.monotonic()-self.last_state_at>.05:
            from std_msgs.msg import String
            msg=String();msg.data=json.dumps(self.status());self.state_publisher.publish(msg)
            self.last_state_at=time.monotonic()
        if time.monotonic()-self.last_idle_at>=1/30:
            for side in ('left','right'):
                if self.latch.side_state(side,time.monotonic())!='ACTIVE':self.idle_snapshot(side)
            self.last_idle_at=time.monotonic()

    def apply_source_holds(self):
        if self.severity_policy:
            for side,age in list(self.latch.pending_source_holds.items()):
                self.hold_side(side,SoftHold('Source arrival timeout; waiting for fresh baseline',joint=None,source_joint=None,source_value=None,target_value=None,delta=None,dt=age))

    def idle_snapshot(self,side):
        from sensor_msgs.msg import JointState
        from .retarget import JOINT_LIMITS
        hand=self.node._hands[next(s for s in self.node._hands if s.value==side)]
        measured=np.asarray(hand.read_joint_state().position)
        if measured.shape!=(22,) or not np.isfinite(measured).all() or np.any(measured<JOINT_LIMITS[:,0]) or np.any(measured>JOINT_LIMITS[:,1]):
            self.node.stop_validation('Invalid idle hand feedback');return
        msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.header.frame_id=f'supervisor_no_send:{side}:{self.latch.side_state(side,time.monotonic())}'
        msg.name=list(JOINT_NAMES);msg.position=[float(q) for q in measured]
        self.publishers[side].publish(msg)

    def keypoint(self,side,message):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        positions=np.array([[p.position.x,p.position.y,p.position.z] for p in message.poses])
        valid=positions.shape==(25,3) and np.isfinite(positions).all()
        self.keypoints[side][stamp]=valid
        if len(self.keypoints[side])>100:self.keypoints[side].pop(next(iter(self.keypoints[side])))
        pending=self.pending[side];self.pending[side]=[]
        for started,msg in pending:
            if self.stamp(msg)==stamp:self.manus(side,msg)
            else:self.pending[side].append((started,msg))

    @staticmethod
    def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def manus(self,side,message):
        try:
            stamp=self.stamp(message)
            if stamp not in self.keypoints[side]:
                if len(self.pending[side])>=100:raise ValueError('Manus validity queue overflow')
                self.pending[side].append((time.monotonic(),message));return
            if not self.keypoints[side][stamp]:raise ValueError('Invalid Manus keypoint input')
            self.sequence[side]+=1
            now_ns=time.monotonic_ns();ros_now=self.node.get_clock().now().nanoseconds
            accepted=self.raw.observe_ros_target(side,list(message.name),list(message.position),stamp,ros_now,
                self.sequence[side],self.config['token'][:16],now_ns)
            if not accepted:self.soft_anomaly();return
            packet={'schema':'litchibot.sharpa_target.v1','dry_run':True,'engaged':True,'side':side,
                'joint_names':list(message.name),'positions_rad':list(message.position),'valid_glove_joint_count':25,
                'valid_target_mask':[True]*22,'sequence':self.sequence[side],'session_id':self.config['token'][:16],
                'source_received_monotonic_ns':now_ns-(ros_now-stamp)}
            self.process(side,packet,stamp)
        except Exception as error:self.node.stop_validation(error)

    @property
    def severity_policy(self):
        return isinstance(self.latch,PerSideMotionLatch)

    def emit_event(self,severity,side,reason,**context):
        # One event per global hard latch, including later driver-stop retries.
        if severity=='HARD_FAULT' and getattr(self,'_hard_fault_logged',False):
            return self._latched_hard_event
        if severity=='HARD_FAULT' and self.latch.phase=='FAULT':reason=self.latch.reason
        if severity=='HARD_FAULT':context['recovery_state']='HARD_FAULT'
        # None explicitly means unavailable, never an invented sensor reading.
        if 'recovery_state' not in context:
            context['recovery_state']=self.latch.side_state(side,time.monotonic()) if side in ('left','right') else self.latch.phase
        if side in ('left','right') and context.get('joint') in JOINT_NAMES and 'measured_value' not in context:
            try:
                hand=self.node._hands[next(s for s in self.node._hands if s.value==side)]
                values=hand.read_joint_state().position
                context['measured_value']=values[JOINT_NAMES.index(context['joint'])]
            except Exception:context['measured_value']=None
        data=event(severity,side,reason,**context)
        if severity!='HARD_FAULT':
            now=time.monotonic()
            if not hasattr(self,'_event_throttle'):
                self._event_throttle={};self._event_states={}
            key=(severity,side,str(reason),data['recovery_state'])
            last,count=self._event_throttle.get(key,(-float('inf'),0))
            transition=self._event_states.get(side)!=data['recovery_state']
            if not transition and now-last<2.0:
                self._event_throttle[key]=(last,count+1)
                return data
            data['suppressed_events']=count
            self._event_throttle[key]=(now,0);self._event_states[side]=data['recovery_state']
        self.last_safety_event=data
        if severity=='HARD_FAULT':
            self._hard_fault_logged=True
            self._latched_hard_event=data
        logger=self.node.get_logger()
        line=json.dumps(data,allow_nan=False)
        if severity=='HARD_FAULT':logger.error(line)
        else:logger.warning(line)
        return data

    def litchibot(self,side,message):
        try:
            packet=json.loads(message.data)
            if self.severity_policy and isinstance(packet,dict):
                self.raw.context[side]=frame_context(packet,self.raw.previous.get(side))
            if packet['side']!=side:raise ValueError('LitchiBot cross-side packet')
            if not self.raw.observe(packet):self.soft_anomaly();return
            now_ns=time.monotonic_ns()
            stamp=self.node.get_clock().now().nanoseconds-(now_ns-packet['source_received_monotonic_ns'])
            if self.severity_policy:self.latch.note_source(side,time.monotonic())
            if self.latch.phase!='FAULT' and side in getattr(self.raw,'warnings',{}):
                reason,context=self.raw.warnings[side]
                self.emit_event('WARNING',side,reason,**context)
            self.process(side,packet,stamp)
        except RejectFrame as error:
            self.latch.note_source(side,time.monotonic())
            if self.latch.phase!='FAULT':self.emit_event('WARNING',side,error,**error.context)
        except SoftHold as error:
            self.latch.note_source(side,time.monotonic())
            self.hold_side(side,error)
        except Exception as error:
            if self.severity_policy:self.hard_fault(side,error,**self.raw.context.get(side,{}))
            else:self.node.stop_validation(error)

    def hard_fault(self,side,reason,**context):
        # Preserve the first fault and retry the authoritative stop without log spam.
        if self.latch.phase=='FAULT':
            self.node.stop_validation(self.latch.reason);return
        # Telemetry failure must never prevent the authoritative hardware stop.
        try:self.emit_event('HARD_FAULT',side,reason,**context)
        finally:self.node.stop_validation(reason)

    def hold_side(self,side,error):
        if self.latch.phase=='FAULT':return
        try:
            hand=self.node._hands[next(s for s in self.node._hands if s.value==side)]
            measured=np.asarray(hand.read_joint_state().position,dtype=float)
            from .retarget import JOINT_LIMITS
            if measured.shape!=(22,) or not np.isfinite(measured).all() or np.any(measured<JOINT_LIMITS[:,0]) or np.any(measured>JOINT_LIMITS[:,1]):
                raise ValueError('Invalid hand feedback during soft hold')
            already=self.latch.holds[side]
            pending_stop=side in self.latch.pending_source_holds
            self.latch.pending_source_holds.pop(side,None)
            self.latch.hold_side(side,error)
            self.gate.clear(side)
            if not already or pending_stop:
                if isinstance(self.latch,MinimumMotionLatch):self.stop_hardware_side(side)
                else:self.request(side,False)
            context=dict(self.raw.context.get(side,{}));context.update(error.context)
            if context.get('joint') in JOINT_NAMES:
                context['measured_value']=measured[JOINT_NAMES.index(context['joint'])]
            self.emit_event('SOFT_HOLD',side,error,**context)
        except Exception as failure:
            self.hard_fault(side,failure)

    def save_delta_stats(self):
        if hasattr(self.raw,'distribution'):
            with self.delta_path.open('w',encoding='utf-8') as stream:
                import os
                os.fchmod(stream.fileno(),0o600)
                json.dump(self.raw.distribution.snapshot(),stream,allow_nan=False)

    def soft_anomaly(self):
        if self.latch.phase=='FAULT':return
        self.latch.pause(self.raw.reason)
        # Attempt both stops even if one actuator fails; failure escalates to FAULT.
        errors=[]
        for side in ('left','right'):
            try:self.request(side,False)
            except Exception as error:errors.append(str(error))
        if errors:self.node.stop_validation('Soft anomaly stop failed: '+'; '.join(errors))
        else:self.node.get_logger().warning('HAND FORWARDING PAUSED: '+self.raw.reason)

    def process(self,side,packet,stamp):
        from sensor_msgs.msg import JointState
        enum=next(s for s in self.node._hands if s.value==side);hand=self.node._hands[enum]
        measured=hand.read_joint_state().position;now_ns=time.monotonic_ns()
        if self.latch.phase=='FAULT':return
        if not self.latch.requested[side]:self.gate.clear(side)
        q=self.gate.accept(packet,feedback=(measured,now_ns),now_ns=now_ns)
        frame_id=f"{self.config['source']}:{side}:{packet['session_id']}:{packet['sequence']}"
        held_before=self.latch.holds[side] if self.severity_policy else False
        forward=self.latch.command(side,JOINT_NAMES,q,stamp,self.node.get_clock().now().nanoseconds,
            frame_id,now_ns/1e9,measured)
        if self.severity_policy:
            if held_before and not self.latch.holds[side]:
                self.emit_event('WARNING',side,self.latch.side_reasons[side],**self.raw.context.get(side,{}))
            raw_q=np.asarray(packet['positions_rad'],dtype=float)
            limited=np.abs(raw_q-q);j=int(np.argmax(limited))
            if limited[j]>1e-8 and forward:
                context=dict(self.raw.context.get(side,{}))
                context.update(joint=JOINT_NAMES[j],target_value=raw_q[j],measured_value=np.asarray(measured)[j],delta=limited[j])
                self.emit_event('WARNING',side,'Target bounded by existing acquisition/slew limit; only limited command forwarded',**context)
        msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.name=list(JOINT_NAMES);msg.position=[float(v) for v in (q if forward else measured)]
        msg.header.frame_id=('supervisor_active:' if forward else 'supervisor_disabled:')+frame_id
        if forward:
            self.node.forward_full(enum,msg)
            if isinstance(self.latch,MinimumMotionLatch):
                self.gate.previous[side]=(q.copy(),now_ns,packet['session_id'])
            self.publishers[side].publish(msg)
        else:
            # Observation-only acquisition must never advance the send trajectory.
            # Start can precede the arm heartbeat by several source callbacks.
            self.gate.clear(side)

    def close(self):
        self.save_delta_stats()
        for side in ('left','right'):
            self.latch.requested[side]=False
        self.server.close()
