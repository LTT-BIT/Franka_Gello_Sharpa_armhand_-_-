"""LitchiBot continuous-operation policy. No ROS or hardware dependencies."""
import time

import numpy as np

from .normal_continuity import DeltaDistribution, NormalContinuityGate
from .retarget import JOINT_NAMES, JOINT_LIMITS
from .transport import validate_packet
from .validation_safety import (RawContinuityGate, MotionLatch, SOURCE_NAMES, TIMEOUT_S,
                                SPEED_RAD_S, WARMUP_FRAMES, EXCURSION_RAD)


# Stop forwarding at the original timeout; latch only a sustained packet loss.
SOURCE_LOSS_HARD_S = 5 * TIMEOUT_S


def scalar(value):
    if value is None:return None
    number=float(value)
    return number if np.isfinite(number) else str(number)


def event(severity, side, reason, *, joint=None, source_value=None, target_value=None,
          measured_value=None, delta=None, dt=None, recovery_state=None, source_joint=None):
    return dict(severity=severity,side=side,joint=joint,source_joint=source_joint,
                source_value=scalar(source_value),target_value=scalar(target_value),
                measured_value=scalar(measured_value),delta=scalar(delta),dt=scalar(dt),
                recovery_state=recovery_state,reason=str(reason))


def frame_context(packet, previous=None):
    def vector(key):
        try:
            q=np.asarray(packet.get(key),dtype=float)
            return q if q.ndim==1 else np.array([])
        except (ValueError,TypeError):return np.array([])
    q=vector('positions_rad');a=vector('glove_joint_angles_rad')
    delta=np.abs(q-previous[0]) if previous and q.shape==(22,) else np.zeros(len(q))
    invalid=np.flatnonzero(~np.isfinite(q))
    j=int(invalid[0]) if len(invalid) else int(np.argmax(delta)) if len(delta) else None
    invalid_a=np.flatnonzero(~np.isfinite(a))
    da=np.abs(a-previous[1]) if previous and a.shape==(20,) else np.zeros(len(a))
    k=int(invalid_a[0]) if len(invalid_a) else int(np.argmax(da)) if len(da) else None
    try:dt=(packet['source_received_monotonic_ns']-previous[4])/1e9 if previous else None
    except (TypeError,KeyError):dt=None
    return dict(joint=JOINT_NAMES[j] if j is not None and j<22 else None,
                source_joint=SOURCE_NAMES[k] if k is not None and k<20 else None,
                target_value=q[j] if j is not None else None,source_value=a[k] if k is not None else None,
                delta=delta[j] if j is not None else None,dt=dt)


class SoftHold(ValueError):
    def __init__(self, reason, **context):
        super().__init__(reason)
        self.context=context


class SeverityRawGate(RawContinuityGate):
    """Retains raw shape/order/finiteness/limits/freshness before severity decisions."""
    def __init__(self, *, diagnostic=False):
        super().__init__(diagnostic=diagnostic)
        self.distribution=DeltaDistribution()
        self.context={}
        self.arrival_ns={}

    def observe(self, packet, now_ns=None):
        now_ns=time.monotonic_ns() if now_ns is None else now_ns
        side=packet.get('side')
        self.context[side]=frame_context(packet,self.previous.get(side))
        q=validate_packet(packet,now_ns=now_ns,max_age_s=TIMEOUT_S,allow_dry_run=True)
        if tuple(packet.get('glove_joint_names',()))!=SOURCE_NAMES:
            raise ValueError('Source joint names/order mismatch')
        angles=np.asarray(packet.get('glove_joint_angles_rad'),dtype=float)
        if angles.shape!=(20,) or not np.isfinite(angles).all():
            raise ValueError('20 finite source joint angles required')
        masks=(packet.get('valid_target_mask'),packet.get('valid_glove_joint_mask'))
        if any(not isinstance(m,list) or len(m)!=n or any(type(v) is not bool for v in m)
               for m,n in zip(masks,(22,20))):
            raise ValueError('Source/target validity mask shape/type mismatch')
        side=packet['side'];old=self.previous.get(side)
        delta=np.zeros(22);source_delta=np.zeros(20);dt=None
        if old:
            if packet['session_id']!=old[2] or packet['sequence']<=old[3] or packet['source_received_monotonic_ns']<=old[4]:
                raise ValueError('Source session/order changed')
            dt=(packet['source_received_monotonic_ns']-old[4])/1e9
            if dt>TIMEOUT_S:
                # A fresh, structurally checked arrival is a new baseline, not an
                # adjacent sample to the pre-gap pose. Never leave that anchor frozen.
                self.arrival_ns[side]=now_ns
                self.previous[side]=(q.copy(),angles.copy(),packet['session_id'],packet['sequence'],packet['source_received_monotonic_ns'])
                self.context[side]=dict(joint=None,source_joint=None,source_value=None,
                                        target_value=None,delta=None,dt=dt)
                raise SoftHold('Fresh source after gap; continuity baseline rebuilt',**self.context[side])
            delta=np.abs(q-old[0]);source_delta=np.abs(angles-old[1])
            self.distribution.add(side,'target',delta);self.distribution.add(side,'source',source_delta)
        self.arrival_ns[side]=now_ns
        j=int(np.argmax(delta));a=int(np.argmax(source_delta))
        self.context[side]=dict(joint=JOINT_NAMES[j],source_joint=SOURCE_NAMES[a],
                                source_value=angles[a],target_value=q[j],
                                delta=max(delta[j],source_delta[a]),dt=dt)
        self.check_extreme(old,delta,source_delta,angles)
        # Observation anchor only; a rejected frame is never a command trajectory.
        self.previous[side]=(q.copy(),angles.copy(),packet['session_id'],packet['sequence'],packet['source_received_monotonic_ns'])
        self.check_quality(packet,q,angles,masks,side)
        self.check_motion(old,side,q,delta,source_delta,dt)
        return True

    def check_extreme(self, old, delta, source_delta,angles):
        if old and (np.any(delta>=NormalContinuityGate.target_extreme) or
                    np.any(source_delta>=NormalContinuityGate.source_extreme)):
            raise ValueError('Extreme raw discontinuity')

    def check_quality(self,packet,q,angles,masks,side):
        partial=not all(masks[0]) or not all(masks[1]) or packet.get('valid_glove_joint_count')!=20
        if partial or packet.get('held_joints') or packet.get('initial_fallback_joints'):
            indices=np.flatnonzero(~np.asarray(masks[0]))
            invalid_source=np.flatnonzero(~np.asarray(masks[1]))
            if len(invalid_source):
                a=int(invalid_source[0]);self.context[side].update(source_joint=SOURCE_NAMES[a],source_value=angles[a])
            if len(indices):
                j=int(indices[0]);self.context[side].update(joint=JOINT_NAMES[j],target_value=q[j])
            elif len(invalid_source):
                # No asserted target dependency: identify the source channel without guessing a mapping.
                self.context[side].update(joint=SOURCE_NAMES[a],target_value=None)
            raise SoftHold(f"Partial-valid input: {packet.get('valid_glove_joint_count')}/20; hold/fallback={bool(packet.get('held_joints') or packet.get('initial_fallback_joints'))}",**self.context[side])
        if packet.get('glove_status')!='solved' or packet.get('clipped_joints'):
            raise SoftHold('Unsolved or schema-clipped source target',**self.context[side])
    def check_motion(self,old,side,q,delta,source_delta,dt):
        if old and (np.any(delta>=self.threshold) or np.any(source_delta>=self.threshold)):
            raise SoftHold('Moderate raw discontinuity',**self.context[side])


class PerSideMotionLatch(MotionLatch):
    """Global hard latch, per-side soft holds; validation-only latch is untouched."""
    recovery_frames=WARMUP_FRAMES

    def __init__(self, permit, now, *, diagnostic=False, **kwargs):
        super().__init__(permit,now,**kwargs)
        if self.validation_only or not self.gui_controlled:
            raise ValueError('Per-side policy requires managed non-validation mode')
        self.diagnostic=diagnostic
        self.holds={s:False for s in self.requested}
        self.side_reasons={s:'' for s in self.requested}
        self.recovered={s:False for s in self.requested}
        self.source_seen={}
        self.pending_source_holds={}
        self.source_gap_sides=set()

    def note_source(self, side, now):
        self.source_seen[side]=now
        self.source_gap_sides.discard(side)

    def hold_side(self, side, reason):
        if self.phase=='FAULT':return
        self.holds[side]=True;self.side_reasons[side]=str(reason);self.recovered[side]=False
        self.requested[side]=False;self.samples[side]=0;self.commanded.pop(side,None)
        if self.phase=='ARMED' and not any(self.requested.values()):
            self.phase='READY';self.last_hold=None;self.armed_at=None

    def request_hand(self, side, enabled, now):
        if enabled and self.holds.get(side):raise ValueError(self.side_reasons[side])
        super().request_hand(side,enabled,now)
        if enabled:self.recovered[side]=False;self.side_reasons[side]=''

    def side_state(self, side, now):
        if self.phase=='FAULT':return 'HARD_FAULT'
        if self.holds[side]:return 'SOFT_HOLD'
        if not self.requested[side]:return 'READY' if self.recovered[side] else 'DISABLED'
        return super().side_state(side,now)

    def heartbeat(self, message, now):
        # The non-validation acquisition authority checks the requested side only.
        if self.phase=='READY' and message.get('token')==self.permit['session_token']:
            stamp=message.get('monotonic_ns');seq=message.get('sequence')
            valid=isinstance(stamp,int) and 0<=now-stamp/1e9<=TIMEOUT_S and isinstance(seq,int) and seq>self.control_sequence
            if valid and message.get('hold') is True:
                if any(self.holds[s] or s not in self.last or now-self.source_seen.get(s,self.started)>TIMEOUT_S
                       for s in self.requested if self.requested[s]):
                    self.trip('Stale acquisition at arm');return
                self.control_sequence=seq;self.last_hold=now
                self.baseline={s:self.measured[s].copy() for s in self.measured}
                self.phase='ARMED';self.armed_at=now;return
        super().heartbeat(message,now)

    def watch(self, now, wall_now):
        if self.phase=='FAULT':return
        if any(self.requested.values()) and (self.last_gui is None or now-self.last_gui>TIMEOUT_S):
            self.trip('GUI enable lease expired');return
        if self.phase=='ARMED' and (self.last_hold is None or now-self.last_hold>TIMEOUT_S):
            self.trip('Deadman watchdog expired');return
        # Every structurally checked source frame refreshes input liveness, even while held.
        for side,stamp in self.source_seen.items():
            age=now-stamp
            if age>SOURCE_LOSS_HARD_S:
                self.trip(f'Sustained source packet loss: {side}, arrival age={age:.6f}s');return
            if age>TIMEOUT_S and side not in self.source_gap_sides:
                self.source_gap_sides.add(side)
                self.hold_side(side,'Source arrival timeout; waiting for fresh baseline')
                self.pending_source_holds[side]=age
        if len(self.source_seen)<2 and now-self.started>30:
            self.trip('Acquisition timeout')

    def command(self, side, names, positions, stamp_ns, ros_now_ns, frame_id, now, feedback):
        if self.phase=='FAULT':raise ValueError(self.reason)
        if side not in self.requested or tuple(names)!=JOINT_NAMES:
            raise ValueError('Left/right joint names/order mismatch')
        fields=frame_id.split(':')
        if len(fields)!=4 or fields[:2]!=[self.source,side] or not fields[2] or not fields[3].isdigit():
            raise ValueError('Missing side/session/sequence identity')
        if not 0<=(ros_now_ns-stamp_ns)/1e9<=TIMEOUT_S:raise ValueError('Stale/future ROS command')
        q=np.asarray(positions,dtype=float);measured=np.asarray(feedback,dtype=float)
        if any(v.shape!=(22,) or not np.isfinite(v).all() for v in (q,measured)):
            raise ValueError('Expected finite 22D command and feedback')
        if any(np.any(v<JOINT_LIMITS[:,0]) or np.any(v>JOINT_LIMITS[:,1]) for v in (q,measured)):
            raise ValueError('Command/feedback joint limit violation')
        previous=self.last.get(side)
        if previous and (fields[2]!=previous[1][0] or int(fields[3])<=previous[1][1]):
            raise ValueError('ROS session/order changed')
        if side in self.source_seen and now-self.source_seen[side]>TIMEOUT_S:
            raise ValueError('ROS command gap watchdog expired')
        self.last[side]=(q.copy(),(fields[2],int(fields[3])),now);self.measured[side]=measured.copy()
        forward=self.phase=='ARMED' and self.requested[side] and not self.holds[side]
        self.check_output_motion(side,q,measured,forward,now)
        self.samples[side]+=1
        if self.holds[side] and self.samples[side]>=self.recovery_frames:
            self.holds[side]=False;self.recovered[side]=True
            self.side_reasons[side]='Stable recovery; READY' if self.diagnostic else 'Stable recovery; READY; Start this hand again'
        if self.phase=='WARMUP' and any(n>=WARMUP_FRAMES for n in self.samples.values()):self.phase='READY'
        self.watch(now,time.time())
        if forward and self.phase=='ARMED':self.commanded[side]=(q.copy(),now)
        return forward and self.phase=='ARMED'

    def check_output_motion(self,side,q,measured,forward,now):
        if forward:
            actual=self.commanded.get(side)
            base,dt=(actual[0],min(.05,now-actual[1])) if actual else (measured,1/30)
            delta=np.abs(q-base);j=int(np.argmax(delta))
            if dt<0:raise ValueError('Command clock moved backwards')
            if delta[j]>SPEED_RAD_S*dt+1e-8:
                raise SoftHold('Hardware command slew/acquisition exceeded',joint=JOINT_NAMES[j],
                               target_value=q[j],measured_value=measured[j],delta=delta[j],dt=dt)
        elif self.requested[side] and np.abs(q-measured).max()>EXCURSION_RAD:
            j=int(np.argmax(np.abs(q-measured)))
            raise SoftHold('Acquisition target is too far from measured pose',joint=JOINT_NAMES[j],
                           target_value=q[j],measured_value=measured[j],delta=abs(q[j]-measured[j]),dt=None)
