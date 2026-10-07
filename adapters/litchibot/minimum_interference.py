"""LitchiBot normal operation: persistent permission, validated targets, pattern holds."""
from collections import deque

import numpy as np

from .fault_severity import SeverityRawGate, PerSideMotionLatch, SoftHold
from .retarget import JOINT_NAMES, JOINT_LIMITS
from .validation_safety import TIMEOUT_S, SOURCE_NAMES


class RejectFrame(ValueError):
    """Drop an isolated unusable frame without stopping the side or clearing its key."""
    def __init__(self,reason,**context):
        super().__init__(reason);self.context=context


class MinimumRawGate(SeverityRawGate):
    history_frames=8
    history_seconds=.5
    reversal_count=3
    partial_count=3
    # Geometry-scaled candidate references classify history, never gate one frame.
    target_reference=.12*np.ptp(JOINT_LIMITS,axis=1)
    source_reference=np.array([.25,.25,.12,.08]+[x for _ in range(4) for x in (.10,.12,.10,.08)])*np.pi

    def __init__(self,*,diagnostic=False):
        super().__init__(diagnostic=False)
        self.threshold=None  # no fixed shared single-frame cutoff in normal operation
        self.history={s:deque(maxlen=self.history_frames) for s in ('left','right')}
        self.partial={s:0 for s in self.history}
        self.warnings={}

    def observe(self,packet,now_ns=None):
        side=packet.get('side');self.warnings.pop(side,None)
        old=self.previous.get(side)
        if old and (packet.get('source_received_monotonic_ns',old[4])-old[4])/1e9>TIMEOUT_S:
            self.history[side].clear();self.partial[side]=0
        return super().observe(packet,now_ns)

    def check_extreme(self,old,delta,source_delta,angles):
        # Within-limit target motion alone cannot prove a corrupt signal. Native
        # driver/SDK motion handling remains responsible for executing valid targets.
        # A canonical channel changing by more than a full turn is branch explosion.
        if old and (not np.isfinite(delta).all() or not np.isfinite(source_delta).all() or
                    np.any((source_delta>2*np.pi)&((np.abs(angles)>2*np.pi)|(np.abs(old[1])>2*np.pi)))):
            raise ValueError('Extreme invalid source discontinuity / branch explosion')

    def check_quality(self,packet,q,angles,masks,side):
        try:super().check_quality(packet,q,angles,masks,side)
        except SoftHold as error:
            self.partial[side]+=1
            self.history[side].clear()
            if self.partial[side]>=self.partial_count:
                raise SoftHold('Repeated unusable/partial-valid source frames: '+str(error),**error.context)
            raise RejectFrame('Isolated unusable/partial-valid frame rejected: '+str(error),**error.context)
        self.partial[side]=0

    def check_motion(self,old,side,q,delta,source_delta,dt):
        if not old:return
        signed_target=q-old[0]
        signed_source=self.previous[side][1]-old[1]
        signs=np.concatenate((np.where(delta>=self.target_reference,np.sign(signed_target),0),
                              np.where(source_delta>=self.source_reference,np.sign(signed_source),0)))
        history=self.history[side];stamp=self.previous[side][4]
        while history and (stamp-history[0][0])/1e9>self.history_seconds:history.popleft()
        history.append((stamp,signs))
        values=np.array([v for _,v in history])
        flips=[]
        for j in range(values.shape[1]):
            nonzero=values[:,j][values[:,j]!=0]
            flips.append(int(np.count_nonzero(nonzero[1:]!=nonzero[:-1])))
        if max(flips,default=0)>=self.reversal_count:
            j=int(np.argmax(flips));context=dict(self.context[side])
            if j<22:
                context.update(joint=JOINT_NAMES[j],target_value=q[j],delta=delta[j])
            else:
                k=j-22
                context.update(joint=SOURCE_NAMES[k],source_joint=SOURCE_NAMES[k],
                               source_value=self.previous[side][1][k],target_value=None,delta=source_delta[k])
            history.clear()
            raise SoftHold('Persistent oscillating / sign-reversal pattern',**context)
        if np.any(signs):
            self.warnings[side]=('Single fast/moderate movement; validated target forwarding continues',dict(self.context[side]))


class MinimumMotionLatch(PerSideMotionLatch):
    tracking_seconds=2.0
    tracking_reference=.15*np.ptp(JOINT_LIMITS,axis=1)

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.persistent_permit=True
        self.divergence={s:None for s in self.requested}

    def hold_side(self,side,reason):
        if self.phase=='FAULT':return
        self.holds[side]=True;self.side_reasons[side]=str(reason);self.recovered[side]=False
        self.samples[side]=0;self.commanded.pop(side,None);self.divergence[side]=None
        # requested, phase, GUI lease and opposite-side state remain untouched.

    def heartbeat(self,message,now):
        if self.phase=='READY' and message.get('token')==self.permit['session_token']:
            stamp=message.get('monotonic_ns');seq=message.get('sequence')
            valid=isinstance(stamp,int) and 0<=now-stamp/1e9<=TIMEOUT_S and isinstance(seq,int) and seq>self.control_sequence
            if valid and message.get('hold') is True and any(self.holds[s] for s in self.requested if self.requested[s]):
                self.control_sequence=seq;self.last_hold=now;return
        super().heartbeat(message,now)

    def check_output_motion(self,side,q,measured,forward,now):
        # Normal/full-fake has no artificial shared slew or acquisition amplitude gate.
        # Structure, finite values, physical limits, identity and freshness are checked
        # by the parent command method; driver/SDK/native motion limits stay untouched.
        actual=self.commanded.get(side)
        if forward and actual and now<actual[1]:
            raise ValueError('Command clock moved backwards')

    def command(self,side,names,positions,stamp_ns,ros_now_ns,frame_id,now,feedback):
        was_held=self.holds.get(side,False)
        if not was_held and self.requested.get(side) and self.phase=='ARMED':
            error=np.abs(np.asarray(positions)-np.asarray(feedback))
            if np.any(error>self.tracking_reference):
                if self.divergence[side] is None:self.divergence[side]=now
                if now-self.divergence[side]>=self.tracking_seconds:
                    j=int(np.argmax(error/self.tracking_reference))
                    raise SoftHold('Persistent command / measured divergence',joint=JOINT_NAMES[j],target_value=positions[j],measured_value=feedback[j],delta=error[j],dt=now-self.divergence[side])
            else:self.divergence[side]=None
        forward=super().command(side,names,positions,stamp_ns,ros_now_ns,frame_id,now,feedback)
        if was_held and not self.holds[side]:
            self.side_reasons[side]='Stable recovery; ACTIVE; persistent operator permit retained' if self.requested[side] else 'Stable recovery; READY; operator permit OFF'
        return forward
