"""Normal-mode raw discontinuity classification; never authorizes motion."""
from collections import deque

import numpy as np

from .retarget import JOINT_LIMITS, JOINT_NAMES
from .validation_safety import RawContinuityGate, JUMP_RAD, SOURCE_NAMES


class DeltaDistribution:
    """Bounded histograms include rejected frames; no adaptive safety thresholds."""
    edges = np.array([0, .001, .002, .005, .01, .02, .05, .1, .15, .2,
                      .3, .5, 1, 1.5, 2, 3, float('inf')])

    def __init__(self):
        self.layers = {}

    def add(self, side, layer, delta):
        key = (side, layer)
        if key not in self.layers:
            self.layers[key] = [0, np.zeros(len(delta)), np.zeros(len(delta)),
                                np.zeros((len(delta), len(self.edges)-1), dtype=int)]
        data = self.layers[key]
        data[0] += 1; data[1] += delta; data[2] = np.maximum(data[2], delta)
        bins = np.searchsorted(self.edges, delta, side='right')-1
        data[3][np.arange(len(delta)), bins] += 1

    def snapshot(self):
        return {'definition': 'absolute adjacent valid raw frame deltas, including rejected anomalies',
                'histogram_edges_rad': [float(x) if np.isfinite(x) else 'inf' for x in self.edges],
                'layers': [{'side': side, 'layer': layer, 'samples': data[0],
                            'per_joint': [{'joint': name, 'mean_rad': float(data[1][i]/data[0]),
                                           'max_rad': float(data[2][i]), 'histogram': data[3][i].tolist()}
                                          for i, name in enumerate(JOINT_NAMES if layer=='target' else SOURCE_NAMES)]}
                           for (side, layer), data in self.layers.items()]}


class NormalContinuityGate(RawContinuityGate):
    # Preserve the existing 0.10 detection boundary as a warning/reject trigger.
    # Extreme boundaries are provisional geometry-based caps, not calibrated values.
    target_moderate = np.full(22, JUMP_RAD)
    target_extreme = np.maximum(3*JUMP_RAD, .5*np.ptp(JOINT_LIMITS, axis=1))
    source_moderate = np.full(20, JUMP_RAD)
    source_extreme = np.full(20, np.pi/2)
    repeat_window_s = 1.0
    repeat_count = 3

    def __init__(self):
        super().__init__()
        self.events = {side: deque() for side in ('left', 'right')}
        self.distribution = DeltaDistribution()
        self.reason = ''

    def check_delta(self, side, delta, source_delta, stamp_ns):
        self.distribution.add(side, 'target', delta)
        if source_delta is not None:
            self.distribution.add(side, 'source', source_delta)
        extreme = np.any(delta >= self.target_extreme) or (
            source_delta is not None and np.any(source_delta >= self.source_extreme))
        moderate = np.any(delta >= self.target_moderate) or (
            source_delta is not None and np.any(source_delta >= self.source_moderate))
        if not moderate:
            return True
        j = int(np.argmax(delta))
        source = '' if source_delta is None else f', source {SOURCE_NAMES[int(np.argmax(source_delta))]} delta={source_delta.max():.6f} rad'
        self.reason = f'Raw discontinuity: {side} {JOINT_NAMES[j]}, target delta={delta[j]:.6f} rad{source}'
        if extreme:
            raise ValueError('Extreme '+self.reason)
        events = self.events[side]
        while events and (stamp_ns-events[0])/1e9 > self.repeat_window_s:
            events.popleft()
        events.append(stamp_ns)
        if len(events) >= self.repeat_count:
            raise ValueError('Repeated '+self.reason)
        # The checked frame becomes an observation anchor only, never a command.
        # This permits a single spike and its return edge without permanent FAULT.
        return False


def full_continuity_gate(source, mode):
    if source=='litchibot' and mode in ('normal','fake'):
        from .normal_control import BasicNormalTarget
        return BasicNormalTarget()
    if mode in ('normal','fake'):
        return NormalContinuityGate()
    return RawContinuityGate()
