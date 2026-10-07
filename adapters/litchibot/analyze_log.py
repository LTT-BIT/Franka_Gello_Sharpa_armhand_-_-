"""Validate a dry-run JSONL and summarize timing, shape, limits and jumps."""
import argparse
from collections import Counter
import json

import numpy as np

from .retarget import JOINT_NAMES, JOINT_LIMITS


def analyze(path):
    sides = {s: [] for s in ('left','right')}
    sent = 0
    summary = None
    for line in open(path, encoding='utf-8'):
        row = json.loads(line)
        if row['type'] == 'summary':
            summary = row
        if row['type'] != 'target':
            continue
        if row['side'] not in sides or tuple(row['joint_names']) != JOINT_NAMES:
            raise ValueError('Side/joint order mismatch')
        q = np.asarray(row['positions_rad'])
        if q.shape != (22,) or not np.isfinite(q).all():
            raise ValueError('Invalid target shape/value')
        if np.any(q<JOINT_LIMITS[:,0]-1e-8) or np.any(q>JOINT_LIMITS[:,1]+1e-8):
            raise ValueError('Target outside Sharpa limits')
        if not row['dry_run']:
            raise ValueError('Log is not a dry-run')
        sent += bool(row['sent'])
        sides[row['side']].append(row)
    if sent:
        raise ValueError('Dry-run recorded a send')
    result = {'path': str(path), 'sent': sent, 'summary': summary, 'sides': {}}
    for side, rows in sides.items():
        times=np.array([r['generated_monotonic_ns'] for r in rows],dtype=np.int64)
        dt=np.diff(times)/1e9
        if dt.size and (dt<=0).any():
            raise ValueError('Non-monotonic target timestamps')
        q=np.array([r['positions_rad'] for r in rows])
        changes=np.diff(q,axis=0) if len(rows)>1 else np.empty((0,22))
        invalid=Counter(n for r in rows for n in r['invalid_glove_joints'])
        peaks=[]
        for i,delta in enumerate(changes):
            joint=int(np.argmax(np.abs(delta)))
            if abs(delta[joint])>0.2:
                peaks.append({'sequence':rows[i+1]['sequence'],'joint':JOINT_NAMES[joint],
                    'delta_rad':float(delta[joint]),'from_rad':float(q[i,joint]),'to_rad':float(q[i+1,joint])})
        result['sides'][side] = {'count':len(rows), 'shape':[22], 'joint_order_verified':True,
            'limits_verified':True,'invalid_glove_joints':dict(invalid),
            'frequency_hz':float(1/dt.mean()) if dt.size else None,
            'mean_dt_ms':float(dt.mean()*1000) if dt.size else None,
            'p50_dt_ms':float(np.percentile(dt,50)*1000) if dt.size else None,
            'p95_dt_ms':float(np.percentile(dt,95)*1000) if dt.size else None,
            'max_dt_ms':float(dt.max()*1000) if dt.size else None,
            'max_delta_rad':float(np.abs(changes).max()) if changes.size else 0,
            'jumps_above_0_2_rad':peaks,
            'statuses':dict(Counter(r['glove_status'] for r in rows))}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    args=parser.parse_args()
    print(json.dumps(analyze(args.log),indent=2))


if __name__=='__main__':
    main()
