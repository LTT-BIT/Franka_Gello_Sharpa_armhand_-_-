"""Offline target continuity analysis; no ROS, SDK or hardware connection."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def analyze(path, threshold=0.1):
    sides = {'left': [], 'right': []}
    with Path(path).open() as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            if row.get('type') == 'target':
                sides[row['side']].append((line_number, row))
    result = {'input': str(Path(path).resolve()), 'anomaly_threshold_rad': threshold,
              'percentile_definition': 'all absolute per-joint adjacent-frame deltas; first frame excluded',
              'sides': {}}
    for side, records in sides.items():
        if len(records) < 2:
            continue
        rows = [r for _, r in records]
        values = np.asarray([r['positions_rad'] for r in rows], dtype=float)
        if values.shape != (len(rows), 22) or not np.isfinite(values).all():
            raise ValueError('Expected finite 22D targets')
        names = rows[0]['joint_names']
        if any(r['joint_names'] != names for r in rows):
            raise ValueError('Joint names/order changed within log')
        delta = np.abs(np.diff(values, axis=0))
        times = np.asarray([r['source_received_monotonic_ns'] for r in rows], dtype=np.int64)
        max_frame, max_joint = np.unravel_index(np.argmax(delta), delta.shape)
        peak_index = int(max_frame)+1

        def event(frame, joint):
            index = int(frame)+1; joint = int(joint)
            a, b = rows[index-1], rows[index]
            return {'side': side, 'frame_index_0based': index, 'jsonl_line_1based': records[index][0],
                    'sequence_before': a['sequence'], 'sequence_after': b['sequence'],
                    'joint_index_0based': joint, 'joint_name': names[joint],
                    'source_received_monotonic_ns': b['source_received_monotonic_ns'],
                    'source_device_timestamp_ns': b.get('source_device_timestamp_ns'),
                    'elapsed_source_s': (b['source_received_monotonic_ns']-times[0])/1e9,
                    'source_interval_ms': (b['source_received_monotonic_ns']-a['source_received_monotonic_ns'])/1e6,
                    'before_rad': a['positions_rad'][joint], 'after_rad': b['positions_rad'][joint],
                    'signed_delta_rad': b['positions_rad'][joint]-a['positions_rad'][joint],
                    'abs_delta_rad': float(delta[frame,joint])}

        indices = np.argsort(-delta.reshape(-1), kind='stable')[:10]
        ranking = np.argsort(-delta[max_frame], kind='stable')
        per_joint = [{'joint_index_0based': j, 'joint_name': n,
                      'max_abs_delta_rad': float(delta[:,j].max()),
                      'samples': len(delta),
                      'p50_abs_delta_rad': float(np.percentile(delta[:,j],50)),
                      'p95_abs_delta_rad': float(np.percentile(delta[:,j],95)),
                      'p99_abs_delta_rad': float(np.percentile(delta[:,j],99)),
                      'p99_9_abs_delta_rad': float(np.percentile(delta[:,j],99.9)),
                      'anomaly_count': int((delta[:,j] >= threshold).sum())} for j,n in enumerate(names)]
        result['sides'][side] = {'frames': len(rows),
            'hz_source_timestamps': (len(rows)-1)*1e9/int(times[-1]-times[0]),
            'max_abs_delta_rad': float(delta.max()),
            **{f'p{p}_abs_delta_rad': float(np.percentile(delta,p)) for p in (50,95,99)},
            'per_frame_max_percentiles_rad': {str(p): float(np.percentile(delta.max(axis=1),p)) for p in (50,95,99)},
            'per_joint': per_joint, 'peak_event': event(max_frame,max_joint),
            'top10_joint_events': [event(*np.unravel_index(int(k),delta.shape)) for k in indices],
            'peak_frame_22_joint_ranking': [event(max_frame,j) for j in ranking],
            'peak_context': [{'frame_index_0based': i, 'jsonl_line_1based': records[i][0], **rows[i]}
                             for i in range(max(0,peak_index-10),min(len(rows),peak_index+11))]}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--anomaly-threshold', type=float, default=0.1)
    args = parser.parse_args()
    if not np.isfinite(args.anomaly_threshold) or args.anomaly_threshold <= 0:
        parser.error('positive finite anomaly threshold required')
    result = analyze(args.input,args.anomaly_threshold)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    with args.output.with_suffix('.csv').open('w') as stream:
        fields = ['side','joint_index_0based','joint_name','samples','max_abs_delta_rad','p50_abs_delta_rad',
                  'p95_abs_delta_rad','p99_abs_delta_rad','p99_9_abs_delta_rad','anomaly_count']
        writer = csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for side,stats in result['sides'].items():
            for row in stats['per_joint']:writer.writerow({'side':side,**row})
    print(json.dumps({s:{k:v for k,v in r.items() if k not in
        ('peak_context','per_joint','top10_joint_events','peak_frame_22_joint_ranking')}
        for s,r in result['sides'].items()},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
