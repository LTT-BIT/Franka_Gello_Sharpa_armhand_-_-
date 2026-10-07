"""Read-only ROS topic contract monitor. Creates no hand or hardware objects."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from adapters.litchibot.retarget import JOINT_NAMES, JOINT_LIMITS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=40)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, Image
    from geometry_msgs.msg import WrenchStamped
    rclpy.init(args=[])
    node = Node('terminal2_contract_monitor')
    streams = {}
    errors = Counter()
    driver_counts = []
    def sample(topic, msg, kind, side=None):
        now = time.monotonic_ns()
        record = streams.setdefault(topic, {'count':0, 'times':[], 'max_delta_rad':0,
                                           'last_q':None, 'last_stamp_ns':None})
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if record['last_stamp_ns'] is not None and stamp<=record['last_stamp_ns']:
            errors[topic+': nonincreasing stamp'] += 1
        record['last_stamp_ns'] = stamp
        if kind in ('command', 'state'):
            expected = list(JOINT_NAMES) if kind=='command' else [side+'_'+n for n in JOINT_NAMES]
            if list(msg.name)!=expected:
                errors[topic+': name/order'] += 1
            q = np.asarray(msg.position)
            if q.shape!=(22,) or not np.isfinite(q).all():
                errors[topic+': shape/finite'] += 1
                return
            if np.any(q<JOINT_LIMITS[:,0]-1e-8) or np.any(q>JOINT_LIMITS[:,1]+1e-8):
                errors[topic+': limit'] += 1
            if kind=='command' and not msg.header.frame_id.startswith('litchibot:'+side+':'):
                errors[topic+': side identity'] += 1
            if record['last_q'] is not None:
                record['max_delta_rad']=max(record['max_delta_rad'],float(np.max(np.abs(q-record['last_q']))))
            record['last_q']=q
        record['count']+=1
        record['times'].append(now)
    subscriptions=[]
    for side in ('left','right'):
        for kind, suffix, qos in [('command','command',10),('state','joint_states',qos_profile_sensor_data)]:
            topic=f'/sharpa/{side}/{suffix}'
            subscriptions.append(node.create_subscription(JointState, topic,
                lambda msg,t=topic,k=kind,s=side:sample(t,msg,k,s),qos))
        for finger in ('thumb','index','middle','ring','pinky'):
            for suffix, cls in [('wrench',WrenchStamped),('deformation',Image),('raw',Image)]:
                topic=f'/sharpa/{side}/tactile/{finger}/{suffix}'
                subscriptions.append(node.create_subscription(cls,topic,
                    lambda msg,t=topic:sample(t,msg,'tactile'),qos_profile_sensor_data))
    started=time.monotonic()
    next_graph=started
    try:
        while time.monotonic()-started<args.duration:
            rclpy.spin_once(node,timeout_sec=0.01)
            if time.monotonic()>=next_graph:
                driver_counts.append(sum(name=='sharpa_driver' for name,ns in node.get_node_names_and_namespaces()))
                next_graph=time.monotonic()+0.5
    finally:
        result={'errors':dict(errors),'topics':{},'max_sharpa_driver_nodes':max(driver_counts,default=0),
                'hardware_send_count':0,'hardware_mode':'fake; asserted by production launch and bridge guard'}
        for topic, record in streams.items():
            dt=np.diff(record['times'])/1e9
            result['topics'][topic]={'frames':record['count'],
                'hz':float(1/dt.mean()) if dt.size else None,
                'mean_dt_ms':float(dt.mean()*1000) if dt.size else None,
                'p95_dt_ms':float(np.percentile(dt,95)*1000) if dt.size else None,
                'max_dt_ms':float(dt.max()*1000) if dt.size else None,
                'max_delta_rad':record['max_delta_rad']}
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
