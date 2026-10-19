"""One-shot task metadata editing; initializes no ROS nodes or devices."""
import argparse
import json
import os
from operator_tasks import add_operator_task,delete_operator_task


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('add','delete'))
    parser.add_argument('value')
    args=parser.parse_args()
    os.environ['TELEOP_TASK_HELPER']='1'
    result=(add_operator_task(args.value) if args.action=='add' else delete_operator_task(args.value))
    print(json.dumps(dict(result=result),ensure_ascii=False))

if __name__=='__main__':main()
