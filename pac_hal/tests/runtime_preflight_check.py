"""Negative startup test on an already running workcell with a bad task sample.
Actual C must remain FAULT and never forward an otherwise valid TCP command.
"""
import argparse
from copy import deepcopy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from intelion_interfaces.msg import HalCommand,HalState,HalJointStatus
p=argparse.ArgumentParser();p.add_argument('--session',type=int,required=True);p.add_argument('--output',required=True);args=p.parse_args()
rclpy.init();node=rclpy.create_node('preflight_rejection_check')
states=[];receivers=[];reports=[]
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(HalState,'/hal/state',states.append,q)
node.create_subscription(HalJointStatus,'/hal_position_controller/status',receivers.append,q)
node.create_subscription(String,'/hal/preflight',lambda m:reports.append(json.loads(m.data)),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub=node.create_publisher(HalCommand,'/hal/final_command',q)
def spin(duration):
 end=time.monotonic()+duration
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
try:
 deadline=time.monotonic()+20
 while not (states and receivers and reports and pub.get_subscription_count()) and time.monotonic()<deadline:spin(.02)
 assert states and receivers and reports,'startup topics unavailable'
 assert reports[-1]['status']=='INCOMPATIBLE' and reports[-1]['error_code']==4,reports
 assert all(s.hal_state==HalState.HAL_FAULT for s in states),[s.hal_state for s in states]
 baseline=states[-1].tcp_pose.position
 initial_seq=receivers[-1].last_applied_command_seq
 for seq in range(1,21):
  c=HalCommand();c.session_id=args.session;c.command_seq=seq;c.header.frame_id='map';c.mode=c.MODE_POSITION
  c.target_pose=deepcopy(states[-1].tcp_pose);c.target_pose.position.z+=.001
  c.created_mono_ns=time.monotonic_ns();c.expires_mono_ns=c.created_mono_ns+5000000
  pub.publish(c);spin(.02)
 spin(.2)
 assert receivers[-1].last_applied_command_seq==initial_seq==0
 assert all(s.hal_state==HalState.HAL_FAULT for s in states)
 actual=states[-1].tcp_pose.position;delta=math.dist([baseline.x,baseline.y,baseline.z],[actual.x,actual.y,actual.z])
 assert delta<.0001,delta
 report={'preflight':reports[-1],'attempted_commands':20,'receiver_applied_seq':initial_seq,'tcp_drift_m':delta,'fault_never_ready':True,'passed':True}
 Path(args.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:
 node.destroy_node();rclpy.shutdown()
