"""Robot-independent actual B / common C Gazebo regression; synthetic A only.
Run on an already started workcell with no other B publisher.
Arguments select session, output folder and independently published TCP TF link.
"""
import math
import argparse
import collections
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
from intelion_interfaces.msg import ControlCommand, HalCommand, HalState, HalJointStatus, HalJointCommand, HalError
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_prefix

parser=argparse.ArgumentParser();parser.add_argument('--session',type=int,required=True);parser.add_argument('--output',required=True);parser.add_argument('--tcp-link',required=True);parser.add_argument('--inject-receiver-expired',action='store_true');args=parser.parse_args()
OUT=Path(args.output).resolve();OUT.mkdir(parents=True,exist_ok=True)
rclpy.init()
node = rclpy.create_node('real_b_common_check')
tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node)
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
states, commands, receiver = [], collections.deque(maxlen=3000), []
node.create_subscription(HalState, '/hal/state', states.append, qos)
node.create_subscription(HalCommand, '/hal/final_command', lambda msg: commands.append(msg), qos)
node.create_subscription(HalJointStatus, '/hal_position_controller/status', receiver.append, qos)
pub = node.create_publisher(ControlCommand, '/ai/control_command', qos)
client = node.create_client(Trigger, '/rt/reset_fault')
seq = 0
base = None
goal = None
errors = []
node.create_subscription(HalError, '/hal/error', errors.append, QoSProfile(depth=32))
version = 1
send_ai = False
last_ai = 0.
process = None
report = {}
log = open(OUT/'B.log', 'w')

def spin(seconds):
    global seq, last_ai
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.002)
        now = time.monotonic()
        if send_ai and now-last_ai >= 1/15:
            msg = ControlCommand()
            msg.seq, msg.writer_boot_id, msg.layout_version = seq, 78901, version
            seq += 1
            msg.command_type, msg.status = 0, 0
            msg.target = [*goal, 0.]
            msg.confidence, msg.age_ns = 1., 0
            pub.publish(msg)
            last_ai = now

def wait(predicate, timeout=12):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        spin(.02)
        if predicate():
            return
    raise RuntimeError('condition timeout')

def launch(session=None):
    session=args.session if session is None else session
    quat = states[-1].tcp_pose.orientation
    params = {'session_id':str(session),'fifo':'false','cpu':'-1','mlock':'false',
              'use_sim_time':'true','v_max':'0.005','a_max':'0.01',
              'planned_position':str([.35,0.,.81]),'tool_orientation':str([quat.x,quat.y,quat.z,quat.w]),
              'log_root':str(OUT/'trace'),'trace_enable':'true',
              'c_state_timeout_ms':'30.0'}
    cmd = [str(Path(get_package_prefix('rt_control'))/'lib/rt_control/rt_control_node'),'--ros-args']
    for key, val in params.items():
        cmd += ['-p', key+':='+val]
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())

try:
    wait(lambda: receiver and states and states[-1].pose_valid and not states[-1].stale and states[-1].hal_state in (2,3))
    p = states[-1].tcp_pose.position
    base = [p.x,p.y,p.z]
    goal = [.35,0.,.812]
    report['initial_tcp'] = base
    report['initial_task_error_m'] = math.dist(base,[.35,0.,.81])
    assert report['initial_task_error_m'] < .001
    report['joint_count'] = receiver[-1].joint_count
    assert node.count_publishers('/hal/final_command') == 0, 'another B publisher exists'
    process = launch(args.session+1)
    spin(1.)
    assert not commands, 'B must wait when C registered session differs'
    process.send_signal(signal.SIGINT)
    process.wait(timeout=8)
    report['wrong_session_waits_without_commands'] = True
    process = launch()
    wait(lambda: commands and states[-1].last_command_result == 2)
    spin(1.)
    report['no_ai_start_hold_error_m'] = abs(states[-1].tcp_pose.position.z-base[2])
    assert report['no_ai_start_hold_error_m'] < .0003
    assert all(c.session_id == args.session and c.expires_mono_ns-c.created_mono_ns == 5_000_000 for c in commands)
    report['session_and_ttl'] = True
    send_ai = True
    spin(8.)
    report['measured_z_move_m'] = states[-1].tcp_pose.position.z-base[2]
    assert report['measured_z_move_m'] > .001, report
    assert any(c.source_result_id > 0 for c in commands)
    report['ai_to_real_b_to_c'] = True
    report['common_moves'] = []
    for dx,dy in [( .02,0.),(-.02,0.),(0.,.02),(0.,0.)]:
        goal = [.35+dx,dy,.812]
        mark = len(errors)
        spin(10.)
        actual=states[-1].tcp_pose.position
        distance=math.dist([actual.x,actual.y,actual.z],goal)
        events=[{'code':e.code,'detail':e.detail} for e in errors[mark:] if e.active]
        tf=tf_buffer.lookup_transform('map',args.tcp_link,rclpy.time.Time()).transform
        tf_xyz=[tf.translation.x,tf.translation.y,tf.translation.z]
        tf_error=math.dist(tf_xyz,goal)
        recent=[st for st in states if st.received_mono_ns >= states[-1].received_mono_ns-100000000]
        stable=all(math.dist([st.tcp_pose.position.x,st.tcp_pose.position.y,st.tcp_pose.position.z],goal)<.001 for st in recent)
        assert stable and tf_error<.001
        report['common_moves'].append({'tf_xyz':tf_xyz,'tf_error_m':tf_error,'within_1mm_last_100ms':stable,'target':goal,'actual':[actual.x,actual.y,actual.z],'error_m':distance,'active_errors':events})
        assert distance < .001, report['common_moves'][-1]
        # Timing events on a non-RT host (IK deadline, command expired in C or at
        # the receiver) are counted separately and make clean_motion false; any
        # other motion fault still fails this comparison.
        assert all((e['code']==5 and 'IK deadline exceeded' in e['detail']) or e['code']==3
                   for e in events), report['common_moves'][-1]

    if args.inject_receiver_expired:
        # Pause the actual B sender at a stationary pose. An expired receiver
        # packet is a transport expiry: C reports EXPIRED(3) with its exact seq,
        # not DEVICE/FAULT, and valid commands continue without restart.
        process.send_signal(signal.SIGSTOP)
        spin(.15)
        joint_pub=node.create_publisher(HalJointCommand,'/hal_position_controller/commands',qos)
        wait(lambda: joint_pub.get_subscription_count()>0)
        bad=HalJointCommand();bad.session_id=args.session;bad.joint_count=receiver[-1].joint_count
        bad.command_seq=commands[-1].command_seq+100000
        bad.positions=receiver[-1].commanded_positions
        bad.created_mono_ns=time.monotonic_ns()-10000000;bad.expires_mono_ns=bad.created_mono_ns+5000000
        mark=len(errors);rmark=len(receiver);joint_pub.publish(bad)
        wait(lambda: any(e.code==3 and e.active and e.command_seq==bad.command_seq for e in errors[mark:]))
        assert not any(r.rejected for r in receiver[rmark:])
        assert not any(e.code==10 and e.active for e in errors[mark:])
        report['expired_receiver_packet_reported_as_expired']=True
        process.send_signal(signal.SIGCONT)
        wait(lambda: states[-1].hal_state in (2,3) and 3 not in states[-1].active_errors)
        spin(2.)
        report['receiver_rejection_recovers_without_restart']=True
    send_ai = False
    spin(.8)
    assert '[모드]' in (OUT/'B.log').read_text()
    report['ai_loss_hold_observed'] = '-> HOLD' in (OUT/'B.log').read_text() or '→ HOLD' in (OUT/'B.log').read_text()
    assert report['ai_loss_hold_observed']
    version, send_ai = 2, True
    spin(.8)
    report['version_fault_observed'] = '→ FAULT (이유: VERSION)' in (OUT/'B.log').read_text()
    assert report['version_fault_observed']
    version = 1
    spin(.5)
    wait(lambda: client.service_is_ready())
    future = client.call_async(Trigger.Request())
    wait(lambda: future.done())
    report['fault_reset_success'] = future.result().success
    report['fault_reset_message'] = future.result().message
    assert report['fault_reset_success'], report
    spin(2.)
    assert commands[-1].session_id == args.session and states[-1].active_session_id == args.session
    assert 'FAULT → RUN' in (OUT/'B.log').read_text()
    report['recovery_keeps_registered_session'] = True
    send_ai = False
    prev_seq = commands[-1].command_seq
    process.send_signal(signal.SIGINT)
    process.wait(timeout=8)
    spin(1.)
    assert receiver and receiver[-1].watchdog_active and max(abs(v) for v in receiver[-1].commanded_velocities) < 1e-8
    report['b_stop_receiver_watchdog'] = True
    # Snapshot after the old B has exited and its late DDS packets have drained.
    # Otherwise an old in-flight packet could satisfy the restart assertion.
    prev_seq=commands[-1].command_seq
    process = launch()
    wait(lambda: commands[-1].command_seq > prev_seq and states[-1].last_command_result == 2)
    spin(.3)
    report['b_restart_sequence_continuity'] = True
    report['motion_timing_error_events'] = sum(len(m['active_errors']) for m in report['common_moves'])
    report['clean_motion'] = report['motion_timing_error_events']==0
    report['passed'] = True
finally:
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGCONT)
        process.send_signal(signal.SIGINT)
        process.wait(timeout=8)
    log.close()
    (OUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
