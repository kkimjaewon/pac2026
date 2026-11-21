"""Synthetic invalid B commands for the Gazebo C integration check."""
import argparse
import copy
import json
from pathlib import Path
import time

from intelion_interfaces.msg import HalCommand, HalError, HalState
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-id', type=int, default=42)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('hal_synthetic_b_fault_probe')
    states, errors = [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(HalState, '/hal/state', states.append, qos)
    node.create_subscription(HalError, '/hal/error', errors.append, QoSProfile(depth=32))
    pub = node.create_publisher(HalCommand, '/hal/final_command', qos)

    def wait_for(predicate, timeout=5.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.001)
            if predicate():
                return
        raise RuntimeError('timed out waiting for HAL result')

    def message(seq):
        s = states[-1]
        msg = HalCommand()
        msg.header.frame_id, msg.header.stamp = 'map', s.header.stamp
        msg.session_id, msg.command_seq = args.session_id, seq
        msg.target_pose = copy.deepcopy(s.tcp_pose)
        msg.created_mono_ns = time.monotonic_ns()
        msg.expires_mono_ns = msg.created_mono_ns+5_000_000
        return msg

    try:
        wait_for(lambda: bool(states) and states[-1].pose_valid and not states[-1].stale
                 and states[-1].hal_state == HalState.HAL_READY
                 and states[-1].active_session_id == args.session_id)
        if node.count_publishers('/hal/final_command') != 1:
            raise RuntimeError('another B publisher exists')
        seq = states[-1].last_command_seq+1
        baseline = message(seq)
        pub.publish(baseline)
        wait_for(lambda: states[-1].last_command_seq == seq)
        if states[-1].last_command_result != HalState.CMD_SENT:
            raise RuntimeError('baseline hold command rejected; retry with fresh state')
        results = []
        cases = [('expired', 3, seq+1), ('replay', 1, seq),
                 ('workspace', 4, seq+2), ('unreachable', 5, seq+3)]
        for label, expected, command_seq in cases:
            wait_for(lambda: states[-1].pose_valid and not states[-1].stale)
            msg = message(command_seq)
            if label == 'expired':
                msg.created_mono_ns -= 20_000_000
                msg.expires_mono_ns -= 20_000_000
            elif label == 'workspace':
                msg.target_pose.position.x = 10.
            elif label == 'unreachable':
                msg.target_pose.position.x = .99
                msg.target_pose.position.y = .99
                msg.target_pose.position.z = 1.49
            before = len(errors)
            pub.publish(msg)
            wait_for(lambda: any(e.active and e.code == expected and
                                 e.command_seq == command_seq for e in errors[before:]))
            wait_for(lambda: states[-1].last_command_seq == command_seq and
                     states[-1].last_command_result == HalState.CMD_REJECTED)
            detail = next(e.detail for e in errors[before:] if e.active and e.code == expected)
            results.append({'case': label, 'error_code': expected, 'rejected': True,
                            'detail': detail})
        rendered = json.dumps({'input': 'synthetic invalid B', 'checks': results,
                               'passed': True}, indent=2)
        print(rendered, flush=True)
        if args.output:
            Path(args.output).write_text(rendered+'\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()
