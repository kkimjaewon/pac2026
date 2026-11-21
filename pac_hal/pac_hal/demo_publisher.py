"""Synthetic B input for C-only Gazebo integration, never an RT benchmark."""
import argparse
import copy
import json
import math
from pathlib import Path
import time

from intelion_interfaces.msg import HalCommand, HalError, HalJointStatus, HalState
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-id', type=int, default=42)
    parser.add_argument('--duration', type=float, default=6.)
    parser.add_argument('--amplitude', type=float, default=.002)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if args.session_id <= 0 or args.duration < 4 or not 0 < args.amplitude <= .005:
        parser.error('session must be positive, duration >=4s and amplitude in (0, .005]m')
    rclpy.init()
    node = rclpy.create_node('hal_synthetic_b')
    latest = {'state': None, 'receiver': None}
    observations, errors = [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def state_callback(msg):
        latest['state'] = msg
        if msg.pose_valid and not msg.stale:
            observations.append(msg.tcp_pose.position.z)

    def receiver_callback(msg):
        if msg.session_id == args.session_id:
            latest['receiver'] = msg

    node.create_subscription(HalState, '/hal/state', state_callback, qos)
    node.create_subscription(HalJointStatus, '/hal_position_controller/status',
                             receiver_callback, qos)
    node.create_subscription(HalError, '/hal/error',
                             lambda msg: errors.append((msg.code, msg.detail)) if msg.active
                             else None, QoSProfile(depth=32))
    pub = node.create_publisher(HalCommand, '/hal/final_command', qos)
    try:
        end = time.monotonic()+30
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005)
            s, receiver = latest['state'], latest['receiver']
            if (s is not None and receiver is not None and s.pose_valid and not s.stale
                    and s.hal_state == HalState.HAL_READY
                    and s.active_session_id == args.session_id and receiver.active):
                break
        else:
            raise RuntimeError('matching session has no fresh READY state; start HAL simulation')
        if node.count_publishers('/hal/final_command') != 1:
            raise RuntimeError('another B publisher exists; stop it before using synthetic B')
        initial = copy.deepcopy(s.tcp_pose)
        seq = max(s.last_command_seq, receiver.last_applied_command_seq)+1
        first_seq = seq
        observations.clear()
        start, sent = time.monotonic(), 0
        # Nominal 1ms offers the real B schema; Python/Linux scheduling is not certified RT.
        while time.monotonic()-start < args.duration:
            s = latest['state']
            if (s.active_session_id != args.session_id or not s.pose_valid or s.stale
                    or time.monotonic_ns()-s.received_mono_ns >= 30_000_000):
                if s.active_session_id != args.session_id:
                    raise RuntimeError('HAL session changed')
                if time.monotonic_ns()-s.received_mono_ns >= 100_000_000:
                    raise RuntimeError('HAL measurements stopped for 100ms')
                rclpy.spin_once(node, timeout_sec=.001)
                continue
            msg = HalCommand()
            msg.header.frame_id, msg.header.stamp = 'map', s.header.stamp
            msg.session_id, msg.command_seq = args.session_id, seq
            seq += 1
            msg.target_pose = copy.deepcopy(initial)
            t = (time.monotonic()-start)/args.duration
            msg.target_pose.position.z += args.amplitude*(1-math.cos(2*math.pi*t))/2
            msg.created_mono_ns = time.monotonic_ns()
            msg.expires_mono_ns = msg.created_mono_ns+5_000_000
            pub.publish(msg)
            sent += 1
            deadline = time.monotonic()+.001
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0)
        # No B stop command: the C receiver must brake independently when input ends.
        end = time.monotonic()+1
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.002)
        receiver = latest['receiver']
        peak = max(observations, default=initial.position.z)-initial.position.z
        result = {
            'input': 'synthetic B; simulation only', 'session_id': args.session_id,
            'sent_messages': sent, 'receiver_applied_seq': receiver.last_applied_command_seq,
            'measured_peak_displacement_m': peak,
            'watchdog_active_after_publish_stop': receiver.watchdog_active,
            'receiver_stopped': all(abs(v) < 1e-6 for v in receiver.commanded_velocities),
            'active_error_events': sorted(set(errors)),
            'passed': (receiver.last_applied_command_seq >= first_seq and
                       peak > args.amplitude*.5 and receiver.watchdog_active and
                       all(abs(v) < 1e-6 for v in receiver.commanded_velocities)),
        }
        rendered = json.dumps(result, indent=2, ensure_ascii=False)
        print(rendered, flush=True)
        if args.output:
            Path(args.output).write_text(rendered+'\n')
        if not result['passed']:
            raise RuntimeError('synthetic B movement/watchdog verification failed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
