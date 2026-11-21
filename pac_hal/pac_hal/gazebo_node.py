"""Gazebo HAL: measured FK, bounded IK, canonical/device mapping, receiver ACK."""
from collections import OrderedDict
import math
import time

from intelion_interfaces.msg import HalError, HalJointCommand, HalJointStatus, HalState
import PyKDL as kdl
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .config_node import HalConfigNode
from .kinematics import Kinematics


def pose_frame(pose):
    p, q = pose.position, pose.orientation
    return kdl.Frame(kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w), kdl.Vector(p.x, p.y, p.z))


def put_pose(frame, pose):
    pose.position.x, pose.position.y, pose.position.z = [frame.p[i] for i in range(3)]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        frame.M.GetQuaternion())


class GazeboHalNode(HalConfigNode):
    """Single non-RT executor; B never waits for this node."""

    def __init__(self):
        super().__init__()
        self.model = None
        self.samples = None
        self.sample_seq = 0
        self.sample_stamp = None
        self.received_ns = 0
        self.receiver = None
        self.receiver_received_ns = 0
        self.last_q = None
        self.last_v = None
        self.last_created = None
        self.last_msg = None
        self.records = OrderedDict()
        self.last_logged_ack = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        description_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.description_sub = self.create_subscription(
            String, '/' + self.config['robot']['robot_description_parameter'],
            self.on_description, description_qos)
        self.joints_sub = self.create_subscription(
            JointState, self.config['transport']['joint_state_topic'], self.on_joints, qos)
        self.receiver_sub = self.create_subscription(
            HalJointStatus, self.config['transport']['receiver_status_topic'],
            self.on_receiver, qos)
        self.joint_pub = self.create_publisher(
            HalJointCommand, self.config['transport']['joint_command_topic'], qos)
        self.get_logger().info('Gazebo execution enabled; waiting for matching URDF, '
                               'fresh measurements and receiver session status.')

    def on_description(self, msg):
        if self.model is not None:
            return  # Model changes require a new HAL/controller session.
        try:
            self.model = Kinematics(msg.data, self.config)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(str(exc))

    def on_joints(self, msg):
        if self.model is None:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.sample_stamp is not None and stamp <= self.sample_stamp:
            return  # Paused/replayed samples do not refresh freshness; sim reset needs restart.
        index = {name: i for i, name in enumerate(msg.name)}
        if any(name not in index or index[name] >= len(msg.position) for name in self.model.names):
            return
        q = [msg.position[index[name]] for name in self.model.names]
        if any(not math.isfinite(v) for v in q):
            return
        # joint_states already uses canonical URDF radians; do NOT apply hardware sign twice.
        dq = None
        if all(index[name] < len(msg.velocity) for name in self.model.names):
            candidate = [msg.velocity[index[name]] for name in self.model.names]
            if all(math.isfinite(v) for v in candidate):
                dq = candidate
        self.samples = (q, dq, msg.header)
        self.sample_stamp = stamp
        self.received_ns = time.monotonic_ns()
        self.sample_seq += 1
        self.on_state()

    def on_receiver(self, msg):
        if (msg.session_id != self.config['runtime']['active_session_id'] or
                msg.joint_count != len(self.config['joints'])):
            return
        if self.receiver is not None and msg.sampled_mono_ns <= self.receiver.sampled_mono_ns:
            return
        self.receiver = msg
        self.receiver_received_ns = time.monotonic_ns()
        key = (msg.session_id, msg.last_applied_command_seq)
        if msg.applied_mono_ns > 0 and key != self.last_logged_ack and key in self.records:
            record = self.records[key]
            if self.writer:
                self.writer.writerow(record[:6] + ['', self.receiver_received_ns, 'applied', 0])
            self.last_logged_ack = key
        if self.last_msg is not None:
            self.error_transition(HalError.E_DEVICE, msg.rejected, self.last_msg,
                                  'receiver rejected a joint command or hit a hard boundary')

    def healthy(self, now):
        state_ok = (self.samples is not None and
                    now-self.received_ns < self.config['timing']['state_stale_ns'])
        controller_ok = (self.receiver is not None and self.receiver.active and
                         now-self.receiver_received_ns < self.config['timing']['state_lost_ns'])
        return state_ok and controller_ok

    def on_state(self):
        now = time.monotonic_ns()
        msg = HalState()
        msg.header.frame_id = self.config['frames']['reference']
        msg.active_session_id = self.config['runtime']['active_session_id']
        msg.state_seq, msg.received_mono_ns = self.sample_seq, self.received_ns
        msg.stale = (self.samples is None or
                     now-self.received_ns >= self.config['timing']['state_stale_ns'])
        msg.last_command_seq = self.last_command_seq
        msg.last_command_result = self.last_command_result
        msg.hal_state = HalState.HAL_CONNECTING
        if self.samples is not None:
            q, dq, header = self.samples
            msg.header.stamp = header.stamp
            try:
                put_pose(self.model.fk(q), msg.tcp_pose)
                msg.pose_valid = not msg.stale
                if dq is not None and not msg.stale:
                    linear, angular = self.model.twist(q, dq)
                    msg.tcp_twist.linear.x, msg.tcp_twist.linear.y, msg.tcp_twist.linear.z = (
                        [linear[i] for i in range(3)])
                    msg.tcp_twist.angular.x, msg.tcp_twist.angular.y, msg.tcp_twist.angular.z = (
                        [angular[i] for i in range(3)])
                    msg.twist_valid = True
            except (ValueError, RuntimeError):
                msg.pose_valid = False
        if self.healthy(now) and msg.pose_valid:
            moving = self.receiver is not None and any(
                abs(v) > 1e-5 for v in self.receiver.commanded_velocities)
            msg.hal_state = HalState.HAL_ACTIVE if moving else HalState.HAL_READY
        if self.last_msg is not None:
            self.error_transition(HalError.E_STATE_STALE,
                                  self.samples is None or now-self.received_ns >=
                                  self.config['timing']['state_lost_ns'],
                                  self.last_msg, 'robot measurements stopped')
            self.error_transition(HalError.E_DISCONNECTED,
                                  self.receiver is None or now-self.receiver_received_ns >=
                                  self.config['timing']['state_lost_ns'],
                                  self.last_msg, 'receiver status missing or stopped')
        if msg.stale and self.samples is not None:
            msg.hal_state = HalState.HAL_FAULT
        msg.active_errors = sorted(self.active_errors)
        self.state_pub.publish(msg)

    def on_command(self, msg):
        t1 = time.monotonic_ns()
        self.last_msg = msg
        p, q = msg.target_pose.position, msg.target_pose.orientation
        data = {'session_id': msg.session_id, 'command_seq': msg.command_seq,
                'created_mono_ns': msg.created_mono_ns, 'expires_mono_ns': msg.expires_mono_ns,
                'mode': msg.mode, 'frame_id': msg.header.frame_id,
                'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w]}
        code, detail = self.validator.check(data, t1)
        t2 = time.monotonic_ns()
        t3 = t4 = ''
        result = 'rejected'
        if code == 0 and not self.healthy(t2):
            code, detail = HalError.E_STATE_STALE, 'fresh measured state and receiver are required'
        if code == 0:
            seed = self.last_q if self.last_q is not None else self.samples[0]
            try:
                solution, _ = self.model.solve(pose_frame(msg.target_pose), seed)
                t3 = time.monotonic_ns()
                dt = (self.config['timing']['command_period_ns'] * 1e-9
                      if self.last_created is None
                      else (msg.created_mono_ns-self.last_created)*1e-9)
                if dt <= 0:
                    raise ValueError('command generation time did not increase')
                previous_q = self.last_q if self.last_q is not None else self.samples[0]
                previous_v = self.last_v if self.last_v is not None else [0.]*len(solution)
                velocity = [(v-old)/dt for v, old in zip(solution, previous_q)]
                acceleration = [(v-old)/dt for v, old in zip(velocity, previous_v)]
                for i, joint in enumerate(self.config['joints']):
                    if (abs(velocity[i]) > joint['max_velocity_rad_s']+1e-6 or
                            abs(acceleration[i]) > joint['max_acceleration_rad_s2']+1e-5):
                        raise ValueError('joint velocity/acceleration limit exceeded; '
                                         'B must slow trajectory')
                output = HalJointCommand()
                output.session_id, output.command_seq = msg.session_id, msg.command_seq
                output.created_mono_ns = msg.created_mono_ns
                output.expires_mono_ns = msg.expires_mono_ns
                output.joint_count = len(solution)
                mapped = [0.]*7
                for i, joint in enumerate(self.config['joints']):
                    mapped[joint['device_index']] = (
                        joint['direction']*solution[i]+joint['zero_offset_rad'])
                output.positions = mapped
                t4 = time.monotonic_ns()
                if t4 >= msg.expires_mono_ns:
                    code, detail = HalError.E_EXPIRED, 'expired before robot transport'
                elif not self.healthy(t4):
                    code, detail = HalError.E_STATE_STALE, 'state went stale before transport'
                elif self.joint_pub.get_subscription_count() == 0:
                    code, detail = HalError.E_SEND_FAILED, 'no watchdog controller subscription'
                else:
                    self.joint_pub.publish(output)
                    self.last_q, self.last_v = solution, velocity
                    self.last_created = msg.created_mono_ns
                    result = 'sent'
            except (ValueError, RuntimeError) as exc:
                detail = str(exc)
                code = (HalError.E_OUT_OF_RANGE if 'limit exceeded' in detail
                        else HalError.E_IK_FAILED)
        self.last_command_seq = msg.command_seq
        self.last_command_result = HalState.CMD_SENT if result == 'sent' else HalState.CMD_REJECTED
        transient = {HalError.E_FORMAT, HalError.E_UNSUPPORTED, HalError.E_EXPIRED,
                     HalError.E_OUT_OF_RANGE, HalError.E_IK_FAILED, HalError.E_TF,
                     HalError.E_SEND_FAILED}
        for old in tuple(self.active_errors & transient):
            if old != code:
                self.error_transition(old, False, msg, 'command condition cleared')
        if code:
            self.error_transition(code, True, msg, detail)
        row = [msg.session_id, msg.command_seq, t1, t2, t3, t4, '', '', result, code]
        if self.writer:
            self.writer.writerow(row)
        if result == 'sent':
            self.records[(msg.session_id, msg.command_seq)] = row
            while len(self.records) > 256:
                self.records.popitem(last=False)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GazeboHalNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
