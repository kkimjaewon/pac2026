#!/usr/bin/env python3
"""가짜 C (통신·HAL 계층 흉내)

B가 보내는 /hal/final_command를 받아 검사하고, 간단한 1차 지연 로봇 흉내로
/hal/state를 발행한다. 오류는 /hal/error로 켜질 때·꺼질 때 한 번씩 알린다.

실행: ros2 run rt_control fake_c_node.py
오류 주입(실행 중 변경 가능):
  ros2 param set /fake_c_node fault_mode ik_fail      # 명령 거부 + E_IK_FAILED 켜기, 로봇 제자리
  ros2 param set /fake_c_node fault_mode state_stop   # /hal/state 발행 중단
  ros2 param set /fake_c_node fault_mode disconnect   # E_DISCONNECTED 켜기 + 상태 발행 중단
  ros2 param set /fake_c_node fault_mode none         # 켜진 오류를 모두 끔
"""
import collections
import gc
import math
import sys
import threading
import time

import rclpy
from intelion_interfaces.msg import HalCommand, HalError, HalState
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

QOS_CMD = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
QOS_STATE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
QOS_ERROR = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=32)

FAULTS = ('none', 'ik_fail', 'state_stop', 'disconnect')
# 오류 주입 모드별로 켜는 오류 코드
FAULT_ERRORS = {
    'none': (),
    'ik_fail': (HalError.E_IK_FAILED,),
    'state_stop': (),
    'disconnect': (HalError.E_DISCONNECTED,),
}
# 이 시간 안에 명령을 받아들였으면 ACTIVE, 아니면 READY
ACTIVE_WINDOW_NS = 50_000_000
ROBOT_PERIOD_NS = 1_000_000  # 로봇 흉내 1ms
SOURCE = 'fake_c'

HAL_STATE_NAMES = {0: 'INITIALIZING', 1: 'CONNECTING', 2: 'READY', 3: 'ACTIVE', 4: 'FAULT', 5: 'RECONNECTING'}


class FakeC(Node):
    def __init__(self):
        super().__init__('fake_c_node')
        self.declare_parameter('robot_tau', 0.02)                 # 로봇 1차 지연 시상수 [s]
        self.declare_parameter('robot_start', [0.45, 0.0, 0.75])  # 로봇 시작 TCP 위치 (map) [m]
        self.declare_parameter('state_rate_hz', 250.0)
        self.declare_parameter('fault_mode', 'none')              # none | ik_fail | state_stop | disconnect
        self.add_on_set_parameters_callback(self._check_params)

        self.tau = float(self.get_parameter('robot_tau').value)
        start = [float(v) for v in self.get_parameter('robot_start').value]
        rate = float(self.get_parameter('state_rate_hz').value)
        if len(start) != 3 or not self.tau > 0.0 or not rate > 0.0:
            raise ValueError('robot_start는 [x, y, z], robot_tau와 state_rate_hz는 0보다 커야 함')

        # 로봇 스레드와 같이 쓰는 값은 lock으로 보호 (ROS 콜백끼리는 한 스레드라 겹치지 않음)
        self.lock = threading.Lock()
        self.pos = list(start)      # 지금 로봇 TCP 위치
        self.target = list(start)   # 마지막으로 받아들인 목표
        self.quat = [1.0, 0.0, 0.0, 0.0]

        self.session = 0            # 지금 따르는 B 세션
        self.last_seq = 0           # 이 세션에서 마지막으로 처리한 command_seq (받아들임·거부 모두)
        self.last_result = HalState.CMD_ACCEPTED
        self.last_accept_ns = 0
        self.state_seq = 0
        self.fault = 'none'
        self.active_errors = set()

        self.received = 0
        self.accepted = 0
        self.discards = collections.Counter()  # 이유별 폐기 수 (누적)
        self.prev_received = 0
        self.prev_accepted = 0
        self.prev_discards = collections.Counter()

        self.cmd_sub = self.create_subscription(HalCommand, '/hal/final_command', self.on_command, QOS_CMD)
        self.state_pub = self.create_publisher(HalState, '/hal/state', QOS_STATE)
        self.error_pub = self.create_publisher(HalError, '/hal/error', QOS_ERROR)
        self.create_timer(1.0 / rate, self.publish_state)
        self.create_timer(0.02, self.check_fault_mode)
        self.create_timer(1.0, self.summary)

        self.running = True
        self.robot_thread = threading.Thread(target=self.robot_main, daemon=True)
        self.robot_thread.start()
        self.get_logger().info(
            f'가짜 C 시작: 로봇 시작 위치 {fmt(start)}, tau {self.tau:.3f}s, 상태 {rate:.0f}Hz')

    def _check_params(self, params):
        for p in params:
            if p.name == 'fault_mode' and p.value not in FAULTS:
                return SetParametersResult(successful=False, reason=f'fault_mode는 {FAULTS} 중 하나')
            if p.name in ('robot_tau', 'robot_start', 'state_rate_hz'):
                return SetParametersResult(successful=False, reason=f'{p.name}는 시작할 때만 정할 수 있음')
        return SetParametersResult(successful=True)

    # ---------------- 명령 수신 ----------------
    def on_command(self, msg):
        now = time.monotonic_ns()  # 가장 먼저 받은 시각부터 찍음
        self.received += 1

        # 현재 세션만 받음. B가 새 세션을 시작하면(session_id는 시작 시각이라 더 큼) 갈아탐
        if msg.session_id != self.session:
            if msg.session_id < self.session:
                self.discards['old_session'] += 1
                return
            self.get_logger().info(f'새 B 세션으로 갈아탐: {self.session} -> {msg.session_id}')
            self.session = msg.session_id
            self.last_seq = 0
        if msg.command_seq <= self.last_seq:
            self.discards['old_seq'] += 1
            return

        p = msg.target_pose.position
        if now > msg.expires_mono_ns:
            self.reject(msg, 'expired', HalError.E_EXPIRED, f'{(now - msg.expires_mono_ns) / 1e6:.2f} ms 지남')
            return
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.reject(msg, 'format', HalError.E_FORMAT, '위치에 NaN·무한대')
            return
        if self.fault == 'disconnect':
            self.reject(msg, 'disconnect', None, '')
            return
        if self.fault == 'ik_fail':
            self.reject(msg, 'ik_fail', None, '')  # E_IK_FAILED는 fault_mode가 켜 둠
            return

        q = msg.target_pose.orientation
        with self.lock:
            self.target = [p.x, p.y, p.z]
            self.quat = [q.x, q.y, q.z, q.w]
        self.last_seq = msg.command_seq
        self.last_result = HalState.CMD_ACCEPTED
        self.last_accept_ns = now
        self.accepted += 1
        # 명령 하나에 붙는 오류는 정상 명령을 받으면 끔
        for code in (HalError.E_EXPIRED, HalError.E_FORMAT):
            self.set_error(code, False, msg.command_seq, '정상 명령 수신')

    def reject(self, msg, reason, code, detail):
        self.discards[reason] += 1
        self.last_seq = msg.command_seq
        self.last_result = HalState.CMD_REJECTED
        if code is not None:
            self.set_error(code, True, msg.command_seq, detail)

    # ---------------- 오류 ----------------
    def set_error(self, code, active, command_seq, detail):
        """켜짐/꺼짐이 바뀔 때만 /hal/error를 한 번 발행"""
        if active == (code in self.active_errors):
            return
        if active:
            self.active_errors.add(code)
        else:
            self.active_errors.discard(code)
        e = HalError()
        e.header.stamp = self.get_clock().now().to_msg()
        e.header.frame_id = 'map'
        e.code = int(code)
        e.active = bool(active)
        e.event_mono_ns = time.monotonic_ns()
        e.session_id = self.session
        e.command_seq = int(command_seq)
        e.source = SOURCE
        e.detail = detail
        self.error_pub.publish(e)
        self.get_logger().warn(f'오류 {code} {"켜짐" if active else "꺼짐"} ({detail})')

    def check_fault_mode(self):
        new = str(self.get_parameter('fault_mode').value)
        if new == self.fault:
            return
        self.get_logger().warn(f'fault_mode: {self.fault} -> {new}')
        for code in FAULT_ERRORS[self.fault]:
            self.set_error(code, False, self.last_seq, f'fault_mode {self.fault} 끝')
        if new == 'none':
            for code in sorted(self.active_errors):
                self.set_error(code, False, self.last_seq, 'fault_mode none')
        self.fault = new
        for code in FAULT_ERRORS[new]:
            self.set_error(code, True, self.last_seq, f'fault_mode {new}')

    # ---------------- 로봇 흉내 ----------------
    def robot_main(self):
        """1ms마다 로봇 위치가 목표를 1차 지연(시상수 tau)으로 따라감"""
        next_ns = time.monotonic_ns()
        prev_ns = next_ns
        while self.running:
            next_ns += ROBOT_PERIOD_NS
            wait = next_ns - time.monotonic_ns()
            if wait > 0:
                time.sleep(wait / 1e9)
            now = time.monotonic_ns()
            if now - next_ns > 10 * ROBOT_PERIOD_NS:  # 많이 늦었으면 몰아서 돌지 않고 지금부터 다시
                next_ns = now
            dt = (now - prev_ns) / 1e9
            prev_ns = now
            alpha = 1.0 - math.exp(-dt / self.tau)
            with self.lock:
                for i in range(3):
                    self.pos[i] += alpha * (self.target[i] - self.pos[i])

    # ---------------- 상태 발행 ----------------
    def publish_state(self):
        if self.fault in ('state_stop', 'disconnect'):
            return
        now = time.monotonic_ns()
        with self.lock:
            pos = list(self.pos)
            quat = list(self.quat)
        s = HalState()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'map'
        receiving = self.last_accept_ns > 0 and now - self.last_accept_ns < ACTIVE_WINDOW_NS
        s.hal_state = HalState.HAL_ACTIVE if receiving else HalState.HAL_READY
        self.state_seq += 1
        s.state_seq = self.state_seq
        s.received_mono_ns = now
        s.stale = False
        s.tcp_pose.position.x, s.tcp_pose.position.y, s.tcp_pose.position.z = pos
        s.tcp_pose.orientation.x, s.tcp_pose.orientation.y, s.tcp_pose.orientation.z, s.tcp_pose.orientation.w = quat
        s.pose_valid = True
        s.twist_valid = False
        s.active_session_id = self.session
        s.last_command_seq = self.last_seq
        s.last_command_result = self.last_result
        s.active_errors = sorted(int(c) for c in self.active_errors)
        self.state_pub.publish(s)

    # ---------------- 1초 요약 ----------------
    def summary(self):
        with self.lock:
            pos = list(self.pos)
        d = self.discards - self.prev_discards
        dropped = ' '.join(f'{k} {v}' for k, v in sorted(d.items())) or '없음'
        total = ' '.join(f'{k} {v}' for k, v in sorted(self.discards.items())) or '없음'
        self.get_logger().info(
            f'받음 {self.received - self.prev_received} | 받아들임 {self.accepted - self.prev_accepted} | '
            f'폐기 {dropped} (누적 {total}) | 로봇 {fmt(pos)} | 세션 {self.session} | '
            f'처리 seq {self.last_seq} | fault {self.fault} | 켜진 오류 {sorted(self.active_errors)}')
        self.prev_received = self.received
        self.prev_accepted = self.accepted
        self.prev_discards = collections.Counter(self.discards)

    def stop(self):
        self.running = False
        self.robot_thread.join(timeout=1.0)


def fmt(v):
    return '[' + ', '.join(f'{x:.4f}' for x in v) + ']'


def main():
    # 파이썬 지연 줄이기: 스레드 사이 GIL 넘기는 간격 5ms -> 0.5ms (명령 콜백이 로봇 스레드를 오래 기다리지 않게),
    # 시작할 때 만든 객체는 GC 검사 대상에서 빼서 GC 멈춤을 짧게
    sys.setswitchinterval(0.0005)
    rclpy.init()
    node = FakeC()
    gc.freeze()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
