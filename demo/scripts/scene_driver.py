#!/usr/bin/env python3
"""시연 장면 1~6을 노드 하나로 실행 (장면마다 ros2 명령 프로세스를 새로 띄우지 않음).

demo_sequence.sh와 같은 장면·시간이다. 철판 이동은 Gazebo set_pose, A 고장 주입은 A의
set_parameters 서비스를 이 노드가 직접 부른다. 장면마다 ros2 param set/service call을
새로 띄우면 그 순간 Gazebo·컨트롤러 출력이 100ms 넘게 멈춘 회차가 있어(C code8/9 → B FAULT)
리허설·녹화는 이 스크립트를 쓴다.

사용: python3 scene_driver.py <timeline.json> <label>
  timeline.json: 장면별 시작 시각(mono_ns, wall_ns)과 성공 여부
  같은 폴더에 demo_start, demo_end(wall 초)도 쓴다 → gap_check.py가 그대로 읽는다.
A·B·C·시뮬과 같은 ROS_DOMAIN_ID에서 실행한다.
"""
import json, sys, time
import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

out, label = sys.argv[1], sys.argv[2]
rclpy.init()
n = rclpy.create_node('scene_driver')
pose = n.create_client(SetEntityPose, '/world/franka_world/set_pose')
params = n.create_client(SetParameters, '/ai_inference_node/set_parameters')
timeline = []


def wait_future(f, timeout=10.0):
    end = time.monotonic() + timeout
    while not f.done() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.01)
    return f.result() if f.done() else None


def mark(name, ok=True):
    timeline.append({'label': label, 'scene': name, 'mono_ns': time.monotonic_ns(), 'wall_ns': time.time_ns(), 'ok': ok})
    print(f'[{label}] {name} ok={ok}', flush=True)


def move(x):
    r = SetEntityPose.Request()
    r.entity.name, r.entity.type = 'steel_plate', Entity.MODEL
    r.pose.position.x, r.pose.position.z, r.pose.orientation.w = float(x), 0.705, 1.0
    res = wait_future(pose.call_async(r))
    return bool(res and res.success)


def set_param(name, value):
    p = Parameter(name=name)
    if isinstance(value, int):
        p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    else:
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    res = wait_future(params.call_async(SetParameters.Request(parameters=[p])))
    return bool(res and res.results and res.results[0].successful)


def hold(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)


ready = pose.wait_for_service(timeout_sec=30) and params.wait_for_service(timeout_sec=30)
mark('services ready', ready)
if not ready:
    raise SystemExit(2)
mark('fault_delay_ms=250', set_param('fault_delay_ms', 250))
mark('scene1 normal'); hold(5)
mark('scene2 plate x0.545', move(0.545)); hold(10)
mark('scene3 AI delay 250ms', set_param('fault_mode', 'delay')); hold(10)
mark('scene3 recover', set_param('fault_mode', 'none')); hold(6)
mark('scene4 AI stop', set_param('fault_mode', 'stop')); hold(10)
mark('scene4 recover', set_param('fault_mode', 'none')); hold(6)
mark('scene5 AI bad value 9.9m', set_param('fault_mode', 'bad_value')); hold(10)
mark('scene5 recover', set_param('fault_mode', 'none')); hold(6)
mark('scene6 plate x0.50', move(0.50)); hold(12)
mark('end')
json.dump(timeline, open(out, 'w'), indent=1)
import os
d = os.path.dirname(os.path.abspath(out))
open(os.path.join(d, 'demo_start'), 'w').write(f"{timeline[0]['wall_ns'] / 1e9:.9f}\n")
open(os.path.join(d, 'demo_end'), 'w').write(f"{timeline[-1]['wall_ns'] / 1e9:.9f}\n")
