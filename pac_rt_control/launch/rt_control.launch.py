"""B 계층 실행 (rt_control_node + 선택: 가짜 C, 가짜 A)

  ros2 launch rt_control rt_control.launch.py
  ros2 launch rt_control rt_control.launch.py fake_a:=true fake_c:=true run_tag:=test
  ros2 launch rt_control rt_control.launch.py run_id:=exp01   # ~/pac_logs/exp01/ 에 기록
  ros2 launch rt_control rt_control.launch.py planned_position:="[0.5, 0.0, 0.70]"
  ros2 launch rt_control rt_control.launch.py config:=/다른/경로.yaml

켜는 순서: 가짜 C -> 가짜 A -> B
  B는 INIT에서 HalState를 기다렸다가 실제 로봇 위치에서 시작하므로, 가짜 C가 먼저 떠 있어야 한다.
"""
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 앞 단계 노드가 뜰 시간을 주는 지연 [s]
FAKE_A_DELAY_S = 1.0
B_DELAY_S = 2.0


def as_bool(text):
    return str(text).strip().lower() in ('true', '1', 'yes', 'on')


def parse_position(text, name='planned_position'):
    """'[0.5, 0.0, 0.70]' -> [0.5, 0.0, 0.7] (실수 3개가 아니면 ValueError)"""
    value = yaml.safe_load(text)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{name}은 [x, y, z] 3개여야 함: {text}')
    return [float(v) for v in value]


def launch_setup(context):
    config = LaunchConfiguration('config').perform(context)
    run_id = LaunchConfiguration('run_id').perform(context)
    run_tag = LaunchConfiguration('run_tag').perform(context)
    planned = LaunchConfiguration('planned_position').perform(context)
    session = LaunchConfiguration('session_id').perform(context)
    offset = LaunchConfiguration('target_offset').perform(context)
    fake_a = as_bool(LaunchConfiguration('fake_a').perform(context))
    fake_c = as_bool(LaunchConfiguration('fake_c').perform(context))

    actions = []
    if not os.path.isfile(config):
        raise RuntimeError(f'설정 파일이 없음: {config}')
    actions.append(LogInfo(msg=f'[rt_control.launch] 설정 파일: {config}'))

    # yaml 위에 덮어쓸 값 (launch 인자를 준 것만)
    overrides = {}
    if fake_c:
        # 가짜 C는 처음 받은 세션을 따르므로 미리 등록된 세션 상태를 보고하지 않음
        overrides['require_registered_state'] = False
    if session:
        if int(session) < 0:
            raise ValueError('session_id는 0(B가 발급) 또는 C에 등록된 양수여야 함')
        overrides['session_id'] = int(session)
    if run_id:
        overrides['run_id'] = run_id
        actions.append(LogInfo(msg=f'[rt_control.launch] run_id 지정: 기록 폴더 <log_root>/{run_id}/'))
    if run_tag:
        overrides['run_tag'] = run_tag
    if planned:
        overrides['planned_position'] = parse_position(planned)
        actions.append(LogInfo(msg=f'[rt_control.launch] planned_position 덮어씀: {overrides["planned_position"]}'))
    if offset:
        overrides['target_offset'] = parse_position(offset, 'target_offset')
        actions.append(LogInfo(msg=f'[rt_control.launch] target_offset 덮어씀: {overrides["target_offset"]}'))

    delay_b = 0.0
    if fake_c:
        actions.append(Node(package='rt_control', executable='fake_c_node.py', name='fake_c_node', output='screen'))
        delay_b = B_DELAY_S
    if fake_a:
        try:
            get_package_share_directory('ai_mock')
            fake_a_node = Node(package='ai_mock', executable='fake_ai_node', name='fake_ai_node', output='screen')
            actions.append(TimerAction(period=FAKE_A_DELAY_S if fake_c else 0.0, actions=[fake_a_node]))
            delay_b = B_DELAY_S
        except PackageNotFoundError:
            actions.append(LogInfo(msg='[rt_control.launch] 경고: ai_mock 패키지가 없어 가짜 A를 켜지 않음 '
                                       '(~/pac2026_ws/install/setup.bash 를 source 했는지 확인)'))

    b_node = Node(package='rt_control', executable='rt_control_node', name='rt_control_node', output='screen',
                  parameters=[config, overrides])
    actions.append(TimerAction(period=delay_b, actions=[b_node]) if delay_b > 0 else b_node)
    return actions


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('rt_control'), 'config', 'rt_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config, description='rt_control_node 파라미터 yaml'),
        DeclareLaunchArgument('session_id', default_value='',
                              description='세션 번호 덮어쓰기. 0 = B가 발급, 양수 = C 등록 세션 (비우면 yaml 값)'),
        DeclareLaunchArgument('run_id', default_value='',
                              description='기록 폴더 이름 고정 (<log_root>/<run_id>/, 이미 rt.csv가 있으면 B가 멈춤). '
                                          '비우면 시각+run_tag'),
        DeclareLaunchArgument('run_tag', default_value='', description='기록 폴더 이름 뒤에 붙일 태그 (run_id가 비었을 때만)'),
        DeclareLaunchArgument('planned_position', default_value='',
                              description='계획 위치 덮어쓰기, 예: "[0.5, 0.0, 0.70]" (비우면 yaml 값)'),
        DeclareLaunchArgument('target_offset', default_value='',
                              description='AI 위치에 더할 작업 높이 덮어쓰기, 예: "[0.0, 0.0, 0.10]" (비우면 yaml 값)'),
        DeclareLaunchArgument('fake_a', default_value='false', description='가짜 A(ai_mock/fake_ai_node)도 켜기'),
        DeclareLaunchArgument('fake_c', default_value='false', description='가짜 C(fake_c_node.py)도 켜기'),
        OpaqueFunction(function=launch_setup),
    ])
