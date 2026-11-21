import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('ai_inference'), 'config', 'sensor_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('detector_mode', default_value='auto',
                              description='verifier | auto | primary'),
        DeclareLaunchArgument('primary_impl', default_value='yolo',
                              description='none | mock_depth | color | yolo'),
        DeclareLaunchArgument('color_v_min', default_value='170',
                              description='색 검출 밝기 하한. 팀 월드 170, A 월드 200'),
        DeclareLaunchArgument('yolo_model', default_value=os.path.join(get_package_share_directory('ai_inference'), 'models', 'plate_yolov8n_320.onnx'),
                              description='primary_impl:=yolo 일 때 쓰는 ONNX 모델 경로'),
        DeclareLaunchArgument('run_id', default_value='',
                              description='시험 이름. 주면 ~/pac_logs/<run_id>/ai.csv에 저장, 비우면 기존처럼 ai_<시각>.csv'),
        Node(
            package='ai_inference',
            executable='ai_inference_node',
            name='ai_inference_node',
            output='screen',
            parameters=[cfg, {
                'detector_mode': ParameterValue(LaunchConfiguration('detector_mode'), value_type=str),
                'primary_impl': ParameterValue(LaunchConfiguration('primary_impl'), value_type=str),
                'color_v_min': ParameterValue(LaunchConfiguration('color_v_min'), value_type=int),
                'yolo_model_path': ParameterValue(LaunchConfiguration('yolo_model'), value_type=str),
                'run_id': ParameterValue(LaunchConfiguration('run_id'), value_type=str),
            }],
        ),
    ])
