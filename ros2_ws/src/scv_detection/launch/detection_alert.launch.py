"""사용자가 고른 로봇의 카메라와 요청형 YOLO·랙 검사 서비스를 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """기본 robot4 카메라를 표시하고 두 검사 서비스를 준비한다. 모델 경로는 자동 선택한다."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot4', description='설비 점검 로봇 이름공간'),
        DeclareLaunchArgument('weights', default_value='', description='빈 값이면 설치된 models/best.pt 자동 선택'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('rack_enabled', default_value='true'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('device', default_value='auto'),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_ns')),
            Node(package='scv_detection', executable='scv_detection_manager', output='screen',
                 parameters=[{'weights': LaunchConfiguration('weights'),
                              'enable_yolo': ParameterValue(LaunchConfiguration('enable_yolo'), value_type=bool),
                              'rack_enabled': ParameterValue(LaunchConfiguration('rack_enabled'), value_type=bool),
                              'device': LaunchConfiguration('device'),
                              'show': ParameterValue(LaunchConfiguration('show'), value_type=bool)}]),
        ]),
    ])
