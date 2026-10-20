"""로봇 RPi 한 대: 우리 노드는 없다 — 드라이버(Create 3 · LiDAR · OAK-D)는 turtlebot4 기본 bringup 이 띄운다.

10/12 저녁: rssi_scanner 는 로봇 위에 얹은 로봇 PC 에서 돈다 (robot_pc.launch.py). 이 파일은 자리만 남긴다.

    ros2 launch scv_bringup robot_rpi.launch.py robot_ns:=robot4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    nodes = [
        # RPi 에서 띄울 우리 노드는 없다 (10/12 저녁)
    ]
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot4', description='로봇 이름공간 (robot4, robot3)'),
        GroupAction([PushRosNamespace(LaunchConfiguration('robot_ns')), *nodes]),
    ])
