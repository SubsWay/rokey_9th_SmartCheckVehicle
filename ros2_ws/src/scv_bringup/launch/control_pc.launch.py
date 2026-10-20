"""관제 PC: cell_map_writer(칸 맵 저장) + inspection_manager(작업 배정 · 재측정 · 회피 마스크 · 설비 점검).

로봇 PC 의 Nav2 보다 먼저 띄운다 (inspection_manager 가 빈 회피 마스크를 먼저 내야 주행의 costmap 이 준비된다).

    ros2 launch scv_bringup control_pc.launch.py                                   # 두 대, 칸 맵은 파일
    ros2 launch scv_bringup control_pc.launch.py robots:=robot4                    # robot4 한 대
    ros2 launch scv_bringup control_pc.launch.py backend:=db ap_ssid:=<핫스팟>     # 칸 맵을 DB 에 (SCV_DB_DSN 필요)
    … backend:=db ap_ssid:=<핫스팟> close_stale_run:=true    # 관제를 끄면서 남은 running 회차를 닫고 새로 시작

    … backend:=db ap_ssid:=<핫스팟> web:=true      # 관제 웹(server_page)까지 함께 (10/14 ROS 통합)

순찰 주인은 inspection_manager 다 (10/14 PM 결정). web:=true 면 server_page 를 SCV_RUN_OWNER=ros 로 띄우고,
그 안의 ros_bridge(scv_app/ros_bridge.py)가 상태 · 알림 · 판정 · 표본을 받아 DB · 화면에 넣고
시작 · 비상정지 · 재개 버튼은 /inspection/* 서비스를 부른다. 웹은 backend:=db 일 때만 (같은 SCV_DB_DSN).
웹 폴더는 이 워크스페이스의 src/scv_app (scv_app_dir 로 바꿈), 파이썬은 web_python (fastapi · psycopg 가 있는 것).
network_map_engine 은 로봇 PC 쪽 미리보기로 뜬다.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ZONE_OF = {'robot4': 'Z1', 'robot3': 'Z2'}      # zones.yaml 의 robot: 과 같게


def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    robots = list(ZONE_OF) if arg('robots') == 'both' else [arg('robots')]
    store = {'ap_ssid': arg('ap_ssid')}
    nodes = [
        Node(package='scv_survey', executable='cell_map_writer', output='screen',
             parameters=[{'sample_topics': [f'/{r}/survey/sample' for r in robots],
                          'backend': arg('backend'), **store}]),
        Node(package='scv_manager', executable='inspection_manager', output='screen',
             parameters=[{'robot_ids': robots, 'zone_ids': [ZONE_OF[r] for r in robots],
                          'cellmap_backend': arg('backend'),
                          'close_stale_run': arg('close_stale_run') == 'true', **store}]),
    ]
    if arg('web') == 'true':
        if arg('backend') != 'db':
            raise RuntimeError('web:=true 는 backend:=db 와 함께 씁니다 (웹과 inspection_manager 가 같은 DB 를 봐야 함)')
        app = Path(arg('scv_app_dir') or Path(get_package_share_directory('scv_bringup')).parents[3] / 'src' / 'scv_app')
        if not (app / 'server_page.py').exists():
            raise RuntimeError(f'server_page.py 가 없습니다: {app} (scv_app_dir:=<경로>)')
        nodes.append(ExecuteProcess(cmd=[arg('web_python'), 'server_page.py'], cwd=str(app), output='screen',
                                    additional_env={'SCV_RUN_OWNER': 'ros',
                                                    'SCV_ZONES': os.path.join(get_package_share_directory('scv_bringup'),
                                                                              'config', 'zones.yaml')}))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # 관제 노드는 일반 클라이언트로 (turtlebot4_discovery 는 SUPER_CLIENT=True — 네트워크의 모든 참가자 discovery 를 다 받음).
        # 10/15 Victus Wi-Fi 수신이 discovery · 죽은 엔드포인트로 오는 데이터로 가득 참. robot_pc.launch 와 같은 설정. ros2 CLI 는 셸 값 그대로
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        DeclareLaunchArgument('robots', default_value='both', description='both · robot4 · robot3'),
        DeclareLaunchArgument('backend', default_value='file', description='칸 맵 저장: file(~/scv_cellmap) · db'),
        DeclareLaunchArgument('ap_ssid', default_value='', description='측정 대상 핫스팟 SSID (db 면 필수, rssi_scanner 와 같은 값)'),
        DeclareLaunchArgument('close_stale_run', default_value='false',
                              description='db: 닫히지 않은 running 회차가 남아 있으면 aborted 로 닫고 시작 (true · false)'),
        DeclareLaunchArgument('web', default_value='false', description='관제 웹 server_page 를 ros 모드로 함께 (true · false)'),
        DeclareLaunchArgument('scv_app_dir', default_value='', description='server_page.py 폴더 (기본: 워크스페이스/src/scv_app)'),
        DeclareLaunchArgument('web_python', default_value='python3', description='server_page 를 돌릴 파이썬 (fastapi · psycopg 설치된 것)'),
        OpaqueFunction(function=_nodes),
    ])
