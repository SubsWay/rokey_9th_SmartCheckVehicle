"""노트북 단위시험: test_map 위에 노트북으로 잰 핫스팟 RSSI 를 0.5 m 칸 색으로 표시한다.

측정은 iw scan 으로 주변 AP 중 ssid 를 듣는다 (노트북은 turtle08 에 붙은 채로). ssid 를 비우면
SSID 에 ssid_pattern(iPhone) 이 들어간 AP 중 처음 가장 센 것으로 자동 고정.

노트북만 들고 (위치는 RViz 클릭):
  ros2 launch scv_survey laptop_test.launch.py ssid:="csh의 iPhone"
노트북을 TurtleBot4 위에 올리고 (위치는 로봇 TF, 지도·AMCL 은 turtlebot4_navigation localization 이 띄움):
  ros2 launch scv_survey laptop_test.launch.py pose_source:=tf robot_ns:=robot3 ssid:="csh의 iPhone"

회피 마스크 시험 (manual): 다른 터미널에서 ros2 run scv_survey survey_keys → 다 쟀으면 s.
  manual 에서는 Nav2 코스트맵(global_costmap, KeepoutFilter 포함)을 노트북에 따로 띄워 /costmap 으로 보여 준다
  (keepout_costmap:=false 로 끔). 로봇 시험(tf)에서는 띄우지 않는다 — 로봇 Nav2 와 섞이지 않게
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    share = get_package_share_directory('scv_survey')
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    use_tf = arg('pose_source') == 'tf'
    ns = arg('robot_ns') or ('robot4' if use_tf else 'laptop')
    csv_path = arg('csv_path') or f'~/scv_survey_logs/{ns}_{arg("pose_source")}.csv'

    if use_tf:
        tagger_params = {'pose_source': 'tf', 'base_frame': arg('base_frame'),
                         'max_pose_age': float(arg('max_pose_age'))}
        # TurtleBot4 는 TF·지도·AMCL 을 로봇 이름공간 아래에 둔다
        tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        rviz_remaps = [(f'/{t}', f'/{ns}/{t}')
                       for t in ('tf', 'tf_static', 'map', 'initialpose', 'scan', 'robot_description')]
        map_topic = f'/{ns}/map'
    else:
        tagger_params = {'pose_source': 'manual',
                         'init_x': float(arg('init_x')), 'init_y': float(arg('init_y')),
                         'samples_per_point': int(arg('samples_per_point'))}
        tf_remaps, rviz_remaps, map_topic = [], [], '/map'

    ssid = arg('ssid')
    target = f"'{ssid}' (직접 입력)" if ssid else f"자동 — SSID 에 '{arg('ssid_pattern')}' 가 들어간 AP"
    nodes = [
        LogInfo(msg=f"[scv_survey] RSSI 측정: iw {arg('method')} · {arg('iface')} · 대상 {target} · "
                    f"{arg('rate_hz')} Hz · /{ns}/survey/raw · CSV {csv_path}"),
        Node(package='scv_survey', executable='rssi_scanner', namespace=ns, output='screen',
             parameters=[{'iface': arg('iface'), 'method': arg('method'),
                          # 숫자로만 된 SSID 도 문자열로 넘기려고 ParameterValue
                          'target_ssid': ParameterValue(ssid, value_type=str),
                          'ssid_pattern': ParameterValue(arg('ssid_pattern'), value_type=str),
                          'robot_id': ns, 'rate_hz': float(arg('rate_hz')),
                          'rescan_after_miss': int(arg('rescan_after_miss')),
                          'scan_retries': int(arg('scan_retries')),
                          'full_scan_interval': float(arg('full_scan_interval')),
                          'fixed_freq': int(arg('rssi_freq')),
                          'log_samples': arg('log_samples') == 'true',
                          'status_period': float(arg('status_period'))}]),
        Node(package='scv_survey', executable='manual_tagger', namespace=ns, output='screen',
             parameters=[tagger_params], remappings=tf_remaps),
        Node(package='scv_survey', executable='network_map_engine', output='screen',
             parameters=[{'sample_topics': [f'/{ns}/survey/sample'],
                          'map_topic': map_topic,
                          'cell_size': float(arg('cell_size')),
                          'rssi_strong': float(arg('rssi_strong')),
                          'rssi_weak': float(arg('rssi_weak')),
                          'csv_path': csv_path,
                          'mask_topic': arg('mask_topic'),
                          'shadow_dbm': float(arg('shadow_dbm')),
                          'mask_margin': float(arg('mask_margin'))}]),
    ]
    if not use_tf:
        # 로봇 시험에서는 localization.launch.py 의 map_server 가 /{ns}/map 을 낸다
        nodes += [
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[{'yaml_filename': arg('map')}], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_map', output='screen',
                 parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        ]
        if arg('keepout_costmap') == 'true':
            nodes += _keepout_costmap(arg('mask_topic'))
    if arg('rviz') == 'true':
        nodes.append(
            Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                 arguments=['-d', os.path.join(share, 'rviz', 'survey.rviz')],
                 remappings=rviz_remaps))
    return nodes


def _keepout_costmap(mask_topic):
    """노트북에서 Nav2 전역 코스트맵만 띄워 회피 마스크가 장애물로 바뀌는지 본다 (로봇 B 설정의 축소판).

    KeepoutFilter 는 costmap_filter_info 로 마스크 토픽을 알고, 마스크 100 → LETHAL. 노트북엔 로봇이 없어
    코스트맵이 기다리는 map → base_link 는 원점 고정 TF 로 준다.
    """
    costmap = {
        'global_frame': 'map', 'robot_base_frame': 'base_link',
        'update_frequency': 2.0, 'publish_frequency': 2.0,
        'robot_radius': 0.19, 'track_unknown_space': True, 'always_send_full_costmap': True,
        'plugins': ['static_layer', 'inflation_layer'],
        'filters': ['keepout_filter'],
        'static_layer.plugin': 'nav2_costmap_2d::StaticLayer',
        'static_layer.map_topic': '/map',
        'static_layer.map_subscribe_transient_local': True,
        'inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
        'inflation_layer.inflation_radius': 0.45,           # turtlebot4_navigation nav2.yaml 과 같게
        'inflation_layer.cost_scaling_factor': 4.0,
        'keepout_filter.plugin': 'nav2_costmap_2d::KeepoutFilter',
        'keepout_filter.enabled': True,
        'keepout_filter.filter_info_topic': '/costmap_filter_info',
    }
    return [
        Node(package='tf2_ros', executable='static_transform_publisher', name='keepout_test_base_link',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_link'], output='log'),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', output='screen',
             parameters=[{'type': 0, 'filter_info_topic': '/costmap_filter_info', 'mask_topic': mask_topic,
                          'base': 0.0, 'multiplier': 1.0}]),
        # 단독 코스트맵 노드는 이름을 global_costmap 으로, 코스트맵 토픽은 /costmap
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d', name='global_costmap',
             output='screen', parameters=[costmap]),
        # 단독 코스트맵 노드는 bond 를 만들지 않아 bond_timeout 0 (끔)
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_keepout', output='screen',
             parameters=[{'autostart': True, 'bond_timeout': 0.0,
                          'node_names': ['costmap_filter_info_server', 'global_costmap']}]),
    ]


def generate_launch_description():
    share = get_package_share_directory('scv_survey')
    return LaunchDescription([
        DeclareLaunchArgument('pose_source', default_value='manual',
                              description='manual: RViz 클릭 위치 / tf: 로봇 TF map→base_link'),
        DeclareLaunchArgument('robot_ns', default_value='',
                              description='비우면 manual=laptop, tf=robot4'),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'test_map.yaml'),
                              description='manual 에서만 씀'),
        DeclareLaunchArgument('iface', default_value='wlo1'),
        DeclareLaunchArgument('method', default_value='scan',
                              description='scan: 주변 AP 스캔 / link: 붙어 있는 AP (예전 방식)'),
        DeclareLaunchArgument('ssid', default_value='',
                              description='측정할 SSID (예: "csh의 iPhone"). 비우면 ssid_pattern 으로 자동 선택'),
        DeclareLaunchArgument('ssid_pattern', default_value='iPhone',
                              description='ssid 를 비웠을 때 이 글자가 들어간 AP 중 가장 센 것으로 고정'),
        DeclareLaunchArgument('rate_hz', default_value='1.0'),
        DeclareLaunchArgument('rescan_after_miss', default_value='3',
                              description='이 횟수 연속 미검출이면 전체 대역 스캔으로 채널 재탐색'),
        DeclareLaunchArgument('scan_retries', default_value='2',
                              description='한 채널 스캔에서 못 들으면 같은 채널을 다시 스캔하는 횟수'),
        DeclareLaunchArgument('rssi_freq', default_value='0',
                              description='측정 AP 채널 고정 [MHz] (예 5745). 0 이면 찾기 — 전체 대역 스캔이 통신을 끊으니 시연은 고정 권장'),
        DeclareLaunchArgument('full_scan_interval', default_value='60.0',
                              description='재탐색 중 전체 대역 스캔 간격 [s] (한 번에 약 5 s)'),
        DeclareLaunchArgument('log_samples', default_value='true',
                              description='측정마다 SSID·dBm 한 줄 로그'),
        DeclareLaunchArgument('status_period', default_value='10.0',
                              description='측정 상태 요약 로그 간격 [s], 0 이면 끔'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('samples_per_point', default_value='5'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('max_pose_age', default_value='0.5',
                              description='측정 시각·지금과 TF 시각 차 허용 [s] (지연·시계 차), 0 이면 검사 안 함'),
        DeclareLaunchArgument('cell_size', default_value='0.5'),
        DeclareLaunchArgument('rssi_strong', default_value='-35.0',
                              description='이 값 이상은 빨강 [dBm]'),
        DeclareLaunchArgument('rssi_weak', default_value='-75.0',
                              description='이 값 이하는 파랑(음영) [dBm]'),
        DeclareLaunchArgument('csv_path', default_value='',
                              description='비우면 ~/scv_survey_logs/{robot_ns}_{pose_source}.csv'),
        DeclareLaunchArgument('mask_topic', default_value='/keepout_mask',
                              description='회피 마스크 (IF-10). 실제 구성은 점검 로봇의 /{ns}/keepout_mask'),
        DeclareLaunchArgument('shadow_dbm', default_value='-72.0',
                              description='칸 중앙값이 이 값보다 작으면 음영 → 회피 [dBm] (BR-T04)'),
        DeclareLaunchArgument('mask_margin', default_value='0.0',
                              description='회피 칸을 넓히는 폭 [m]. 0 = 0.5 m 칸 그대로 (KeepoutFilter 는 팽창되지 않음)'),
        DeclareLaunchArgument('keepout_costmap', default_value='true',
                              description='manual 에서 Nav2 전역 코스트맵(KeepoutFilter)을 노트북에 띄워 /costmap 으로'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
