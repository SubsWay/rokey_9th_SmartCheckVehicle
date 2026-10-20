#!/usr/bin/env python3
# PLAN 5-1·5-2·5-3·3-4·4-2: IF-11 {ns}/execute_task 액션 서버 + IF-12 {ns}/status 1 Hz. 작업 한 건씩 받아 Nav2 로 실행
#   python3 mission_executor.py --ros-args -r __ns:=/robot3
#   (undock 상태로 다른 자리에서 다시 켤 때만 -p x:=… -p y:=… -p yaw:=… 로 그 자리 좌표를 줌)
# 시작: Dock 에 있으면 라이다만 켜고 그대로 → 초기 pose 설정 → Nav2 활성화 대기. undock 은 첫 작업 때 (10/15 통합 브랜치)
# HOME 의 x, y(m), yaw(도)는 undock 직후 로봇의 지도(testbed_v1) 좌표. Dock 에서는 lidar 가 꺼져 있어 이 위치를 기준으로 함
# 이 위치는 Dock 바로 앞이므로 배터리 부족·DOCK 작업 때 여기로 돌아와 dock 한다
# 속도 상한 0.2 m/s 는 Nav2 controller·velocity_smoother 설정에서 건다 (이 파일에서 속도를 내지 않음)
# 작업 종류
#   SURVEY_SEGMENT: waypoints 를 navigate_through_poses 로 통과
#   FACILITY_CHECK: 마지막 waypoint 에 정밀 도착 → {ns}/inspect_facility 서비스 (IF-05, 10/14 PM 결정: 액션 → 서비스, 자세 보정 없음)
#                   관측 자리가 장애물에 막히면 둘레 0.15–0.45 m 빈 자리(최대 3곳)에서 같은 방향으로 점검, 결과 reason 에 ALT_VIEW
#   REINSPECTION:   마지막 waypoint 에 정밀 도착 → {ns}/survey/precise 10~15초
#   DOCK:           waypoints(없으면 undock 위치)로 가서 dock
# 막힘(E1): 5초 동안 5 cm·0.3 rad 도 못 움직이면 BLOCKED, BLOCKED 때 길 없는 점이 있으면 바로, 아니면 10초에 지금 향하던 점을 건너뜀
#   향하던 점이 global costmap 에서 장애물·inscribed 칸이 되면(지도에 없는 장애물이 점 위에 있음) 다가가기 전에 바로 건너뜀
#   (그대로 두면 RPP 가 장애물에 2–5 cm 까지 붙어 돌지도 못하고 갇힘 — 10/14 시뮬, BackUp 복구 없음)
# 배터리(E2): 30 % 이하 새 작업 거절(DOCK 제외), DOCK_PCT 이하 작업 취소 후 dock
import math
import re
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry
from action_msgs.msg import GoalStatus
from nav2_simple_commander.robot_navigator import TaskResult
from rcl_interfaces.srv import SetParameters
from scv_msgs.action import ExecuteTask, PreciseMeasure
from scv_msgs.srv import InspectFacility
from scv_msgs.msg import InspectionResult, RobotStatus
from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

T = ExecuteTask.Goal
S = RobotStatus
NORMAL_TOL = (0.05, 0.087)   # nav2_rpp.yaml general_goal_checker 와 같은 값 (m, rad). 3-4 시험 (2026-10-12)
PRECISE_TOL = (0.03, 0.052)  # 설비 점검 지점 3 cm, 3° (PLAN 0-1 목표 5 cm, 5° 안쪽으로)
# shortcut: facilities.yaml id 앞 세 글자로 종류를 정함, 관제가 kind 를 보내 주면 그 값으로 바꿈
KIND = {'EXT': InspectionResult.KIND_EXTINGUISHER, 'SHT': InspectionResult.KIND_SHUTTER,
        'EXL': InspectionResult.KIND_EXIT_LIGHT, 'RCK': InspectionResult.KIND_RACK}
# undock 직후 위치 (x m, y m, yaw 도). pose_check 3회 평균 — robot3 10/12 재측정(좌우 spin 으로 AMCL 수렴 뒤, yaw 범위 2.1°), robot4 10/11 (docs/driving/data/2026-10-1{1,2}_pose_check.csv)
DOCK_BACKOFF = 0.3   # dock 명령 내리는 자리: undock 위치보다 dock 에서 이만큼 더 먼 곳 (m, 10/12 hhj 요청)
DOCK_PCT = 20.0   # 스스로 dock 하는 배터리 % (process_flow E2 는 15, 10/12 hhj 요청으로 20)
HOME = {'/robot3': (2.806, 4.610, 179.9), '/robot4': (2.863, 0.948, 175.8)}
INSCRIBED_COST = 253   # nav2 costmap: 로봇 중심이 여기 오면 부딪힘 (254 장애물, 255 모름)
BACKOFF = 0.25         # 막힌 점 대신 갈 줄 위의 점: 줄이 inscribed 칸에 닿는 곳에서 이만큼 앞 (장애물과 약 0.44 m, 로봇 가장자리 25 cm).
                       # inflation 비용으로 고르면(100 미만) 6 cm 까지 붙어 돌지 못하고 55 초 서 있었음 (10/14 시뮬)
NEAR_GOAL = 0.5        # 골 둘레 이 안에 새 장애물(지도엔 빈칸, costmap 엔 장애물)이 보여야 줄 막힘을 골 막힘으로 봄 (m)
MIN_STEP = 0.15        # 대신 갈 점이 앞 점(또는 지금 위치)에서 이보다 가까우면 그냥 뺌 (m)
ALT_RINGS = (0.15, 0.25, 0.35, 0.45)   # 설비 관측 자리가 막혔을 때 대체 자리를 찾는 거리 (m, 10/15)
ALT_MAX_COST = 100     # 대체 자리는 costmap 비용이 이 미만인 곳만 (inscribed 253 보다 충분히 여유)
CMD_LIN, CMD_ANG = 0.08, 0.4   # 회피 구역 cmd_vel 접근 속도 (m/s, rad/s) — Nav2 충돌 감시를 거치지 않으므로 느리게 (10/16)
CMD_TOL = (0.05, 0.087)        # cmd_vel 접근 도착 허용 (m, rad)


class MissionExecutor(Node):
    def __init__(self, nav, home):
        super().__init__('mission_executor')
        self.nav = nav   # nav 는 자기 스스로 spin 하므로 이 노드의 executor 에 넣지 않는다
        self.home = home
        self.busy = False
        self.state = S.STATE_IDLE
        self.task = None   # 수행 중인 ExecuteTask.Goal
        self.run_id = 0    # 마지막으로 받은 작업의 회차 (RobotStatus.msg: 작업 사이에도 유지)
        self.progress = 0.0
        self.warning = ''
        self.battery = 100.0
        self.odom = None  # (수신 monotonic 시각, 선속도, 각속도)
        self.odom_pose = None  # odom 프레임 (x, y, yaw) — cmd_vel 접근 때 amcl 사이를 메움 (10/16)
        self.precise_duration = int(self.declare_parameter("precise_duration_s", 15).value)
        self.cell_size = float(self.declare_parameter("cell_size", 0.5).value)
        if not 10 <= self.precise_duration <= 15:
            raise ValueError("precise_duration_s는 10~15초여야 합니다")
        self.pose = None   # (x, y, yaw rad, 위치 표준편차 m)
        self.costmap = None   # 마지막 global costmap (nav2_msgs/Costmap, map 프레임)
        self.static_map = None   # 지도 (map_server). costmap 장애물 중 지도에 없는 것을 가리는 데 씀
        self.goal_index, self.skipped, self.shortened = -1, set(), set()   # 작업 중 막힌 점 기록 (follow_loop)
        self.blocked_spots = []   # 이번 회차에 장애물로 못 간 waypoint 자리 (x, y). 다음 작업이 같은 자리에서 시작하면 뺌
        self.robot_id = self.get_namespace().strip('/')
        self.pose_std_max = self.declare_parameter('pose_std_max', 0.15).value   # PLAN 2-3: 정상 9–11 cm, 틀림 52 cm (docs/driving/data/2026-10-11_robot4_pose_ok.csv)

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(RobotStatus, 'status', 10)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, qos_profile_sensor_data)
        # 손으로 올리거나 Dock 명령을 따로 보낸 것도 status 에 DOCKED 로 (관제가 도크 위 표본을 버림 · 작업 때 undock, 10/15)
        from irobot_create_msgs.msg import DockStatus
        self.create_subscription(DockStatus, 'dock_status', self.on_dock, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Costmap, 'global_costmap/costmap_raw', self.on_costmap, 1)
        self.create_subscription(OccupancyGrid, 'map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.param_cli = self.create_client(SetParameters, 'controller_server/set_parameters', callback_group=cb)
        self.inspect_cli = self.create_client(InspectFacility, 'inspect_facility', callback_group=cb)
        self.inspect_timeout = float(self.declare_parameter('inspect_timeout_s', 30.0).value)   # 비전 판정 최대 시간
        # 회피 구역(미검출 칸) 안 설비: Nav2 로 못 가면 cmd_vel 로 느리게 직선 접근해 점검 (10/16, 기본 꺼짐 · 실기 미검증)
        self.keepout_cmd_vel = bool(self.declare_parameter('keepout_cmd_vel', False).value)
        self.keepout_cmd_vel_max = float(self.declare_parameter('keepout_cmd_vel_max_m', 1.5).value)
        self.precise_cli = ActionClient(self, PreciseMeasure, 'survey/precise', callback_group=cb)
        ActionServer(self, ExecuteTask, 'execute_task', self.execute,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=cb)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('mission_executor ready')

    # ---------- 상태 (IF-12) ----------
    def on_battery(self, msg):
        self.battery = msg.percentage * 100.0   # Create 3 는 0–1

    def on_dock(self, msg):
        if self.busy:                           # 작업 중(undock · dock 포함)에는 작업 쪽이 state 를 정함
            return
        if msg.is_docked:
            self.state = S.STATE_DOCKED
        elif self.state == S.STATE_DOCKED:
            self.state = S.STATE_IDLE

    def on_costmap(self, msg):
        self.costmap = msg

    def on_map(self, msg):
        self.static_map = msg

    def new_obstacle_near(self, pose, radius=NEAR_GOAL):
        # pose 둘레 radius 안에 지도에 없는 장애물 칸(costmap 254, 지도는 빈칸)이 있는가
        m = self.costmap
        if m is None:
            return False
        md = m.metadata
        x0, y0 = xy(pose)
        r = int(radius / md.resolution) + 1
        cx, cy = int((x0 - md.origin.position.x) / md.resolution), int((y0 - md.origin.position.y) / md.resolution)
        for row in range(max(cy - r, 0), min(cy + r + 1, md.size_y)):
            for col in range(max(cx - r, 0), min(cx + r + 1, md.size_x)):
                if m.data[row * md.size_x + col] != 254:
                    continue
                x = md.origin.position.x + (col + 0.5) * md.resolution
                y = md.origin.position.y + (row + 0.5) * md.resolution
                if math.dist((x, y), (x0, y0)) <= radius and not self.on_static_map(x, y):
                    return True
        return False

    def on_static_map(self, x, y):
        # 지도(map_server)에서 점유 칸인가 (지도를 못 받았으면 지도에 있는 것으로 봄 → 새 장애물로 세지 않음)
        g = self.static_map
        if g is None:
            return True
        gi = g.info
        col, row = int((x - gi.origin.position.x) / gi.resolution), int((y - gi.origin.position.y) / gi.resolution)
        if not (0 <= col < gi.width and 0 <= row < gi.height):
            return True
        return g.data[row * gi.width + col] >= 50 or g.data[row * gi.width + col] < 0

    def cost(self, x, y):
        # global costmap 비용. 모르는 칸·costmap 밖·아직 못 받음은 0
        m = self.costmap
        if m is None:
            return 0
        md = m.metadata
        col = int((x - md.origin.position.x) / md.resolution)
        row = int((y - md.origin.position.y) / md.resolution)
        if not (0 <= col < md.size_x and 0 <= row < md.size_y):
            return 0
        c = m.data[row * md.size_x + col]
        return 0 if c == 255 else c

    def occupied(self, pose):
        # 점이 장애물·inscribed 칸인가 (지도에 없는 장애물이 점 위에 있음)
        return self.cost(pose.pose.position.x, pose.pose.position.y) >= INSCRIBED_COST

    def line_hit(self, start, pose):
        # start(x, y) → pose 직선이 처음 inscribed 칸에 닿는 거리 (안 닿으면 None)
        x0, y0 = start
        x1, y1 = pose.pose.position.x, pose.pose.position.y
        length = math.dist((x0, y0), (x1, y1))
        step = self.costmap.metadata.resolution / 2 if self.costmap else 0.025
        return next((k * step for k in range(1, int(length / step) + 1)
                     if self.cost(x0 + (x1 - x0) * k * step / length,
                                  y0 + (y1 - y0) * k * step / length) >= INSCRIBED_COST), None)

    def blocked(self, before, pose):
        # 점이 장애물 칸이거나, 앞 waypoint(before) 에서 그 점까지의 측정 줄이 막혔고 점 둘레 NEAR_GOAL 안에 지도에 없는
        # 장애물이 보임. 앞에서만 보면 줄 가운데만 막은 장애물(돌아가면 골에 감)과 골을 덮은 장애물(안쪽은 라이다가 못 봐
        # 빈칸으로 남고, 돌아가다 모서리에 붙어 갇힘)이 구분되지 않아 막힘 쪽으로 봄 — 잘못 봐도 골 앞 한 칸만 잃음 (10/14 시뮬:
        # 세 쪽이 막혀야 골 막힘으로 보면 큰 상자에서 갇혀 이후 작업을 모두 잃음)
        return self.occupied(pose) or (before is not None and self.line_hit(xy(before), pose) is not None
                                       and self.new_obstacle_near(pose))

    def last_free(self, start, pose):
        # start(x, y) → pose 직선이 처음 inscribed 칸에 닿는 곳에서 BACKOFF 만큼 앞의 점 (막힌 끝점 대신 그 줄을 장애물 앞까지 잼).
        # 그 점이 start 에서 MIN_STEP 안이면 None
        x0, y0 = start
        x1, y1 = pose.pose.position.x, pose.pose.position.y
        length = math.dist((x0, y0), (x1, y1))
        hit = self.line_hit(start, pose)
        best = (length if hit is None else hit) - BACKOFF
        if best < MIN_STEP:
            return None
        p = PoseStamped(header=pose.header)
        p.pose.position.x, p.pose.position.y = x0 + (x1 - x0) * best / length, y0 + (y1 - y0) * best / length
        p.pose.orientation = pose.pose.orientation
        return p

    def alternative_views(self, view, limit=3):
        """막힌 관측 자리 둘레(ALT_RINGS m)에서 갈 만한 빈 자리 — 가까운 순, 같은 거리면 보는 방향 뒤쪽(설비에서 조금 멀어지는 쪽) 먼저.
        방향(yaw)은 원래 관측 자세 그대로 (정밀 도착이 마지막에 그 방향으로 돎). 지도 장애물 · costmap ALT_MAX_COST 이상은 뺌"""
        x0, y0 = xy(view)
        q = view.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)
        found = []
        for r in ALT_RINGS:
            for k in range(16):
                a = k * math.tau / 16
                x, y = x0 + r * math.cos(a), y0 + r * math.sin(a)
                c = self.cost(x, y)
                if c >= ALT_MAX_COST or self.on_static_map(x, y):
                    continue
                behind = abs(math.remainder(a - (yaw + math.pi), math.tau))   # 0 = 바로 뒤
                found.append((r, behind, c, x, y))
        out = []
        for r, _, _, x, y in sorted(found)[:limit]:
            p = PoseStamped(header=view.header)
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation = view.pose.orientation
            out.append(p)
        return out

    def on_odom(self, msg):
        v = msg.twist.twist
        self.odom = (time.monotonic(), math.hypot(v.linear.x, v.linear.y), abs(v.angular.z))
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom_pose = (p.x, p.y, 2 * math.atan2(q.z, q.w))

    def on_pose(self, msg):
        p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw, math.sqrt(max(c[0], c[7], 0.0)))

    def publish_status(self):
        m = RobotStatus(robot_id=self.robot_id, state=self.state, battery_percent=self.battery,
                        accepting_tasks=self.battery > 30.0, progress=self.progress, warning=self.warning)
        m.stamp = self.get_clock().now().to_msg()
        m.run_id = self.run_id
        if self.task:
            m.task_id, m.task_type = self.task.task_id, self.task.type
        if self.pose:
            m.x, m.y, m.yaw = self.pose[:3]
            m.pose_ok = self.pose[3] <= self.pose_std_max
        self.status_pub.publish(m)

    # ---------- 작업 (IF-11) ----------
    def on_goal(self, goal):
        # 한 번에 한 작업만 (관제가 이전 작업을 취소한 뒤 새로 보냄). 배터리 30 % 이하는 DOCK 만 받음
        if self.busy or (self.battery <= 30.0 and goal.type != T.DOCK):
            return GoalResponse.REJECT
        self.busy = True   # 수락과 실행 사이에 다른 goal 이 들어오지 않게 여기서 잡음
        return GoalResponse.ACCEPT

    def execute(self, gh):
        g = gh.request
        if g.run_id != self.run_id:
            self.blocked_spots = []
        self.task, self.run_id, self.progress, self.warning = g, g.run_id, 0.0, ''
        measured = None   # REINSPECTION 의 PreciseMeasure 결과 (interfaces v0.2.10)
        if self.state == S.STATE_DOCKED and g.type != T.DOCK:
            self.nav.undock()
            localize(self.nav, self.home)   # 시작할 때와 같은 이유 (Dock 에서는 lidar 꺼짐)
            settle(self.nav)
        self.state = S.STATE_EXECUTING
        self.get_logger().info(f'task {g.task_id} type={g.type} waypoints={len(g.waypoints)}')
        try:
            if g.type == T.SURVEY_SEGMENT:
                ok, reason = self.follow(gh, g.waypoints)
            elif g.type == T.FACILITY_CHECK:
                ok, reason = self.follow(gh, g.waypoints, precise=True)
                alt = None
                # 관측 자리가 장애물에 막히면 둘레의 빈 자리에서 같은 방향을 보고 점검 (10/15 통합 브랜치, cmd_vel 강제 이동 대신)
                for alt in ([] if ok or not reason.startswith('BLOCKED') else self.alternative_views(g.waypoints[-1])):
                    if gh.is_cancel_requested:
                        break
                    self.get_logger().warn(f'{g.target_ref}: 관측 자리 막힘 → 대체 자리 '
                                           f'({alt.pose.position.x:.2f}, {alt.pose.position.y:.2f}) 에서 같은 방향으로 점검')
                    ok, reason = self.follow(gh, [alt], precise=True)
                    if ok:
                        break
                via_cmd_vel = False
                # 그래도 못 가면(회피 구역 안 관측 자리 등) keepout_cmd_vel 이 켜졌을 때만 cmd_vel 로 접근 (10/16)
                back = None
                if (not ok and self.keepout_cmd_vel and g.waypoints and reason.startswith('BLOCKED')
                        and not gh.is_cancel_requested and self.pose is not None):
                    self.get_logger().warn(f'{g.target_ref}: Nav2 로 관측 자리에 못 감({reason}) → cmd_vel 로 접근')
                    view = g.waypoints[-1]
                    q = view.pose.orientation
                    back = self.pose[:3]                         # 점검 뒤 되돌아 나올 자리 (회피 칸 밖, Nav2 가 다시 출발할 수 있게)
                    ok, reason = self.cmd_vel_approach(gh, (view.pose.position.x, view.pose.position.y,
                                                            2 * math.atan2(q.z, q.w)))
                    alt, via_cmd_vel = None, ok
                if ok:
                    ok, reason = self.inspect(gh, g)
                    if alt is not None:
                        reason = f'ALT_VIEW ({alt.pose.position.x:.2f}, {alt.pose.position.y:.2f}) {reason}'
                    if via_cmd_vel:
                        back_ok, back_reason = self.cmd_vel_approach(gh, back)
                        reason = f'KEEPOUT_CMD_VEL {reason}' + ('' if back_ok else f' RETURN_FAILED {back_reason}')
            elif g.type == T.REINSPECTION:
                ok, reason = self.follow(gh, g.waypoints, precise=True)
                if ok:
                    ok, reason, measured = self.measure(gh)
            elif g.type == T.DOCK:
                ok, reason = self.go_dock(gh, g.waypoints)
            else:
                ok, reason = False, 'UNKNOWN_TYPE'
            if reason == 'LOW_BATTERY':
                self.go_dock(gh, [])
        finally:
            self.task, self.busy = None, False
            if self.state != S.STATE_DOCKED:
                self.state = S.STATE_IDLE
        self.get_logger().info(f'task {g.task_id} ok={ok} reason={reason}')
        if gh.is_cancel_requested:
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        r = ExecuteTask.Result(ok=ok, reason=reason)
        if measured:
            for field in ('rssi_median', 'rssi_iqr', 'n_samples', 'n_total', 'n_missed',
                          'rssi_mean', 'duration_s', 'measurement_valid', 'measurement_reason'):
                setattr(r, field, getattr(measured, field))
        return r

    def follow(self, gh, waypoints, precise=False):
        if not waypoints:
            return False, 'NO_WAYPOINTS'
        try:
            # 정밀 값으로 못 바꾸면 일반 오차로 도착해 버리므로 출발하지 않음 (원복은 finally)
            if precise and not self.set_tolerance(*PRECISE_TOL):
                return False, 'TOLERANCE_FAILED'
            return self.follow_loop(gh, list(waypoints), shorten=not precise)
        finally:
            if precise:
                self.set_tolerance(*NORMAL_TOL)

    def follow_loop(self, gh, waypoints, shorten=True):
        # 막힌 점 처리 (10/14, 관제가 측정 줄을 0.5 m 마다 중간 점으로 나눠 보냄):
        #   중간 점이 장애물 칸이면 그 점만 빼고 다음 점으로 (플래너가 돌아감). 여러 개가 막혀도 하나씩 뺌
        #   마지막 점(골)이 장애물 칸이거나 앞 점에서의 줄이 막히면 줄 위 마지막 빈 점까지만 가고 작업을 끝냄 (없으면 뺌)
        #   (shorten=False — 설비·재측정처럼 정확한 자리가 필요하면 골은 줄이지 않고 장애물 칸일 때 뺌)
        #   결과 reason 에 원래 번호를 남김: BLOCKED skipped=[…] shortened=[…] → 관제가 그 칸을 채우기(미측정 재측정)에서 뺌
        n = len(waypoints)
        self.goal_index = n - 1 if shorten else -1
        self.skipped, self.shortened = set(), set()
        left = self.drop_unreachable(list(enumerate(waypoints)))   # (원래 번호, pose)
        while left:
            self.nav.goThroughPoses([w for _, w in left])
            fb, last_xy, last_t, moved = None, None, time.time(), False
            while not self.nav.isTaskComplete():
                if gh.is_cancel_requested or self.battery <= DOCK_PCT:
                    self.nav.cancelTask()
                    self.stop()
                    return False, 'CANCELED' if gh.is_cancel_requested else 'LOW_BATTERY'
                fb = self.nav.getFeedback() or fb
                if fb:
                    i = left[min(len(left) - fb.number_of_poses_remaining, len(left) - 1)][0]
                    self.progress = i / n
                    gh.publish_feedback(ExecuteTask.Feedback(waypoint_index=i, progress=self.progress))
                    k = max(len(left) - fb.number_of_poses_remaining, 0)
                    if k < len(left) and self.must_change(left[k - 1][1] if k else None, *left[k]):
                        self.get_logger().warn(f'waypoint {left[k][0]} 가 장애물에 막힘 → 다가가지 않음')
                        moved = True
                        break
                    q = fb.current_pose.pose.orientation
                    xy_yaw = (fb.current_pose.pose.position.x, fb.current_pose.pose.position.y,
                              2 * math.atan2(q.z, q.w))
                    # 5 cm 또는 0.3 rad 바뀌면 움직임 (제자리 회전을 막힘으로 세지 않게)
                    if last_xy is None or math.dist(xy_yaw[:2], last_xy[:2]) > 0.05 or \
                            abs(math.remainder(xy_yaw[2] - last_xy[2], math.tau)) > 0.3:
                        last_xy, last_t = xy_yaw, time.time()
                        if self.state == S.STATE_BLOCKED:
                            self.state, self.warning = S.STATE_EXECUTING, ''
                    elif time.time() - last_t > 40.0:
                        break   # 막힘 40초 (10/15 통합 브랜치, 원래 10초): 그 사이 Nav2 복구(costmap 지우기 · 회전 · 대기 · 재계획)가
                                # 상자를 돌아가게 둔다. 10초면 Nav2 진척 검사(10초)와 같아 복구 전에 끊고 점을 건너뜀 (실기 4회차)
                    elif time.time() - last_t > 5.0 and self.state != S.STATE_BLOCKED:
                        self.state, self.warning = S.STATE_BLOCKED, 'BLOCKED'   # 5초: Nav2 가 재계획하는 중
                        k = max(len(left) - fb.number_of_poses_remaining, 0)
                        rest = left[k:]
                        # 여기서는 검사만 (바꾼 점은 쓰지 않으므로 기록도 되돌림)
                        saved = set(self.skipped), set(self.shortened), list(self.blocked_spots)
                        dropped = len(self.drop_unreachable(rest, left[k - 1][1] if k else None)) < len(rest)
                        self.skipped, self.shortened, self.blocked_spots = saved
                        if dropped:
                            break   # 길이 사라진 점이 있으면 10초를 기다리지 않고 바로 뺌
                time.sleep(0.2)
            else:
                res = self.nav.getResult()
                if res == TaskResult.SUCCEEDED:
                    return self.result()
                if res != TaskResult.FAILED:
                    return False, str(res)
            # 막힘 10초 또는 Nav2 실패: 지난 점을 버리고, 남은 점 중 길 없는 점만 뺌. 다 갈 수 있으면(사람·상자 같은 일시 막힘) 지금 향하던 점을 뺌
            self.cancel_nav()
            k = max(len(left) - fb.number_of_poses_remaining, 0) if fb else 0
            before = left[k - 1][1] if k else None
            left = left[k:]
            kept = self.drop_unreachable(left, before)
            if len(kept) == len(left) and not moved:   # 장애물에 막힌 점은 drop_unreachable 이 바꾸거나 뺐음
                kept = left[1:]
            gone = [i for i, _ in left if all(i != j for j, _ in kept)]
            self.get_logger().warn(f'BLOCKED → waypoint {gone} 건너뜀')
            self.skipped.update(gone)
            left = kept
            self.state = S.STATE_EXECUTING
        return self.result()   # 남은 waypoint 가 모두 막힘

    def cancel_nav(self):
        # 취소가 실제로 끝날 때까지 기다림. 바로 다음 goal 을 보내면 bt_navigator 가 늦게 받은 취소를 새 goal 에 적용해
        # 새 작업이 0 초에 실패함 (10/14 시뮬: 다음 작업의 점을 모두 건너뜀)
        self.nav.cancelTask()
        deadline = time.time() + 3.0
        while not self.nav.isTaskComplete() and time.time() < deadline:
            time.sleep(0.05)

    def result(self):
        if not self.skipped and not self.shortened:
            return True, ''
        return False, f'BLOCKED skipped={sorted(self.skipped)} shortened={sorted(self.shortened)}'

    def must_change(self, before, i, w):
        # 중간 점은 장애물 칸일 때만, 골은 앞 점에서의 줄이 막혀도 (골을 줄 위 빈 점으로 줄임)
        return self.blocked(before, w) if i == self.goal_index else self.occupied(w)

    def drop_unreachable(self, left, before=None):
        # 앞 점 → 다음 점 경로가 없는 waypoint 를 뺌 (첫 구간은 지금 위치에서). 플래너 tolerance(0.5 m) 안이면 갈 수 있는 것으로 봄.
        # 장애물에 막힌 점(must_change)은: 중간 점은 뺌, 골은 앞 점(없으면 지금 위치)에서 골로 가는 직선 위의 마지막 빈 점으로
        # 바꾸고 그런 점이 없으면 뺌. 작업의 첫 점이 앞서 막혔던 자리면 뺌 (앞 작업 골을 줄였으면 다음 작업은 그 자리에서
        # 시작하므로). before 는 left 앞에 이미 지난 점 (줄 검사용).
        # 앞 점에서 길이 없으면 지금 위치에서 한 번 더 봄 — 앞 점이 장애물 가장자리면 거기서 출발하는 경로가 막혀
        # 갈 수 있는 다음 점까지 버리던 것 (10/14 시뮬)
        kept, prev = [], None
        saved = self.nav.goal_handle, self.nav.result_future   # getPath 가 이 둘을 덮어써 cancelTask·isTaskComplete 가 주행 goal 을 못 봄 (10/13)
        for i, w in left:
            line_from = prev or before
            if line_from is None and any(math.dist(xy(w), b) < 0.05 for b in self.blocked_spots):
                self.get_logger().warn(f'waypoint {i} 는 앞서 장애물로 못 간 자리 → 뺌')
                self.skipped.add(i)
                continue
            if self.must_change(line_from, i, w):
                self.blocked_spots.append(xy(w))
                start = xy(line_from) if line_from else self.pose[:2] if self.pose else None
                w = self.last_free(start, w) if i == self.goal_index and start else None
                if w is None:
                    self.get_logger().warn(f'waypoint {i} 가 장애물에 막힘 → 뺌')
                    self.skipped.add(i)
                    continue
                self.get_logger().warn(f'골 {i} 가 장애물에 막힘 → 줄 위 ({w.pose.position.x:.2f}, {w.pose.position.y:.2f}) 로 줄임')
                self.shortened.add(i)
            path = self.nav.getPath(prev or w, w, use_start=prev is not None)
            if not (path and path.poses) and prev is not None and not self.occupied(w):
                path = self.nav.getPath(w, w, use_start=False)
            if path and path.poses:
                kept.append((i, w))
                prev = w
            else:
                self.get_logger().warn(f'waypoint {i} 로 가는 길 없음')
                self.skipped.add(i)
        self.nav.goal_handle, self.nav.result_future = saved
        return kept

    def inspect(self, gh, g):
        kind = KIND.get(g.target_ref[:3], 0)
        if not kind:
            self.get_logger().warn(f'{g.target_ref}: KIND 표에 없는 종류 → kind 0 으로 보냄')
        if not self.inspect_cli.wait_for_service(timeout_sec=5.0):
            return False, 'NO_SERVER inspect_facility'
        req = InspectFacility.Request(task_id=g.task_id, run_id=g.run_id, facility_id=g.target_ref, kind=kind)
        fut = self.inspect_cli.call_async(req)
        t0 = time.time()
        while not fut.done():            # 서비스는 취소가 없다 — 작업이 취소되면 응답을 기다리지 않고 나감
            if gh.is_cancel_requested:
                return False, 'CANCELED'
            if time.time() - t0 > self.inspect_timeout:
                return False, f'TIMEOUT inspect_facility {self.inspect_timeout:.0f}s'
            time.sleep(0.1)
        res = fut.result()
        if res is None:
            return False, 'INSPECT_FAILED'
        return res.verdict != InspectionResult.UNCONFIRMED, f'verdict={res.verdict} {res.detail}'

    def measurement_guard(self, target_cell, start_pose):
        if self.battery <= DOCK_PCT:
            return 'LOW_BATTERY'
        if self.pose is None or self.pose[3] > self.pose_std_max:
            return 'POSE_UNCERTAIN'
        if self.static_map is None:
            return 'NO_MAP'
        ox, oy = self.static_map.info.origin.position.x, self.static_map.info.origin.position.y
        actual = (math.floor((self.pose[0] - ox) / self.cell_size),
                  math.floor((self.pose[1] - oy) / self.cell_size))
        if actual != target_cell:
            return 'OUTSIDE_TARGET_CELL'
        if (math.dist(self.pose[:2], start_pose[:2]) > .05
                or abs(math.atan2(math.sin(self.pose[2] - start_pose[2]),
                                  math.cos(self.pose[2] - start_pose[2]))) > .10):
            return 'MOVED_DURING_MEASUREMENT'
        if self.odom is None or time.monotonic() - self.odom[0] > 1:
            return 'NO_FRESH_ODOM'
        if self.odom[1] > .02 or self.odom[2] > .05:
            return 'NOT_STATIONARY'
        return ''

    def measure(self, gh):
        target = re.fullmatch(r'cell:(-?\d+),(-?\d+)', gh.request.target_ref)
        if target is None or self.pose is None:
            return False, 'INVALID_MEASUREMENT_TARGET', None
        cell = tuple(map(int, target.groups()))
        self.stop()
        # 남은 속도로 바로 실패하지 않게 멈출 때까지 최대 2초 기다린 뒤 그 자리를 기준으로 삼는다 (10/15 통합 브랜치)
        until = time.monotonic() + 2
        while time.monotonic() < until and not gh.is_cancel_requested and not (
                self.odom and time.monotonic() - self.odom[0] <= 1 and self.odom[1] <= .02 and self.odom[2] <= .05):
            time.sleep(.1)
        start = self.pose
        # 새 창을 열기 전에 1초간 정지 확인. 측정 중에도 동일한 조건을 계속 확인한다.
        until = time.monotonic() + 1
        while time.monotonic() < until:
            reason = 'CANCELED' if gh.is_cancel_requested else self.measurement_guard(cell, start)
            if reason:
                return False, reason, None
            time.sleep(.1)
        res, reason = self.call_action(
            self.precise_cli, PreciseMeasure.Goal(duration_s=self.precise_duration), gh,
            each=lambda: self.measurement_guard(cell, start))
        if res is None:
            return False, reason, None
        reason = self.measurement_guard(cell, start)
        if reason:
            res.measurement_valid, res.measurement_reason = False, reason
        return res.measurement_valid, res.measurement_reason, res

    def call_action(self, client, goal, gh, on_fb=None, each=None):
        # 다른 노드의 액션을 보내고 결과를 기다림. 작업이 취소되면 그 액션도 취소
        if not client.wait_for_server(timeout_sec=5.0):
            return None, 'NO_SERVER'
        f = client.send_goal_async(goal, feedback_callback=on_fb)
        while not f.done():
            time.sleep(0.05)
        h = f.result()
        if not h.accepted:
            return None, 'REJECTED'
        r = h.get_result_async()
        while not r.done():
            if gh.is_cancel_requested:
                h.cancel_goal_async()
                return None, 'CANCELED'
            if each:
                reason = each()
                if reason:
                    h.cancel_goal_async()
                    return None, reason
            time.sleep(0.1)
        outcome = r.result()
        if outcome.status != GoalStatus.STATUS_SUCCEEDED:
            return None, getattr(outcome.result, 'measurement_reason', '') or 'ACTION_FAILED'
        return outcome.result, ''

    def go_dock(self, gh, waypoints):
        if self.nav.getDockedStatus():
            self.state = S.STATE_DOCKED
            return True, ''
        ok, reason = self.follow(gh, waypoints) if waypoints else self.go_home(gh)
        if not ok:
            return ok, reason
        self.state = S.STATE_DOCKING
        self.nav.dock()   # 결과를 돌려주지 않아 dock 상태로 확인
        if not self.nav.getDockedStatus():
            self.state = S.STATE_IDLE
            return False, 'DOCK_FAILED'
        self.state = S.STATE_DOCKED
        return True, ''

    def go_home(self, gh):
        # dock 명령은 undock 위치에서 DOCK_BACKOFF 만큼 앞(dock 반대쪽)으로 나간 자리에서 내림
        p, q = self.home.pose.position, self.home.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)
        self.nav.goToPose(self.nav.getPoseStamped(
            [p.x + DOCK_BACKOFF * math.cos(yaw), p.y + DOCK_BACKOFF * math.sin(yaw)], math.degrees(yaw)))
        self.wait_nav(gh)   # 쉬는 중 자동 dock 은 gh=None
        if gh and gh.is_cancel_requested:   # cancelTask 는 status 를 안 바꿔 getResult 가 직전 결과(SUCCEEDED)를 줌 → 취소 자리에서 dock 하지 않게
            return False, 'CANCELED'
        res = self.nav.getResult()
        return res == TaskResult.SUCCEEDED, '' if res == TaskResult.SUCCEEDED else str(res)

    def set_tolerance(self, xy, yaw):
        # SimpleGoalChecker 는 실행 중 파라미터 변경을 받는다 (ros2 param set 과 같음)
        req = SetParameters.Request(parameters=[
            Parameter('general_goal_checker.xy_goal_tolerance', value=xy).to_parameter_msg(),
            Parameter('general_goal_checker.yaw_goal_tolerance', value=yaw).to_parameter_msg()])
        if not self.param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('controller_server 파라미터 서비스 없음, 허용 오차 못 바꿈')
            return False
        f = self.param_cli.call_async(req)
        while not f.done():
            time.sleep(0.05)
        ok = all(r.successful for r in f.result().results)
        self.get_logger().info(f'goal 허용 오차 xy={xy} yaw={yaw} ok={ok}')
        return ok

    def wait_nav(self, gh=None):
        while not self.nav.isTaskComplete():
            if gh and gh.is_cancel_requested:   # 작업 취소(E7)면 보정 동작도 바로 멈춤
                self.nav.cancelTask()
                self.stop()
                return
            time.sleep(0.1)

    def cmd_vel_approach(self, gh, goal):
        """Nav2 대신 cmd_vel 로 goal (x, y, yaw)까지 느리게 간다 → (ok, reason). 회피 구역 안 관측 자리로 들어갈 때와
        점검 뒤 들어온 자리로 되돌아 나올 때 쓴다.

        keepout_cmd_vel 이 켜졌을 때만 부른다(기본 꺼짐). Nav2 충돌 감시를 거치지 않으므로 지금 위치에서
        keepout_cmd_vel_max_m 이내인 자리만 · 시간 제한 · 취소되면 멈춘다. 움직임은 approach_twist 가 정한다.
        """
        if self.pose is None or self.odom_pose is None:
            return False, 'CMD_VEL_NO_POSE'
        dist = math.dist(self.pose[:2], goal[:2])
        if dist > self.keepout_cmd_vel_max:
            return False, f'CMD_VEL_TOO_FAR {dist:.2f}m'
        # amcl_pose 는 0.1 m · 0.1 rad 움직여야 갱신되므로, 시작 때 amcl 자세에 odom 변화량을 더해 10 Hz 로 추정한다
        start_map, start_odom = self.pose[:3], self.odom_pose
        end = time.time() + 15.0 + dist / CMD_LIN + 2 * math.tau / CMD_ANG   # 직진 + 회전 두 바퀴 여유 (robot3 QA)
        try:
            while True:
                if gh.is_cancel_requested:
                    return False, 'CANCELED'
                if time.time() > end:
                    return False, 'CMD_VEL_TIMEOUT'
                lin, ang, done = approach_twist(odom_compose(start_map, start_odom, self.odom_pose), goal)
                if done:
                    return True, ''
                m = TwistStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = 'base_link'
                m.twist.linear.x, m.twist.angular.z = lin, ang
                self.cmd_pub.publish(m)
                time.sleep(0.1)
        finally:
            self.stop()

    def stop(self):
        for _ in range(5):
            self.cmd_pub.publish(TwistStamped())
            time.sleep(0.05)


def xy(pose):
    return pose.pose.position.x, pose.pose.position.y


def odom_compose(start_map, start_odom, now_odom):
    """시작 때 지도 자세 start_map 에, odom 으로 잰 그 뒤의 움직임(start_odom → now_odom)을 더한 지금 지도 자세 (x, y, yaw)."""
    dx, dy = now_odom[0] - start_odom[0], now_odom[1] - start_odom[1]
    c, s = math.cos(-start_odom[2]), math.sin(-start_odom[2])
    fx, fy = c * dx - s * dy, s * dx + c * dy                     # 시작 때 로봇 기준 이동량
    c, s = math.cos(start_map[2]), math.sin(start_map[2])
    return (start_map[0] + c * fx - s * fy, start_map[1] + s * fx + c * fy,
            math.remainder(start_map[2] + now_odom[2] - start_odom[2], math.tau))


def approach_twist(pose, goal):
    """cmd_vel 접근 한 걸음: pose · goal = (x, y, yaw) → (linear, angular, done).
    목표 쪽으로 먼저 돌고(20° 넘게 틀어졌으면 제자리 회전), 느리게 직진, 도착하면 목표 방향으로 돈다."""
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    dist = math.hypot(dx, dy)
    if dist > CMD_TOL[0]:
        err = math.remainder(math.atan2(dy, dx) - pose[2], math.tau)
        if abs(err) > 0.35:
            return 0.0, math.copysign(CMD_ANG, err), False
        return min(CMD_LIN, dist), 1.5 * err, False
    err = math.remainder(goal[2] - pose[2], math.tau)
    if abs(err) > CMD_TOL[1]:
        return 0.0, math.copysign(CMD_ANG, err), False
    return 0.0, 0.0, True


def localize(nav, home):
    # setInitialPose 는 공분산 0 이라 AMCL 이 몇 도 틀린 것도 못 고침 (10/12 robot3 6° 틀어짐)
    # → 위치 0.15 m, 각도 10° 불확실성을 넣어 보내고 amcl_pose 가 올 때까지 반복
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose = home.pose
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.15 ** 2
    cov[35] = math.radians(10.0) ** 2
    msg.pose.covariance = cov
    nav.initial_pose_received = False
    since, restarts = time.time(), 0
    while not nav.initial_pose_received:
        # localization bringup 이 bond 4 s 초과로 중단되면 AMCL 이 inactive 로 남아 여기서 영원히 기다림 (10/15 robot3)
        # → 30 s 동안 amcl_pose 가 없고 amcl 이 active 가 아니면 lifecycle_manager_localization 을 RESET · STARTUP (최대 3번)
        if time.time() - since > 30.0 and restarts < 3:
            since = time.time()
            st = node_state(nav, 'amcl')
            if st is None:                               # 서비스를 못 찾음(discovery 느림) — 멀쩡한 amcl 을 RESET 하지 않음
                nav.get_logger().warn('amcl_pose 30 s 없음 · amcl 상태 확인 못 함 (discovery?) — scan · amcl 확인 필요, 계속 기다림')
            elif st != 'active':
                restarts += 1
                nav.get_logger().warn(f'amcl 이 active 아님 — localization bringup 중단으로 보고 다시 띄움 ({restarts}/3)')
                restart_nav2(nav, 'lifecycle_manager_localization')
            else:
                nav.get_logger().warn('amcl 은 active 인데 amcl_pose 30 s 없음 — scan(라이다)이 오는지 확인 필요')
        msg.header.stamp = nav.get_clock().now().to_msg()
        nav.initial_pose_pub.publish(msg)
        # spin_once 는 메시지가 하나라도 오면 바로 돌아온다 (이 노드가 initialpose 도 받음) → 1초 동안은
        # 다시 보내지 않고 기다림. 그러지 않으면 초기 pose 를 수천~수십만 번 보내 AMCL 이 계속 초기화돼
        # amcl_pose 가 안 나오고 기동이 멈춘다 (10/14 robot3: 25만 번)
        deadline = time.time() + 1.0
        while not nav.initial_pose_received and time.time() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.1)


def wait_nav2_active(nav, node_name='bt_navigator'):
    # waitUntilNav2Active(localizer='robot_localization') 와 같이 bt_navigator 가 active 가 될 때까지 기다리되,
    # get_state 한 번에 3초 제한 — 응답을 놓치면 클라이언트를 새로 만들어 다시 부름
    from lifecycle_msgs.srv import GetState
    stuck_since, restarts = None, 0
    while rclpy.ok():
        cli = nav.create_client(GetState, f'{node_name}/get_state')
        if not cli.wait_for_service(timeout_sec=2.0):
            nav.get_logger().info(f'{node_name}/get_state 서비스 대기 중')
            nav.destroy_client(cli)
            stuck_since = stuck_since or time.time()     # 서비스를 오래 못 찾아도 아래와 같이 셈 (120 s)
            if time.time() - stuck_since > 120.0 and restarts < 3:
                restarts += 1
                nav.get_logger().warn(f'{node_name} 서비스 120초 없음 — Nav2 다시 띄움 ({restarts}/3)')
                restart_nav2(nav)
                stuck_since = None
            continue
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(nav, fut, timeout_sec=3.0)
        state = fut.result().current_state.label if fut.done() and fut.result() is not None else None
        nav.destroy_client(cli)
        if state == 'active':
            nav.get_logger().info('Nav2 is ready for use!')
            return
        nav.get_logger().info(f'{node_name} 상태 {state or "응답 없음 — 다시 물음"}')
        # lifecycle_manager 가 bond 4 s 초과로 "Aborting bringup" 하면 inactive 로 영원히 남음 (10/15 robot3) → RESET · STARTUP
        if state in ('inactive', 'unconfigured'):        # RESET 뒤 STARTUP 이 configure 전에 실패하면 unconfigured 로 남음
            stuck_since = stuck_since or time.time()
            if time.time() - stuck_since > 30.0 and restarts < 3:
                restarts += 1
                nav.get_logger().warn(f'{node_name} 30초 넘게 {state} — Nav2 bringup 중단으로 보고 다시 띄움 ({restarts}/3)')
                restart_nav2(nav)
                stuck_since = None
        elif state is not None:                         # 응답 없음(None)은 타이머를 유지 — 섞여 들어와도 30초를 채움
            stuck_since = None                          # (activating 등 전환 중)
        time.sleep(1.0)


def node_state(nav, node_name):
    from lifecycle_msgs.srv import GetState
    cli = nav.create_client(GetState, f'{node_name}/get_state')
    state = None
    if cli.wait_for_service(timeout_sec=2.0):
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(nav, fut, timeout_sec=3.0)
        if fut.done() and fut.result() is not None:
            state = fut.result().current_state.label
    nav.destroy_client(cli)
    return state


def restart_nav2(nav, manager='lifecycle_manager_navigation'):
    from nav2_msgs.srv import ManageLifecycleNodes
    cli = nav.create_client(ManageLifecycleNodes, f'{manager}/manage_nodes')
    if not cli.wait_for_service(timeout_sec=3.0):
        nav.get_logger().warn(f'{manager}/manage_nodes 서비스 없음')
        nav.destroy_client(cli)
        return
    ok = False
    for cmd in (ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP):
        if cmd == ManageLifecycleNodes.Request.STARTUP and not ok:
            nav.get_logger().warn(f'{manager} RESET 실패/응답 없음 — STARTUP 은 다음 주기로 미룸')
            break
        req = ManageLifecycleNodes.Request()
        req.command = cmd
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(nav, fut, timeout_sec=60.0)
        ok = fut.done() and fut.result() is not None and fut.result().success
        nav.get_logger().info(f'manage_nodes {"RESET" if cmd == ManageLifecycleNodes.Request.RESET else "STARTUP"} → {"성공" if ok else "실패/응답 없음"}')
    nav.destroy_client(cli)
    refresh_action_clients(nav)


def refresh_action_clients(nav):
    # RESET · STARTUP 뒤 Nav2 액션 서버가 새로 만들어지면 기존 클라이언트가 다시 매칭되지 않아
    # "'Spin' action server not available" 로 영원히 기다림 (10/15 robot3, Discovery Server) → 같은 이름 · 타입으로 다시 만듦
    from rclpy.action import ActionClient as AC
    n = 0
    for attr, cli in list(vars(nav).items()):
        if isinstance(cli, AC):
            kind, name = cli._action_type, cli._action_name
            try:
                cli.destroy()
            except Exception:   # 이미 없어진 클라이언트
                pass
            setattr(nav, attr, AC(nav, kind, name))
            n += 1
    nav.get_logger().info(f'Nav2 액션 클라이언트 {n}개 다시 만듦')


def start_lidar(nav):
    # TurtleBot4 power_saver 가 Dock 에서 끈 RPLIDAR 모터를 켬 (rplidar_ros 의 {ns}/start_motor, std_srvs/Empty).
    # 서비스 발견이 늦을 수 있어(10/15 robot4: 5 s 안에 못 찾아 포기 → scan 0 Hz → Nav2 bringup 중단) 60 s 까지 찾고,
    # 부른 뒤 scan 이 실제로 오는지 보고 안 오면 다시 부름 (최대 5번)
    from std_srvs.srv import Empty
    from sensor_msgs.msg import LaserScan
    got = []
    sub = nav.create_subscription(LaserScan, 'scan', lambda m: got.append(1), qos_profile_sensor_data)
    end_t = time.time() + 4.0                          # 이미 돌고 있으면 부르지 않음 — 도는 모터에 start_motor 를 다시 보내면
    while time.time() < end_t and not got:             # 드라이버가 죽는 것으로 보임 (10/15 robot4 · robot3, undock 뒤 rplidar 노드 사라짐)
        rclpy.spin_once(nav, timeout_sec=0.2)
    if got:
        nav.get_logger().info('Dock 위 — 라이다가 이미 돎, start_motor 부르지 않음 (작업이 오면 undock)')
        nav.destroy_subscription(sub)
        return
    cli = nav.create_client(Empty, 'start_motor')
    deadline = time.time() + 60.0
    while not cli.wait_for_service(timeout_sec=2.0):
        if time.time() > deadline:
            nav.get_logger().warn('start_motor 서비스 60 s 동안 없음 — 라이다가 꺼져 있으면 Nav2 bringup 이 멈출 수 있음')
            nav.destroy_subscription(sub)
            return
        nav.get_logger().info('start_motor 서비스 기다리는 중')
    for n in range(1, 6):
        fut = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(nav, fut, timeout_sec=5.0)
        got.clear()
        end_t = time.time() + 6.0
        while time.time() < end_t and not got:
            rclpy.spin_once(nav, timeout_sec=0.2)
        if got:
            nav.get_logger().info(f'Dock 위 — undock 하지 않고 라이다를 켬 (작업이 오면 undock), scan 확인 ({n}번째)')
            break
        nav.get_logger().warn(f'start_motor 뒤 scan 이 안 옴 — 다시 부름 ({n}/5)')
    nav.destroy_subscription(sub)


def settle(nav):
    if not nav.spin_client.wait_for_server(timeout_sec=20.0):
        refresh_action_clients(nav)                    # 서버가 다시 만들어진 뒤 매칭이 안 된 경우 (10/15 robot3)
        if not nav.spin_client.wait_for_server(timeout_sec=20.0):
            nav.get_logger().warn('Spin 서버 없음 — settle 건너뜀 (AMCL 방향 보정 없이 진행)')
            return
    _settle(nav)


def _settle(nav):
    # 제자리에서 좌우로 0.26 rad(15°) 돌려 AMCL 이 라이다로 방향을 맞추게 함 (10/12 시험: ±0.5 두 번과 차이 없음)
    for yaw in (0.26, -0.26):
        nav.spin(yaw)
        while not nav.isTaskComplete():
            time.sleep(0.1)
        time.sleep(1.0)   # spin 은 목표 각도에서 바로 끝나지만 로봇은 아직 도는 중 → 멈춘 뒤 반대로 (바로 돌면 한쪽으로 치우침, 10/13)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = TurtleBot4Navigator()
    xyyaw = [nav.declare_parameter(k, math.nan).value for k in ('x', 'y', 'yaw')]
    if any(map(math.isnan, xyyaw)):
        if nav.get_namespace() not in HOME:
            raise SystemExit(f'HOME 에 {nav.get_namespace()} 없음 — -p x y yaw 를 주세요')
        xyyaw = HOME[nav.get_namespace()]
    home = nav.getPoseStamped(xyyaw[:2], xyyaw[2])
    # stay_docked:=true 일 때만: 켤 때 undock 하지 않고 도크에서 라이다만 켜 Nav2 를 띄운다 (Dock 에서는 power_saver 가
    # 라이다를 꺼 AMCL 이 map TF 를 못 냄). 작업이 오면 execute() 가 undock → home 다시 → settle 로 맞춘다
    # getDockedStatus 는 dock_status 가 올 때까지 로그 없이 기다림 (Pi 를 막 켰을 때 수십 초) — 기다리는 중임을 남김
    nav.get_logger().info('dock_status 기다리는 중 (Pi 의 create3 가 올라오면 이어 감)')
    docked = nav.getDockedStatus()
    nav.get_logger().info(f'dock_status 받음: {"도크 위" if docked else "도크 밖"}')
    # 10/15 오후: 도크 위에서 start_motor 를 부르면 rplidar 드라이버가 죽는 일이 4번 재현 (robot4 ×3, robot3 ×1, 호출 직후 노드 사라짐)
    # → 기본은 오전 방식(켤 때 undock, 1~5회차 라이다 문제 0번). -p stay_docked:=true 일 때만 도크에 머물며 라이다를 켬
    stay = nav.declare_parameter('stay_docked', False).value
    if docked and not stay:
        for n in range(1, 4):                          # undock 이 거절 · 실패해도 navigator 는 완료로 봄 → 도크 상태로 다시 확인
            nav.undock()
            end_t = time.time() + 1.5                  # dock_status 가 갱신될 때까지 (마지막 값이 아직 True 일 수 있음)
            while time.time() < end_t:
                rclpy.spin_once(nav, timeout_sec=0.1)
            if not nav.getDockedStatus():
                break
            nav.get_logger().warn(f'undock 뒤에도 도크 위 — 다시 undock ({n}/3)')
        docked = nav.getDockedStatus()
    if docked:
        start_lidar(nav)
    # undock 상태로 시작해도 home 을 줌. Nav2 bringup 이 map TF 를 기다리므로 Nav2 보다 먼저 보냄
    # 도크 위에서는 도크를 보고 있어 HOME(undock 뒤, 도크 반대쪽을 봄)과 방향이 180° 다르다 — 위치는 HOME 과 몇 cm 차이
    # (10/15 실측 robot3 도킹 (2.85, 4.64, 0°) · HOME (2.81, 4.61, 180°)). 움직이지 않으면 AMCL 이 방향을 못 고치므로 뒤집어 줌
    localize(nav, nav.getPoseStamped(xyyaw[:2], xyyaw[2] + 180.0) if docked else home)
    wait_nav2_active(nav)   # nav.waitUntilNav2Active 는 get_state 응답을 시간 제한 없이 기다려 놓치면 영원히 멈춤 (10/15 robot3 두 번)
    if not docked:
        settle(nav)   # 도크 위에서는 돌지 않음
    node = MissionExecutor(nav, home)
    if docked:
        node.state = S.STATE_DOCKED
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        while rclpy.ok():
            # 놀고 있을 때 배터리 DOCK_PCT 이하면 스스로 dock (작업 중이면 follow 가 처리)
            if not node.busy and node.battery <= DOCK_PCT and node.state != S.STATE_DOCKED:
                node.busy = True
                node.go_dock(None, [])
                node.busy = False
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if node.busy:
            nav.cancelTask()
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
