import math
import re
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scv_msgs.action import ExecuteTask
from scv_msgs.msg import Alert, Candidate, InspectionResult, RobotStatus
from scv_msgs.srv import Reassign
from scv_survey.cellmap_store import open_store, read_map_yaml
from scv_survey.heatmap import cell_center, cell_index, keepout_mask
from scv_survey.remeasurement import MIN_SAMPLES, classify
from std_srvs.srv import Trigger

from .manager_core import (
    ManagerCore,
    PatrolPhase,
    RobotStatusSnapshot,
    RunState,
    TERMINAL_TASK_STATES,
    TaskState,
    Waypoint,
    blocked_cells,
    mask_cells,
    remeasure_targets,
)
from .route_builder import RouteBuilder, wall_points

# DB tasks 의 상태 · 종류 이름 (01_schema.sql CHECK)
TASK_DB_STATUS = {
    TaskState.PENDING: "PENDING", TaskState.ASSIGNED: "ACTIVE", TaskState.RUNNING: "ACTIVE",
    TaskState.SUCCEEDED: "DONE", TaskState.BLOCKED: "FAILED", TaskState.FAILED: "FAILED",
    TaskState.CANCELED: "CANCELED",
}
TASK_DB_TYPE = {
    ExecuteTask.Goal.SURVEY_SEGMENT: "SURVEY_SEGMENT", ExecuteTask.Goal.FACILITY_CHECK: "FACILITY_CHECK",
    ExecuteTask.Goal.REINSPECTION: "REINSPECTION", ExecuteTask.Goal.DOCK: "DOCK",
}


STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

CANDIDATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

INSPECTION_RESULT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


MASK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class InspectionManager(Node):
    def __init__(self):
        super().__init__("inspection_manager")

        self.declare_parameter("robot_ids", ["robot4", "robot3"])
        self.declare_parameter("zone_ids", ["Z1", "Z2"])
        self.declare_parameter("offline_timeout_sec", 29.0)
        self.declare_parameter("require_robot_ready", True)

        bringup_share = Path(get_package_share_directory("scv_bringup"))
        self.declare_parameter(
            "zones_file",
            str(bringup_share / "config" / "zones.yaml"),
        )
        self.declare_parameter(
            "facilities_file",
            str(bringup_share / "config" / "facilities.yaml"),
        )

        # 칸 맵(RSSI 측정 결과) 저장소와 회피 마스크 (묶음 시험)
        self.declare_parameter("cellmap_backend", "file")
        self.declare_parameter("cellmap_dir", "~/scv_cellmap")
        self.declare_parameter("db_dsn_env", "SCV_DB_DSN")
        self.declare_parameter("ap_ssid", "")
        self.declare_parameter(
            "map_yaml",
            str(bringup_share / "maps" / "testbed_v1.yaml"),
        )
        self.declare_parameter("cell_size", 0.5)
        self.declare_parameter("wall_bounds", [0.08, -0.05, 3.18, 6.48])
        self.declare_parameter("wall_clearance_m", 0.45)
        # 로봇 반경 0.19 m + 벽과의 간격 0.10 m
        self.declare_parameter("obstacle_clearance_m", 0.30)
        # 측정 줄 간격. 칸(cell_size)보다 작으면 칸 한 줄을 여러 번 지난다 (0.25 = 칸마다 두 줄)
        self.declare_parameter("survey_spacing_m", 0.25)
        # 모서리 처리: angle = corner_break_deg 이상 꺾는 점을 작업 끝점으로 (10/14 채택, PLAN N-10).
        # 지움 반경은 주행 설정(BT RemovePassedGoals)과 같게
        self.declare_parameter("corner_mode", "angle")
        self.declare_parameter("corner_break_deg", 90.0)
        # 측정 줄을 이 간격의 중간 점으로 나눔 (0 이면 안 나눔). 막힌 점만 건너뛰고 나머지를 잼 (10/14)
        self.declare_parameter("survey_split_m", 0.5)
        self.declare_parameter("passed_goal_radius_m", 0.35)
        self.declare_parameter("shadow_dbm", -72.0)
        self.declare_parameter("max_remeasure_per_robot", 0)  # 0: 전체 후보
        # 1차 측정에서 미검출된 칸만 10~15초 정지 재측정 (선택 2차 작업, 10/15 PM). 약한 신호 칸 · 미측정 메우기는 넣지 않는다
        self.declare_parameter("include_unmeasured", False)
        self.declare_parameter("dock_keepout_radius_m", 0.6)
        # 회피 구역 안 설비도 점검 작업을 보내 로봇이 cmd_vel 로 접근하게 (로봇 mission_executor keepout_cmd_vel 과 함께, 10/16 · 기본 꺼짐)
        self.declare_parameter("keepout_facility_cmd_vel", False)
        self.declare_parameter("writer_node", "cell_map_writer")
        self.declare_parameter("commit_timeout_sec", 10.0)
        self.declare_parameter("close_stale_run", False)

        robot_ids = list(self.get_parameter("robot_ids").value)
        zone_ids = list(self.get_parameter("zone_ids").value)
        self.offline_timeout_sec = float(
            self.get_parameter("offline_timeout_sec").value
        )
        self.require_robot_ready = bool(
            self.get_parameter("require_robot_ready").value
        )

        if not robot_ids or len(robot_ids) != len(zone_ids):
            raise ValueError(
                "robot_ids와 zone_ids는 비어 있지 않고 "
                "항목 수가 같아야 합니다."
            )
        if len(set(robot_ids)) != len(robot_ids):
            raise ValueError("robot_ids에 중복된 로봇이 있습니다.")
        if len(set(zone_ids)) != len(zone_ids):
            raise ValueError("zone_ids에 중복된 구역이 있습니다.")

        self.zone_by_robot = dict(zip(robot_ids, zone_ids))

        self.core = ManagerCore(
            robot_ids=robot_ids,
            offline_timeout_sec=self.offline_timeout_sec,
        )
        self.alert_publisher = self.create_publisher(
            Alert,
            "/inspection/alerts",
            10,
        )
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.map = read_map_yaml(str(self.get_parameter("map_yaml").value))
        self.route_builder = RouteBuilder(
            zones_file=str(self.get_parameter("zones_file").value),
            facilities_file=str(
                self.get_parameter("facilities_file").value
            ),
            wall_bounds=tuple(self.get_parameter("wall_bounds").value),
            wall_clearance=float(
                self.get_parameter("wall_clearance_m").value
            ),
            origin=self.map.origin,
            walls=wall_points(self.map),
            obstacle_clearance=float(
                self.get_parameter("obstacle_clearance_m").value
            ),
            cell_size=self.cell_size,
            line_spacing=float(self.get_parameter("survey_spacing_m").value),
            corner_mode=str(self.get_parameter("corner_mode").value),
            passed_goal_radius=float(self.get_parameter("passed_goal_radius_m").value),
            corner_break_deg=float(self.get_parameter("corner_break_deg").value),
            split=float(self.get_parameter("survey_split_m").value) or None,
        )

        # 노드를 켜자마자 빈 마스크를 낸다. 주행의 keepout_layer 는
        # 마스크를 한 번 받아야 costmap 이 준비된다.
        self.mask_publishers = {
            robot_id: self.create_publisher(
                OccupancyGrid,
                f"/{robot_id}/keepout_mask",
                MASK_QOS,
            )
            for robot_id in robot_ids
        }
        for robot_id in robot_ids:
            self.publish_mask(robot_id, set())

        writer_node = str(self.get_parameter("writer_node").value)
        self.writer_reset_client = self.create_client(
            Trigger,
            f"/{writer_node}/reset",
        )
        self.writer_commit_clients = {
            zone_id: self.create_client(
                Trigger,
                f"/{writer_node}/commit_{zone_id}",
            )
            for zone_id in zone_ids
        }
        self.commit_futures = []
        self.commit_deadline = 0.0

        # 저장소(DB)를 못 열어도 노드는 살려 둔다. 위에서 낸 빈 마스크가 없으면
        # 주행의 costmap 이 준비되지 않기 때문이다. 순찰 시작만 거절한다.
        try:
            self.store = open_store(
                str(self.get_parameter("cellmap_backend").value),
                str(self.get_parameter("map_yaml").value),
                self.cell_size,
                str(self.get_parameter("cellmap_dir").value),
                str(self.get_parameter("db_dsn_env").value),
                str(self.get_parameter("ap_ssid").value),
            )
        except (RuntimeError, ImportError) as exc:
            self.store = None
            self.get_logger().error(f"칸 맵 저장소를 열지 못했습니다: {exc}")
            self.publish_alert(
                level=Alert.LEVEL_ERROR,
                code="DB_UNAVAILABLE",
                robot_id="",
                message=f"칸 맵 저장소를 열지 못했습니다: {exc}",
            )
        # 관제를 다시 띄우면 DB 에 running 회차가 남아 웹 「순찰 시작」이 막힘 (10/15 실기 run 4) — 기동 때 닫음
        if self.store is not None and bool(self.get_parameter("close_stale_run").value) \
                and hasattr(self.store, "close_stale_runs"):
            try:
                for run_id in self.store.close_stale_runs():
                    self.get_logger().warning(f"관제 재시작 — 남아 있던 run_id={run_id} 를 aborted 로 닫음")
            except RuntimeError as exc:
                self.get_logger().error(f"남은 회차를 닫지 못했습니다: {exc}")

        self.execute_task_clients = {
            robot_id: ActionClient(
                self,
                ExecuteTask,
                f"/{robot_id}/execute_task",
            )
            for robot_id in robot_ids
        }

        self.active_goals = {}
        self.canceling_task_ids = set()
        self.seen_candidate_ids = set()
        self.pending_candidates = {}
        self.pending_remeasure_results = {}
        self.confirmed_cells = set()
        # 이번 회차 측정 작업에서 장애물로 못 간 칸 (mission_executor 결과 reason). 미측정이면 채우기(재측정 후보)에서 뺀다
        self.blocked_cells = set()
        self.seen_inspection_results = set()
        self.unavailable_action_servers = set()
        self.unready_robot_reasons = {}

        self.status_subscriptions = [
            self.create_subscription(
                RobotStatus,
                f"/{robot_id}/status",
                lambda msg, robot_id=robot_id: self.on_robot_status(
                    robot_id,
                    msg,
                ),
                STATUS_QOS,
            )
            for robot_id in robot_ids
        ]
        self.candidate_subscription = self.create_subscription(
            Candidate,
            "/inspection/candidates",
            self.on_candidate,
            CANDIDATE_QOS,
        )
        self.inspection_result_subscriptions = [
            self.create_subscription(
                InspectionResult,
                f"/{robot_id}/inspect/result",
                self.on_inspection_result,
                INSPECTION_RESULT_QOS,
            )
            for robot_id in robot_ids
        ]

        self.start_service = self.create_service(
            Trigger,
            "/inspection/start",
            self.on_start,
        )
        self.estop_service = self.create_service(
            Trigger,
            "/inspection/estop",
            self.on_estop,
        )
        self.abort_service = self.create_service(      # 10/15: 웹 「순찰 종료」 — 회차를 aborted 로 닫음
            Trigger,
            "/inspection/abort",
            self.on_abort,
        )
        self.resume_service = self.create_service(
            Trigger,
            "/inspection/resume",
            self.on_resume,
        )
        self.reassign_service = self.create_service(
            Reassign,
            "/inspection/reassign",
            self.on_reassign,
        )

        self.watchdog_timer = self.create_timer(
            1.0,
            self.check_robot_timeouts,
        )
        self.dispatch_timer = self.create_timer(
            0.5,
            self.dispatch_next_task,
        )
        # 작업 줄을 DB tasks 에 쓴다 (10/14 ROS 통합 — 작업의 원본은 이 노드, 웹은 읽기만). 파일 저장소면 안 함
        self.synced_tasks = {}
        self.task_sync_timer = self.create_timer(1.0, self.sync_tasks)

        self.get_logger().info(
            "inspection_manager 시작: "
            + ", ".join(
                f"{robot_id}={zone_id}"
                for robot_id, zone_id in self.zone_by_robot.items()
            )
        )

    def on_robot_status(self, robot_id: str, msg: RobotStatus) -> None:
        status = RobotStatusSnapshot(
            state=msg.state,
            battery_percent=msg.battery_percent,
            accepting_tasks=msg.accepting_tasks,
            task_id=msg.task_id,
            run_id=msg.run_id,
            task_type=msg.task_type,
            progress=msg.progress,
            warning=msg.warning,
        )
        self.core.update_robot(
            robot_id=robot_id,
            now=time.monotonic(),
            status=status,
        )

    def check_robot_timeouts(self) -> None:
        for robot_id in self.core.check_offline(now=time.monotonic()):
            failed_task_ids = self.core.fail_active_tasks_for_robot(
                robot_id,
                reason="OFFLINE",
            )

            for task_id in failed_task_ids:
                goal_handle = self.active_goals.pop(task_id, None)

                if goal_handle is not None:
                    goal_handle.cancel_goal_async()

                self.publish_task_failure(
                    robot_id,
                    task_id,
                    "로봇이 OFFLINE 되어 작업을 종료했습니다.",
                )

            self.get_logger().error(
                f"{robot_id}: RobotStatus 수신 중단"
            )
            self.publish_alert(
                level=Alert.LEVEL_ERROR,
                code="E3",
                robot_id=robot_id,
                message=(
                    f"{self.offline_timeout_sec:g}초 동안 "
                    "RobotStatus가 수신되지 않았습니다."
                ),
            )

        self.complete_run_if_ready()

    def on_start(self, request, response):
        del request

        if self.core.run_state in {RunState.RUNNING, RunState.ESTOPPED}:
            response.success = False
            response.message = "이미 진행 중인 순찰이 있습니다."
            return response

        if self.store is None:
            response.success = False
            response.message = "칸 맵 저장소를 열지 못했습니다."
            return response

        try:
            survey_routes = self.route_builder.build_survey_routes()
        except (OSError, KeyError, TypeError, ValueError) as exc:
            response.success = False
            response.message = f"경로 설정을 읽을 수 없습니다: {exc}"
            self.get_logger().error(response.message)
            return response

        if not survey_routes:
            response.success = False
            response.message = "zones.yaml에 측정 경로가 필요합니다."
            return response

        routes_by_zone = {
            route.zone_id: route
            for route in survey_routes
        }
        missing_zone_ids = [
            zone_id
            for zone_id in self.zone_by_robot.values()
            if zone_id not in routes_by_zone
        ]

        if missing_zone_ids:
            response.success = False
            response.message = (
                "zones.yaml에 다음 구역이 없습니다: "
                + ", ".join(missing_zone_ids)
            )
            return response

        try:
            run_id = self.store.open_run(
                {
                    "".join(filter(str.isdigit, zone_id)) or zone_id: robot_id
                    for robot_id, zone_id in self.zone_by_robot.items()
                },
                bool(self.get_parameter("close_stale_run").value),
            )
        except RuntimeError as exc:
            response.success = False
            response.message = f"회차를 열 수 없습니다: {exc}"
            self.get_logger().error(response.message)
            return response

        run_id = self.core.start_run(run_id=run_id)
        self.blocked_cells = set()
        self.writer_reset_client.call_async(Trigger.Request())
        for robot_id in self.core.robots:
            self.publish_mask(robot_id, set())
        self.seen_candidate_ids.clear()
        self.pending_candidates.clear()
        self.pending_remeasure_results.clear()
        self.confirmed_cells.clear()
        self.seen_inspection_results.clear()

        for robot_id, zone_id in self.zone_by_robot.items():
            route = routes_by_zone[zone_id]
            # 구간마다 작업 하나. 구간의 마지막 점은 끝까지 도착한다
            segments = route.segments or (route.waypoints,)
            for index, waypoints in enumerate(segments, start=1):
                name = "survey" if len(segments) == 1 else f"survey-{index}"
                self.core.create_task(
                    task_id=f"run-{run_id}-{robot_id}-{name}",
                    run_id=run_id,
                    task_type=ExecuteTask.Goal.SURVEY_SEGMENT,
                    priority=1,
                    target_ref=route.zone_id,
                    preferred_robot_id=robot_id,
                    allowed_robot_ids=(robot_id,),
                    waypoints=waypoints,
                )
            self.core.create_task(
                task_id=f"run-{run_id}-{robot_id}-survey-dock",
                run_id=run_id,
                task_type=ExecuteTask.Goal.DOCK,
                priority=1,
                target_ref="dock",
                preferred_robot_id=robot_id,
                allowed_robot_ids=(robot_id,),
                waypoints=(),
            )

        task_count = len(self.core.tasks)
        response.success = True
        response.message = f"run_id={run_id}, 작업 {task_count}건 생성"
        self.publish_alert(
            level=Alert.LEVEL_INFO,
            code="RUN_STARTED",
            robot_id="",
            ref=str(run_id),
            message=response.message,
        )
        self.get_logger().info(response.message)
        return response

    def sync_tasks(self) -> None:
        """바뀐 작업만 DB tasks 에 upsert (상태 · 로봇 · 진행률 · 사유). 실패하면 다음 주기에 다시"""
        if not hasattr(self.store, "sync_tasks"):
            return
        rows, keys = [], {}
        for task in list(self.core.tasks.values()):
            robot_id = task.robot_id or task.preferred_robot_id
            if not robot_id:
                continue
            status = TASK_DB_STATUS[task.state]
            key = (status, robot_id, round(task.progress, 2), task.reason)
            if self.synced_tasks.get(task.task_id) == key:
                continue
            keys[task.task_id] = key
            task_type = TASK_DB_TYPE.get(task.task_type, "SURVEY_SEGMENT")
            rows.append({
                "task_id": task.task_id, "run_id": task.run_id, "robot_id": robot_id, "type": task_type,
                "target_ref": task.target_ref or (None if task_type == "DOCK" else task.task_id),
                "waypoints": [{"x": w.x, "y": w.y, "yaw": w.yaw} for w in task.waypoints],
                "status": status, "progress": max(0.0, min(1.0, task.progress)), "reason": task.reason or None,
                "result_ok": {"DONE": True, "FAILED": False}.get(status),
            })
        if not rows:
            return
        try:
            self.store.sync_tasks(rows)
        except RuntimeError as exc:
            self.get_logger().warning(f"작업 줄 DB 저장 실패 — 다음 주기에 다시: {exc}")
            return
        self.synced_tasks.update(keys)

    def retry_remeasure_results(self) -> None:
        """저장 못 한 재측정 결과를 같은 값으로 다시 저장 (다시 재지 않음). 순찰은 멈추지 않는다 (10/15 PM)."""
        for task_id, (robot_id, result) in list(self.pending_remeasure_results.items()):
            task = self.core.tasks.get(task_id)
            if task is None or self.apply_remeasure(robot_id, task, result):
                self.pending_remeasure_results.pop(task_id, None)

    def dispatch_next_task(self) -> None:
        if self.core.run_state != RunState.RUNNING:
            return
        if self.pending_remeasure_results:
            self.retry_remeasure_results()
        if self.core.patrol_phase in {
            PatrolPhase.WAITING_CANDIDATES,
            PatrolPhase.WAITING_KEEPOUT,
        }:
            self.advance_waiting_phase()
            return

        for robot_id in self.core.robots:
            task = self.core.next_pending_task(robot_id)
            if task is None:
                continue

            ready, reason = self.robot_readiness(robot_id)
            status = self.core.robots[robot_id].status
            # 로봇 자신이 하는 일이 없을 때만 (status.task_id 비어 있음) — 점검 판정을 기다리는 중에 끼어들면
            # 그 작업이 취소되고, 동시에 보낸 DOCK 도 executor 안에서 함께 취소된다 (10/15 5회차 SHT-02 · facility-dock)
            if not ready and status is not None and self.core.robots[robot_id].online \
                    and not status.accepting_tasks and not status.task_id \
                    and status.state in {RobotStatus.STATE_IDLE, RobotStatus.STATE_DOCKED}:
                # mission_executor 는 배터리가 낮으면 DOCK 만 받는다. 남은 작업을 기다리면 DOCK 도 영영 안 나가므로
                # 그 로봇 몫을 실패로 닫고 바로 귀환시킨다 (10/15 실기: robot4 27% 에서 서서 교착)
                dock = self.core.abandon_for_dock(
                    robot_id, ExecuteTask.Goal.DOCK,
                    f"LOW_BATTERY ({status.battery_percent:.0f}%)")
                if dock is not None and self.execute_task_clients[robot_id].server_is_ready():
                    self.get_logger().warning(
                        f"{robot_id}: 작업을 받지 않음({reason}) → 남은 작업을 닫고 귀환")
                    self.publish_alert(level=Alert.LEVEL_WARN, code="E2", robot_id=robot_id,
                                       ref=dock.task_id, message=f"배터리 부족 귀환 ({status.battery_percent:.0f}%)")
                    self.send_task(dock.task_id, robot_id)
                    continue
            if not ready:
                previous_reason = self.unready_robot_reasons.get(robot_id)

                if previous_reason != reason:
                    self.get_logger().warning(
                        f"{robot_id}: 작업 배정 대기 - {reason}"
                    )
                    self.unready_robot_reasons[robot_id] = reason
                continue

            self.unready_robot_reasons.pop(robot_id, None)

            client = self.execute_task_clients[robot_id]
            if not client.server_is_ready():
                if robot_id not in self.unavailable_action_servers:
                    self.get_logger().warning(
                        f"{robot_id}: ExecuteTask 액션 서버 대기 중"
                    )
                    self.unavailable_action_servers.add(robot_id)
                continue

            self.unavailable_action_servers.discard(robot_id)
            self.send_task(task.task_id, robot_id)

        self.complete_run_if_ready()

    def robot_readiness(self, robot_id: str) -> tuple[bool, str]:
        if not self.require_robot_ready:
            return True, ""

        robot = self.core.robots[robot_id]

        if robot.status is None:
            return False, "RobotStatus가 아직 수신되지 않음"

        if not robot.online:
            return False, "로봇이 OFFLINE 상태임"

        if not robot.status.accepting_tasks:
            return (
                False,
                "accepting_tasks=false "
                f"(battery={robot.status.battery_percent:.1f}%)",
            )

        ready_states = {
            RobotStatus.STATE_IDLE,
            RobotStatus.STATE_DOCKED,
        }

        if robot.status.state not in ready_states:
            return (
                False,
                f"작업을 받을 수 없는 상태임(state={robot.status.state})",
            )

        return True, ""

    def send_task(self, task_id: str, robot_id: str) -> None:
        task = self.core.tasks[task_id]
        goal = ExecuteTask.Goal()
        goal.task_id = task.task_id
        goal.run_id = task.run_id
        goal.type = task.task_type
        goal.priority = task.priority
        goal.waypoints = [
            self.to_pose_stamped(waypoint)
            for waypoint in task.waypoints
        ]
        goal.target_ref = task.target_ref

        self.core.assign_task(task_id, robot_id)
        self.get_logger().info(
            f"{robot_id}에 작업 전송: {task_id} ({task.target_ref})"
        )

        send_future = self.execute_task_clients[robot_id].send_goal_async(
            goal,
            feedback_callback=lambda feedback, rid=robot_id, tid=task_id: (
                self.on_task_feedback(rid, tid, feedback)
            ),
        )
        send_future.add_done_callback(
            lambda future, rid=robot_id, tid=task_id: (
                self.on_goal_response(rid, tid, future)
            )
        )

    def on_goal_response(self, robot_id: str, task_id: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if task_id in self.canceling_task_ids:
                self.canceling_task_ids.discard(task_id)
                return
            self.core.finish_task(
                task_id,
                ok=False,
                reason="SEND_FAILED",
            )
            self.publish_task_failure(
                robot_id,
                task_id,
                f"작업 전송 실패: {exc}",
            )
            return

        task = self.core.tasks.get(task_id)
        if task is None or task.state != TaskState.ASSIGNED:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            self.canceling_task_ids.discard(task_id)
            return

        if not goal_handle.accepted:
            if task_id in self.canceling_task_ids:
                self.canceling_task_ids.discard(task_id)
                return
            self.core.finish_task(
                task_id,
                ok=False,
                reason="GOAL_REJECTED",
            )
            self.publish_task_failure(
                robot_id,
                task_id,
                "로봇이 작업을 거절했습니다.",
            )
            return

        self.active_goals[task_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, rid=robot_id, tid=task_id: (
                self.on_task_result(rid, tid, result)
            )
        )

        task = self.core.tasks[task_id]
        if (
            self.core.run_state == RunState.ESTOPPED
            or task.state == TaskState.CANCELED
        ):
            goal_handle.cancel_goal_async()
            return

        self.core.start_task(task_id)
        self.get_logger().info(f"{robot_id}: 작업 승인됨: {task_id}")

    def on_task_feedback(
        self,
        robot_id: str,
        task_id: str,
        feedback_message,
    ) -> None:
        task = self.core.tasks.get(task_id)
        if task is None or task.state not in {
            TaskState.ASSIGNED,
            TaskState.RUNNING,
        }:
            return

        feedback = feedback_message.feedback
        self.core.update_task_progress(task_id, feedback.progress, feedback.waypoint_index)
        self.get_logger().info(
            f"{robot_id}: {task_id} 진행률 "
            f"{feedback.progress * 100:.0f}% "
            f"(waypoint={feedback.waypoint_index})"
        )

    def on_task_result(self, robot_id: str, task_id: str, future) -> None:
        self.active_goals.pop(task_id, None)
        task = self.core.tasks.get(task_id)
        if task is None or task.state == TaskState.CANCELED:
            return

        if task_id in self.canceling_task_ids:
            self.canceling_task_ids.discard(task_id)
            self.get_logger().warning(
                f"{robot_id}: 비상정지로 작업 취소됨: {task_id}"
            )
            return

        if task.state in {
            TaskState.SUCCEEDED,
            TaskState.BLOCKED,
            TaskState.FAILED,
        }:
            return

        try:
            wrapped = future.result()
            result = wrapped.result
            if getattr(wrapped, 'status', GoalStatus.STATUS_SUCCEEDED) != GoalStatus.STATUS_SUCCEEDED:
                result.ok = False
                result.reason = result.reason or 'ACTION_NOT_SUCCEEDED'
        except Exception as exc:
            if task.state == TaskState.CANCELED:
                return
            self.core.finish_task(
                task_id,
                ok=False,
                reason="RESULT_FAILED",
            )
            self.publish_task_failure(
                robot_id,
                task_id,
                f"작업 결과 수신 실패: {exc}",
            )
            return

        if result.ok and task.task_type == ExecuteTask.Goal.REINSPECTION:
            state = self.remeasure_state(result)
            if state is None or re.fullmatch(r'cell:(-?\d+),(-?\d+)', task.target_ref) is None:
                result.ok, result.reason = False, 'UNCONFIRMED: invalid or insufficient measurement'
            elif not self.apply_remeasure(robot_id, task, result):
                # 저장 실패: 판정 알림(CLEARED/REMAINS)은 저장된 뒤에만. 작업은 끝내고 다음으로 진행, 저장은 dispatch 마다 재시도
                self.pending_remeasure_results[task_id] = (robot_id, result)
                result.reason = 'SAVE_PENDING'
                self.publish_alert(level=Alert.LEVEL_WARN, code='SAVE_PENDING', robot_id=robot_id,
                                   ref=task.target_ref, message='재측정 결과 저장 실패 — 판정은 저장된 뒤 알림, 순찰은 계속')
        if not result.ok and task.task_type == ExecuteTask.Goal.REINSPECTION:
            self.publish_alert(level=Alert.LEVEL_WARN, code='UNCONFIRMED', robot_id=robot_id,
                               ref=task.target_ref, message=result.reason or '재점검 판정 불가')

        self.core.finish_task(
            task_id,
            ok=result.ok,
            reason=result.reason,
        )

        if result.ok:
            self.get_logger().info(f"{robot_id}: 작업 완료: {task_id}")
            if task.task_type == ExecuteTask.Goal.REINSPECTION:
                self.publish_alert(
                    level=Alert.LEVEL_INFO,
                    code="REINSPECTION_DONE",
                    robot_id=robot_id,
                    ref=task.candidate_id,
                    message="네트워크 후보 재측정 작업을 완료했습니다.",
                )
        else:
            message = f"작업 실패: {result.reason or '사유 없음'}"
            self.publish_task_failure(robot_id, task_id, message)
            if task.task_type == ExecuteTask.Goal.SURVEY_SEGMENT:
                cells = blocked_cells(task.waypoints, result.reason, self.map.origin, self.cell_size)
                if cells:
                    self.blocked_cells |= cells
                    self.get_logger().warning(
                        f"{robot_id}: {task_id}: 장애물로 못 간 칸 {sorted(cells)} "
                        "— 미측정이면 채우기에서 뺌"
                    )

        self.complete_run_if_ready()

    def complete_run_if_ready(self) -> None:
        if self.core.run_state != RunState.RUNNING:
            return
        if getattr(self, "aborting_run", False):        # 순찰 종료 뒤 귀환 중 — 귀환 DOCK 이 다 끝나면 마침 (회차는 이미 aborted)
            docks = [t for t in self.core.tasks.values() if t.task_id.endswith("-abort-dock")]
            if all(t.state in TERMINAL_TASK_STATES or t.state == TaskState.CANCELED for t in docks):
                self.aborting_run = False
                self.core.complete_run()
                self.get_logger().info(f"run_id={self.core.run_id}: 순찰 종료 뒤 귀환 끝")
            return
        if not self.core.all_tasks_terminal():
            return

        if self.core.patrol_phase == PatrolPhase.SURVEY:
            self.core.patrol_phase = PatrolPhase.WAITING_CANDIDATES
            self.get_logger().info(
                f"run_id={self.core.run_id}: 두 섹터 측정·Dock 종료, "
                "칸 맵 확정 요청"
            )
            self.commit_futures = [
                client.call_async(Trigger.Request())
                for client in self.writer_commit_clients.values()
            ]
            self.commit_deadline = time.monotonic() + float(
                self.get_parameter("commit_timeout_sec").value
            )
            return

        if self.core.patrol_phase == PatrolPhase.REINSPECTION:
            self.core.patrol_phase = PatrolPhase.WAITING_KEEPOUT
            self.get_logger().info(
                f"run_id={self.core.run_id}: 재측정·Dock 종료, "
                "회피 마스크 발행 후 설비 점검"
            )
            return

        if self.core.patrol_phase != PatrolPhase.FACILITY:
            return

        run_id = self.core.run_id
        all_ok = all(
            task.state == TaskState.SUCCEEDED
            for task in self.core.tasks.values()
        )
        try:
            self.store.close_run(run_id, "done" if all_ok else "partial")
        except RuntimeError as exc:
            self.get_logger().error(f"회차를 닫지 못했습니다: {exc}")
        # 회차가 끝난 뒤 손으로 몰 때 막히지 않게 마스크를 비운다
        for robot_id in self.core.robots:
            self.publish_mask(robot_id, set())

        self.core.complete_run()
        self.publish_alert(
            level=Alert.LEVEL_INFO,
            code="RUN_COMPLETED",
            robot_id="",
            ref=str(run_id),
            message=f"run_id={run_id}의 모든 작업이 종료되었습니다.",
        )
        self.get_logger().info(f"run_id={run_id} 순찰 종료")

    def advance_waiting_phase(self) -> None:
        """대기 단계에서 다음 단계의 작업을 만든다 (dispatch 타이머에서 호출)."""
        if self.core.patrol_phase == PatrolPhase.WAITING_KEEPOUT:
            self.begin_facility_phase()
            return

        pending = [f for f in self.commit_futures if not f.done()]
        if pending and time.monotonic() < self.commit_deadline:
            return
        if pending:
            self.get_logger().warning(
                "칸 맵 확정 응답이 없어 지금 저장된 값으로 진행합니다."
            )
        for future in self.commit_futures:
            if future.done() and future.result() is not None:
                self.get_logger().info(
                    f"칸 맵 확정: {future.result().message}"
                )

        self.queue_cell_candidates()
        count = self.begin_reinspection_phase()
        self.get_logger().info(
            f"run_id={self.core.run_id}: 재측정 대상 {count}칸"
        )

    def read_cells(self) -> dict:
        try:
            return self.store.read_cells(self.core.run_id)
        except RuntimeError as exc:
            self.get_logger().error(f"칸 맵을 읽지 못했습니다: {exc}")
            return {}

    def queue_cell_candidates(self) -> None:
        """자기 섹터에서 1차 측정 때 미검출된 칸(include_unmeasured 면 미측정 칸도)을 그 로봇의 재측정 후보로 넣는다."""
        cells = self.read_cells()
        if not cells:
            return

        grid_cells = self.store.grid_cells()
        origin = self.map.origin

        for robot_id, zone_id in self.zone_by_robot.items():
            targets = remeasure_targets(
                cells,
                self.route_builder.reachable_cells(zone_id, grid_cells),
                int(self.get_parameter("max_remeasure_per_robot").value),
                bool(self.get_parameter("include_unmeasured").value),
                self.blocked_cells,
                min_samples=MIN_SAMPLES,
            )

            for ix, iy in targets:
                candidate = Candidate()
                candidate.candidate_id = f"cell-{ix}-{iy}"
                candidate.run_id = self.core.run_id
                candidate.robot_id = robot_id
                candidate.source = Candidate.SOURCE_NETWORK
                candidate.ref = f"cell:{ix},{iy}"
                candidate.x, candidate.y = cell_center(
                    ix,
                    iy,
                    origin[0],
                    origin[1],
                    self.cell_size,
                )
                self.pending_candidates.setdefault(
                    candidate.candidate_id,
                    candidate,
                )

    def remeasure_state(self, result):
        # 이전 액션/문자열 결과에는 미검출 수가 없다. 추측해 정상으로 해제하지 않는다.
        nd, nm, total = (getattr(result, k, 0) for k in ('n_samples', 'n_missed', 'n_total'))
        valid = (getattr(result, 'measurement_valid', False)
                 and 10 <= getattr(result, 'duration_s', 0) <= 16
                 and total == nd + nm)
        return classify(getattr(result, 'rssi_median', None), nd, nm, valid=valid,
                        threshold=float(self.get_parameter('shadow_dbm').value))

    def apply_remeasure(self, robot_id: str, task, result) -> bool:
        """판정 가능하고 저장에 성공했을 때만 알림·회피 상태를 확정한다."""
        cell = re.fullmatch(r'cell:(-?\d+),(-?\d+)', task.target_ref)
        state = self.remeasure_state(result)
        if cell is None or state is None:
            return False
        ix, iy = int(cell.group(1)), int(cell.group(2))
        try:
            saved = self.store.update_cell(
                self.core.run_id, ix, iy, state,
                result.rssi_median if result.n_samples else None, result.n_total,
                mean=result.rssi_mean if result.n_samples else None,
                n_detected=result.n_samples, n_missed=result.n_missed)
        except (RuntimeError, OSError) as exc:
            self.get_logger().error(f'재측정 저장 실패 — 판정 확정 보류: {exc}')
            return False
        if not saved:
            self.get_logger().error(f'{task.target_ref}: 저장 대상 칸 없음 — 판정 확정 보류')
            return False
        if state == 'normal':
            self.confirmed_cells.discard((ix, iy))
        else:
            self.confirmed_cells.add((ix, iy))
        self.publish_alert(
            level=Alert.LEVEL_INFO if state == 'normal' else Alert.LEVEL_WARN,
            code='CLEARED' if state == 'normal' else 'REMAINS', robot_id=robot_id,
            ref=task.target_ref,
            message=f'{task.target_ref} 재점검: {state} '
                    f'(median={result.rssi_median if result.n_samples else None}, '
                    f'detected={result.n_samples}, missed={result.n_missed}, total={result.n_total})')
        return True

    def begin_facility_phase(self) -> None:
        """남은 음영·미검출 칸을 회피 마스크로 내고 설비 점검·Dock 작업을 만든다."""
        self.core.patrol_phase = PatrolPhase.FACILITY
        run_id = self.core.run_id
        origin = self.map.origin
        zones = self.route_builder.zones()

        cells = self.read_cells()
        blocked = mask_cells(
            {c: row for c, row in cells.items() if c in self.confirmed_cells},
            self.cell_size,
            origin,
            [
                (float(zone["dock"]["x"]), float(zone["dock"]["y"]))
                for zone in zones.values()
                if zone.get("dock")
            ],
            float(self.get_parameter("dock_keepout_radius_m").value),
        )
        for robot_id in self.core.robots:
            self.publish_mask(robot_id, blocked)
        self.get_logger().info(
            f"run_id={run_id}: 회피 칸 {len(blocked)}개 {sorted(blocked)}"
        )
        low = sorted(
            cell
            for cell, row in cells.items()
            if row.state != "normal" and row.confidence != "ok"
        )
        self.get_logger().info(
            f"마스크 밖 low 음영·미검출 칸 {len(low)}개 {low}"
        )

        try:
            routes = self.route_builder.build_facility_routes()
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"설비 설정을 읽을 수 없습니다: {exc}")
            routes = []

        if self.route_builder.skipped_facilities:
            self.publish_alert(
                level=Alert.LEVEL_INFO,
                code="FACILITY_SKIPPED",
                robot_id="",
                message="좌표가 없어 건너뛴 설비: "
                + ", ".join(self.route_builder.skipped_facilities),
            )

        # 설비는 관측 자세(view)가 들어 있는 섹터를 맡은 로봇이 점검한다. 어느 섹터에도 없거나 맡은 로봇이 없으면 알림
        orphans = [r.facility_id for r in routes if r.zone_id not in self.zone_by_robot.values()]
        if orphans:
            self.publish_alert(
                level=Alert.LEVEL_WARN,
                code="FACILITY_NO_SECTOR",
                robot_id="",
                message="관측 자세가 이번 회차 섹터 밖이라 점검하지 않는 설비: " + ", ".join(orphans),
            )

        for robot_id, zone_id in self.zone_by_robot.items():
            task_count = 0

            for route in routes:
                if route.zone_id != zone_id:
                    continue

                view = route.waypoints[0]
                if cell_index(
                    view.x,
                    view.y,
                    origin[0],
                    origin[1],
                    self.cell_size,
                ) in blocked:
                    if not bool(self.get_parameter("keepout_facility_cmd_vel").value):
                        self.publish_alert(
                            level=Alert.LEVEL_WARN,
                            code="KEEPOUT_BLOCKED",
                            robot_id=robot_id,
                            ref=route.facility_id,
                            message=(
                                f"{route.facility_id}: 회피 구역에 막혀 미확인"
                            ),
                        )
                        continue
                    self.publish_alert(
                        level=Alert.LEVEL_WARN,
                        code="KEEPOUT_CMD_VEL",
                        robot_id=robot_id,
                        ref=route.facility_id,
                        message=(
                            f"{route.facility_id}: 회피 구역 안 — 로봇이 cmd_vel 로 접근해 점검"
                        ),
                    )

                self.core.create_task(
                    task_id=(
                        f"run-{run_id}-{robot_id}-"
                        f"facility-{route.facility_id}"
                    ),
                    run_id=run_id,
                    task_type=ExecuteTask.Goal.FACILITY_CHECK,
                    priority=1,
                    target_ref=route.facility_id,
                    preferred_robot_id=robot_id,
                    allowed_robot_ids=(robot_id,),
                    waypoints=route.waypoints,
                )
                task_count += 1

            if task_count:
                self.core.create_task(
                    task_id=f"run-{run_id}-{robot_id}-facility-dock",
                    run_id=run_id,
                    task_type=ExecuteTask.Goal.DOCK,
                    priority=1,
                    target_ref="dock",
                    preferred_robot_id=robot_id,
                    allowed_robot_ids=(robot_id,),
                    waypoints=(),
                )

    def publish_mask(self, robot_id: str, cells: set) -> None:
        info = self.map
        mask = OccupancyGrid()
        mask.header.frame_id = "map"
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info.resolution = info.resolution
        mask.info.width = info.width
        mask.info.height = info.height
        mask.info.origin.position.x = info.origin[0]
        mask.info.origin.position.y = info.origin[1]
        mask.info.origin.orientation.w = 1.0
        mask.data = keepout_mask(
            cells,
            self.cell_size,
            info.origin,
            info.resolution,
            info.width,
            info.height,
        )
        self.mask_publishers[robot_id].publish(mask)

    def on_candidate(self, msg: Candidate) -> None:
        if msg.source != Candidate.SOURCE_NETWORK:
            return
        if msg.run_id != self.core.run_id:
            return
        if self.core.run_state not in {
            RunState.RUNNING,
            RunState.ESTOPPED,
        }:
            return
        if msg.candidate_id in self.seen_candidate_ids:
            return
        if msg.robot_id not in self.core.robots:
            self.get_logger().warning(
                f"알 수 없는 후보 로봇 ID: {msg.robot_id}"
            )
            return
        if self.core.patrol_phase not in {
            PatrolPhase.SURVEY,
            PatrolPhase.WAITING_CANDIDATES,
        }:
            self.get_logger().warning(
                f"{self.core.patrol_phase.value} 단계에서 도착한 "
                f"후보를 무시합니다: {msg.candidate_id}"
            )
            return

        self.seen_candidate_ids.add(msg.candidate_id)
        self.pending_candidates[msg.candidate_id] = msg
        self.publish_alert(
            level=Alert.LEVEL_WARN,
            code="CANDIDATE",
            robot_id=msg.robot_id,
            ref=msg.candidate_id,
            message=(
                f"네트워크 후보 {msg.candidate_id}: "
                f"{msg.robot_id} 재측정 대기열에 저장"
            ),
        )
        self.get_logger().info(
            f"후보 저장: {msg.candidate_id} -> {msg.robot_id}"
        )

    def begin_reinspection_phase(self) -> int:
        """RSSI 후보 분석 완료 신호를 받을 콜백에서 호출할 내부 진입점."""
        if self.core.patrol_phase != PatrolPhase.WAITING_CANDIDATES:
            raise RuntimeError("후보 분석 대기 단계가 아닙니다.")

        self.core.patrol_phase = PatrolPhase.REINSPECTION
        candidate_counts = {
            robot_id: 0
            for robot_id in self.core.robots
        }

        for candidate_id, candidate in self.pending_candidates.items():
            robot_id = candidate.robot_id
            candidate_counts[robot_id] += 1
            self.core.create_task(
                task_id=(
                    f"run-{candidate.run_id}-{robot_id}-"
                    f"reinspect-{candidate_id}"
                ),
                run_id=candidate.run_id,
                task_type=ExecuteTask.Goal.REINSPECTION,
                priority=10,
                target_ref=candidate.ref,
                preferred_robot_id=robot_id,
                allowed_robot_ids=(robot_id,),
                waypoints=(
                    Waypoint(
                        candidate.x,
                        candidate.y,
                        candidate.yaw,
                    ),
                ),
                candidate_id=candidate_id,
            )

        for robot_id, candidate_count in candidate_counts.items():
            if candidate_count == 0:
                continue

            self.core.create_task(
                task_id=(
                    f"run-{self.core.run_id}-{robot_id}-reinspect-dock"
                ),
                run_id=self.core.run_id,
                task_type=ExecuteTask.Goal.DOCK,
                priority=1,
                target_ref="dock",
                preferred_robot_id=robot_id,
                allowed_robot_ids=(robot_id,),
                waypoints=(),
            )

        candidate_count = len(self.pending_candidates)
        if candidate_count == 0:
            self.core.patrol_phase = PatrolPhase.WAITING_KEEPOUT

        return candidate_count

    def on_inspection_result(self, msg: InspectionResult) -> None:
        if msg.run_id != self.core.run_id:
            return

        result_key = (
            msg.robot_id,
            msg.run_id,
            msg.task_id,
            msg.facility_id,
            msg.verdict,
            msg.attempts,
            msg.stamp.sec,
            msg.stamp.nanosec,
        )
        if result_key in self.seen_inspection_results:
            return
        self.seen_inspection_results.add(result_key)

        if msg.verdict == InspectionResult.NORMAL:
            self.publish_alert(
                level=Alert.LEVEL_INFO,
                code="INSPECTION_NORMAL",
                robot_id=msg.robot_id,
                ref=msg.facility_id,
                message=f"{msg.facility_id}: 정상",
            )
            return

        if msg.verdict == InspectionResult.UNCONFIRMED:
            self.publish_alert(
                level=Alert.LEVEL_WARN,
                code="E6",
                robot_id=msg.robot_id,
                ref=msg.facility_id,
                message=(
                    msg.detail
                    or f"{msg.facility_id}: {msg.attempts}회 관측 후 미확인"
                ),
            )
            return

        self.publish_alert(
            level=Alert.LEVEL_ERROR,
            code="CONFIRMED",
            robot_id=msg.robot_id,
            ref=msg.facility_id,
            message=msg.detail or f"{msg.facility_id}: 이상 판정",
        )

    def on_estop(self, request, response):
        del request

        if self.core.run_state == RunState.ESTOPPED:
            response.success = True
            response.message = "이미 비상정지 상태입니다."
            return response
        if self.core.run_state != RunState.RUNNING:
            response.success = False
            response.message = "진행 중인 순찰이 없습니다."
            return response

        self.core.estop()
        self.canceling_task_ids.update(
            task.task_id
            for task in self.core.tasks.values()
            if task.state == TaskState.CANCELED
        )
        for goal_handle in self.active_goals.values():
            goal_handle.cancel_goal_async()

        self.publish_alert(
            level=Alert.LEVEL_ERROR,
            code="E7",
            robot_id="",
            message="비상정지: 실행 중인 로봇 작업 취소를 요청했습니다.",
        )
        response.success = True
        response.message = "비상정지 요청 완료"
        return response

    def on_abort(self, request, response):
        """순찰 종료 — 로봇 작업을 모두 취소하고 회차를 aborted 로 닫은 뒤, 로봇마다 DOCK 을 보내 도크로 돌려보낸다
        (비상정지는 그 자리 정지 · 재개 대기, 순찰 종료는 끝내고 귀환 — 10/15 PM)"""
        del request
        if self.core.run_state not in {RunState.RUNNING, RunState.ESTOPPED}:
            response.success = False
            response.message = "진행 중인 순찰이 없습니다."
            return response
        run_id = self.core.run_id
        self.core.estop()
        for task in self.core.tasks.values():          # 대기 작업도 취소
            if task.state == TaskState.PENDING:
                task.state, task.reason = TaskState.CANCELED, "ABORT"
        self.canceling_task_ids.update(
            task.task_id for task in self.core.tasks.values() if task.state == TaskState.CANCELED
        )
        for goal_handle in self.active_goals.values():
            goal_handle.cancel_goal_async()
        try:
            self.store.close_run(run_id, "aborted")
        except (RuntimeError, AttributeError) as exc:
            self.get_logger().error(f"회차를 닫지 못했습니다: {exc}")
        for robot_id in self.core.robots:
            self.publish_mask(robot_id, set())
        # 귀환 — 로봇마다 DOCK (도크 위면 executor 가 바로 끝냄). 다 끝나면 complete_run_if_ready 가 회차를 마침
        for robot_id in self.core.robots:
            self.core.create_task(
                task_id=f"run-{run_id}-{robot_id}-abort-dock",
                run_id=run_id,
                task_type=ExecuteTask.Goal.DOCK,
                priority=1,
                target_ref="dock",
                preferred_robot_id=robot_id,
                allowed_robot_ids=(robot_id,),
                waypoints=(),
            )
        self.aborting_run = True
        self.core.run_state = RunState.RUNNING
        self.publish_alert(
            level=Alert.LEVEL_WARN,
            code="RUN_ABORTED",
            robot_id="",
            ref=str(run_id),
            message=f"run_id={run_id} 순찰을 종료했습니다 (관리자) — 로봇은 도크로 귀환합니다.",
        )
        self.get_logger().warning(f"run_id={run_id} 순찰 종료 (관리자 abort) — 귀환 DOCK 전송")
        response.success = True
        response.message = f"run {run_id} 종료"
        return response

    def on_resume(self, request, response):
        del request

        if self.core.run_state != RunState.ESTOPPED:
            response.success = False
            response.message = "비상정지 상태가 아닙니다."
            return response

        self.core.resume()
        response.success = True
        response.message = "취소된 작업을 대기열에 넣고 순찰을 재개합니다."
        self.publish_alert(
            level=Alert.LEVEL_INFO,
            code="RESUMED",
            robot_id="",
            message=response.message,
        )
        return response

    def on_reassign(self, request, response):
        from_robot_id = request.from_robot_id
        to_robot_id = request.to_robot_id

        if not to_robot_id:
            to_robot_id = next(
                (
                    robot_id
                    for robot_id in self.core.robots
                    if robot_id != from_robot_id
                ),
                "",
            )

        if (
            from_robot_id not in self.core.robots
            or to_robot_id not in self.core.robots
            or from_robot_id == to_robot_id
        ):
            response.ok = False
            response.message = "from/to 로봇 ID를 확인하세요."
            response.moved_tasks = 0
            return response

        moved_tasks = self.core.reassign_pending_tasks(
            from_robot_id,
            to_robot_id,
        )
        response.moved_tasks = moved_tasks
        response.ok = moved_tasks > 0

        if moved_tasks:
            response.message = (
                f"{from_robot_id} -> {to_robot_id}: "
                f"대기 작업 {moved_tasks}건 재배정"
            )
        else:
            response.message = (
                "재배정 가능한 대기 작업이 없습니다. "
                "현재 시나리오는 로봇별 담당 섹터가 고정되어 있습니다."
            )
        return response

    def publish_task_failure(
        self,
        robot_id: str,
        task_id: str,
        message: str,
    ) -> None:
        task = self.core.tasks[task_id]
        blocked = task.state == TaskState.BLOCKED
        self.publish_alert(
            level=Alert.LEVEL_WARN if blocked else Alert.LEVEL_ERROR,
            code="E1" if blocked else "TASK_FAILED",
            robot_id=robot_id,
            ref=task_id,
            message=message,
        )
        self.get_logger().error(f"{robot_id}: {task_id}: {message}")

    def publish_alert(
        self,
        *,
        level: int,
        code: str,
        robot_id: str,
        message: str,
        ref: str = "",
    ) -> None:
        alert = Alert()
        alert.stamp = self.get_clock().now().to_msg()
        alert.level = level
        alert.code = code
        alert.robot_id = robot_id
        alert.ref = ref
        alert.message = message
        alert.delayed = False
        self.alert_publisher.publish(alert)

    def to_pose_stamped(self, waypoint: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = InspectionManager()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
