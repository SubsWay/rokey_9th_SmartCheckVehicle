from types import SimpleNamespace

from scv_msgs.action import ExecuteTask
from scv_msgs.msg import Candidate

from scv_manager.inspection_manager import InspectionManager
from scv_manager.manager_core import (
    ManagerCore,
    PatrolPhase,
    TaskState,
    Waypoint,
)
from scv_manager.route_builder import SurveyRoute


class FakeLogger:
    def info(self, message: str) -> None:
        del message

    def warning(self, message: str) -> None:
        del message

    def error(self, message: str) -> None:
        del message


class FakeRouteBuilder:
    def build_survey_routes(self) -> list[SurveyRoute]:
        return [
            SurveyRoute("Z1", (Waypoint(1.0, 1.0, 0.0),)),
            SurveyRoute("Z2", (Waypoint(2.0, 2.0, 0.0),)),
        ]


class FakeStore:
    def open_run(self, sector_robots, close_stale=False) -> int:
        del sector_robots, close_stale
        return 7


class FakeClient:
    def call_async(self, request) -> None:
        del request


class FakeInspectionManager:
    """ROS 노드를 띄우지 않고 관제 콜백을 시험한다."""

    def __init__(self, core: ManagerCore):
        self.core = core
        self.route_builder = FakeRouteBuilder()
        self.zone_by_robot = {
            "robot4": "Z1",
            "robot3": "Z2",
        }
        self.seen_candidate_ids = set()
        self.pending_candidates = {}
        self.pending_remeasure_results = {}
        self.confirmed_cells = set()
        self.seen_inspection_results = set()
        self.alerts = []
        self.store = FakeStore()
        self.writer_reset_client = FakeClient()
        self.writer_commit_clients = {}

    def get_parameter(self, name: str) -> SimpleNamespace:
        del name
        return SimpleNamespace(value=0)

    def publish_mask(self, robot_id: str, cells: set) -> None:
        del robot_id, cells

    def publish_alert(self, **kwargs) -> None:
        self.alerts.append(kwargs)

    def get_logger(self) -> FakeLogger:
        return FakeLogger()


def make_candidate(
    run_id: int,
    candidate_id: str = "candidate-1",
    robot_id: str = "robot4",
) -> Candidate:
    candidate = Candidate()
    candidate.run_id = run_id
    candidate.candidate_id = candidate_id
    candidate.robot_id = robot_id
    candidate.source = Candidate.SOURCE_NETWORK
    candidate.ref = f"cell-{candidate_id}"
    candidate.x = 1.0
    candidate.y = 2.0
    candidate.yaw = 0.0
    return candidate


def test_start_creates_one_survey_task_per_segment_then_dock():
    core = ManagerCore(["robot3"])
    manager = FakeInspectionManager(core)
    manager.zone_by_robot = {"robot3": "Z2"}
    a, b, c = Waypoint(1.0, 1.0, 0.0), Waypoint(2.0, 1.0, 0.0), Waypoint(2.0, 2.0, 0.0)
    manager.route_builder.build_survey_routes = lambda: [
        SurveyRoute("Z2", (a, b, c), segments=((a, b), (c,))),
    ]

    InspectionManager.on_start(manager, None, SimpleNamespace(success=False, message=""))

    tasks = list(core.tasks.values())
    assert [task.task_id for task in tasks] == [
        "run-7-robot3-survey-1",
        "run-7-robot3-survey-2",
        "run-7-robot3-survey-dock",
    ]
    assert [task.waypoints for task in tasks] == [(a, b), (c,), ()]
    # 한 로봇에는 만든 순서대로 하나씩 나간다
    assert core.next_pending_task("robot3").task_id == "run-7-robot3-survey-1"


def test_start_creates_survey_and_dock_for_each_robot():
    core = ManagerCore(["robot4", "robot3"])
    manager = FakeInspectionManager(core)
    response = SimpleNamespace(success=False, message="")

    result = InspectionManager.on_start(manager, None, response)

    assert result.success is True
    assert len(core.tasks) == 4
    assert core.patrol_phase == PatrolPhase.SURVEY

    robot4_tasks = [
        task
        for task in core.tasks.values()
        if task.preferred_robot_id == "robot4"
    ]
    robot3_tasks = [
        task
        for task in core.tasks.values()
        if task.preferred_robot_id == "robot3"
    ]

    assert [task.task_type for task in robot4_tasks] == [
        ExecuteTask.Goal.SURVEY_SEGMENT,
        ExecuteTask.Goal.DOCK,
    ]
    assert [task.task_type for task in robot3_tasks] == [
        ExecuteTask.Goal.SURVEY_SEGMENT,
        ExecuteTask.Goal.DOCK,
    ]
    assert robot4_tasks[0].target_ref == "Z1"
    assert robot3_tasks[0].target_ref == "Z2"


def test_duplicate_candidate_is_buffered_only_once():
    core = ManagerCore(["robot3", "robot4"])
    run_id = core.start_run(timestamp_sec=1000)
    core.patrol_phase = PatrolPhase.WAITING_CANDIDATES
    manager = FakeInspectionManager(core)
    candidate = make_candidate(run_id)

    InspectionManager.on_candidate(manager, candidate)
    InspectionManager.on_candidate(manager, candidate)

    assert core.tasks == {}
    assert manager.seen_candidate_ids == {"candidate-1"}
    assert set(manager.pending_candidates) == {"candidate-1"}
    assert len(manager.alerts) == 1


def test_candidate_with_different_run_id_is_ignored():
    core = ManagerCore(["robot3", "robot4"])
    run_id = core.start_run(timestamp_sec=1000)
    manager = FakeInspectionManager(core)

    candidate = make_candidate(run_id + 1)
    InspectionManager.on_candidate(manager, candidate)

    assert manager.pending_candidates == {}
    assert manager.seen_candidate_ids == set()
    assert manager.alerts == []


def test_candidate_from_unknown_robot_is_ignored():
    core = ManagerCore(["robot3", "robot4"])
    run_id = core.start_run(timestamp_sec=1000)
    manager = FakeInspectionManager(core)

    candidate = make_candidate(run_id, robot_id="robot9")
    InspectionManager.on_candidate(manager, candidate)

    assert manager.pending_candidates == {}
    assert manager.seen_candidate_ids == set()


def test_reinspection_tasks_stay_with_candidate_robot_then_dock():
    core = ManagerCore(["robot4", "robot3"])
    run_id = core.start_run(timestamp_sec=1000)
    core.patrol_phase = PatrolPhase.WAITING_CANDIDATES
    manager = FakeInspectionManager(core)

    InspectionManager.on_candidate(
        manager,
        make_candidate(run_id, "candidate-r4", "robot4"),
    )
    InspectionManager.on_candidate(
        manager,
        make_candidate(run_id, "candidate-r3", "robot3"),
    )

    count = InspectionManager.begin_reinspection_phase(manager)

    assert count == 2
    assert core.patrol_phase == PatrolPhase.REINSPECTION

    robot4_tasks = [
        task
        for task in core.tasks.values()
        if task.preferred_robot_id == "robot4"
    ]
    robot3_tasks = [
        task
        for task in core.tasks.values()
        if task.preferred_robot_id == "robot3"
    ]

    assert [task.task_type for task in robot4_tasks] == [
        ExecuteTask.Goal.REINSPECTION,
        ExecuteTask.Goal.DOCK,
    ]
    assert [task.task_type for task in robot3_tasks] == [
        ExecuteTask.Goal.REINSPECTION,
        ExecuteTask.Goal.DOCK,
    ]
    assert robot4_tasks[0].allowed_robot_ids == ("robot4",)
    assert robot3_tasks[0].allowed_robot_ids == ("robot3",)


def test_no_candidates_skips_reinspection_phase():
    core = ManagerCore(["robot4", "robot3"])
    core.start_run(timestamp_sec=1000)
    core.patrol_phase = PatrolPhase.WAITING_CANDIDATES
    manager = FakeInspectionManager(core)

    count = InspectionManager.begin_reinspection_phase(manager)

    assert count == 0
    assert core.tasks == {}
    assert core.patrol_phase == PatrolPhase.WAITING_KEEPOUT


def test_survey_completion_waits_for_candidate_analysis():
    core = ManagerCore(["robot4"])
    run_id = core.start_run(timestamp_sec=1000)
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=ExecuteTask.Goal.SURVEY_SEGMENT,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
    )
    core.finish_task("survey", ok=True, reason="")
    manager = FakeInspectionManager(core)

    InspectionManager.complete_run_if_ready(manager)

    assert core.patrol_phase == PatrolPhase.WAITING_CANDIDATES
    assert core.run_state.value == "RUNNING"


def test_offline_failure_is_terminal():
    core = ManagerCore(["robot4"])
    run_id = core.start_run(timestamp_sec=1000)
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=ExecuteTask.Goal.SURVEY_SEGMENT,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
    )
    core.assign_task("survey", "robot4")
    core.start_task("survey")
    core.fail_active_tasks_for_robot("robot4", "OFFLINE")

    assert core.tasks["survey"].state == TaskState.FAILED


def test_start_is_rejected_without_cell_map_store():
    core = ManagerCore(["robot4", "robot3"])
    manager = FakeInspectionManager(core)
    manager.store = None
    response = SimpleNamespace(success=True, message="")

    result = InspectionManager.on_start(manager, None, response)

    assert result.success is False
    assert core.tasks == {}
