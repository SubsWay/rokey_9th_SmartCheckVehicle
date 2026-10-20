import math
import re
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class RunState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    ESTOPPED = "ESTOPPED"
    COMPLETED = "COMPLETED"


class PatrolPhase(str, Enum):
    IDLE = "IDLE"
    SURVEY = "SURVEY"
    WAITING_CANDIDATES = "WAITING_CANDIDATES"
    REINSPECTION = "REINSPECTION"
    WAITING_KEEPOUT = "WAITING_KEEPOUT"
    FACILITY = "FACILITY"
    COMPLETED = "COMPLETED"


class TaskState(str, Enum):
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    BLOCKED = "BLOCKED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RobotStatusSnapshot:
    """RobotStatus 메시지에서 받은 최신 상태."""

    state: int
    battery_percent: float
    accepting_tasks: bool
    task_id: str
    run_id: int
    task_type: int
    progress: float
    warning: str


@dataclass
class RobotRecord:
    """관제에서 관리하는 로봇의 최신 상태와 수신 시각."""

    robot_id: str
    online: bool = False
    last_seen: Optional[float] = None
    status: Optional[RobotStatusSnapshot] = None


@dataclass
class TaskRecord:
    task_id: str
    run_id: int
    task_type: int
    priority: int
    target_ref: str
    preferred_robot_id: str = ""
    allowed_robot_ids: tuple[str, ...] = ()
    waypoints: tuple[Waypoint, ...] = ()
    candidate_id: str = ""
    state: TaskState = TaskState.PENDING
    robot_id: Optional[str] = None
    progress: float = 0.0
    reason: str = ""
    waypoint_index: int = 0     # 로봇이 지금 향하는 점 (feedback, 보낸 waypoints 기준)


TERMINAL_TASK_STATES = {
    TaskState.SUCCEEDED,
    TaskState.BLOCKED,
    TaskState.FAILED,
}


class ManagerCore:
    """ROS에 의존하지 않는 로봇·순찰·작업 상태 관리 로직."""

    def __init__(
        self,
        robot_ids: list[str],
        offline_timeout_sec: float = 29.0,
    ):
        self.offline_timeout_sec = offline_timeout_sec
        self.robots = {
            robot_id: RobotRecord(robot_id=robot_id)
            for robot_id in robot_ids
        }
        self.run_id = 0
        self.run_state = RunState.IDLE
        self.patrol_phase = PatrolPhase.IDLE
        self.tasks: dict[str, TaskRecord] = {}
        self.task_order: list[str] = []

    def update_robot(
        self,
        robot_id: str,
        now: float,
        status: RobotStatusSnapshot,
    ) -> None:
        robot = self.robots[robot_id]
        robot.online = True
        robot.last_seen = now
        robot.status = status

    def check_offline(self, now: float) -> list[str]:
        newly_offline = []

        for robot_id, robot in self.robots.items():
            if not robot.online or robot.last_seen is None:
                continue

            if now - robot.last_seen >= self.offline_timeout_sec:
                robot.online = False
                newly_offline.append(robot_id)

        return newly_offline

    def start_run(
        self,
        timestamp_sec: Optional[int] = None,
        run_id: Optional[int] = None,
    ) -> int:
        next_run_id = self.run_id + 1

        if timestamp_sec is not None:
            next_run_id = max(next_run_id, timestamp_sec)
        if run_id is not None:
            next_run_id = run_id

        self.run_id = next_run_id
        self.run_state = RunState.RUNNING
        self.patrol_phase = PatrolPhase.SURVEY
        self.tasks.clear()
        self.task_order.clear()

        return self.run_id

    def create_task(
        self,
        *,
        task_id: str,
        run_id: int,
        task_type: int,
        priority: int,
        target_ref: str,
        preferred_robot_id: str = "",
        allowed_robot_ids: tuple[str, ...] = (),
        waypoints: tuple[Waypoint, ...] = (),
        candidate_id: str = "",
    ) -> None:
        self.tasks[task_id] = TaskRecord(
            task_id=task_id,
            run_id=run_id,
            task_type=task_type,
            priority=priority,
            target_ref=target_ref,
            preferred_robot_id=preferred_robot_id,
            allowed_robot_ids=allowed_robot_ids,
            waypoints=waypoints,
            candidate_id=candidate_id,
        )
        self.task_order.append(task_id)

    def next_pending_task(self, robot_id: str) -> Optional[TaskRecord]:
        if self.run_state != RunState.RUNNING:
            return None

        if any(
            task.robot_id == robot_id
            and task.state in {TaskState.ASSIGNED, TaskState.RUNNING}
            for task in self.tasks.values()
        ):
            return None

        for task_id in self.task_order:
            task = self.tasks[task_id]

            if task.state != TaskState.PENDING:
                continue
            if (
                task.allowed_robot_ids
                and robot_id not in task.allowed_robot_ids
            ):
                continue
            if (
                task.preferred_robot_id
                and task.preferred_robot_id != robot_id
            ):
                continue

            return task

        return None

    def abandon_for_dock(self, robot_id: str, dock_type: int, reason: str) -> Optional[TaskRecord]:
        """배터리 부족 등으로 로봇이 DOCK 외 작업을 안 받을 때: 그 로봇 몫 대기 작업을 실패로 닫고 DOCK 작업을 돌려줌.
        DOCK 작업이 없거나 로봇이 작업 중이면 None (아무것도 안 바꿈)."""
        if self.run_state != RunState.RUNNING or any(
            task.robot_id == robot_id and task.state in {TaskState.ASSIGNED, TaskState.RUNNING}
            for task in self.tasks.values()
        ):
            return None
        mine = [self.tasks[t] for t in self.task_order
                if self.tasks[t].state == TaskState.PENDING
                and (self.tasks[t].preferred_robot_id == robot_id
                     or robot_id in self.tasks[t].allowed_robot_ids)]
        dock = next((t for t in mine if t.task_type == dock_type), None)
        if dock is None:
            return None
        for task in mine:
            if task is not dock:
                task.state, task.reason = TaskState.FAILED, reason
        return dock

    def assign_task(self, task_id: str, robot_id: str) -> None:
        task = self.tasks[task_id]
        task.robot_id = robot_id
        task.state = TaskState.ASSIGNED
        task.reason = ""

    def start_task(self, task_id: str) -> None:
        self.tasks[task_id].state = TaskState.RUNNING

    def update_task_progress(self, task_id: str, progress: float, waypoint_index: int = 0) -> None:
        self.tasks[task_id].progress = progress
        self.tasks[task_id].waypoint_index = waypoint_index

    def finish_task(self, task_id: str, *, ok: bool, reason: str) -> None:
        task = self.tasks[task_id]

        if ok:
            task.state = TaskState.SUCCEEDED
            task.progress = 1.0
            task.reason = ""
            return

        task.reason = reason
        task.state = (
            TaskState.BLOCKED
            if reason.upper().startswith("BLOCKED")
            else TaskState.FAILED
        )

    def fail_active_tasks_for_robot(
        self,
        robot_id: str,
        reason: str,
    ) -> list[str]:
        failed_task_ids = []

        for task in self.tasks.values():
            if task.robot_id != robot_id:
                continue
            if task.state not in {
                TaskState.ASSIGNED,
                TaskState.RUNNING,
            }:
                continue

            self.finish_task(
                task.task_id,
                ok=False,
                reason=reason,
            )
            failed_task_ids.append(task.task_id)

        return failed_task_ids

    def requeue_task(self, task_id: str, reason: str = "") -> None:
        task = self.tasks[task_id]
        task.state = TaskState.PENDING
        task.robot_id = None
        task.progress = 0.0
        task.reason = reason

    def estop(self) -> None:
        self.run_state = RunState.ESTOPPED

        for task in self.tasks.values():
            if task.state in {TaskState.ASSIGNED, TaskState.RUNNING}:
                task.state = TaskState.CANCELED
                task.reason = "ESTOP"
                # 재개 때 지나온 점으로 되돌아가지 않게, 향하던 점부터 남긴다 (그 점은 아직 못 감)
                if 0 < task.waypoint_index < len(task.waypoints):
                    task.waypoints = task.waypoints[task.waypoint_index:]
                task.waypoint_index = 0

    def resume(self) -> None:
        self.run_state = RunState.RUNNING

        for task in self.tasks.values():
            if task.state == TaskState.CANCELED:
                self.requeue_task(task.task_id)

    def all_tasks_terminal(self) -> bool:
        return bool(self.tasks) and all(
            task.state in TERMINAL_TASK_STATES
            for task in self.tasks.values()
        )

    def complete_run(self) -> None:
        self.run_state = RunState.COMPLETED
        self.patrol_phase = PatrolPhase.COMPLETED

    def reopen_run(self) -> None:
        if self.run_state == RunState.COMPLETED:
            self.run_state = RunState.RUNNING

    def reassign_pending_tasks(
        self,
        from_robot_id: str,
        to_robot_id: str,
    ) -> int:
        moved_tasks = 0

        for task in self.tasks.values():
            if task.state != TaskState.PENDING:
                continue
            if task.preferred_robot_id != from_robot_id:
                continue
            if (
                task.allowed_robot_ids
                and to_robot_id not in task.allowed_robot_ids
            ):
                continue

            task.preferred_robot_id = to_robot_id
            moved_tasks += 1

        return moved_tasks


def remeasure_targets(
    cells: dict,
    reachable: set,
    limit: int,
    include_unmeasured: bool = True,
    blocked: set = frozenset(),
    min_samples: int = 0,
) -> list[tuple[int, int]]:
    """재측정할 칸: 1차 측정 경로에서 미검출된 칸 → (include_unmeasured 면) 미측정.

    재측정은 1차 측정 뒤 미검출 칸만 바로 가서 다시 재는 선택 2차 작업이다 (10/15 PM). 약한 신호(shadow) 칸은 넣지 않는다.
    cells 는 {(ix, iy): 줄(state, median, …)}, reachable 은 그 로봇이 갈 수 있는 칸.
    blocked 는 이번 회차 측정 중 장애물로 못 간 칸 — 미측정이면 채우러 가지 않는다.
    """
    eligible = {c for c in cells if cells[c].n >= min_samples}
    miss = sorted(c for c in reachable if c in eligible and cells[c].state == "miss")
    unmeasured = sorted(reachable - set(cells) - set(blocked)) if include_unmeasured else []
    targets = miss + unmeasured
    return targets[:limit] if limit > 0 else targets


def mask_cells(
    cells: dict,
    cell_size: float,
    origin: tuple[float, float],
    docks: list[tuple[float, float]],
    dock_radius: float,
) -> set[tuple[int, int]]:
    """회피 칸: 음영·미검출이고 confidence 가 ok 인 칸. Dock 반경에 걸치는 칸은 뺀다."""
    out = set()

    for (ix, iy), row in cells.items():
        if row.state == "normal" or row.confidence != "ok":
            continue

        x0 = origin[0] + ix * cell_size
        y0 = origin[1] + iy * cell_size
        near_dock = any(
            max(x0 - dx, 0.0, dx - x0 - cell_size) ** 2
            + max(y0 - dy, 0.0, dy - y0 - cell_size) ** 2
            <= dock_radius ** 2
            for dx, dy in docks
        )
        if not near_dock:
            out.add((ix, iy))

    return out


def blocked_cells(
    waypoints: tuple,
    reason: str,
    origin: tuple[float, float],
    cell: float,
) -> set[tuple[int, int]]:
    """측정 작업 결과 reason(mission_executor: "BLOCKED skipped=[…] shortened=[…]")의 막힌 점이 든 칸.

    skipped 는 그 점의 칸, shortened(골을 장애물 앞까지 줄임)는 앞 점 → 골 직선이 지나는 칸 (줄인 자리를 모르므로 그 구간 전체).
    """
    def indices(key: str) -> list[int]:
        found = re.search(key + r"=\[([\d,\s]*)\]", reason)
        return [int(v) for v in found.group(1).split(",") if v.strip()] if found else []

    def index(x: float, y: float) -> tuple[int, int]:
        return math.floor((x - origin[0]) / cell), math.floor((y - origin[1]) / cell)

    out = set()
    for i in indices("skipped"):
        if 0 <= i < len(waypoints):
            out.add(index(waypoints[i].x, waypoints[i].y))
    for i in indices("shortened"):
        if 0 < i < len(waypoints):
            a, b = waypoints[i - 1], waypoints[i]
            steps = max(int(math.dist((a.x, a.y), (b.x, b.y)) / (cell / 4)), 1)
            out.update(index(a.x + (b.x - a.x) * k / steps, a.y + (b.y - a.y) * k / steps) for k in range(1, steps + 1))
    return out
