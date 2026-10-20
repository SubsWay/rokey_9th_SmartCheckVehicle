from scv_manager.manager_core import (
    ManagerCore,
    PatrolPhase,
    RobotStatusSnapshot,
    RunState,
    TaskState,
    Waypoint,
)


def test_task_success_flow():
    core = ManagerCore(["robot4", "robot3"])

    core.create_task(
        task_id="task-001",
        run_id=1,
        task_type=0,
        priority=1,
        target_ref="Z1",
    )

    task = core.tasks["task-001"]
    assert task.state == TaskState.PENDING

    core.assign_task("task-001", "robot4")
    assert task.state == TaskState.ASSIGNED
    assert task.robot_id == "robot4"

    core.start_task("task-001")
    assert task.state == TaskState.RUNNING

    core.update_task_progress("task-001", 0.5)
    assert task.progress == 0.5

    core.finish_task(
        "task-001",
        ok=True,
        reason="",
    )

    assert task.state == TaskState.SUCCEEDED
    assert task.progress == 1.0
    assert task.reason == ""


def test_task_failure_flow():
    core = ManagerCore(["robot4"])

    core.create_task(
        task_id="task-002",
        run_id=1,
        task_type=0,
        priority=1,
        target_ref="Z1",
    )

    core.assign_task("task-002", "robot4")
    core.start_task("task-002")
    core.finish_task(
        "task-002",
        ok=False,
        reason="BLOCKED",
    )

    task = core.tasks["task-002"]

    assert task.state == TaskState.BLOCKED
    assert task.reason == "BLOCKED"


def test_two_robots_can_receive_tasks_concurrently():
    core = ManagerCore(["robot4", "robot3"])
    run_id = core.start_run()

    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
    )
    core.create_task(
        task_id="robot3-survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z2",
        preferred_robot_id="robot3",
        allowed_robot_ids=("robot3",),
    )
    core.create_task(
        task_id="robot4-dock",
        run_id=run_id,
        task_type=3,
        priority=1,
        target_ref="dock",
        preferred_robot_id="robot4",
        allowed_robot_ids=("robot4",),
    )

    assert core.next_pending_task("robot4").task_id == "survey"
    core.assign_task("survey", "robot4")
    core.start_task("survey")

    assert core.next_pending_task("robot4") is None
    assert core.next_pending_task("robot3").task_id == "robot3-survey"

    core.assign_task("robot3-survey", "robot3")
    core.start_task("robot3-survey")
    core.finish_task("survey", ok=True, reason="")
    assert core.next_pending_task("robot4").task_id == "robot4-dock"
    assert core.next_pending_task("robot3") is None


def test_robot_cannot_take_task_for_another_robot():
    core = ManagerCore(["robot4", "robot3"])
    run_id = core.start_run()
    core.create_task(
        task_id="robot4-only",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
        allowed_robot_ids=("robot4",),
    )

    assert core.next_pending_task("robot3") is None
    assert core.next_pending_task("robot4").task_id == "robot4-only"


def test_estop_and_resume_requeue_active_task():
    core = ManagerCore(["robot4"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
    )
    core.assign_task("survey", "robot4")
    core.start_task("survey")

    core.estop()
    assert core.run_state == RunState.ESTOPPED
    assert core.tasks["survey"].state == TaskState.CANCELED

    core.resume()
    assert core.run_state == RunState.RUNNING
    assert core.tasks["survey"].state == TaskState.PENDING
    assert core.tasks["survey"].robot_id is None


def test_resume_keeps_only_waypoints_not_reached():
    core = ManagerCore(["robot3"])
    run_id = core.start_run()
    points = tuple(Waypoint(float(i), 0.0, 0.0) for i in range(5))
    core.create_task(task_id="survey", run_id=run_id, task_type=0, priority=1,
                     target_ref="Z2", waypoints=points)
    core.assign_task("survey", "robot3")
    core.start_task("survey")
    core.update_task_progress("survey", 0.4, 2)     # 점 2 로 가던 중

    core.estop()
    core.resume()
    assert core.tasks["survey"].waypoints == points[2:]
    assert core.tasks["survey"].waypoint_index == 0


def test_low_battery_closes_pending_and_returns_dock():
    core = ManagerCore(["robot4"])
    run_id = core.start_run()
    for tid, kind in (("s1", 0), ("s2", 0), ("dock", 3)):
        core.create_task(task_id=tid, run_id=run_id, task_type=kind, priority=1,
                         target_ref="Z1", preferred_robot_id="robot4")
    dock = core.abandon_for_dock("robot4", 3, "LOW_BATTERY (27%)")
    assert dock is core.tasks["dock"] and dock.state == TaskState.PENDING
    assert core.tasks["s1"].state == TaskState.FAILED and core.tasks["s2"].reason == "LOW_BATTERY (27%)"


def test_fixed_role_task_is_not_reassigned_to_other_robot():
    core = ManagerCore(["robot4", "robot3"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
        allowed_robot_ids=("robot4",),
    )

    moved = core.reassign_pending_tasks("robot4", "robot3")
    assert moved == 0
    assert core.tasks["survey"].preferred_robot_id == "robot4"


def test_run_completes_only_when_all_tasks_are_terminal():
    core = ManagerCore(["robot4"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
    )

    assert not core.all_tasks_terminal()
    core.finish_task("survey", ok=True, reason="")
    assert core.all_tasks_terminal()

    core.complete_run()
    assert core.run_state == RunState.COMPLETED


def make_robot_status() -> RobotStatusSnapshot:
    return RobotStatusSnapshot(
        state=0,
        battery_percent=100.0,
        accepting_tasks=True,
        task_id="",
        run_id=0,
        task_type=0,
        progress=0.0,
        warning="",
    )


def test_timestamp_based_run_id():
    core = ManagerCore(["robot4"])

    first_run_id = core.start_run(timestamp_sec=1_700_000_000)
    second_run_id = core.start_run(timestamp_sec=1_700_000_000)

    assert first_run_id == 1_700_000_000
    assert second_run_id == 1_700_000_001
    assert core.patrol_phase == PatrolPhase.SURVEY


def test_robot_goes_offline_at_timeout_and_recovers():
    core = ManagerCore(
        ["robot4"],
        offline_timeout_sec=29.0,
    )

    core.update_robot(
        robot_id="robot4",
        now=100.0,
        status=make_robot_status(),
    )

    assert core.check_offline(now=128.999) == []
    assert core.robots["robot4"].online is True

    assert core.check_offline(now=129.0) == ["robot4"]
    assert core.robots["robot4"].online is False

    # 이미 OFFLINE인 로봇을 계속 반환하지 않는다.
    assert core.check_offline(now=130.0) == []

    # RobotStatus가 다시 오면 ONLINE으로 복귀한다.
    core.update_robot(
        robot_id="robot4",
        now=131.0,
        status=make_robot_status(),
    )

    assert core.robots["robot4"].online is True
    assert core.robots["robot4"].last_seen == 131.0


def test_offline_robot_active_task_is_failed():
    core = ManagerCore(["robot4"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
    )
    core.assign_task("survey", "robot4")
    core.start_task("survey")

    failed = core.fail_active_tasks_for_robot(
        "robot4",
        reason="OFFLINE",
    )

    assert failed == ["survey"]
    assert core.tasks["survey"].state == TaskState.FAILED
    assert core.tasks["survey"].reason == "OFFLINE"


def test_blocked_reason_with_details_is_blocked():
    core = ManagerCore(["robot4"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
    )

    core.finish_task(
        "survey",
        ok=False,
        reason="BLOCKED skipped=2",
    )

    assert core.tasks["survey"].state == TaskState.BLOCKED
