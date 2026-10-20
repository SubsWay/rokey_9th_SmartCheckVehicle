#!/usr/bin/env python3
# PLAN 5-1·5-2: 가짜 manager. mission_executor 의 {ns}/execute_task 로 작업 한 건을 보내 왕복·취소 확인
#   python3 fake_manager.py --ros-args -r __ns:=/robot3                      # 완주까지
#   python3 fake_manager.py --ros-args -r __ns:=/robot3 -p cancel_after:=5.0 # 5초 뒤 취소
#   python3 fake_manager.py --ros-args -r __ns:=/robot3 -p type:=facility -p target:=EXT-01   # survey·facility·reinspection·dock
# facility·reinspection 은 fake_servers.py 를 같이 켬. 마지막 waypoint 가 정밀 도착 지점
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scv_msgs.action import ExecuteTask

# (x m, y m, yaw 도). course_run 첫 직선 위 두 점 + 가운데 틈 너머 한 점. 4-2 막힘 시험은 가운데 틈(y 3.25–3.75, x 0–2.0)을 keepout 으로 막음
WAYPOINTS = [(1.0, 1.5, 90.0), (1.0, 2.5, 90.0), (1.0, 4.2, 90.0)]


def pose(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
    return p


def main():
    rclpy.init()
    node = Node('fake_manager')
    cancel_after = node.declare_parameter('cancel_after', 0.0).value
    kind = node.declare_parameter('type', 'survey').value
    target = node.declare_parameter('target', 'EXT-01').value
    run_id = node.declare_parameter('run_id', 1).value   # 작업 뒤에도 RobotStatus.run_id 에 남는지 확인용
    client = ActionClient(node, ExecuteTask, 'execute_task')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('execute_task 서버 없음 — mission_executor 실행·namespace 확인')
        return

    types = {'survey': ExecuteTask.Goal.SURVEY_SEGMENT, 'facility': ExecuteTask.Goal.FACILITY_CHECK,
             'reinspection': ExecuteTask.Goal.REINSPECTION, 'dock': ExecuteTask.Goal.DOCK}
    wps = [] if kind == 'dock' else [pose(*w) for w in WAYPOINTS]   # dock 은 waypoint 없이 → undock 위치로
    goal = ExecuteTask.Goal(task_id='fake-1', type=types[kind], waypoints=wps, target_ref=target, run_id=run_id)
    fb = lambda m: node.get_logger().info(f'feedback: {m.feedback.waypoint_index} ({m.feedback.progress:.0%})')
    future = client.send_goal_async(goal, feedback_callback=fb)
    rclpy.spin_until_future_complete(node, future)
    gh = future.result()
    if not gh.accepted:
        node.get_logger().error('거절됨 (이미 작업 중?)')
        return
    node.get_logger().info('수락됨')

    result = gh.get_result_async()
    t0 = time.time()
    while not result.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if cancel_after and time.time() - t0 > cancel_after:
            node.get_logger().info('취소 요청')
            rclpy.spin_until_future_complete(node, gh.cancel_goal_async())
            cancel_after = 0.0
    r = result.result()
    node.get_logger().info(f'결과: status={r.status} ok={r.result.ok} reason="{r.result.reason}" '
                           f'rssi_median={r.result.rssi_median:.1f} rssi_iqr={r.result.rssi_iqr:.1f} n_samples={r.result.n_samples}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
