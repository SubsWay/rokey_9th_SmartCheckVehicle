#!/usr/bin/env python3
# PLAN 3-2·3-3: 같은 코스를 한 번 주행 (컨트롤러마다 5회 반복 실행)
#   python3 course_run.py --ros-args -r __ns:=/robot3
# 순서: 시작점으로 이동 → Enter 대기(이때 track_check 를 켬) → COURSE 웨이포인트를 차례로 통과
# 코스(testbed_v1, 약 7.2 m): 가운데 틈(y≈3.55)·위쪽 틈(y≈4.8)을 가운데로 지나고 90° 코너 3개. 골판지 레일(x 1.9–2.0, y 2.1–2.8)은 피함
import rclpy
from rclpy.signals import SignalHandlerOptions
from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

# (x m, y m, yaw 도). 첫 점이 시작점. yaw 는 진행 방향 (navigate_through_poses 는 마지막 점의 yaw 만 맞춤)
COURSE = [
    (1.0, 1.0, 90.0),
    (1.0, 3.55, 90.0),    # 가운데 틈 (x 0.05–1.95, 실측 1.98 m)
    (1.0, 4.2, 0.0),
    (2.2, 4.2, 90.0),
    (2.2, 5.6, 180.0),    # 위쪽 틈 (x 1.15–3.2, 실측 2.05 m) 지나서
    (0.8, 5.6, 180.0),
]


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 뒤에도 goal 취소를 보낼 수 있게
    nav = TurtleBot4Navigator()
    # 'amcl' 이면 amcl_pose 를 받을 때까지 (0,0) 초기 pose 를 보냄 → 초기 pose 는 건드리지 않고 Nav2 만 기다림
    nav.waitUntilNav2Active(localizer='robot_localization')
    poses = [nav.getPoseStamped([x, y], yaw) for x, y, yaw in COURSE]

    try:
        nav.info('시작점으로 이동')
        nav.startToPose(poses[0])
        input('시작점 도착. track_check 를 켠 뒤 Enter')

        nav.startThroughPoses(poses[1:])
        result = nav.getResult()
        nav.info(f'코스 결과: {result}')
        if result != TaskResult.SUCCEEDED:
            nav.error('코스 실패 — 이 회차는 비교에서 제외')
    except KeyboardInterrupt:
        nav.cancelTask()   # Ctrl+C 로 꺼도 Nav2 goal 이 남아 로봇이 계속 가지 않게
        nav.info('goal 취소')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
