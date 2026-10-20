#!/usr/bin/env python3
# PLAN 3-4: goal 허용 오차 단계 시험. A↔B 를 오가며 단계마다 도착 시간·도착 근처 수정 횟수·최종 오차를 기록
#   python3 tol_check.py --ros-args -r __ns:=/robot3 -p rounds:=3   (-p level:=0 이면 LEVELS[0] 한 단계만)
# 도착 방향은 진행 방향에서 90° 돌린 값 → 매번 "전진 → 제자리 90° 회전" 으로 끝나야 정상 (수정 0회)
# 수정 횟수: 목표 0.3 m 안의 움직임 종류(전진 F·좌회전 L·우회전 R) 구간 수 - 2 ("F→회전" 한 번은 정상 = 0회)
# 최종 오차는 Nav2 feedback 의 현재 위치(AMCL 기준). 실제 바닥 오차는 테이프 실측으로 따로
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

LOG = os.path.expanduser('~/scv_logs/tol_check.csv')
A, B = (1.0, 1.5, 0.0), (1.0, 3.0, 180.0)        # (x m, y m, 도착 yaw 도)
LEVELS = [(0.03, 3.0), (0.05, 5.0), (0.08, 8.0), (0.10, 10.0)]   # (xy m, yaw 도)
NORMAL = (0.05, 0.087)                             # 끝나면 되돌릴 nav2_rpp.yaml 값 (m, rad)


def set_tol(nav, cli, xy, yaw_rad):
    cli.wait_for_service(timeout_sec=5.0)
    f = cli.call_async(SetParameters.Request(parameters=[
        Parameter('general_goal_checker.xy_goal_tolerance', value=xy).to_parameter_msg(),
        Parameter('general_goal_checker.yaw_goal_tolerance', value=yaw_rad).to_parameter_msg()]))
    rclpy.spin_until_future_complete(nav, f)


def run_leg(nav, goal, cmd):
    pose = nav.getPoseStamped(goal[:2], goal[2])
    cmd.clear()
    t0 = time.time()
    nav.goToPose(pose)
    last = None
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            last = fb.current_pose.pose
    dt = time.time() - t0
    # 목표 0.3 m 안에 들어온 뒤의 움직임 종류 변화
    kinds = []
    for near, vx, wz in cmd:
        if not near:
            continue
        k = 'F' if vx > 0.02 else ('L' if wz > 0.05 else ('R' if wz < -0.05 else None))
        if k and (not kinds or kinds[-1] != k):
            kinds.append(k)
    if last is None:
        return dt, -1, ''.join(kinds), math.nan, math.nan, str(nav.getResult())
    p, q = last.position, last.orientation
    err_xy = math.hypot(p.x - goal[0], p.y - goal[1])
    yaw = math.degrees(2 * math.atan2(q.z, q.w))
    err_yaw = abs((yaw - goal[2] + 180) % 360 - 180)
    return dt, max(len(kinds) - 2, 0), ''.join(kinds), err_xy, err_yaw, str(nav.getResult())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = TurtleBot4Navigator()
    rounds = nav.declare_parameter('rounds', 3).value
    level = nav.declare_parameter('level', -1).value   # -1: 모든 단계
    nav.waitUntilNav2Active(localizer='robot_localization')   # 초기 pose 는 건드리지 않음 (course_run 과 같은 이유)
    cli = nav.create_client(SetParameters, 'controller_server/set_parameters')
    cmd, goal_now = [], [None]

    def on_cmd(m):
        if goal_now[0] and nav.feedback:
            p = nav.feedback.current_pose.pose.position
            near = math.hypot(p.x - goal_now[0][0], p.y - goal_now[0][1]) < 0.3
            cmd.append((near, m.twist.linear.x, m.twist.angular.z))
    nav.create_subscription(TwistStamped, 'cmd_vel', on_cmd, 10)

    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    new = not os.path.exists(LOG)
    try:
        with open(LOG, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['time', 'robot', 'tol_xy_m', 'tol_yaw_deg', 'leg', 'sec', 'corrections',
                            'motions', 'err_xy_cm', 'err_yaw_deg', 'result'])
            nav.info('시작점 A 로 이동')
            goal_now[0] = A
            run_leg(nav, A, cmd)
            for xy, yaw_deg in (LEVELS if level < 0 else [LEVELS[level]]):
                set_tol(nav, cli, xy, math.radians(yaw_deg))
                for i in range(rounds * 2):
                    goal = B if i % 2 == 0 else A
                    goal_now[0] = goal
                    dt, n, kinds, exy, eyaw, res = run_leg(nav, goal, cmd)
                    leg = 'A→B' if goal is B else 'B→A'
                    nav.info(f'{xy} m/{yaw_deg}° {leg}: {dt:.1f}초 수정 {n}회({kinds}) '
                             f'오차 {exy * 100:.1f} cm {eyaw:.1f}° {res}')
                    w.writerow([time.strftime('%F %T'), nav.get_namespace(), xy, yaw_deg, leg,
                                f'{dt:.1f}', n, kinds, f'{exy * 100:.1f}', f'{eyaw:.1f}', res])
                    f.flush()
    except KeyboardInterrupt:
        nav.cancelTask()   # Ctrl+C 로 꺼도 Nav2 goal 이 남지 않게
    finally:
        set_tol(nav, cli, *NORMAL)
        nav.info(f'허용 오차 되돌림 {NORMAL} → {LOG}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
