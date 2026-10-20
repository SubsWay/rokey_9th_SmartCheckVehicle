#!/usr/bin/env python3
# PLAN 2-1·2-2·2-3: AMCL 위치(x, y, yaw)와 표준편차를 1초마다 출력, Enter 로 그 순간 값을 CSV에 기록
#   python3 pose_check.py --ros-args -r __ns:=/robot3
# 2-1: undock 직후 RViz 2D Pose Estimate 로 맞춘 뒤 x, y, yaw 를 mission_executor 인자로 사용
# 2-2: 테이프 기준점 이름(예: P1)을 입력하고 Enter → ~/scv_logs/pose_check.csv 에 한 줄 추가
# 2-3: 같은 CSV 의 표준편차로 pose_ok 기준값을 정함
import csv
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped

LOG = os.path.expanduser('~/scv_logs/pose_check.csv')


def to_row(msg):
    # (x, y, yaw°, std_x cm, std_y cm, std_yaw °)
    p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return (p.x, p.y, math.degrees(yaw),
            math.sqrt(max(c[0], 0.0)) * 100, math.sqrt(max(c[7], 0.0)) * 100, math.degrees(math.sqrt(max(c[35], 0.0))))


class PoseCheck(Node):
    def __init__(self):
        super().__init__('pose_check')
        self.row = None
        # amcl_pose 는 transient_local 로 발행됨 → 마지막 값을 바로 받음
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, qos)
        self.create_timer(1.0, self.show)

    def on_pose(self, msg):
        self.row = to_row(msg)

    def show(self):
        if self.row is None:
            self.get_logger().warn('amcl_pose 수신 없음 — Nav2·초기 pose 확인', throttle_duration_sec=5.0)
            return
        self.get_logger().info('x=%.3f y=%.3f yaw=%.1f° | std x=%.1fcm y=%.1fcm yaw=%.1f°' % self.row)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PoseCheck()
    spin = threading.Thread(target=rclpy.spin, args=(node,))
    spin.start()
    try:
        while True:
            label = input('기준점 이름 입력 후 Enter 로 기록 (Ctrl+C 종료)\n').strip()
            if node.row is None:
                print('아직 amcl_pose 없음, 기록 안 함')
                continue
            os.makedirs(os.path.dirname(LOG), exist_ok=True)
            new = not os.path.exists(LOG)
            with open(LOG, 'a', newline='') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(['time', 'robot', 'label', 'x', 'y', 'yaw_deg',
                                'std_x_cm', 'std_y_cm', 'std_yaw_deg'])
                w.writerow([time.strftime('%F %T'), node.get_namespace(), label]
                           + [f'{v:.4f}' for v in node.row])
            print(f'기록: {label} → {LOG}')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.shutdown()
        spin.join()


if __name__ == '__main__':
    main()
