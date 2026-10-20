#!/usr/bin/env python3
# PLAN 3-2·3-3: 경로 이탈(cross-track). Nav2 첫 전역 경로를 기준선으로, 로봇(map→base_link)과의 최단 거리를 10 Hz로 기록
#   python3 track_check.py --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static -p tag:=mppi_1
# 이 스크립트를 먼저 켜고 주행 goal 을 보냄. 도착 후 Ctrl+C → 평균·최대 이탈 출력, ~/scv_logs/track_check.csv 에 한 줄 추가
# AMCL 추정 위치 기준이므로 컨트롤러 추종 정도를 보는 값 (실제 바닥 오차는 pose_check + 테이프 실측)
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

LOG = os.path.expanduser('~/scv_logs/track_check.csv')


def dist_to_path(px, py, pts):
    # 점(px, py)에서 꺾은선 pts 까지 최단 거리
    best = math.inf
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(px - ax - t * dx, py - ay - t * dy))
    return best


class TrackCheck(Node):
    def __init__(self):
        super().__init__('track_check')
        self.tag = self.declare_parameter('tag', '').value
        self.path = None   # 첫 경로만 사용 (재계획 무시, 같은 기준선으로 비교)
        self.last = None
        self.errs = []
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_timer(0.1, self.sample)
        self.get_logger().info('경로 대기 중 — goal 을 보내세요')

    def on_plan(self, msg):
        if self.path is None and len(msg.poses) > 1:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.get_logger().info(f'기준 경로 저장: {len(self.path)}점')

    def sample(self):
        if self.path is None:
            return
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform.translation
        except Exception:
            self.get_logger().warn('tf map→base_link 없음', throttle_duration_sec=5.0)
            return
        # 정지 중(도착 후 등) 샘플은 평균을 낮추므로 1 cm 이상 움직였을 때만 기록
        if self.last and math.hypot(t.x - self.last[0], t.y - self.last[1]) < 0.01:
            return
        self.last = (t.x, t.y)
        self.errs.append(dist_to_path(t.x, t.y, self.path))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TrackCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    e = node.errs
    if e:
        mean, mx = sum(e) / len(e) * 100, max(e) * 100
        print(f'{node.tag} 샘플 {len(e)}개: 평균 {mean:.1f} cm, 최대 {mx:.1f} cm')
        os.makedirs(os.path.dirname(LOG), exist_ok=True)
        new = not os.path.exists(LOG)
        with open(LOG, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['time', 'robot', 'tag', 'samples', 'mean_cm', 'max_cm'])
            w.writerow([time.strftime('%F %T'), node.get_namespace(), node.tag,
                        len(e), f'{mean:.2f}', f'{mx:.2f}'])
        print(f'→ {LOG}')
    else:
        print('기록 없음 (경로 또는 tf 를 받지 못함)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
