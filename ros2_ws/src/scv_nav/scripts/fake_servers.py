#!/usr/bin/env python3
# PLAN 5-2·4-1: 다른 팀 노드 대신 쓰는 가짜 서버
#   {ns}/inspect_facility (비전, 서비스 IF-05): 2초 뒤 NORMAL
#   {ns}/survey/precise (RSSI): duration_s 동안 1초마다 피드백 → 고정 값
#   {ns}/keepout_mask (RSSI): -p mask:=true 일 때 map 과 같은 크기의 마스크에 사각형 하나를 점유(100)로 발행
#   python3 fake_servers.py --ros-args -r __ns:=/robot3
#   python3 fake_servers.py --ros-args -r __ns:=/robot3 -p mask:=true -p rect:="[1.0, 2.0, 2.0, 3.0]" -p clear_after:=30.0
# rect = [x0, y0, x1, y1] (m, map 좌표). clear_after 초 뒤 빈 마스크로 바꿔 해제도 확인 (0 이면 해제 안 함)
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from scv_msgs.action import PreciseMeasure
from scv_msgs.srv import InspectFacility
from scv_msgs.msg import InspectionResult

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FakeServers(Node):
    def __init__(self):
        super().__init__('fake_servers')
        cb = ReentrantCallbackGroup()
        self.create_service(InspectFacility, 'inspect_facility', self.inspect, callback_group=cb)
        ActionServer(self, PreciseMeasure, 'survey/precise', self.precise, callback_group=cb)
        if self.declare_parameter('mask', False).value:
            self.rect = self.declare_parameter('rect', [1.0, 2.0, 2.0, 3.0]).value
            self.clear_after = self.declare_parameter('clear_after', 0.0).value
            self.mask_pub = self.create_publisher(OccupancyGrid, 'keepout_mask', LATCHED)
            self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)
        self.get_logger().info('fake_servers ready')

    def inspect(self, g, res):
        self.get_logger().info(f'inspect_facility {g.facility_id} kind={g.kind} run_id={g.run_id}')
        time.sleep(2.0)
        res.verdict, res.detail, res.image_ref = InspectionResult.NORMAL, 'fake', ''
        return res

    def precise(self, gh):
        n = gh.request.duration_s
        self.get_logger().info(f'survey/precise {n} s')
        for i in range(n):
            if gh.is_cancel_requested:
                gh.canceled()
                return PreciseMeasure.Result()
            gh.publish_feedback(PreciseMeasure.Feedback(elapsed_s=i + 1))
            time.sleep(1.0)
        gh.succeed()
        return PreciseMeasure.Result(rssi_median=-55.0, rssi_iqr=2.0, n_samples=n)

    def on_map(self, m):
        info = m.info
        mask = OccupancyGrid(header=m.header, info=info)
        mask.data = [0] * (info.width * info.height)   # 모르는 칸도 0 (-1 이면 통과 불가로 보일 수 있음)
        empty = list(mask.data)
        x0, y0, x1, y1 = self.rect
        r = info.resolution
        for row in range(int((y0 - info.origin.position.y) / r), int((y1 - info.origin.position.y) / r)):
            for col in range(int((x0 - info.origin.position.x) / r), int((x1 - info.origin.position.x) / r)):
                if 0 <= row < info.height and 0 <= col < info.width:
                    mask.data[row * info.width + col] = 100
        self.mask_pub.publish(mask)
        self.get_logger().info(f'keepout_mask 발행: {self.rect}')
        if self.clear_after > 0:
            time.sleep(self.clear_after)   # 한 번만 받는 latched 토픽이라 콜백에서 기다려도 됨
            mask.data = empty
            self.mask_pub.publish(mask)
            self.get_logger().info('keepout_mask 해제 (빈 마스크)')


def main():
    rclpy.init()
    ex = MultiThreadedExecutor()
    ex.add_node(FakeServers())
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
