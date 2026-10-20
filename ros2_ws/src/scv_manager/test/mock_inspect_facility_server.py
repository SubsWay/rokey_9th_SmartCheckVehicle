"""묶음 시험용 가짜 비전: {ns}/inspect_facility 서비스(IF-05)를 받아 2초 뒤 NORMAL 을 돌려주고 {ns}/inspect/result 를 낸다.

    python3 mock_inspect_facility_server.py --ros-args -r __ns:=/robot4

자세 보정은 없다 (10/14 PM 결정).
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from scv_msgs.srv import InspectFacility
from scv_msgs.msg import InspectionResult


class MockInspectFacilityServer(Node):
    def __init__(self):
        super().__init__("mock_inspect_facility_server")
        self.result_publisher = self.create_publisher(
            InspectionResult,
            "inspect/result",
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.service = self.create_service(
            InspectFacility,
            "inspect_facility",
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def execute_callback(self, request, response):
        self.get_logger().info(f"설비 점검 요청: {request.facility_id}")
        time.sleep(2.0)

        message = InspectionResult()
        message.stamp = self.get_clock().now().to_msg()
        message.robot_id = self.get_namespace().strip("/")
        message.run_id = request.run_id
        message.task_id = request.task_id
        message.facility_id = request.facility_id
        message.kind = request.kind
        message.verdict = InspectionResult.NORMAL
        message.detail = "mock"
        message.attempts = 1
        self.result_publisher.publish(message)

        response.verdict = InspectionResult.NORMAL
        response.detail = "mock"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockInspectFacilityServer()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
