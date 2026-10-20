import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scv_msgs.action import ExecuteTask
from scv_msgs.msg import NetSample, RobotStatus


class MockExecuteTaskServer(Node):
    def __init__(self):
        super().__init__("mock_execute_task_server")
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("step_delay_sec", 0.2)
        self.step_delay_sec = float(
            self.get_parameter("step_delay_sec").value
        )
        self.declare_parameter("status_duration_sec", 0.0)
        self.status_duration_sec = float(
            self.get_parameter("status_duration_sec").value
        )
        # 묶음 시험: SURVEY 중 경로를 따라 가짜 RSSI 샘플을 낸다.
        # 사각형은 [x0, y0, x1, y1] 을 이어 붙인 목록 (map 좌표, 칸 경계에 맞춤)
        self.declare_parameter("publish_samples", True)
        self.declare_parameter("sample_step_m", 0.25)
        self.declare_parameter(
            "shadow_rects",
            [1.35, 1.75, 1.85, 2.25, 0.85, 4.75, 1.35, 5.25],
        )
        self.declare_parameter("miss_rects", [2.35, 2.75, 2.85, 3.25])
        self.declare_parameter("clear_on_remeasure", ["cell:3,4"])
        self.started_at = time.monotonic()
        self.robot_ids = ("robot4", "robot3")
        self.sample_seq = 0
        self.sample_publishers = {
            robot_id: self.create_publisher(
                NetSample,
                f"/{robot_id}/survey/sample",
                50,
            )
            for robot_id in self.robot_ids
        }
        self.status_publishers = {
            robot_id: self.create_publisher(
                RobotStatus,
                f"/{robot_id}/status",
                10,
            )
            for robot_id in self.robot_ids
        }
        self.robot_status = {
            robot_id: {
                "state": RobotStatus.STATE_DOCKED,
                "task_id": "",
                "run_id": 0,
                "task_type": 0,
                "progress": 0.0,
            }
            for robot_id in self.robot_ids
        }
        self.action_servers = [
            ActionServer(
                self,
                ExecuteTask,
                f"/{robot_id}/execute_task",
                lambda goal_handle, rid=robot_id: self.execute_callback(
                    rid,
                    goal_handle,
                ),
                cancel_callback=self.cancel_callback,
                callback_group=self.callback_group,
            )
            for robot_id in self.robot_ids
        ]
        self.status_timer = self.create_timer(1.0, self.publish_statuses)
        self.get_logger().info(
            "mock 서버 시작: /robot4/execute_task, /robot3/execute_task"
        )

    @staticmethod
    def cancel_callback(goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def publish_statuses(self):
        if (
            self.status_duration_sec > 0.0
            and time.monotonic() - self.started_at
            >= self.status_duration_sec
        ):
            return

        for robot_id, values in self.robot_status.items():
            message = RobotStatus()
            message.stamp = self.get_clock().now().to_msg()
            message.robot_id = robot_id
            message.state = values["state"]
            message.battery_percent = 100.0
            message.accepting_tasks = True
            message.task_id = values["task_id"]
            message.run_id = values["run_id"]
            message.task_type = values["task_type"]
            message.progress = values["progress"]
            message.pose_ok = True
            message.warning = ""
            self.status_publishers[robot_id].publish(message)

    def in_rects(self, name, x, y):
        values = list(self.get_parameter(name).value)
        return any(
            values[k] <= x < values[k + 2] and values[k + 1] <= y < values[k + 3]
            for k in range(0, len(values) - 3, 4)
        )

    def publish_samples(self, robot_id, request):
        step = float(self.get_parameter("sample_step_m").value)
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in request.waypoints
        ]

        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            count = max(int(math.dist((x0, y0), (x1, y1)) / step), 1)

            for k in range(count):
                sample = NetSample()
                sample.seq = self.sample_seq
                self.sample_seq += 1
                sample.stamp = self.get_clock().now().to_msg()
                sample.robot_id = robot_id
                sample.run_id = request.run_id
                sample.x = x0 + (x1 - x0) * k / count
                sample.y = y0 + (y1 - y0) * k / count
                sample.pose_ok = True
                sample.detected = not self.in_rects(
                    "miss_rects",
                    sample.x,
                    sample.y,
                )
                sample.rssi_dbm = (
                    -80
                    if self.in_rects("shadow_rects", sample.x, sample.y)
                    else -55
                )
                self.sample_publishers[robot_id].publish(sample)

    def execute_callback(self, robot_id, goal_handle):
        request = goal_handle.request
        self.get_logger().info(
            f"{robot_id} 작업 수신: {request.task_id} type={request.type}"
        )
        values = self.robot_status[robot_id]
        values["state"] = RobotStatus.STATE_EXECUTING
        values["task_id"] = request.task_id
        values["run_id"] = request.run_id
        values["task_type"] = request.type
        values["progress"] = 0.0
        waypoint_count = max(len(request.waypoints), 1)

        if (
            request.type == ExecuteTask.Goal.SURVEY_SEGMENT
            and self.get_parameter("publish_samples").value
        ):
            self.publish_samples(robot_id, request)

        for index in range(waypoint_count):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteTask.Result()
                result.ok = False
                result.reason = "ESTOP"
                values["state"] = RobotStatus.STATE_IDLE
                values["task_id"] = ""
                values["progress"] = 0.0
                return result

            feedback = ExecuteTask.Feedback()
            feedback.waypoint_index = index
            feedback.progress = float(index + 1) / waypoint_count
            goal_handle.publish_feedback(feedback)
            values["progress"] = feedback.progress
            time.sleep(self.step_delay_sec)

        goal_handle.succeed()
        values["state"] = (
            RobotStatus.STATE_DOCKED
            if request.type == ExecuteTask.Goal.DOCK
            else RobotStatus.STATE_IDLE
        )
        values["task_id"] = ""
        values["progress"] = 1.0
        result = ExecuteTask.Result()
        result.ok = True
        result.reason = ""

        if request.type == ExecuteTask.Goal.REINSPECTION:
            cleared = request.target_ref in list(
                self.get_parameter("clear_on_remeasure").value
            )
            # mission_executor 와 같은 꼴 (Result 필드와 reason 둘 다)
            result.rssi_median = -60.0 if cleared else -80.0
            result.rssi_iqr = 2.0
            result.n_samples = result.n_total = 15
            result.n_missed = 0
            result.rssi_mean = result.rssi_median
            result.duration_s = 15.0
            result.measurement_valid = True
            result.reason = (
                f"median={result.rssi_median:.1f} iqr=2.0 n=30"
            )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockExecuteTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
