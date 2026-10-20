"""IF-04: 정지 재점검용 새 NetRaw를 10~15초 모아 검출/미검출 모두 요약한다."""
import statistics
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from scv_msgs.action import PreciseMeasure
from scv_msgs.msg import NetRaw
from scv_survey.remeasurement import MIN_SAMPLES, MeasurementWindow
from scv_survey.rssi_scanner import RAW_QOS


class PreciseServer(Node):
    def __init__(self):
        super().__init__('precise_server')
        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.window = None
        self.busy = False
        self.robot_id = self.get_namespace().strip('/')
        self.create_subscription(NetRaw, 'survey/raw', self.on_raw, RAW_QOS, callback_group=cb)
        ActionServer(self, PreciseMeasure, 'survey/precise', self.execute,
                     goal_callback=self.on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)

    def on_goal(self, goal):
        with self.lock:
            if self.busy or not 10 <= goal.duration_s <= 15:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def on_raw(self, m):
        with self.lock:
            if self.window is not None:
                self.window.add(m.stamp.sec * 10**9 + m.stamp.nanosec, m.robot_id,
                                m.detected, m.rssi_dbm, self.get_clock().now().nanoseconds)

    def execute(self, gh):
        res = PreciseMeasure.Result()
        t0 = time.monotonic()
        try:
            with self.lock:
                self.window = MeasurementWindow(self.get_clock().now().nanoseconds,
                                                gh.request.duration_s, self.robot_id)
            while time.monotonic() - t0 < gh.request.duration_s and not gh.is_cancel_requested:
                gh.publish_feedback(PreciseMeasure.Feedback(elapsed_s=int(time.monotonic() - t0)))
                time.sleep(.1)
            with self.lock:
                st = self.window.stats
                self.window = None
            res.duration_s = float(time.monotonic() - t0)
            res.n_samples, res.n_missed, res.n_total = st.n_detected, st.n_missed, st.n_total
            if st.rssi:
                res.rssi_mean, res.rssi_median = float(st.mean), float(st.median)
                if len(st.rssi) >= 2:
                    q = statistics.quantiles(st.rssi, n=4)
                    res.rssi_iqr = float(q[2] - q[0])
            res.measurement_valid = not gh.is_cancel_requested and st.n_total >= MIN_SAMPLES
            res.measurement_reason = '' if res.measurement_valid else (
                'CANCELED' if gh.is_cancel_requested else 'INSUFFICIENT_SAMPLES')
            if gh.is_cancel_requested:
                gh.canceled()
            elif res.measurement_valid:
                gh.succeed()
            else:
                gh.abort()
            return res
        finally:
            with self.lock:
                self.window, self.busy = None, False


def main(args=None):
    rclpy.init(args=args)
    node = PreciseServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
