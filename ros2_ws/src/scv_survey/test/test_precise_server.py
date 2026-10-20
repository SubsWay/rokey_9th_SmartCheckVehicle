"""격리 ROS 액션 시험: 스캔 발행만 하며 주행 노드·동작 명령은 사용하지 않는다."""
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scv_msgs.action import PreciseMeasure
from scv_msgs.msg import NetRaw
from scv_survey.precise_server import PreciseServer
from scv_survey.remeasurement import classify
from scv_survey.rssi_scanner import RAW_QOS


def wait(future, timeout=15):
    until = time.monotonic() + timeout
    while not future.done() and time.monotonic() < until:
        time.sleep(.02)
    assert future.done(), '액션 응답 시간 초과'
    return future.result()


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '191')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot4'])
    server, client_node = PreciseServer(), Node('remeasure_test_client')
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(server)
    ex.add_node(client_node)
    thread = threading.Thread(target=ex.spin)
    thread.start()
    client = ActionClient(client_node, PreciseMeasure, 'survey/precise')
    pub = client_node.create_publisher(NetRaw, 'survey/raw', RAW_QOS)
    assert client.wait_for_server(timeout_sec=5)
    yield server, client_node, client, pub
    ex.shutdown(timeout_sec=3)
    thread.join(timeout=3)
    client_node.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('mode', ['mixed', 'all_missed', 'no_messages'])
def test_measurement_action_preserves_attempts_and_fails_without_samples(ros, mode):
    _, node, client, pub = ros
    handle = wait(client.send_goal_async(PreciseMeasure.Goal(duration_s=10)))
    assert handle.accepted
    future = handle.get_result_async()
    sent = 0
    until = time.monotonic() + 11
    while not future.done() and time.monotonic() < until:
        if mode != 'no_messages':
            msg = NetRaw(robot_id='robot4', detected=mode == 'mixed' and sent == 0, rssi_dbm=-60)
            msg.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            sent += 1
        time.sleep(.15)
    outcome = wait(future)
    result = outcome.result
    if mode == 'no_messages':
        assert outcome.status == GoalStatus.STATUS_ABORTED
        assert result.n_total == 0 and not result.measurement_valid
    else:
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert result.n_missed >= 3
        assert result.n_total == result.n_samples + result.n_missed
        assert result.n_samples <= 1
        assert classify(result.rssi_mean, result.n_samples, result.n_missed) == 'miss'


def test_rejects_old_duration_concurrent_goal_and_cancels_without_success(ros):
    _, _, client, _ = ros
    assert not wait(client.send_goal_async(PreciseMeasure.Goal(duration_s=30))).accepted
    first = wait(client.send_goal_async(PreciseMeasure.Goal(duration_s=10)))
    assert first.accepted
    assert not wait(client.send_goal_async(PreciseMeasure.Goal(duration_s=10))).accepted
    wait(first.cancel_goal_async())
    outcome = wait(first.get_result_async())
    assert outcome.status == GoalStatus.STATUS_CANCELED
    assert not outcome.result.measurement_valid
