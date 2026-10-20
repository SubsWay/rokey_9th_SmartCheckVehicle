#!/usr/bin/env python3
"""Exercise the installed ROS executable with a synthetic compressed camera."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    """화면 없이 실제 실행기를 켜고 가짜 카메라 사진을 보내 결과 사진이 오는지 확인한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--namespace', default='review_camera')
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--bringup', action='store_true')
    parser.add_argument('--rack-only', action='store_true')
    args = parser.parse_args()
    env = os.environ.copy()
    env.pop('DISPLAY', None)
    env.pop('WAYLAND_DISPLAY', None)
    namespace = '/' + args.namespace.strip('/')
    if args.bringup:
        command = ['ros2', 'launch', 'scv_bringup', 'robot_pc.launch.py',
                   'robot_ns:=' + args.namespace, 'detection_weights:=' + args.weights,
                   'start_detection_diagnostics:=true', 'detection_show:=false',
                   'detection_device:=cpu']
        if args.rack_only:
            command.extend(['detection_enable_yolo:=false', 'detection_rack_enabled:=true'])
    elif args.launch:
        command = ['ros2', 'launch', 'scv_detection', 'detection_alert.launch.py',
                   'robot_ns:=' + args.namespace, 'weights:=' + args.weights,
                   'show:=false', 'device:=cpu']
        if args.rack_only:
            command.extend(['enable_yolo:=false', 'rack_enabled:=true'])
    else:
        command = ['ros2', 'run', 'scv_detection', 'scv_detection_manager', '--ros-args',
                   '-r', '__ns:=' + namespace, '-p', 'weights:=' + args.weights,
                   '-p', 'show:=false', '-p', 'device:=cpu']
        if args.rack_only:
            command.extend(['-p', 'enable_yolo:=false', '-p', 'rack_enabled:=true'])
    rclpy.init(args=[])
    node = Node('scv_headless_smoke', namespace=namespace)
    received = []
    subscription = node.create_subscription(
        CompressedImage, namespace + '/fire_extinguisher/detections/compressed',
        received.append, qos_profile_sensor_data)
    publisher = node.create_publisher(CompressedImage,
        namespace + '/oakd/rgb/image_raw/compressed', qos_profile_sensor_data)
    _, jpeg = cv2.imencode('.jpg', np.zeros((480, 640, 3), dtype=np.uint8))
    message = CompressedImage(format='jpeg', data=jpeg.tobytes())
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open('w') as log:
        process = subprocess.Popen(command, env=env, stdout=log, stderr=log,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 25.0
            while not received and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'Node exited early: {process.returncode}')
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.2)
            if not received:
                raise RuntimeError('No annotated image received')
            output = cv2.imdecode(np.frombuffer(received[0].data, np.uint8), cv2.IMREAD_COLOR)
            assert output.shape == (480, 640, 3)
            assert process.poll() is None
            print('PASS: installed scv_detection_manager runs without DISPLAY; 640x480 JPEG output received')
        finally:
            if process.poll() is None:
                # launch가 자식 노드에 종료 신호를 전달하므로 부모에게만 한 번 보낸다.
                if args.launch or args.bringup:
                    process.send_signal(signal.SIGINT)
                else:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
    print(args.log.read_text())


if __name__ == '__main__':
    main()
