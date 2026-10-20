#!/usr/bin/env python3
"""실제 카메라 입력과 검출 노드 출력을 읽어서 연결 상태를 기록한다."""

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    """로봇을 움직이지 않고 원본·검출 영상의 수신 장수와 크기를 확인한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=25)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    namespace = '/' + args.namespace.strip('/')
    report = {'namespace': namespace, 'raw': {'frames': 0}, 'detection': {'frames': 0},
              'accuracy': 'Not measured: no ground-truth labels'}
    rclpy.init(args=[])
    node = Node('scv_camera_connection_check')

    def receive(message, name):
        """JPEG을 읽어 사진 크기와 장수를 기록하고 마지막 사진을 보관한다."""
        frame = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        report[name]['frames'] += 1
        report[name]['width'] = frame.shape[1]
        report[name]['height'] = frame.shape[0]
        cv2.imwrite(str(args.output_dir / (name + '.jpg')), frame)

    subscriptions = [
        node.create_subscription(CompressedImage, namespace + '/oakd/rgb/image_raw/compressed',
                                 lambda msg: receive(msg, 'raw'), qos_profile_sensor_data),
        node.create_subscription(CompressedImage, namespace + '/fire_extinguisher/detections/compressed',
                                 lambda msg: receive(msg, 'detection'), qos_profile_sensor_data),
    ]
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    (args.output_dir / 'connection.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['raw']['frames'] or not report['detection']['frames']:
        raise SystemExit('Camera input or detection output not received')


if __name__ == '__main__':
    main()
