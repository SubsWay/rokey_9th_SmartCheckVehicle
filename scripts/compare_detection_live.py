#!/usr/bin/env python3
"""Compare old/new models on the same read-only compressed camera frames."""

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

from scv_detection.detection_render import annotate_detections


def main():
    """같은 카메라 사진을 기존 모델과 새 모델에 넣어 판정과 신뢰도를 비교한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--old', type=Path, required=True)
    parser.add_argument('--new', type=Path, required=True)
    parser.add_argument('--topic', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=12)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()
    models = {'old': YOLO(str(args.old)), 'new': YOLO(str(args.new))}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = []
    last_time = 0.0
    rclpy.init(args=[])
    node = Node('scv_live_model_comparison')

    def callback(message):
        """카메라 사진 한 장을 두 모델로 검사하고 결과와 사진 크기를 기록한다."""
        nonlocal last_time
        now = time.monotonic()
        if now - last_time < 0.2:
            return
        last_time = now
        frame = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        cv2.imwrite(str(args.output_dir / 'last_frame.jpg'), frame)
        record = {'width': frame.shape[1], 'height': frame.shape[0]}
        for name, model in models.items():
            result = model.predict(frame, conf=0.25, iou=0.45, imgsz=640, device=0, verbose=False)[0]
            record[name] = [{'class': result.names[int(box.cls.item())],
                             'confidence': float(box.conf.item())} for box in result.boxes]
            if name == 'new' and result.boxes is not None and len(result.boxes):
                cv2.imwrite(str(args.output_dir / 'live_detection.jpg'), annotate_detections(result))
        records.append(record)

    subscription = node.create_subscription(CompressedImage, args.topic, callback,
                                            qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    try:
        while len(records) < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = {'topic': args.topic, 'frames_received': len(records), 'frames': records,
              'sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest()
                         for name, path in [('old', args.old), ('new', args.new)]},
              'accuracy': 'Not measured: live scene ground truth is not annotated',
              'confidence_summary': {}}
    for name in models:
        grouped = defaultdict(list)
        for record in records:
            for box in record[name]:
                grouped[box['class']].append(box['confidence'])
        report['confidence_summary'][name] = {
            label: {'detections': len(values), 'min': min(values), 'max': max(values),
                    'mean': sum(values) / len(values)} for label, values in grouped.items()}
    (args.output_dir / 'live.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not records:
        raise SystemExit('No live frame received; camera performance remains unverified')


if __name__ == '__main__':
    main()
