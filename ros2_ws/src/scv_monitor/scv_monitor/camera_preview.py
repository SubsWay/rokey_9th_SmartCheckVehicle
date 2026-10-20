"""로봇 PC: 관제 블랙박스용 카메라 프레임 (10/15). 실시간 표시는 없고 관제가 회차별로 저장해 이력 탭에서 본다.

{ns}/<source> (CompressedImage) 를 받아 가로 width px · rate Hz JPEG 로 줄여 {ns}/camera/preview 로 낸다.
도킹 중(mission_executor 의 {ns}/status 가 DOCKED · DOCKING)에는 카메라를 구독하지 않는다 — Pi 가 이 PC 로 원본을 보내지 않게.
관제 PC 는 원본이 아니라 이 작은 영상만 받는다 (원본 수 MB/s 가 Wi-Fi 를 채움, 10/15 네트워크 단절).

    ros2 run scv_monitor camera_preview --ros-args -r __ns:=/robot4 -p source:=fire_extinguisher/detections/compressed
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scv_msgs.msg import RobotStatus
from sensor_msgs.msg import CompressedImage

PREVIEW_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
PARKED = {RobotStatus.STATE_DOCKED, RobotStatus.STATE_DOCKING}


class CameraPreview(Node):
    def __init__(self):
        super().__init__('camera_preview')
        self.source = self.declare_parameter('source', 'oakd/rgb/image_raw/compressed').value
        self.width = int(self.declare_parameter('width', 640).value)
        self.period = 1.0 / float(self.declare_parameter('rate', 1.0).value)
        self.quality = int(self.declare_parameter('quality', 60).value)
        self.pub = self.create_publisher(CompressedImage, 'camera/preview', PREVIEW_QOS)
        self.sub = None
        self.last = 0.0
        self.create_subscription(RobotStatus, 'status', self.on_status, 10)
        self.get_logger().info(f'{self.source} → camera/preview ({self.width} px, {1 / self.period:.1f} Hz, 도킹 중엔 끔)')

    def on_status(self, m: RobotStatus):
        parked = m.state in PARKED
        if parked and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.get_logger().info('도킹 — 카메라 구독 끔')
        elif not parked and self.sub is None:
            self.sub = self.create_subscription(CompressedImage, self.source, self.on_image, qos_profile_sensor_data)
            self.get_logger().info('주행 — 카메라 구독 켬')

    def on_image(self, m: CompressedImage):
        now = time.monotonic()
        if now - self.last < self.period:
            return
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        if w > self.width:
            img = cv2.resize(img, (self.width, round(h * self.width / w)), interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            self.last = now
            self.pub.publish(CompressedImage(header=m.header, format='jpeg', data=jpeg.tobytes()))


def main(args=None):
    rclpy.init(args=args)
    node = CameraPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
