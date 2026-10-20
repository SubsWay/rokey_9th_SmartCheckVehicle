#!/usr/bin/env python3
"""평소에는 카메라를 보여 주고 서비스 요청 때 YOLO·랙·방화셔터 중 하나를 검사하는 노드."""

from __future__ import annotations

import os
import json
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scv_msgs.msg import InspectionResult
from scv_msgs.srv import InspectFacility
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from scv_detection import exit_sign, extinguisher_aruco, extinguisher_pressure, rack_conditioner, rack_loading, shutter_obstacle


# 이전 6클래스 상태 모델의 BGR 색상과 로그 상태다. 실제 모델 ID는 이름으로 이 표에 연결한다.
YOLO_CLASS_STYLES = {
    0: ((0, 0, 255), 'ABNORMAL'),      # Abnormal_left
    1: ((0, 0, 255), 'ABNORMAL'),      # Abnormal_right
    2: ((192, 192, 192), 'UNCONFIRMED'),  # Fire Extinguisher: 전체 모습만으로 상태를 알 수 없다.
    3: ((0, 255, 0), 'NORMAL'),        # Normal
    4: ((0, 255, 0), 'NORMAL'),        # Exit_sign_on
    5: ((0, 0, 255), 'ABNORMAL'),      # Exit_sign_off
}

# 단독 압력계와 유도등에 적용할 기존 불량 클래스다. 소화기 내부 압력계는 바늘로 판정한다.
YOLO_DEFECT_KINDS = {
    0: InspectionResult.KIND_EXTINGUISHER,  # 압력계 왼쪽 비정상
    1: InspectionResult.KIND_EXTINGUISHER,  # 압력계 오른쪽 비정상
    5: InspectionResult.KIND_EXIT_LIGHT,    # 비상구 유도등 소등
}


# IF-05 inspect_facility 서비스: 설비 종류 → 이 노드의 검사 모드 (통합 브랜치 10/15)
FACILITY_MODES = {
    InspectionResult.KIND_EXTINGUISHER: 'yolo',   # 압력계 바늘 · ArUco
    InspectionResult.KIND_EXIT_LIGHT: 'yolo',     # 유도등 밝기
    InspectionResult.KIND_SHUTTER: 'shutter',     # 하강 구간 장애물 (RGB-D)
    InspectionResult.KIND_RACK: 'yolo',           # YOLO 랙 · 박스 적재 판정 (rack_loading). 'rack' 은 이전 OpenCV 기준 사진 방식
}
VERDICTS = {'NORMAL': InspectionResult.NORMAL, 'ABNORMAL': InspectionResult.ABNORMAL}
# 관제(ros_bridge · inspection_manager)는 inspect/result 를 TRANSIENT_LOCAL 로 받는다
FACILITY_RESULT_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)


# 객체 위치 모델과 이전 상태 모델을 이름으로 구분하여 ID 순서 변경에 대응한다.
YOLO_LABEL_STYLE_IDS = {'abnormal_left': 0, 'abnormal_right': 1, 'fire_extinguisher': 2,
                        'normal': 3, 'exit_sign_on': 4, 'exit_sign_off': 5}
YOLO_EXTINGUISHER_NAMES = {'fire_extinguisher', 'extinguisher', '소화기'}
YOLO_GAUGE_NAMES = {'pressure_gauge', 'gauge', 'guage', '압력판', '압력계',
                    'normal', 'abnormal_left', 'abnormal_right'}
YOLO_RACK_NAMES = {'rack', '랙'}
YOLO_BOX_NAMES = {'box', '박스'}
YOLO_EXIT_SIGN_NAMES = {'exit_sign', 'exit_sign_on', 'exit_sign_off', 'exit_light',
                        'emergency_exit', '비상구', '비상구표지', '비상구_표지', '유도등'}

# 카메라 설정 조회 대기와 실제 정렬 설정 오류를 구분한다.
SHUTTER_ALIGNMENT_REASONS = {
    'pending': 'depth_alignment_check_pending', 'disabled': 'depth_alignment_disabled',
    'wrong_camera': 'depth_alignment_target_mismatch', 'unsupported': 'depth_alignment_unconfirmed',
}


def normalize_yolo_name(name):
    """모델의 클래스 이름을 소문자·밑줄 형태로 맞춰 대소문자나 띄어쓰기 차이를 줄인다."""
    return '_'.join(str(name).strip().lower().replace('-', ' ').replace('_', ' ').split())


def decode_compressed_depth(message: CompressedImage) -> np.ndarray:
    """compressedDepth 토픽의 PNG 압축 Depth를 풀어 미터 단위 실수 배열로 반환한다.

    message.format은 '16UC1; compressedDepth png' 또는 '32FC1; compressedDepth png' 형식이다.
    data 앞 12바이트는 압축 정보(형식 번호, 32FC1 복원용 상수 2개)이고 그 뒤가 PNG다.
    16UC1은 밀리미터 정수를 그대로 미터로 바꾸며 PNG는 손실이 없으므로 원본 Depth와 값이 같다.
    32FC1은 역수로 양자화된 값을 원래 거리로 되돌린다. 0은 측정 없음(무효)으로 0을 유지한다.
    다른 압축 방식(rvl 등)이나 깨진 데이터는 ValueError로 알린다.
    """
    encoding, _, compression = (part.strip() for part in message.format.partition(';'))
    if compression != 'compressedDepth png' or encoding not in ('16UC1', '32FC1'):
        raise ValueError(f'unsupported compressedDepth format: {message.format!r}')
    data = bytes(message.data)
    if len(data) <= 12:
        raise ValueError('compressedDepth message is too short')
    _, quant_a, quant_b = np.frombuffer(data[:12], dtype='<i4,<f4,<f4')[0]
    image = cv2.imdecode(np.frombuffer(data[12:], np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim != 2 or image.dtype != np.uint16:
        raise ValueError('compressedDepth PNG must decode to a single-channel 16-bit image')
    if encoding == '16UC1':
        return image.astype(np.float64) * 0.001
    depth = np.zeros(image.shape, dtype=np.float64)
    valid = image > 0
    depth[valid] = quant_a / (image[valid].astype(np.float64) - quant_b)
    return depth


class ScvDetectionManager(Node):
    @staticmethod
    def _find_default_weights() -> str:
        """설치된 패키지의 models/best.pt 경로를 찾아 사용자별 절대 경로 없이 반환한다."""
        return str(Path(get_package_share_directory('scv_detection')) / 'models' / 'best.pt')

    @staticmethod
    def _find_default_reference():
        """패키지 소스 config 폴더에서 기준 파일 기본 경로를 찾는다."""
        try:
            from ament_index_python.packages import get_package_prefix
            prefix = Path(get_package_prefix('scv_detection'))
            src_config = prefix.parent.parent / 'src' / 'scv_detection' / 'config'
            if src_config.is_dir():
                return str(src_config / 'rack_reference.npz')
        except Exception:
            pass
        return ''

    def __init__(self, **kwargs) -> None:
        """카메라 정렬 자동 조회와 RGB·Depth 보정·TF·영상 짝·설비 검사 설정을 준비한다."""
        kwargs.setdefault('namespace', '/robot4')
        super().__init__("detection_alert", **kwargs)

        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("show", True)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('rack_enabled', True)
        self.declare_parameter('rack_confirm_frames', 3)
        self.declare_parameter('rack_diff_threshold', 35)
        self.declare_parameter('rack_min_area', 300)
        self.declare_parameter('rack_margin', 30)
        self.declare_parameter('rack_zone_width', 90)
        self.declare_parameter('rack_loading_enabled', True)
        self.declare_parameter('rack_loading_tolerance_ratio', 0.0)
        self.declare_parameter('rack_loading_association_margin', 0.10)
        self.declare_parameter('rack_loading_confirm_frames', 3)
        self.declare_parameter('rack_loading_match_iou', 0.30)
        self.declare_parameter('rack_loading_match_margin', 0.10)
        default_ref = self._find_default_reference()
        self.declare_parameter('rack_reference_path', default_ref)
        self.declare_parameter('camera_timeout', 2.0)
        self.declare_parameter('facility_window_s', 6.0)   # IF-05: 한 설비를 보는 최대 시간
        self.declare_parameter('facility_min_s', 3.0)      # IF-05: 판정이 나도 이만큼은 계속 본다 (첫 한두 프레임으로 응답하지 않게, 10/16)
        self.declare_parameter('pressure_enabled', True)
        self.declare_parameter('aruco_enabled', True)
        self.declare_parameter('exit_sign_enabled', True)
        self.declare_parameter('shutter_enabled', True)
        # 원본 Depth(약 600KB/장)는 Wi-Fi에서 지연·누락이 커서 손실 없는 PNG 압축 Depth(약 160KB/장)를 기본으로 받는다.
        # 이름이 compressedDepth로 끝나지 않으면 원본 Image 토픽으로 구독한다.
        self.declare_parameter('shutter_depth_topic', 'oakd/stereo/image_raw/compressedDepth')
        self.declare_parameter('shutter_camera_info_topic', 'oakd/rgb/camera_info')
        self.declare_parameter('shutter_depth_camera_info_topic', 'oakd/stereo/camera_info')
        self.declare_parameter('shutter_tf_topic', 'tf')
        self.declare_parameter('shutter_tf_static_topic', 'tf_static')
        self.declare_parameter('shutter_ground_frame', 'base_link')
        self.declare_parameter('shutter_camera_node', 'oakd')
        # 기본 실행은 자동 조회한다. true는 기존 수동 정렬 확인 실행과의 호환용이다.
        self.declare_parameter('shutter_depth_aligned', False)
        self.declare_parameter('shutter_rgb_rectified', False)
        # Depth 화면 배율·이동 보정(depth_x = scale_x*rgb_x + offset_x). 1, 0, 1, 0 이면 보정 없음.
        # 2026-10-15 오전 카메라 설정(RGB 640x480)에서는 Depth 가 RGB 보다 넓은 시야를 담아 약 2/3 크기로 어긋났다.
        # 10:40 무렵 카메라 설정을 바꾼 뒤(RGB·Depth 704x704, 둘 다 10Hz) 녹화 shutter_1015_1043 에서는
        # 배율 약 0.98 로 거의 1:1이었고 남은 차이는 RGB 렌즈 왜곡이라 depth_for_rgb 가 이미 보정한다.
        # 그래서 기본값은 보정 없음이다. 카메라 설정이 다시 바뀌면 tools/measure_depth_alignment.py 로 다시 잰다.
        # 바닥 평면 방식: 'ransac' 은 매 프레임 RANSAC, 'fixed' 는 아래 미리 잰 평면을 검증만 하고 쓴다(연산량 절감).
        # 고정 평면은 카메라 광학 좌표계 [nx, ny, nz, d](높이 = p·n + d, d = 카메라의 바닥 위 높이)다.
        # 값은 녹화 shutter_1015_1043 을 초점 거리 1036px(아래 shutter_focal_length_px)로 계산한 RANSAC 48장 중앙값이며
        # d=0.237m 로 TF 카메라 높이 0.244m 와 맞는다(법선 흔들림 최대 0.43°). 초점 거리 569.95 로 쟀던
        # [-0.0017, -0.9994, 0.0334, 0.4237] 과 그 이전 [0.0040, -0.9996, 0.0292, 0.4610] 은 높이가 약 1.8배 부풀려진 값이다.
        # 초점 거리·카메라 설정·장착이 바뀌면 다시 잰다: ransac 모드 로그의 plane= 값을 옮겨 적는다.
        self.declare_parameter('shutter_floor_plane_mode', 'ransac')
        self.declare_parameter('shutter_fixed_floor_plane', [-0.0061, -0.9999, 0.0162, 0.2371])
        self.declare_parameter('shutter_depth_pixel_scale_x', 1.0)
        self.declare_parameter('shutter_depth_pixel_offset_x', 0.0)
        self.declare_parameter('shutter_depth_pixel_scale_y', 1.0)
        self.declare_parameter('shutter_depth_pixel_offset_y', 0.0)
        # 초점 거리 덮어쓰기(픽셀). 0이면 CameraInfo 값을 그대로 쓴다. 2026-10-15 카메라 설정(RGB 1080P → ISP 2/3 =
        # 1280x720 → 가운데 704x704 잘라냄)에서 CameraInfo 는 fx=fy=569.95 를 내지만, 이는 1280→704 로 줄였다고 본 값이고
        # 실제로는 잘라낸 영상이라 569.95 x 1280/704 = 약 1036 이다. 녹화 shutter_1015_1043 에서 569.95 를 쓰면
        # 바닥 평면 카메라 높이 0.43m(TF 0.244m), 상자 높이 약 357mm(실측 185mm)로 약 1.8배 크게 나왔다.
        # 1036 이면 카메라 높이 약 0.237m, 상자 앞면 높이 약 197mm 다(초점 1030~1060 에서 198~192mm). 카메라 해상도·ISP 설정이 바뀌면 다시 정한다.
        # 실행 중 로그·화면의 '카메라 높이 확인' 비율이 1에서 크게 벗어나면 이 값이 틀린 것이다.
        self.declare_parameter('shutter_focal_length_px', 1036.0)
        # 셔터 검사는 로봇이 멈춘 상태에서 하므로 0.1초 정도 앞뒤 사진을 짝지어도 장면이 같다.
        # 시각 차이는 10Hz 한 주기(0.1초)까지, Depth 수신 경과는 Wi-Fi 지연을 생각해 1초까지 허용한다.
        self.declare_parameter('shutter_depth_timeout', 1.0)
        self.declare_parameter('shutter_sync_tolerance', 0.10)
        self.declare_parameter('shutter_font_path', '')
        self.declare_parameter('shutter_font_size', 24)
        for name, default in shutter_obstacle.DEFAULT_OPTIONS.items():
            self.declare_parameter('shutter_' + name, default)
        for name, default in exit_sign.DEFAULT_OPTIONS.items():
            self.declare_parameter('exit_sign_' + name, list(default) if isinstance(default, tuple) else default)
        self.declare_parameter('pressure_min_gauge_diameter', 24)
        self.declare_parameter('pressure_needle_min_pixels', 3)
        self.declare_parameter('pressure_normal_confirm_seconds', 0.5)
        self.declare_parameter('pressure_normal_confirm_frames', 3)
        # 이전 실행 설정과의 호환용이다. 새 바늘 존재 판정에는 각도 경계 여유를 적용하지 않는다.
        self.declare_parameter('pressure_boundary_margin_deg', 5.0)
        self.declare_parameter('pressure_normal_margin_deg', 10.0)   # 정상 구간 양옆 여유 — 비스듬한 시점의 바늘 시차 (10/16 실기)
        self.declare_parameter('pressure_normal_start_deg', -1.0)
        self.declare_parameter('pressure_normal_end_deg', -1.0)
        self.declare_parameter('pressure_yellow_lower', [8, 40, 70])
        self.declare_parameter('pressure_yellow_upper', [35, 255, 255])
        self.declare_parameter('pressure_green_lower', [38, 25, 20])
        self.declare_parameter('pressure_green_upper', [120, 255, 255])
        self.declare_parameter('rack_roi_x1', 0.25)
        self.declare_parameter('rack_roi_y1', 0.30)
        self.declare_parameter('rack_roi_x2', 0.75)
        self.declare_parameter('rack_roi_y2', 0.83)
        self.enable_yolo = bool(self.get_parameter('enable_yolo').value)
        self.rack_enabled = bool(self.get_parameter('rack_enabled').value)
        self.rack_loading_enabled = bool(self.get_parameter('rack_loading_enabled').value)
        self.rack_loading_options = rack_loading.validate_options(
            tolerance_ratio=self.get_parameter('rack_loading_tolerance_ratio').value,
            association_margin=self.get_parameter('rack_loading_association_margin').value,
            required_frames=self.get_parameter('rack_loading_confirm_frames').value,
            match_iou=self.get_parameter('rack_loading_match_iou').value,
            match_margin=self.get_parameter('rack_loading_match_margin').value)
        self.pressure_enabled = bool(self.get_parameter('pressure_enabled').value)
        self.exit_sign_enabled = bool(self.get_parameter('exit_sign_enabled').value)
        self.exit_sign_options = exit_sign.validate_options(**{
            name: self.get_parameter('exit_sign_' + name).value for name in exit_sign.DEFAULT_OPTIONS})
        self.shutter_enabled = bool(self.get_parameter('shutter_enabled').value)
        self.shutter_options = shutter_obstacle.validate_options(**{
            name: self.get_parameter('shutter_' + name).value for name in shutter_obstacle.DEFAULT_OPTIONS})
        self.shutter_depth_aligned = bool(self.get_parameter('shutter_depth_aligned').value)
        self._shutter_alignment_manual = self.shutter_depth_aligned
        self.shutter_rgb_rectified = bool(self.get_parameter('shutter_rgb_rectified').value)
        self.shutter_floor_plane_mode = str(self.get_parameter('shutter_floor_plane_mode').value).strip().lower()
        if self.shutter_floor_plane_mode not in ('fixed', 'ransac'):
            raise ValueError("shutter_floor_plane_mode must be 'fixed' or 'ransac'")
        fixed_plane = np.asarray(self.get_parameter('shutter_fixed_floor_plane').value, dtype=float)
        if fixed_plane.shape != (4,) or not np.isfinite(fixed_plane).all() or np.linalg.norm(fixed_plane[:3]) < 1e-6:
            raise ValueError('shutter_fixed_floor_plane must be four finite numbers [nx, ny, nz, d]')
        fixed_plane[:3] /= np.linalg.norm(fixed_plane[:3])
        self.shutter_fixed_floor_plane = fixed_plane if self.shutter_floor_plane_mode == 'fixed' else None
        self.shutter_depth_pixel_mapping = tuple(
            float(self.get_parameter(name).value) for name in (
                'shutter_depth_pixel_scale_x', 'shutter_depth_pixel_offset_x',
                'shutter_depth_pixel_scale_y', 'shutter_depth_pixel_offset_y'))
        if self.shutter_depth_pixel_mapping[0] <= 0 or self.shutter_depth_pixel_mapping[2] <= 0:
            raise ValueError('shutter_depth_pixel_scale_x/y must be positive')
        self.shutter_focal_length_px = float(self.get_parameter('shutter_focal_length_px').value)
        if not np.isfinite(self.shutter_focal_length_px) or self.shutter_focal_length_px < 0:
            raise ValueError('shutter_focal_length_px must be 0 (use CameraInfo) or a positive pixel value')
        self.shutter_depth_timeout = float(self.get_parameter('shutter_depth_timeout').value)
        self.shutter_sync_tolerance = float(self.get_parameter('shutter_sync_tolerance').value)
        self.shutter_ground_frame = str(self.get_parameter('shutter_ground_frame').value).strip()
        self.shutter_font_path = str(self.get_parameter('shutter_font_path').value).strip()
        if self.shutter_font_path:
            self.shutter_font_path = str(Path(self.shutter_font_path).expanduser())
        self.shutter_font_size = int(self.get_parameter('shutter_font_size').value)
        if (not np.isfinite((self.shutter_depth_timeout, self.shutter_sync_tolerance)).all()
                or self.shutter_depth_timeout <= 0 or self.shutter_sync_tolerance < 0
                or self.shutter_font_size < 8 or not self.shutter_ground_frame):
            raise ValueError('invalid shutter sensor timeout, sync tolerance, ground frame or font size')
        if self.shutter_enabled:
            try:
                shutter_obstacle.load_font(self.shutter_font_path, self.shutter_font_size)
            except OSError as error:
                self.shutter_enabled = False
                self.get_logger().error(f'Shutter inspection disabled: {error}')
        self.aruco_enabled = bool(self.get_parameter('aruco_enabled').value)
        self.extinguisher_aruco = None
        if self.aruco_enabled:
            try:
                self.extinguisher_aruco = extinguisher_aruco.ExtinguisherAruco()
            except (RuntimeError, cv2.error) as error:
                self.aruco_enabled = False
                self.get_logger().error(f'ArUco inspection disabled: {error}')
        self.pressure_options = extinguisher_pressure.validate_options(
            min_gauge_diameter=self.get_parameter('pressure_min_gauge_diameter').value,
            needle_min_pixels=self.get_parameter('pressure_needle_min_pixels').value,
            boundary_margin_deg=self.get_parameter('pressure_boundary_margin_deg').value,
            normal_margin_deg=self.get_parameter('pressure_normal_margin_deg').value,
            normal_start_deg=self.get_parameter('pressure_normal_start_deg').value,
            normal_end_deg=self.get_parameter('pressure_normal_end_deg').value,
            yellow_lower=self.get_parameter('pressure_yellow_lower').value,
            yellow_upper=self.get_parameter('pressure_yellow_upper').value,
            green_lower=self.get_parameter('pressure_green_lower').value,
            green_upper=self.get_parameter('pressure_green_upper').value)
        self.pressure_normal_confirm_seconds = float(self.get_parameter('pressure_normal_confirm_seconds').value)
        confirm_frames = self.get_parameter('pressure_normal_confirm_frames').value
        if (not np.isfinite(self.pressure_normal_confirm_seconds) or self.pressure_normal_confirm_seconds <= 0
                or not np.isfinite(confirm_frames) or confirm_frames < 2 or confirm_frames != int(confirm_frames)):
            raise ValueError('pressure normal confirmation requires positive seconds and at least two frames')
        self.pressure_normal_confirm_frames = int(confirm_frames)
        self.rack_inspect_options = dict(
            margin=int(self.get_parameter('rack_margin').value),
            zone_width=int(self.get_parameter('rack_zone_width').value),
            diff_threshold=int(self.get_parameter('rack_diff_threshold').value),
            min_area=int(self.get_parameter('rack_min_area').value))
        # 1차 ROI: 랙을 찾을 고정 영역(화면 비율 0~1). 이 밖에서는 랙을 검출하지 않는다.
        self.rack_roi = (
            float(self.get_parameter('rack_roi_x1').value),
            float(self.get_parameter('rack_roi_y1').value),
            float(self.get_parameter('rack_roi_x2').value),
            float(self.get_parameter('rack_roi_y2').value))
        x1, y1, x2, y2 = self.rack_roi
        if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
            raise ValueError('rack_roi must satisfy 0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1')
        self.window_size = (960, 540)
        self.latest_frame = None
        self.last_frame_time = None
        ref_path = str(self.get_parameter('rack_reference_path').value).strip()
        self.rack_reference_path = ref_path if ref_path else None
        self.rack_reference = None
        if self.rack_reference_path:
            self.rack_reference = rack_conditioner.load_reference(self.rack_reference_path)
            if self.rack_reference is not None:
                r = self.rack_reference['rack']
                self.get_logger().info(
                    f'Loaded rack reference from {self.rack_reference_path} '
                    f'(L={r["left"]} R={r["right"]})')
            else:
                self.get_logger().info(
                    f'No rack reference at {self.rack_reference_path} yet; '
                    'press N to capture and save')
        self.rack_confirm_frames = int(self.get_parameter('rack_confirm_frames').value)
        self.camera_timeout = float(self.get_parameter('camera_timeout').value)
        if self.rack_confirm_frames < 1:
            raise ValueError('rack_confirm_frames must be positive')
        if not np.isfinite(self.camera_timeout) or self.camera_timeout <= 0:
            raise ValueError('camera_timeout must be positive')
        self.rack_state = {}
        self._pressure_state = {}
        self._rack_loading_state = {}
        self._exit_sign_state = {}
        self._shutter_state = {}
        self._shutter_report = None
        # Depth가 RGB보다 약 0.8초(최대 1.4초) 늦게 도착하므로 30Hz RGB를 3초 분량 보관해 짝을 놓치지 않게 한다.
        self._shutter_rgb_frames = deque(maxlen=90)
        self._shutter_depth_frames = deque(maxlen=40)
        self._shutter_calibration = None
        self._shutter_depth_calibration = None
        self._shutter_last_pair_stamp = 0
        self._shutter_last_pair_time = None
        # 발행 주기·시각 정렬 확인용 (촬영 시각 ns, 수신 시각) 표본과 최근 짝 찾기 성공 여부다.
        self._shutter_rgb_timing = deque(maxlen=90)
        self._shutter_pair_results = deque(maxlen=50)
        self._shutter_timing_log_time = 0.0
        self._shutter_timeout_reason = None
        self._shutter_tf_buffer = None
        self._shutter_alignment_client = None
        self._shutter_alignment_future = None
        self._shutter_alignment_request_at = 0.0
        self._shutter_alignment_checked_at = None
        self._shutter_alignment_epoch = 0
        self._shutter_alignment_status = 'manual' if self._shutter_alignment_manual else 'pending'
        self._shutter_alignment_detail = ('explicit shutter_depth_aligned=true'
                                          if self._shutter_alignment_manual else 'waiting for camera parameters')
        self._shutter_camera_node = ''
        self._pressure_timeout_active = False
        # 시작·중지·기준 변경을 한 번에 하나씩 처리한다. 검사 모드는 다음 호출까지 유지한다.
        self._inspection_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._inspection_mode = None
        self._inspection_id = 0
        self._inspection_result = None
        self._stop_event = threading.Event()
        srv_group = ReentrantCallbackGroup()
        if self.rack_enabled:
            self.rack_service = self.create_service(
                Trigger, '~/inspect_rack', self._handle_inspect_rack,
                callback_group=srv_group)
        if self.enable_yolo:
            self.yolo_service = self.create_service(
                Trigger, '~/inspect_yolo', self._handle_inspect_yolo,
                callback_group=srv_group)
        if self.shutter_enabled:
            self.shutter_service = self.create_service(
                Trigger, '~/inspect_shutter', self._handle_inspect_shutter,
                callback_group=srv_group)

        # IF-05 {ns}/inspect_facility (mission_executor 가 설비 앞에서 부름). 다른 서비스와 달리 상대 이름
        self.facility_window = float(self.get_parameter('facility_window_s').value)
        self.facility_min = min(float(self.get_parameter('facility_min_s').value), self.facility_window)
        self._facility_defects = None   # 서비스 처리 중에만 리스트: 이번 검사에서 발행한 (kind, detail)
        self._last_jpeg = None          # 마지막 결과 화면 (근거 사진)
        self.facility_service = self.create_service(
            InspectFacility, 'inspect_facility', self._handle_inspect_facility, callback_group=srv_group)

        self.model = None
        weights_value = str(self.get_parameter('weights').value).strip()
        self.weights = Path(weights_value or self._find_default_weights()).expanduser().resolve()
        self.device = 'cpu'
        self.topic = 'oakd/rgb/image_raw/compressed'
        self.output_topic = 'fire_extinguisher/detections/compressed'
        self.result_topic = 'inspect/result'      # IF-05 판정 (회차 · 작업 · 설비 id, TRANSIENT_LOCAL) — 관제가 저장
        self.defect_topic = 'inspect/defects'     # 검사 중 순간 불량 이벤트 (VOLATILE) — 같은 토픽에 QoS 가 다른 발행자가
        #                                           둘이면 관제(TRANSIENT_LOCAL 구독)에 「incompatible QoS」 가 나서 나눔 (10/16)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.show = bool(self.get_parameter("show").value)
        self.window_name = "YOLO and Rack Detection"
        if self.show and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().warning("No desktop display available; disabling show")
            self.show = False
        if self.show:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, *self.window_size)
                waiting_frame = np.zeros((540, 960, 3), dtype=np.uint8)
                cv2.putText(waiting_frame, "Waiting for camera frames...", (230, 280),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow(self.window_name, waiting_frame)
                cv2.waitKey(1)
            except cv2.error as error:
                self.get_logger().warning(f"Cannot open display; disabling show: {error}")
                self.show = False

        self._display_frame = None
        self.last_inference_time = 0.0
        self.last_log_time = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            CompressedImage, self.output_topic, sensor_qos
        )
        # 불량은 저장된 옛 상태가 아닌 현재 검출 이벤트다. 기본 VOLATILE QoS로 새 결과만 inspect/defects 로 보낸다.
        self.result_publisher = self.create_publisher(InspectionResult, self.defect_topic, 10)
        # IF-05 판정 결과 (회차 · 작업 · 설비 id 포함) — 관제가 이것으로 점검 이력 · 사진을 저장한다
        self.facility_result_publisher = self.create_publisher(InspectionResult, self.result_topic, FACILITY_RESULT_QOS)
        self.subscription = self.create_subscription(
            CompressedImage, self.topic, self.image_callback, sensor_qos
        )
        if self.shutter_enabled:
            self._shutter_bridge = CvBridge()
            self._shutter_tf_buffer = Buffer()
            depth_topic = str(self.get_parameter('shutter_depth_topic').value).strip()
            info_topic = str(self.get_parameter('shutter_camera_info_topic').value).strip()
            depth_info_topic = str(self.get_parameter('shutter_depth_camera_info_topic').value).strip()
            tf_topic = str(self.get_parameter('shutter_tf_topic').value).strip()
            static_topic = str(self.get_parameter('shutter_tf_static_topic').value).strip()
            camera_node = str(self.get_parameter('shutter_camera_node').value).strip().rstrip('/')
            if not all((depth_topic, info_topic, depth_info_topic, tf_topic, static_topic, camera_node)):
                raise ValueError('shutter input topics must not be empty')
            self._shutter_camera_node = (camera_node if camera_node.startswith('/')
                                         else self.get_namespace().rstrip('/') + '/' + camera_node)
            # 상대 토픽이므로 __ns:=/robot3 또는 /robot4에 따라 카메라·TF 주소가 함께 바뀐다.
            shutter_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                     reliability=ReliabilityPolicy.BEST_EFFORT)
            sensor_group = MutuallyExclusiveCallbackGroup()
            self.shutter_depth_subscription = self.create_subscription(
                CompressedImage if depth_topic.rstrip('/').endswith('compressedDepth') else Image,
                depth_topic, self._shutter_depth_callback, shutter_qos, callback_group=sensor_group)
            self.shutter_info_subscription = self.create_subscription(
                CameraInfo, info_topic, self._shutter_camera_info_callback, shutter_qos, callback_group=sensor_group)
            self.shutter_depth_info_subscription = self.create_subscription(
                CameraInfo, depth_info_topic, self._shutter_depth_info_callback, shutter_qos, callback_group=sensor_group)
            tf_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            static_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                                   reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.shutter_tf_subscription = self.create_subscription(
                TFMessage, tf_topic, self._shutter_tf_callback, tf_qos, callback_group=sensor_group)
            self.shutter_tf_static_subscription = self.create_subscription(
                TFMessage, static_topic, self._shutter_tf_static_callback, static_qos, callback_group=sensor_group)
            if not self._shutter_alignment_manual:
                # 조회 응답을 기다리느라 영상 수신이나 검사 서비스가 멈추지 않게 한다.
                alignment_group = MutuallyExclusiveCallbackGroup()
                self._shutter_alignment_client = self.create_client(
                    GetParameters, self._shutter_camera_node + '/get_parameters', callback_group=alignment_group)
                self._shutter_alignment_timer = self.create_timer(
                    0.5, self._refresh_shutter_alignment, callback_group=alignment_group)
            self.get_logger().info(
                f'Shutter Depth={self.shutter_depth_subscription.topic_name}; '
                f'CameraInfo={self.shutter_info_subscription.topic_name}; '
                f'Depth CameraInfo={self.shutter_depth_info_subscription.topic_name}; '
                f'TF={self.shutter_tf_subscription.topic_name}; ground frame={self.shutter_ground_frame}')
        # 추론과 별도 그룹에서 영상 단절을 확인해 설비의 옛 화면과 확인 이력을 지운다.
        self._pressure_watchdog = self.create_timer(
            min(0.1, self.camera_timeout / 2), self._check_pressure_camera_timeout,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'weights: {self.weights} (loaded on first YOLO request)')
        self.get_logger().info(f"input: {self.subscription.topic_name}")
        self.get_logger().info(f"output: {self.publisher.topic_name}")
        self.get_logger().info(f"defects: {self.result_publisher.topic_name} (InspectionResult, ABNORMAL only)")
        self.get_logger().info(f'YOLO enabled={self.enable_yolo}; rack enabled={self.rack_enabled}')
        self.get_logger().info(f'Extinguisher needle inspection enabled={self.pressure_enabled}')
        self.get_logger().info(
            f'Exit sign brightness inspection enabled={self.exit_sign_enabled}; '
            f'confirmation={self.exit_sign_options["confirm_seconds"]:.2f}s / '
            f'{self.exit_sign_options["confirm_frames"]} fresh frames; thresholds are initial, uncalibrated values')
        self.get_logger().info(
            f'Extinguisher ArUco enabled={self.aruco_enabled}; dictionary={extinguisher_aruco.DICTIONARY_NAME}; '
            'manufacturing records are DUMMY and logged only; markers require a current extinguisher box')
        self.get_logger().info(
            f'YOLO rack loading enabled={self.rack_loading_enabled}; '
            f'confirmation={self.rack_loading_options["required_frames"]} fresh frames; '
            f'boundary tolerance={self.rack_loading_options["tolerance_ratio"]:.3f}')
        self.get_logger().info(
            f'Pressure NORMAL requires {self.pressure_normal_confirm_seconds:.2f}s '
            f'and {self.pressure_normal_confirm_frames} consecutive fresh frames')
        self.get_logger().info(
            f'Shutter enabled={self.shutter_enabled}; RGB-aligned Depth confirmed={self.shutter_depth_aligned}; '
            f'alignment check={self._shutter_alignment_status}; camera={self._shutter_camera_node}; '
            'height/area thresholds are initial, uncalibrated values')
        self.get_logger().info(
            'Camera preview ready. Call ~/inspect_yolo, ~/inspect_rack or ~/inspect_shutter to toggle detection.')
        self.get_logger().info('Shutter floor plane mode=%s; fixed plane [nx, ny, nz, d]=%s'
                               % (self.shutter_floor_plane_mode, np.round(self.shutter_fixed_floor_plane, 4).tolist()
                                  if self.shutter_fixed_floor_plane is not None else None))
        self.get_logger().info('Shutter depth pixel correction (scale_x, offset_x, scale_y, offset_y)=%s; 1,0,1,0 means none'
                               % (self.shutter_depth_pixel_mapping,))
        self.get_logger().info('Shutter focal length override=%s px (0 means CameraInfo)' % self.shutter_focal_length_px)

    def _set_shutter_alignment(self, confirmed, status, detail, checked_at=None) -> None:
        """정렬 확인 여부와 사유를 보관하고 확인이 취소되면 지난 셔터 판정을 지운다.

        confirmed는 확인 여부, status·detail은 확인 단계와 근거다. checked_at은 응답을 읽은 단조 시각이다.
        """
        with self._state_lock:
            changed = (self.shutter_depth_aligned, self._shutter_alignment_status,
                       self._shutter_alignment_detail) != (confirmed, status, detail)
            if self.shutter_depth_aligned != confirmed:
                self._shutter_alignment_epoch += 1
            self.shutter_depth_aligned = confirmed
            self._shutter_alignment_status = status
            self._shutter_alignment_detail = detail
            self._shutter_alignment_checked_at = checked_at
            if changed and not confirmed:
                shutter_obstacle.reset_stability_state(self._shutter_state)
                reading = shutter_obstacle.unconfirmed(SHUTTER_ALIGNMENT_REASONS[status], detail)
                reading['depth_alignment_epoch'] = self._shutter_alignment_epoch
                self._shutter_report = reading
                if self._inspection_mode == 'shutter':
                    self._inspection_result = None
                    if self.latest_frame is not None:
                        self._display_frame = shutter_obstacle.draw_result(
                            self.latest_frame, reading, font_path=self.shutter_font_path,
                            font_size=self.shutter_font_size)
        if changed:
            self.get_logger().info(f'Shutter alignment={status}; camera={self._shutter_camera_node}; {detail}')

    def _refresh_shutter_alignment(self) -> None:
        """OAK-D 정렬 켜짐과 RGB·Depth 카메라 번호를 비동기로 읽어 자동 확인한다.

        카메라 설정을 변경하지 않는다. 요청 간격은 2초, 응답 대기는 최대 5초다.
        마지막 확인이 10초 이상 오래되면 미확인으로 돌아간다. 조회를 기다리며 영상 처리를 막지 않는다.
        """
        if self._stop_event.is_set() or self._shutter_alignment_manual:
            return
        now = time.monotonic()
        with self._state_lock:
            checked_at = self._shutter_alignment_checked_at
        if checked_at is not None and now - checked_at > 10.0:
            self._set_shutter_alignment(False, 'pending', 'camera alignment confirmation expired')
        future = self._shutter_alignment_future
        if future is not None:
            if not future.done():
                if now - self._shutter_alignment_request_at < 5.0:
                    return
                self._shutter_alignment_client.remove_pending_request(future)
                future.cancel()
                self._shutter_alignment_future = None
                with self._state_lock:
                    confirmed = self.shutter_depth_aligned
                if not confirmed:
                    self._set_shutter_alignment(False, 'pending', 'camera parameter query timed out')
                return
            self._shutter_alignment_future = None
            try:
                response = future.result()
                values = response.values if response is not None else []
                if len(values) != 3 or values[0].type != ParameterType.PARAMETER_BOOL:
                    self._set_shutter_alignment(False, 'unsupported', 'camera alignment parameters unavailable', now)
                elif not values[0].bool_value:
                    self._set_shutter_alignment(False, 'disabled', 'stereo.i_align_depth=false', now)
                elif (any(value.type != ParameterType.PARAMETER_INTEGER for value in values[1:])
                      or any(value.integer_value < 0 for value in values[1:])):
                    self._set_shutter_alignment(False, 'unsupported', 'camera socket identifiers unavailable', now)
                elif values[1].integer_value != values[2].integer_value:
                    self._set_shutter_alignment(False, 'wrong_camera',
                        f'Depth socket={values[1].integer_value}, RGB socket={values[2].integer_value}', now)
                else:
                    self._set_shutter_alignment(True, 'verified',
                        f'stereo.i_align_depth=true; shared RGB socket={values[1].integer_value}', now)
            except Exception as error:
                with self._state_lock:
                    confirmed = self.shutter_depth_aligned
                if not confirmed:
                    self._set_shutter_alignment(False, 'pending', f'camera parameter query failed: {error}')
            return
        if now - self._shutter_alignment_request_at < 2.0:
            return
        if not self._shutter_alignment_client.service_is_ready():
            with self._state_lock:
                confirmed = self.shutter_depth_aligned
            if not confirmed:
                self._set_shutter_alignment(False, 'pending', 'waiting for camera get_parameters service')
            return
        request = GetParameters.Request()
        request.names = ['stereo.i_align_depth', 'stereo.i_board_socket_id', 'rgb.i_board_socket_id']
        self._shutter_alignment_request_at = now
        try:
            self._shutter_alignment_future = self._shutter_alignment_client.call_async(request)
        except RuntimeError as error:
            with self._state_lock:
                confirmed = self.shutter_depth_aligned
            if not confirmed:
                self._set_shutter_alignment(False, 'pending', f'camera parameter query failed: {error}')

    def _shutter_depth_callback(self, message) -> None:
        """셔터용 Depth를 미터 배열로 바꿔 촬영 시각·수신 시각·좌표계와 함께 최근 사진 목록에 보관한다.

        message는 원본 Depth(Image, 16UC1·32FC1) 또는 압축 Depth(CompressedImage, compressedDepth png)다.
        수신 시각은 압축 해제 전에 기록해 해제 시간이 도착 지연으로 섞이지 않게 한다.
        """
        if self._stop_event.is_set():
            return
        received = time.monotonic()
        try:
            if isinstance(message, CompressedImage):
                depth = decode_compressed_depth(message)
            else:
                if message.encoding not in ('16UC1', '32FC1'):
                    raise ValueError('Depth encoding must be 16UC1 (mm) or 32FC1 (m)')
                image = self._shutter_bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
                if image is None or image.ndim != 2 or image.size == 0:
                    raise ValueError('Depth must be a nonempty single-channel image')
                depth = np.asarray(image, dtype=np.float64).copy()
                if message.encoding == '16UC1':
                    depth *= 0.001
            if depth.size == 0:
                raise ValueError('Depth must be a nonempty single-channel image')
            record = {'depth': depth, 'stamp': message.header.stamp,
                      'stamp_ns': message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                      'frame_id': message.header.frame_id, 'received': received}
            with self._state_lock:
                self._shutter_depth_frames.append(record)
        except (ValueError, CvBridgeError, cv2.error, RuntimeError) as error:
            self.get_logger().warning(f'Shutter Depth conversion failed: {error}', throttle_duration_sec=2.0)

    def _shutter_camera_info_callback(self, message: CameraInfo, depth=False) -> None:
        """CameraInfo를 보관한다. depth가 참이면 Depth 보정값이며, 바뀌면 구역 추적을 새로 시작한다."""
        roi = message.roi
        record = {'width': int(message.width), 'height': int(message.height),
                  'k': tuple(message.k), 'p': tuple(message.p), 'r': tuple(message.r), 'd': tuple(message.d),
                  'distortion_model': message.distortion_model, 'frame_id': message.header.frame_id,
                  'cropped_or_binned': (message.binning_x > 1 or message.binning_y > 1
                                       or roi.x_offset > 0 or roi.y_offset > 0
                                       or roi.width not in (0, message.width) or roi.height not in (0, message.height))}
        with self._state_lock:
            name = '_shutter_depth_calibration' if depth else '_shutter_calibration'
            previous = getattr(self, name)
            if previous != record:
                setattr(self, name, record)
                if previous is not None:
                    shutter_obstacle.reset_stability_state(self._shutter_state)

    def _shutter_depth_info_callback(self, message: CameraInfo) -> None:
        """Depth CameraInfo를 RGB와 별도로 보관하도록 공통 보정값 수신 함수를 호출한다."""
        self._shutter_camera_info_callback(message, depth=True)

    def _shutter_tf_callback(self, message: TFMessage) -> None:
        """실행 이름공간의 움직이는 TF를 버퍼에 넣어 촬영 시각의 바닥 방향을 조회할 수 있게 한다."""
        for transform in message.transforms:
            try:
                self._shutter_tf_buffer.set_transform(transform, 'shutter_tf_topic')
            except TransformException as error:
                self.get_logger().warning(f'Shutter TF rejected: {error}', throttle_duration_sec=2.0)

    def _shutter_tf_static_callback(self, message: TFMessage) -> None:
        """실행 이름공간의 고정 TF를 버퍼에 넣어 카메라 장착 방향을 계산할 수 있게 한다."""
        for transform in message.transforms:
            try:
                self._shutter_tf_buffer.set_transform_static(transform, 'shutter_tf_static_topic')
            except TransformException as error:
                self.get_logger().warning(f'Shutter static TF rejected: {error}', throttle_duration_sec=2.0)

    def show_preview_frame(self, frame) -> None:
        """메인 스레드에서 RGB 또는 검출 영상을 창 크기에 맞춰 표시한다. 반환값은 없다."""
        display = cv2.resize(frame, self.window_size)
        cv2.imshow(self.window_name, display)

    def crop_rack_roi(self, frame: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int]]:
        """영상에서 1차 ROI(랙을 찾을 고정 영역)만 잘라낸다.

        rack_roi 파라미터는 0~1 비율이므로 실제 영상 크기에 맞춰 픽셀 좌표로 바꾼다.
        반환: (잘라낸 영상, 전체 영상 기준 (x1, y1, x2, y2) 픽셀 좌표)
        """
        height, width = frame.shape[:2]
        x1 = int(self.rack_roi[0] * width)
        y1 = int(self.rack_roi[1] * height)
        x2 = int(self.rack_roi[2] * width)
        y2 = int(self.rack_roi[3] * height)
        return frame[y1:y2, x1:x2].copy(), (x1, y1, x2, y2)

    def reference_for_roi(self, roi_box: tuple[int, int, int, int]):
        """저장된 기준 자료를 1차 ROI 좌표계에 맞춰 돌려준다.

        N키로 새로 저장한 기준은 이미 ROI 크기라 그대로 쓰고, 예전에 전체 화면으로
        저장한 기준이면 같은 ROI로 잘라내고 랙 좌표도 그만큼 옮긴다. 기준이 없으면 None.
        """
        reference = self.rack_reference
        if reference is None:
            return None
        x1, y1, x2, y2 = roi_box
        ref_frame = reference['frame']
        if ref_frame.shape[:2] == (y2 - y1, x2 - x1):
            return reference
        rack = reference['rack']
        return {'frame': ref_frame[y1:y2, x1:x2],
                'rack': {'left': rack['left'] - x1, 'right': rack['right'] - x1,
                         'top': rack['top'] - y1, 'bottom': rack['bottom'] - y1}}

    def capture_rack_reference(self) -> bool:
        """대기 중인 최신 ROI를 정상 기준으로 저장한다. 검사 중에는 기준을 바꾸지 않는다."""
        if not self._inspection_lock.acquire(blocking=False):
            self.get_logger().warning('Cannot change rack reference during inspection.')
            return False
        try:
            if self._inspection_mode is not None:
                self.get_logger().warning('Stop detection before changing the rack reference.')
                return False
            return self._capture_rack_reference()
        finally:
            self._inspection_lock.release()

    def _capture_rack_reference(self) -> bool:
        """요청 잠금을 확보한 뒤 최신 영상에서 랙을 찾아 기준 사진을 저장한다."""
        if self.latest_frame is None:
            self.get_logger().warning('No frame available yet.')
            return False
        roi_frame, _ = self.crop_rack_roi(self.latest_frame)
        rack = rack_conditioner.detect_rack(roi_frame)
        if rack is None:
            self.get_logger().warning('Cannot detect rack pipes in current frame.')
            return False
        reference = rack_conditioner.capture_reference(roi_frame, rack)
        if reference is None:
            self.get_logger().warning('Rack area has no visible content.')
            return False
        with self._state_lock:
            self.rack_reference = reference
            self.rack_state = {}
        if self.rack_reference_path:
            rack_conditioner.save_reference(reference, self.rack_reference_path)
            self.get_logger().info(
                f'Saved rack reference to {self.rack_reference_path} '
                f'(L={rack["left"]} R={rack["right"]})')
        else:
            self.get_logger().info(
                f'Rack reference set in memory (L={rack["left"]} R={rack["right"]}). '
                'Set rack_reference_path to persist across restarts.')
        return True

    def reset_rack_reference(self) -> bool:
        """검사 대기 중에 메모리의 랙 기준을 지운다. 저장한 NPZ 파일은 그대로 둔다."""
        if not self._inspection_lock.acquire(blocking=False):
            self.get_logger().warning('Cannot reset rack reference during inspection.')
            return False
        try:
            with self._state_lock:
                if self._inspection_mode is not None:
                    self.get_logger().warning('Stop detection before resetting the rack reference.')
                    return False
                self.rack_reference = None
                self.rack_state = {}
            return True
        finally:
            self._inspection_lock.release()

    def _load_yolo_model(self) -> None:
        """첫 YOLO 서비스 요청에서만 자동 선택한 best.pt를 로드하고 이후 요청에서 재사용한다."""
        if self.model is not None:
            return
        if not self.weights.is_file():
            raise FileNotFoundError(f'Weights file not found: {self.weights}')
        import torch
        from ultralytics import YOLO

        requested_device = str(self.get_parameter('device').value)
        self.device = ('0' if torch.cuda.is_available() else 'cpu') if requested_device == 'auto' else requested_device
        self.model = YOLO(str(self.weights))
        self.get_logger().info(f'YOLO classes: {self.model.names}; device: {self.device}')

    def _handle_inspect_yolo(self, request, response):
        """빈 Trigger 요청으로 YOLO를 시작하거나 중지하고 변경 결과를 즉시 응답한다."""
        return self._handle_inspection('yolo', response)

    def _handle_inspect_rack(self, request, response):
        """빈 Trigger 요청으로 랙 검사를 시작하거나 중지하고 변경 결과를 즉시 응답한다."""
        return self._handle_inspection('rack', response)

    def _handle_inspect_shutter(self, request, response):
        """빈 Trigger 요청으로 방화셔터 검사를 켜거나 끄고 변경 결과를 즉시 응답한다."""
        return self._handle_inspection('shutter', response)

    def _handle_inspect_facility(self, request, response):
        """IF-05 서비스 (통합 브랜치 10/15): kind 에 맞는 검사를 켜고 판정을 돌려준다.

        로봇(mission_executor)은 이 응답을 받을 때까지 관측 자리에 서 있다. 판정이 나도 facility_min_s(3 s)
        동안은 계속 보고(첫 한두 프레임으로 응답하지 않게, 10/16), 그 뒤 확정 판정이 있으면 끝, 없으면
        facility_window_s(6 s)까지 기다린다. 판정은 _facility_decision — 마지막 확정 판정을 따른다.
        카메라 없음 · 모델 없음 · 판정 못 함은 UNCONFIRMED. 끝나면 검사를 끄고 결과를 inspect/result 로 낸다.
        """
        kind, verdict, detail = request.kind, InspectionResult.UNCONFIRMED, ''
        mode = FACILITY_MODES.get(kind)
        if mode is None:
            detail = f'unknown kind {kind}'
        else:
            with self._state_lock:
                active = self._inspection_mode
            if active is not None:                       # 수동으로 켜 둔 검사는 끄고 시작 (같은 서비스 = 켜기/끄기)
                self._handle_inspection(active, Trigger.Response())
            self._facility_defects = []
            started = self._handle_inspection(mode, Trigger.Response())
            if not started.success:
                detail = f'{mode}: {started.message}'
            else:
                seen, last, markers = None, None, {}
                start = time.monotonic()
                end, min_end = start + self.facility_window, start + self.facility_min
                while time.monotonic() < end and not self._stop_event.is_set():
                    with self._state_lock:
                        outcome = self._inspection_result
                    if outcome and outcome.get('time') != seen:
                        seen = outcome.get('time')
                        last = self._facility_status(kind, outcome) or last
                        for rep in outcome.get('aruco_markers') or []:     # 소화기 ArUco 제조 정보 (ID 별 마지막 값)
                            for mk in rep.get('markers') or []:
                                markers[mk['marker_id']] = {k: mk.get(k) for k in (
                                    'marker_id', 'manufacture_date', 'manufacture_number', 'expiry_date', 'expiry_status')}
                    if last is not None and time.monotonic() >= min_end:   # 최소 시간을 봤고 확정 판정이 있음
                        break
                    time.sleep(0.1)
                with self._state_lock:
                    still = self._inspection_mode == mode
                if still:
                    self._handle_inspection(mode, Trigger.Response())
                hits = [d for k, d in self._facility_defects if k == kind]
                decision = self._facility_decision(last, bool(hits))
                if decision == 'ABNORMAL':
                    verdict, detail = InspectionResult.ABNORMAL, hits[-1] if hits else f'{mode}: ABNORMAL'
                elif decision == 'NORMAL':
                    verdict, detail = InspectionResult.NORMAL, f'{mode}: NORMAL'
                else:
                    detail = f'{mode}: 판정 못 함 ({last or "결과 없음"}, {self.facility_window:.0f}s)'
                if markers:                                       # 관제가 inspections.aruco 로 저장 (ros_bridge 가 풂)
                    detail = json.dumps({'text': detail, 'aruco': [markers[k] for k in sorted(markers)]}, ensure_ascii=False)
            self._facility_defects = None
        self._publish_facility_result(request, verdict, detail)
        self.get_logger().info(f'inspect_facility {request.facility_id} kind={kind} → verdict={verdict} {detail[:120]}')
        response.verdict, response.detail, response.image_ref = verdict, detail, ''
        return response

    @staticmethod
    def _facility_decision(last, has_defect):
        """IF-05 최종 판정: 마지막 확정 판정(last: 'NORMAL' · 'ABNORMAL' · None)을 따르고,
        확정 판정이 한 번도 없을 때만 그동안 발행한 불량(has_defect)으로 ABNORMAL. 둘 다 없으면 None(판정 못 함).
        한두 프레임의 이상 뒤 정상으로 확정되면 NORMAL 이 된다 (10/16 — 예전에는 불량 한 번이면 바로 ABNORMAL)."""
        if last in ('NORMAL', 'ABNORMAL'):
            return last
        return 'ABNORMAL' if has_defect else None

    @staticmethod
    def _facility_status(kind, outcome):
        """한 프레임 결과에서 그 설비 종류의 판정 (NORMAL · ABNORMAL · None)."""
        if kind == InspectionResult.KIND_SHUTTER:
            status = (outcome.get('shutter') or {}).get('status')
            return status if status in VERDICTS else None
        if kind == InspectionResult.KIND_RACK:            # yolo 모드의 랙 적재 판정 (랙마다 NORMAL · ABNORMAL · UNCONFIRMED)
            reports = (outcome.get('rack_loading') or {}).get('racks')
        else:
            reports = outcome.get('pressures' if kind == InspectionResult.KIND_EXTINGUISHER else 'exit_signs')
        statuses = {r.get('status') for r in reports or []}
        return 'ABNORMAL' if 'ABNORMAL' in statuses else 'NORMAL' if 'NORMAL' in statuses else None

    def _publish_facility_result(self, request, verdict, detail):
        """판정을 회차 · 작업 · 설비 id 와 함께 inspect/result 로 (근거 사진은 긴 변 640 px JPEG)."""
        msg = InspectionResult(robot_id=self.get_namespace().strip('/'), run_id=request.run_id, task_id=request.task_id,
                               facility_id=request.facility_id, kind=request.kind, verdict=verdict, detail=detail,
                               attempts=1)
        msg.stamp = self.get_clock().now().to_msg()
        try:
            if self._last_jpeg is not None:
                img = cv2.imdecode(np.frombuffer(bytes(self._last_jpeg), np.uint8), cv2.IMREAD_COLOR)
                scale = 640 / max(img.shape[:2])
                if scale < 1:
                    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    msg.image.format, msg.image.data = 'jpeg', jpeg.tobytes()
        except (cv2.error, AttributeError) as error:
            self.get_logger().warning(f'inspect_facility 사진 생략: {error}')
        self.facility_result_publisher.publish(msg)

    def _handle_inspection(self, mode, response):
        """같은 서비스를 부를 때마다 검사를 켜거나 끄고 Trigger 변경 결과를 반환한다.

        mode는 yolo·rack·shutter 중 하나다. 시작 후에는 시간 제한 없이 검사한다.
        다른 모드가 켜져 있으면 거절한다. 중지는 영상이나 기준 사진 없이도 가능하다.
        응답은 시작·중지 성공 여부다. 시작·중지마다 압력·비상구·랙·셔터 확인 이력을 지운다.
        셔터의 대기 RGB 목록도 지워 지난 검사에서 받은 사진을 새 검사에 섞지 않는다.
        판정은 영상·로그에 표시하고 불량은 별도 토픽으로 보낸다.
        """
        response.success = False
        if not self._inspection_lock.acquire(blocking=False):
            response.message = 'busy: detection is starting/stopping or reference is updating'
            return response
        try:
            with self._state_lock:
                active_mode = self._inspection_mode
                if active_mode == mode:
                    self._inspection_mode = None
                    self._inspection_id += 1
                    self._inspection_result = None
                    self.rack_state = {}
                    extinguisher_pressure.reset_stability_state(self._pressure_state)
                    rack_loading.reset_stability_state(self._rack_loading_state)
                    exit_sign.reset_stability_state(self._exit_sign_state)
                    shutter_obstacle.reset_stability_state(self._shutter_state)
                    self._shutter_rgb_frames.clear()
                    self._shutter_last_pair_stamp = 0
                    self._shutter_last_pair_time = None
                    self._shutter_pair_results.clear()
                    self._shutter_report = None
                    self._shutter_timeout_reason = None
                    self._pressure_timeout_active = False
                    if self.latest_frame is not None:
                        self._display_frame = self.latest_frame.copy()
                    response.success = True
                    response.message = f'{mode.upper()} detection stopped; camera preview resumed'
                    self.get_logger().info(response.message)
                    return response
                if active_mode is not None:
                    response.message = f'busy: {active_mode.upper()} detection is active; call its service again to stop'
                    return response
            if self._stop_event.is_set():
                response.message = 'node is stopping'
                return response
            enabled = {'yolo': self.enable_yolo, 'rack': self.rack_enabled, 'shutter': self.shutter_enabled}
            if not enabled.get(mode, False):
                response.message = f'{mode} inspection is disabled'
                return response
            if self.last_frame_time is None or time.monotonic() - self.last_frame_time > self.camera_timeout:
                response.message = 'no recent camera frame available'
                return response
            if mode == 'rack' and self.rack_reference is None:
                response.message = 'no rack reference loaded; fit boxes inside and press N'
                return response
            if mode == 'yolo':
                self._load_yolo_model()
            # 첫 모델 로딩 동안 카메라가 끊긴 경우에는 검사를 시작하지 않는다.
            if self.last_frame_time is None or time.monotonic() - self.last_frame_time > self.camera_timeout:
                response.message = 'no recent camera frame available'
                return response
            if self._stop_event.is_set():
                response.message = 'node is stopping'
                return response
            with self._state_lock:
                self.rack_state = {}
                extinguisher_pressure.reset_stability_state(self._pressure_state)
                rack_loading.reset_stability_state(self._rack_loading_state)
                exit_sign.reset_stability_state(self._exit_sign_state)
                shutter_obstacle.reset_stability_state(self._shutter_state)
                self._shutter_rgb_frames.clear()
                self._shutter_last_pair_stamp = 0
                self._shutter_last_pair_time = None
                self._shutter_pair_results.clear()
                self._shutter_report = None
                self._shutter_timeout_reason = None
                self._pressure_timeout_active = False
                self._inspection_result = None
                self._inspection_id += 1
                self._inspection_mode = mode
            response.success = True
            response.message = f'{mode.upper()} detection started; call this service again to stop'
            self.get_logger().info(response.message)
        except Exception as error:
            response.message = f'{mode} inspection failed: {error}'
            self.get_logger().error(response.message)
        finally:
            self._inspection_lock.release()
        return response

    def _select_shutter_pair(self, message, received_at):
        """대기 RGB와 Depth에서 촬영 시각이 맞는 가장 최신 짝을 찾아 검사할 입력을 반환한다.

        message는 이번 압축 RGB이고 received_at은 실제 수신 시각이다.
        (RGB 메시지, RGB 수신 시각, 선택한 Depth 기록, 검사 번호)를 반환한다.
        허용 시간 안에 RGB가 있는 Depth 중 촬영 시각이 가장 늦은 것을 고르고, 그 Depth와 가장 가까운 RGB를 붙인다.
        예전처럼 차이가 가장 작은 옛 쌍을 고르면 처리 결과가 점점 뒤처지므로 최신 쌍을 우선한다.
        이미 처리한 Depth와 수신 후 shutter_depth_timeout이 지난 Depth는 제외한다.
        Depth는 RGB보다 늦게 도착하므로 오래됨 판단은 RGB가 아닌 최신 Depth 촬영 시각을 기준으로 한다.
        맞는 짝이 없으면 이번 RGB와 빈 Depth를 돌려준다. 짝 찾기 성공 여부는 주기 진단용으로 기록한다.
        """
        with self._state_lock:
            if self._inspection_mode != 'shutter' or self._stop_event.is_set():
                return message, received_at, None, None
            request_id = self._inspection_id
            current_time = time.monotonic()
            rgb_records = [record for record in self._shutter_rgb_frames
                           if record['stamp_ns'] > 0 and current_time - record['received'] <= self.camera_timeout]
            depth_records = [record for record in self._shutter_depth_frames
                             if record['stamp_ns'] > self._shutter_last_pair_stamp
                             and current_time - record['received'] <= self.shutter_depth_timeout]
            best = None
            if rgb_records and depth_records:
                latest_depth = max(record['stamp_ns'] for record in depth_records)
                for depth in depth_records:
                    if (latest_depth - depth['stamp_ns']) / 1_000_000_000 > self.shutter_depth_timeout:
                        continue
                    rgb = min(rgb_records, key=lambda item: abs(item['stamp_ns'] - depth['stamp_ns']))
                    difference = abs(depth['stamp_ns'] - rgb['stamp_ns']) / 1_000_000_000
                    if difference <= self.shutter_sync_tolerance and (best is None or depth['stamp_ns'] > best[1]['stamp_ns']):
                        best = rgb, depth
            if best is None:
                # 새 Depth가 아직 없으면 실패로 세지 않는다. 새 Depth가 있는데 짝이 없을 때만 실패다.
                if depth_records:
                    self._shutter_pair_results.append(False)
                return message, received_at, None, request_id
            rgb, depth = best
            self._shutter_pair_results.append(True)
            pending = [record for record in self._shutter_rgb_frames if record['stamp_ns'] > rgb['stamp_ns']]
            self._shutter_rgb_frames.clear()
            self._shutter_rgb_frames.extend(pending)
            self._shutter_last_pair_stamp = depth['stamp_ns']
            self._shutter_last_pair_time = current_time
            return rgb['message'], rgb['received'], depth, request_id

    def _log_shutter_timing(self, now) -> None:
        """셔터 검사 중 5초마다 RGB·Depth 발행 주기, 촬영 시각 위상차, 도착 지연 차이, 짝 성공률을 로그로 남긴다.

        now는 이번 RGB 수신 단조 시각(초)이다. 두 영상의 주기와 위상이 맞는지 화면 없이도 확인하기 위한 진단이다.
        """
        with self._state_lock:
            if now - self._shutter_timing_log_time < 5.0:
                return
            self._shutter_timing_log_time = now
            timing = shutter_obstacle.stream_timing(
                tuple(self._shutter_rgb_timing),
                [(record['stamp_ns'], record['received']) for record in self._shutter_depth_frames],
                self.shutter_sync_tolerance, tuple(self._shutter_pair_results))
        self.get_logger().info(
            'shutter stream timing: ' + ' '.join(
                f'{name}={value:.3f}' if isinstance(value, float) else f'{name}={value}'
                for name, value in timing.items()) + f' sync_tolerance_s={self.shutter_sync_tolerance}')

    def _inspect_shutter(self, frame, header, frame_time, request_id, depth_record=None) -> dict:
        """촬영 시각이 맞는 RGB·Depth·보정·TF를 모아 셔터 함수 모듈의 판정을 반환한다.

        frame은 원본 RGB, header는 촬영 정보, frame_time은 수신 시각, request_id는 검사 번호다.
        depth_record가 있으면 RGB와 함께 고른 Depth를 그대로 사용하며, 없으면 시각 차이 등 실패 사유를 확인한다.
        두 CameraInfo로 Depth 픽셀 위치를 RGB에 맞추고 구역·평면 추적 이력도 함수 모듈에 전달한다.
        자동 조회한 정렬 설정·해상도·좌표계·시각을 확인한다. 계산 중 정렬 확인이 바뀌면 결과를 버린다.
        Depth 수신 경과 시간은 계산 전에만 확인하며, 결과에는 RGB·Depth 발행 주기 진단(stream_timing)을 넣는다.
        """
        with self._state_lock:
            records = tuple(self._shutter_depth_frames)
            timing = shutter_obstacle.stream_timing(
                tuple(self._shutter_rgb_timing),
                [(record['stamp_ns'], record['received']) for record in records],
                self.shutter_sync_tolerance, tuple(self._shutter_pair_results))
            calibration = self._shutter_calibration
            depth_calibration = self._shutter_depth_calibration
            geometry = dict(self._shutter_state.get('geometry', {}))
            aligned = self.shutter_depth_aligned
            alignment_status = self._shutter_alignment_status
            alignment_detail = self._shutter_alignment_detail
            alignment_epoch = self._shutter_alignment_epoch
        reading = None
        observation_time, observation_stamp = frame_time, 0
        rgb_stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        sync_information = {'rgb_stamp_ns': rgb_stamp, 'rgb_depth_sync_tolerance_s': self.shutter_sync_tolerance,
                            'rgb_depth_pair_selected': False, 'depth_alignment_epoch': alignment_epoch,
                            'depth_alignment_check_status': alignment_status,
                            'depth_alignment_camera_node': self._shutter_camera_node,
                            'stream_timing': timing}
        if not aligned:
            reading = shutter_obstacle.unconfirmed(
                SHUTTER_ALIGNMENT_REASONS.get(alignment_status, 'depth_alignment_unconfirmed'), alignment_detail)
        elif not records:
            reading = shutter_obstacle.unconfirmed('depth_missing')
        elif calibration is None or depth_calibration is None:
            reading = shutter_obstacle.unconfirmed('camera_info_missing')
        elif not rgb_stamp:
            reading = shutter_obstacle.unconfirmed('rgb_timestamp_missing')
        else:
            stamped = [record for record in records if record['stamp_ns'] > 0]
            if not stamped:
                reading = shutter_obstacle.unconfirmed('depth_timestamp_missing')
            else:
                record = depth_record if depth_record is not None else min(
                    stamped, key=lambda item: abs(item['stamp_ns'] - rgb_stamp))
                signed_difference = (record['stamp_ns'] - rgb_stamp) / 1_000_000_000
                difference = abs(signed_difference)
                sync_information.update(depth_received=record['received'], depth_stamp_ns=record['stamp_ns'],
                                        rgb_depth_time_diff_s=difference,
                                        rgb_depth_signed_diff_s=signed_difference,
                                        rgb_depth_pair_selected=depth_record is not None)
                latest_stamp = max([rgb_stamp] + [item['stamp_ns'] for item in stamped])
                capture_age = (latest_stamp - min(rgb_stamp, record['stamp_ns'])) / 1_000_000_000
                sync_information['capture_pair_age_s'] = capture_age
                observation_time, observation_stamp = record['received'], record['stamp_ns']
                if time.monotonic() - record['received'] > self.shutter_depth_timeout:
                    reading = shutter_obstacle.unconfirmed('depth_stale')
                elif difference > self.shutter_sync_tolerance:
                    reading = shutter_obstacle.unconfirmed(
                        'rgb_depth_time_mismatch',
                        f'RGB/Depth capture time difference={difference:.3f}s '
                        f'(Depth-RGB={signed_difference:+.3f}s), allowed={self.shutter_sync_tolerance:.3f}s')
                elif capture_age > self.shutter_depth_timeout:
                    reading = shutter_obstacle.unconfirmed('capture_pair_stale', f'capture lag={capture_age:.3f}s')
                elif depth_record is None:
                    reading = shutter_obstacle.unconfirmed('rgb_depth_pair_unavailable')
                elif (not header.frame_id or not calibration['frame_id']
                      or record['frame_id'] != header.frame_id or calibration['frame_id'] != header.frame_id
                      or depth_calibration['frame_id'] != header.frame_id):
                    reading = shutter_obstacle.unconfirmed('optical_frame_mismatch')
                elif record['depth'].shape != frame.shape[:2]:
                    reading = shutter_obstacle.unconfirmed('rgb_depth_size_mismatch')
                else:
                    try:
                        # Depth가 촬영된 시각의 TF로 카메라에서 본 바닥 위쪽 방향을 계산한다.
                        transform = self._shutter_tf_buffer.lookup_transform(
                            self.shutter_ground_frame, header.frame_id, Time.from_msg(record['stamp']))
                        quaternion = transform.transform.rotation
                        rotation = shutter_obstacle.rotation_matrix(
                            (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
                        up = rotation.T @ np.array([0.0, 0.0, 1.0])
                        # 초점 거리·화면 어긋남 보정은 복사본에만 적용하고 원본 보정값은 변경 감지용으로 그대로 둔다.
                        # RGB와 Depth는 같은 픽셀 격자이므로 초점 거리는 둘 다 같은 값으로 바꾼다.
                        rgb_calibration = shutter_obstacle.override_focal_length(
                            calibration, self.shutter_focal_length_px)
                        focal_calibration = shutter_obstacle.override_focal_length(
                            depth_calibration, self.shutter_focal_length_px)
                        mapped_depth_calibration = shutter_obstacle.scale_depth_calibration(
                            focal_calibration, *self.shutter_depth_pixel_mapping)
                        depth, rays = shutter_obstacle.depth_for_rgb(
                            record['depth'], frame.shape, rgb_calibration, mapped_depth_calibration,
                            self.shutter_rgb_rectified)
                        reading = shutter_obstacle.inspect(frame, depth, rays, up,
                                                           geometry_state=geometry, observation_time=observation_time,
                                                           fixed_plane=self.shutter_fixed_floor_plane,
                                                           **self.shutter_options)
                        reading['ground_frame'] = self.shutter_ground_frame
                        used_focal = float(np.asarray(rgb_calibration['k'], dtype=float)[0])
                        reading['focal_length_px'] = used_focal
                        reading['camera_info_focal_px'] = float(np.asarray(calibration['k'], dtype=float)[0])
                        # 바닥 평면으로 잰 카메라 높이와 TF 카메라 높이를 비교해 초점 거리 오류를 드러낸다.
                        reading.update(shutter_obstacle.camera_height_check(
                            reading.get('plane'), float(transform.transform.translation.z), used_focal))
                        reading['depth_pixel_mapping'] = (
                            'rgb_and_depth_camera_info' if mapped_depth_calibration is focal_calibration
                            else 'rgb_and_depth_camera_info+pixel_correction(%.3f,%.1f,%.3f,%.1f)'
                            % self.shutter_depth_pixel_mapping)
                    except TransformException as error:
                        reading = shutter_obstacle.unconfirmed('tf_unavailable', str(error))
                    except (ValueError, np.linalg.LinAlgError, cv2.error) as error:
                        reading = shutter_obstacle.unconfirmed('calibration_or_geometry_invalid', str(error))
        reading.update(sync_information)
        with self._state_lock:
            if (self._stop_event.is_set() or self._inspection_mode != 'shutter'
                    or request_id != self._inspection_id):
                return shutter_obstacle.unconfirmed('inspection_stopped')
            # Depth 신선도는 계산 전에 확인했다. 계산 시간 때문에 정상 결과를 depth_stale로 버리지 않는다.
            if time.monotonic() - frame_time > self.camera_timeout:
                reading = shutter_obstacle.unconfirmed('rgb_stale')
            elif alignment_epoch != self._shutter_alignment_epoch:
                reading = shutter_obstacle.unconfirmed(
                    SHUTTER_ALIGNMENT_REASONS.get(self._shutter_alignment_status, 'depth_alignment_check_pending'),
                    self._shutter_alignment_detail)
                sync_information.update(depth_alignment_epoch=self._shutter_alignment_epoch,
                                        depth_alignment_check_status=self._shutter_alignment_status)
                geometry.clear()
            elif calibration is not self._shutter_calibration or depth_calibration is not self._shutter_depth_calibration:
                reading = shutter_obstacle.unconfirmed('calibration_changed')
                geometry.clear()
            else:
                self._shutter_state['geometry'] = geometry
            reading.update(sync_information)
            confirmed = shutter_obstacle.stabilize_report(
                reading, self._shutter_state, observation_time, observation_stamp, **self.shutter_options)
            confirmed['inspection_id'] = request_id
            return confirmed

    def _inspect_extinguisher_aruco(self, result) -> list[dict]:
        """이번 YOLO 결과의 소화기 좌표만 모아 ArUco 모듈에 전달하고 소화기별 결과를 반환한다.

        소화기가 없으면 모듈의 마커 검출도 호출하지 않는다. 압력 검사 활성 여부와는 별개다.
        영상 수신이나 YOLO 추가 추론은 하지 않고 이번 결과의 원본 영상과 좌표를 사용한다.
        """
        if not self.aruco_enabled or self.extinguisher_aruco is None or result.boxes is None:
            return []
        extinguishers = []
        for index, box in enumerate(result.boxes):
            name = normalize_yolo_name(result.names[int(box.cls.item())])
            if name in YOLO_EXTINGUISHER_NAMES:
                extinguishers.append({'box_index': index, 'bbox': box.xyxy[0].cpu().tolist(),
                                     'confidence': float(box.conf.item())})
        if not extinguishers:
            return []
        return self.extinguisher_aruco.inspect(result.orig_img, extinguishers)

    def _inspect_extinguisher_pressures(self, result) -> list[dict]:
        """소화기 안의 압력계를 찾아 함수 모듈에 판독을 맡긴다. 좌표는 전체 사진 기준이다.

        전체 영상의 압력계 검출을 먼저 사용하고, 없으면 같은 YOLO를 소화기 crop에 한 번 실행한다.
        기존 색 범위로 정상 영역 안의 바늘색 존재 여부를 판독한다.
        압력계·정상 영역을 못 찾으면 미확인이다. 영역 안에 바늘색이 있으면 정상, 없으면 비정상이다.
        반환값은 이번 사진만의 판정이다. 정상 복귀 확인은 다음 단계에서 처리한다.
        """
        reports = []
        if not self.pressure_enabled or result.boxes is None:
            return reports
        boxes = list(result.boxes)
        frame = result.orig_img
        bodies = {}
        for index, box in enumerate(boxes):
            if normalize_yolo_name(result.names[int(box.cls.item())]) in YOLO_EXTINGUISHER_NAMES:
                body = extinguisher_pressure.clip_bbox(box.xyxy[0].cpu().tolist(), frame.shape)
                if body is not None:
                    bodies[index] = body
        candidates = {index: [] for index in bodies}
        ambiguous = {index: [] for index in bodies}
        # 겹친 소화기가 있어도 압력계 한 개를 여러 소화기에 연결하지 않는다.
        for gauge_index, gauge in enumerate(boxes):
            if normalize_yolo_name(result.names[int(gauge.cls.item())]) not in YOLO_GAUGE_NAMES:
                continue
            gauge_box = extinguisher_pressure.clip_bbox(gauge.xyxy[0].cpu().tolist(), frame.shape)
            if gauge_box is None:
                continue
            owners = [index for index, body in bodies.items()
                      if extinguisher_pressure.gauge_belongs_to_extinguisher(gauge_box, body)]
            if len(owners) == 1:
                candidates[owners[0]].append((gauge_index, gauge_box))
            elif len(owners) > 1:
                for owner in owners:
                    ambiguous[owner].append(gauge_index)
        for index, body in bodies.items():
            x1, y1, x2, y2 = body
            gauge_candidates = candidates[index]
            crop_error = False
            if not gauge_candidates and not ambiguous[index]:
                crop = frame[y1:y2, x1:x2].copy()
                try:
                    local = self.model.predict(source=crop, conf=self.conf, iou=self.iou,
                                               imgsz=self.imgsz, device=self.device, verbose=False)[0]
                except Exception as error:
                    crop_error = True
                    local = None
                    self.get_logger().warning(f'Pressure gauge crop inference failed: {error}',
                                              throttle_duration_sec=5.0)
                if local is not None and local.boxes is not None:
                    for gauge in local.boxes:
                        if normalize_yolo_name(local.names[int(gauge.cls.item())]) not in YOLO_GAUGE_NAMES:
                            continue
                        local_box = extinguisher_pressure.clip_bbox(
                            gauge.xyxy[0].cpu().tolist(), crop.shape)
                        if local_box is None:
                            continue
                        gx1, gy1, gx2, gy2 = local_box
                        gauge_box = (gx1 + x1, gy1 + y1, gx2 + x1, gy2 + y1)
                        if extinguisher_pressure.gauge_belongs_to_extinguisher(gauge_box, body):
                            gauge_candidates.append((None, gauge_box))
            readings = []
            for _, gauge_box in gauge_candidates:
                # 소화기 검출 영역 밖의 색상·바늘은 판독에 사용하지 않는다.
                gx1, gy1, gx2, gy2 = gauge_box
                inside_box = (max(x1, gx1), max(y1, gy1), min(x2, gx2), min(y2, gy2))
                readings.append(extinguisher_pressure.inspect_pressure(
                    frame, inside_box, **self.pressure_options))
            report = extinguisher_pressure.select_pressure_report(readings)
            if ambiguous[index]:
                report.update(status='UNCONFIRMED', reason='pressure gauge belongs to multiple extinguishers')
            elif crop_error:
                report.update(status='UNCONFIRMED', reason='pressure gauge crop inference failed')
            report.update(box_index=index, bbox=body, confidence=float(boxes[index].conf.item()),
                          class_id=int(boxes[index].cls.item()),
                          gauge_indices=ambiguous[index] + [gauge_index for gauge_index, _ in gauge_candidates
                                                           if gauge_index is not None])
            reports.append(report)
        return reports

    def _stabilize_extinguisher_pressures(self, reports, frame_time, stamp, request_id) -> list[dict]:
        """현재 검사에 속한 새 사진만으로 압력 판정을 확정해 화면·발행용 목록을 반환한다.

        reports는 한 사진의 판독, frame_time은 수신 시각(초), stamp는 ROS 촬영 시각이다.
        중지된 검사나 오래 걸려 유효 시간이 지난 사진은 확인 이력에 반영하지 않는다.
        """
        with self._state_lock:
            if (self._stop_event.is_set() or self._inspection_mode != 'yolo'
                    or request_id != self._inspection_id):
                return []
            if time.monotonic() - frame_time > self.camera_timeout:
                extinguisher_pressure.reset_stability_state(self._pressure_state)
                return [dict(report, raw_status=report['status'], raw_reason=report['reason'],
                             status='UNCONFIRMED', reason='pressure camera frame expired')
                        for report in reports]
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            confirmed = extinguisher_pressure.stabilize_pressure_reports(
                reports, self._pressure_state, frame_time, observation_stamp=stamp_ns or None,
                normal_confirm_seconds=self.pressure_normal_confirm_seconds,
                normal_confirm_frames=self.pressure_normal_confirm_frames,
                max_gap_seconds=self.camera_timeout)
            for report in confirmed:
                report['inspection_id'] = request_id
            return confirmed

    def _inspect_exit_signs(self, result) -> list[dict]:
        """이번 YOLO 결과의 비상구 영역과 원본 영상을 점등 판독 함수에 전달해 결과 목록을 반환한다.

        표지 종류는 클래스 이름으로 찾으며 점등·소등은 이름이나 YOLO 신뢰도로 정하지 않는다.
        다른 검출 물체의 영역은 주변 밝기를 측정할 때 제외한다. 추가 모델 추론은 하지 않는다.
        """
        if not self.exit_sign_enabled or result.boxes is None:
            return []
        boxes = list(result.boxes)
        excluded = [box.xyxy[0].cpu().tolist() for box in boxes]
        reports = []
        for index, box in enumerate(boxes):
            class_id = int(box.cls.item())
            if normalize_yolo_name(result.names[class_id]) not in YOLO_EXIT_SIGN_NAMES:
                continue
            report = exit_sign.inspect(result.orig_img, excluded[index],
                                       excluded_boxes=excluded, **self.exit_sign_options)
            report.update(box_index=index, class_id=class_id, confidence=float(box.conf.item()))
            reports.append(report)
        return reports

    def _stabilize_exit_signs(self, reports, frame_time, stamp, request_id, frame_id) -> list[dict]:
        """현재 검사와 같은 카메라의 새 사진만 비상구 연속 확인에 반영해 화면·발행용 판정을 반환한다.

        reports는 이번 판독이며 frame_time은 수신 시각, stamp는 촬영 시각, request_id는 검사 번호다.
        """
        with self._state_lock:
            if (self._stop_event.is_set() or self._inspection_mode != 'yolo'
                    or request_id != self._inspection_id or not self.exit_sign_enabled):
                return []
            if time.monotonic() - frame_time > self.camera_timeout:
                exit_sign.reset_stability_state(self._exit_sign_state)
                return exit_sign.unconfirmed_reports(reports, 'exit sign camera frame expired')
            previous_frame_id = self._exit_sign_state.get('frame_id')
            if previous_frame_id is not None and previous_frame_id != frame_id:
                exit_sign.reset_stability_state(self._exit_sign_state)
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            confirmed = exit_sign.stabilize_reports(
                reports, self._exit_sign_state, frame_time, observation_stamp=stamp_ns or None,
                max_gap_seconds=self.camera_timeout, **self.exit_sign_options)
            self._exit_sign_state['frame_id'] = frame_id
            for report in confirmed:
                report['inspection_id'] = request_id
            return confirmed

    def _inspect_rack_loading(self, result) -> dict:
        """같은 YOLO 추론 결과에서 랙·박스 좌표를 뽑아 적재 판독 모듈에 전달한다.

        클래스 ID 숫자는 고정하지 않고 이름으로 구분한다. 추가 모델 추론은 하지 않는다.
        반환값의 racks/boxes 목록에는 이번 사진의 소속과 경계 초과 후보가 들어 있다.
        """
        if not self.rack_loading_enabled or result.boxes is None:
            return {'racks': [], 'boxes': []}
        racks, boxes = [], []
        for index, box in enumerate(result.boxes):
            name = normalize_yolo_name(result.names[int(box.cls.item())])
            if name not in YOLO_RACK_NAMES and name not in YOLO_BOX_NAMES:
                continue
            record = {'box_index': index, 'bbox': box.xyxy[0].cpu().tolist(),
                      'confidence': float(box.conf.item())}
            (racks if name in YOLO_RACK_NAMES else boxes).append(record)
        return rack_loading.inspect_loading(racks, boxes, result.orig_img.shape, **self.rack_loading_options)

    def _stabilize_rack_loading(self, reading, frame_time, stamp, request_id, frame_id) -> dict:
        """현재 YOLO 검사와 같은 카메라의 새 사진만 랙·박스 연속 확인에 반영한다.

        frame_time은 수신 시각(초), stamp는 촬영 시각, request_id는 검사 번호다.
        반환한 확정 결과 하나를 화면·로그·ROS 불량 발행에 함께 전달한다.
        """
        with self._state_lock:
            if (self._stop_event.is_set() or self._inspection_mode != 'yolo'
                    or request_id != self._inspection_id or not self.rack_loading_enabled):
                return {'racks': [], 'boxes': []}
            if time.monotonic() - frame_time > self.camera_timeout:
                rack_loading.reset_stability_state(self._rack_loading_state)
                return rack_loading.unconfirmed_result(reading, 'loading camera frame expired')
            previous_frame_id = self._rack_loading_state.get('frame_id')
            if previous_frame_id is not None and previous_frame_id != frame_id:
                rack_loading.reset_stability_state(self._rack_loading_state)
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            confirmed = rack_loading.stabilize_result(
                reading, self._rack_loading_state, frame_time, observation_stamp=stamp_ns or None,
                max_gap_seconds=self.camera_timeout, **self.rack_loading_options)
            self._rack_loading_state['frame_id'] = frame_id
            for report in confirmed['racks']:
                report['inspection_id'] = request_id
            return confirmed

    def _check_pressure_camera_timeout(self) -> None:
        """설비 검사 중 RGB가 끊기거나 셔터 Depth가 만료되면 옛 결과를 지우고 미확인 화면을 만든다.

        영상 콜백과 별도로 실행하므로 새 사진이 오지 않아도 검사 결과를 지우고 경고를 표시한다.
        셔터는 RGB·Depth 수신이나 시각이 맞는 짝이 camera_timeout 동안 끊기면 옛 결과를 지운다.
        Wi-Fi 지연으로 Depth 한두 장이 늦거나 한 장 계산이 길어지는 정도로는 화면을 지우지 않는다.
        """
        with self._state_lock:
            if self._inspection_mode == 'shutter' and not self._stop_event.is_set():
                reason = None
                now = time.monotonic()
                if self.last_frame_time is None or now - self.last_frame_time > self.camera_timeout:
                    reason = 'rgb_stale'
                elif not self._shutter_depth_frames:
                    reason = 'depth_missing'
                elif now - self._shutter_depth_frames[-1]['received'] > self.camera_timeout:
                    reason = 'depth_stale'
                elif (self._shutter_last_pair_time is not None
                      and now - self._shutter_last_pair_time > self.camera_timeout):
                    reason = 'rgb_depth_pair_stale'
                if reason is None:
                    self._shutter_timeout_reason = None
                    return
                if self._shutter_timeout_reason == reason:
                    return
                self._shutter_timeout_reason = reason
                shutter_obstacle.reset_stability_state(self._shutter_state)
                self._shutter_report = shutter_obstacle.unconfirmed(reason)
                self._inspection_result = None
                if self.latest_frame is not None:
                    self._display_frame = shutter_obstacle.draw_result(
                        self.latest_frame, self._shutter_report,
                        font_path=self.shutter_font_path, font_size=self.shutter_font_size)
                return
            if (self._stop_event.is_set() or self._inspection_mode != 'yolo'
                    or not (self.pressure_enabled or self.rack_loading_enabled
                            or self.aruco_enabled or self.exit_sign_enabled)
                    or self.last_frame_time is None or self._pressure_timeout_active
                    or time.monotonic() - self.last_frame_time <= self.camera_timeout):
                return
            extinguisher_pressure.reset_stability_state(self._pressure_state)
            rack_loading.reset_stability_state(self._rack_loading_state)
            exit_sign.reset_stability_state(self._exit_sign_state)
            self._pressure_timeout_active = True
            self._inspection_result = None
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
                cv2.putText(frame, 'Inspection UNCONFIRMED: camera timeout', (12, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (192, 192, 192), 2, cv2.LINE_AA)
                self._display_frame = frame

    @staticmethod
    def annotate_yolo_result(result, pressure_reports=(), loading_result=None,
                             aruco_reports=(), exit_reports=()) -> tuple[np.ndarray, list[str]]:
        """YOLO 박스·설비 상태·소화기 ArUco 마커를 표시하고 비상구 점등은 초록, 소등은 빨강으로 그린다.

        원본 사진은 보존하고 (표시용 사진, 물체별 로그 문자열 목록)을 반환한다.
        loading_result에는 racks/boxes 목록이 있다. 후보·연속 횟수·초과 방향·대상 연결도 로그에 남긴다.
        aruco_reports는 이번 소화기별 마커 결과다. 더미 만료 색은 압력 판정과 별도로 표시한다.
        exit_reports는 이번 비상구의 연속 확인 결과이며 미확인은 회색이다.
        """
        from ultralytics.utils.plotting import Annotator

        annotator = Annotator(result.orig_img.copy(), example=str(result.names))
        by_index = {report['box_index']: report for report in pressure_reports}
        gauges = {index: report for report in pressure_reports for index in report['gauge_indices']}
        loading_result = loading_result or {'racks': [], 'boxes': []}
        loading_by_index = {report['box_index']: report
                            for report in loading_result['racks'] + loading_result['boxes']}
        exits_by_index = {report['box_index']: report for report in exit_reports}
        detections = []
        if result.boxes is not None:
            for index in reversed(range(len(result.boxes))):
                box = result.boxes[index]
                class_id = int(box.cls.item())
                name = result.names[class_id]
                confidence = float(box.conf.item())
                style_id = YOLO_LABEL_STYLE_IDS.get(normalize_yolo_name(name))
                color, state = YOLO_CLASS_STYLES.get(style_id, ((192, 192, 192), 'UNCONFIRMED'))
                report = by_index.get(index, gauges.get(index, loading_by_index.get(index, exits_by_index.get(index))))
                if report is not None:
                    state = report['status']
                    color = {'NORMAL': (0, 255, 0), 'ABNORMAL': (0, 0, 255),
                             'UNCONFIRMED': (192, 192, 192)}[state]
                    if index in gauges:
                        name = 'Pressure gauge'
                label = f'{name} {confidence:.2f}'
                log = f'{name}:{confidence:.2f} verdict={state}'
                if index in exits_by_index:
                    light = {'NORMAL': 'ON', 'ABNORMAL': 'OFF', 'UNCONFIRMED': 'UNCONFIRMED'}[state]
                    label += f' {light}'
                    log += f" light={light} raw_verdict={report.get('raw_status', state)}"
                    for key in ('y_median', 'y_p90', 'v_median', 'green_ratio', 'bright_ratio', 'relative_contrast'):
                        value = report.get(key)
                        log += f' {key}={round(value, 3) if value is not None else None}'
                    log += f" exit_frames={report.get('confirm_frames', 0)} reason={report['reason']}"
                elif index in by_index:
                    label += f' {state}'
                    log += f" needle_in_normal={report.get('needle_in_normal_range')}"
                    log += f" needle_pixels={report.get('needle_in_normal_pixels', 0)}"
                    log += f" raw_verdict={report.get('raw_status', report['status'])}"
                    log += f" normal_frames={report.get('normal_confirm_frames', 0)}"
                    log += f" normal_elapsed={report.get('normal_confirm_elapsed_s', 0):.2f}s"
                    log += f" reason={report['reason']}"
                elif index in loading_by_index:
                    label += f' {state}'
                    log += f" raw_verdict={report.get('raw_status', report['status'])}"
                    if 'directions' in report:
                        log += f" directions={report['directions']} overflow_px={report['overflow_px']}"
                        log += f" loading_frames={report.get('confirm_frames', 0)}"
                        log += f" box_track={report.get('box_track_id')} tracking={report.get('tracking_reason')}"
                    else:
                        log += f" rack_track={report.get('rack_track_id')} boxes={len(report['boxes'])}"
                        log += f" normal_frames={report.get('normal_confirm_frames', 0)}"
                    log += f" reason={report['reason']}"
                annotator.box_label(box.xyxy[0].cpu().tolist(), label, color=color)
                detections.append(log)
        detections.reverse()
        annotated = annotator.result()
        for report in pressure_reports:
            extinguisher_pressure.draw_result(annotated, report)
        rack_loading.draw_result(annotated, loading_result)
        extinguisher_aruco.ExtinguisherAruco.draw_result(annotated, aruco_reports)
        return annotated, detections

    @staticmethod
    def _collect_yolo_defects(result, pressure_reports=(), loading_result=None,
                              exit_reports=()) -> list[tuple[int, str]]:
        """확정 압력·비상구 소등·랙 적재 불량과 독립 YOLO 불량을 (설비 종류, JSON 설명) 목록으로 만든다.

        소화기에 연결된 압력계는 화면과 같은 확정 판정을 쓴다. 정상·미확인은 발행하지 않는다.
        랙 불량은 랙당 한 건이며 연결된 불량 박스의 초과 방향·픽셀·비율을 JSON 근거로 담는다.
        비상구 후처리 결과가 있으면 이전 on/off 클래스 대신 같은 화면의 확정 판정만 사용한다.
        """
        defects = []
        if result.boxes is None:
            return defects
        associated = {index for report in pressure_reports for index in report['gauge_indices']}
        associated.update(report['box_index'] for report in exit_reports)
        for index, box in enumerate(result.boxes):
            if index in associated:
                continue
            class_id = int(box.cls.item())
            style_id = YOLO_LABEL_STYLE_IDS.get(normalize_yolo_name(result.names[class_id]))
            kind = YOLO_DEFECT_KINDS.get(style_id)
            if kind is None:
                continue
            detail = json.dumps({
                'mode': 'yolo', 'class_id': class_id, 'class_name': result.names[class_id],
                'confidence': float(box.conf.item()), 'bbox': box.xyxy[0].cpu().tolist(),
            }, ensure_ascii=False)
            defects.append((kind, detail))
        for report in exit_reports:
            if report['status'] != 'ABNORMAL':
                continue
            detail = json.dumps({
                'mode': 'yolo', 'source': 'exit_sign_brightness', 'reason': report['reason'],
                'bbox': report['bbox'], 'confidence': report['confidence'],
                'class_id': report['class_id'], 'class_name': result.names[report['class_id']],
                'face_polygon': report['face_polygon'], 'y_median': report['y_median'], 'y_p90': report['y_p90'],
                'v_median': report['v_median'], 'green_ratio': report['green_ratio'],
                'bright_ratio': report['bright_ratio'], 'relative_contrast': report['relative_contrast'],
                'reference_y': report['reference_y'], 'exit_track_id': report.get('exit_track_id'),
                'inspection_id': report.get('inspection_id'), 'confirm_frames': report.get('confirm_frames'),
                'confirm_elapsed_s': report.get('confirm_elapsed_s'),
            }, ensure_ascii=False)
            defects.append((InspectionResult.KIND_EXIT_LIGHT, detail))
        for report in pressure_reports:
            if report['status'] != 'ABNORMAL':
                continue
            detail = json.dumps({
                'mode': 'yolo', 'reason': report['reason'], 'source': 'yellow_needle_presence',
                'class_id': report['class_id'],
                'class_name': result.names[report['class_id']],
                'confidence': report['confidence'], 'bbox': report['bbox'],
                'gauge_bbox': report['gauge_bbox'], 'dial_center': report['dial_center'],
                'needle_angle_deg': None,  # 이전 JSON을 읽는 코드와의 호환용이며 더 이상 측정하지 않는다.
                'needle_in_normal_range': report.get('needle_in_normal_range'),
                'needle_in_normal_pixels': report.get('needle_in_normal_pixels', 0),
                'raw_status': report.get('raw_status', report['status']),
                'raw_reason': report.get('raw_reason', report['reason']),
                'pressure_track_id': report.get('pressure_track_id'),
                'inspection_id': report.get('inspection_id'),
                'normal_confirm_frames': report.get('normal_confirm_frames', 0),
                'normal_confirm_elapsed_s': report.get('normal_confirm_elapsed_s', 0.0),
                'normal_confirm_required_s': report.get('normal_confirm_required_s'),
                'green_start_deg': report['green_start_deg'], 'green_end_deg': report['green_end_deg'],
                'range_source': report['range_source'],
            }, ensure_ascii=False)
            defects.append((InspectionResult.KIND_EXTINGUISHER, detail))
        for rack in (loading_result or {}).get('racks', []):
            if rack['status'] != 'ABNORMAL':
                continue
            protrusions = [{key: box.get(key) for key in (
                'box_index', 'box_track_id', 'bbox', 'confidence', 'directions', 'overflow_px', 'overflow_ratio',
                'association_overlap_ratio', 'raw_status', 'confirm_frames', 'required_frames')}
                for box in rack['boxes'] if box['status'] == 'ABNORMAL']
            detail = json.dumps({
                'mode': 'yolo', 'source': 'yolo_rack_box_containment', 'reason': rack['reason'],
                'rack_bbox': rack['bbox'], 'rack_confidence': rack['confidence'],
                'rack_track_id': rack.get('rack_track_id'), 'inspection_id': rack.get('inspection_id'),
                'coordinate_format': 'xyxy', 'tolerance_ratio': rack['tolerance_ratio'],
                'raw_status': rack.get('raw_status', rack['status']), 'protrusions': protrusions,
            }, ensure_ascii=False)
            defects.append((InspectionResult.KIND_RACK, detail))
        return defects

    def _publish_defects(self, defects, stamp, mode, request_id) -> None:
        """현재 검사에서 나온 (설비 종류, JSON 설명)을 불량 메시지로 즉시 발행한다.

        stamp는 입력 사진 시각이며 0이면 현재 ROS 시각을 사용한다.
        중지·이전 요청·영상 단절 결과는 버리고, 셔터는 Depth 수신 후 camera_timeout 초과와 정렬 확인 변경도 확인한다.
        서비스에 설비·순찰 정보가 없으므로 관련 식별자는 기본값으로 두고 사진은 따로 전송하지 않는다.
        """
        if not defects:
            return
        with self._state_lock:
            if (self._stop_event.is_set() or mode != self._inspection_mode
                    or request_id != self._inspection_id or mode not in ('yolo', 'rack', 'shutter')):
                return
            if mode in ('yolo', 'shutter') and (self.last_frame_time is None
                                  or time.monotonic() - self.last_frame_time > self.camera_timeout):
                return
            if mode == 'shutter':
                reading = self._shutter_report
                if (not reading or reading['status'] != 'ABNORMAL' or reading.get('depth_received') is None
                        or not self.shutter_depth_aligned
                        or reading.get('depth_alignment_epoch') != self._shutter_alignment_epoch
                        or time.monotonic() - reading['depth_received'] > self.camera_timeout):
                    # Depth 신선도는 짝 선택 때 확인했다. 여기서는 계산 시간을 넘긴 오래된 결과만 막는다.
                    return
            event_stamp = stamp if stamp.sec or stamp.nanosec else self.get_clock().now().to_msg()
            for kind, detail in defects:
                event = InspectionResult()
                event.stamp = event_stamp
                event.robot_id = self.get_namespace().strip('/')
                event.kind = kind
                event.verdict = InspectionResult.ABNORMAL
                event.detail = detail
                event.attempts = 1  # 순찰 액션의 자세 재시도 없이 이번 사진에서 얻은 검출이다.
                # 확인부터 발행까지 같은 잠금을 유지하여 중지 응답 뒤에 옛 불량을 보내지 않는다.
                self.result_publisher.publish(event)
                if self._facility_defects is not None:
                    self._facility_defects.append((kind, detail))

    def image_callback(self, message: CompressedImage) -> None:
        """RGB를 받아 선택한 설비를 검사하고 셔터 모드에서는 좌측 상단에 한글 경고·정상 문구를 표시한다.

        더미 제조 정보는 info 로그로만 출력하고 기존 설비 불량 발행 방식을 유지한다.
        ArUco는 이번 사진의 소화기 검출이 있어야 실행하며 실패·중지·영상 만료 시 결과를 버린다.
        셔터 바닥 마스크는 화면 표시에만 사용하고 ROS 상태의 JSON 설명에서는 제외한다.
        셔터 RGB는 처리 속도 제한 전에 보관해 늦게 도착한 Depth와 연결하며, 선택한 RGB 위에 결과를 그린다.
        이번 검사에서 짝을 찾은 뒤에는 다음 짝을 기다리는 동안(camera_timeout 이내) 마지막 결과 화면을 유지한다.
        Depth 신선도는 짝을 고를 때만 확인하고, 계산 시간 때문에 결과를 버리지 않는다.
        셔터 바닥 평면의 실패 단계·기울기·일치 비율·오차도 로그로 출력한다.
        정렬 확인이 계산 중 바뀌면 이전 설정의 결과를 표시하거나 발행하지 않는다.
        """
        if self._stop_event.is_set():
            return
        now = time.monotonic()
        with self._state_lock:
            if self._inspection_mode == 'shutter':
                stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                self._shutter_rgb_frames.append({'message': message, 'received': now, 'stamp_ns': stamp_ns})
                self._shutter_rgb_timing.append((stamp_ns, now))
        if self.max_fps > 0 and now - self.last_inference_time < 1.0 / self.max_fps:
            return
        self.last_inference_time = now
        message, now, shutter_depth_record, shutter_selection_id = self._select_shutter_pair(message, now)
        with self._state_lock:
            if (shutter_selection_id is not None and shutter_selection_id == self._inspection_id
                    and self._inspection_mode == 'shutter' and shutter_depth_record is None
                    and self._shutter_last_pair_time is not None
                    and time.monotonic() - self._shutter_last_pair_time <= self.camera_timeout):
                # 다음 짝을 기다리는 RGB는 화면을 다시 그리지 않는다. 마지막 짝의 결과(바닥 표시)를 유지해
                # 짝이 없는 RGB마다 '시각 불일치' 화면으로 바뀌며 깜빡이는 것을 막는다.
                # 짝이 camera_timeout 동안 없으면 이 조건이 풀리고 감시 타이머가 미확인으로 바꾼다.
                self.last_frame_time = now
                self._log_shutter_timing(now)
                return

        try:
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(f"Failed to decode compressed image: {error}")
            frame = None
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image")
            with self._state_lock:
                if self._inspection_mode == 'yolo':
                    extinguisher_pressure.reset_stability_state(self._pressure_state)
                    rack_loading.reset_stability_state(self._rack_loading_state)
                    exit_sign.reset_stability_state(self._exit_sign_state)
                elif self._inspection_mode == 'shutter':
                    shutter_obstacle.reset_stability_state(self._shutter_state)
                    self._shutter_report = shutter_obstacle.unconfirmed('rgb_decode_error')
                    if self.latest_frame is not None:
                        self._display_frame = shutter_obstacle.draw_result(
                            self.latest_frame, self._shutter_report,
                            font_path=self.shutter_font_path, font_size=self.shutter_font_size)
            return
        with self._state_lock:
            self.latest_frame = frame.copy()
            self.last_frame_time = now
            self._pressure_timeout_active = False
            mode = self._inspection_mode
            request_id = self._inspection_id
            if shutter_selection_id is not None and (mode != 'shutter' or shutter_selection_id != request_id):
                # 짝을 고르는 사이 검사를 바꿨으면 이전 검사 입력은 미리보기로만 표시한다.
                mode = None
            rack_state = self.rack_state

        annotated = frame.copy()
        detections = []
        defects = []
        rack_report = None
        aruco_reports = []
        exit_reports = []
        shutter_report = None
        outcome = None
        try:
            if mode == 'yolo':
                result = self.model.predict(
                    source=frame, conf=self.conf, iou=self.iou, imgsz=self.imgsz,
                    device=self.device, verbose=False,
                )[0]
                pressure_reports = self._inspect_extinguisher_pressures(result)
                aruco_reports = self._inspect_extinguisher_aruco(result)
                exit_reports = self._inspect_exit_signs(result)
                loading_result = self._inspect_rack_loading(result)
                pressure_reports = self._stabilize_extinguisher_pressures(
                    pressure_reports, now, message.header.stamp, request_id)
                loading_result = self._stabilize_rack_loading(
                    loading_result, now, message.header.stamp, request_id, message.header.frame_id)
                exit_reports = self._stabilize_exit_signs(
                    exit_reports, now, message.header.stamp, request_id, message.header.frame_id)
                annotated, detections = self.annotate_yolo_result(
                    result, pressure_reports, loading_result, aruco_reports, exit_reports)
                defects = self._collect_yolo_defects(result, pressure_reports, loading_result, exit_reports)
                summary = ', '.join(detections) if detections else 'none'
                # 내부 결과의 success는 추론 성공이다. 검출 0개도 정상 상태를 뜻하지 않는다.
                outcome = {'success': True,
                           'message': f'YOLO detections={len(detections)} [{summary}]',
                           'pressures': pressure_reports,
                           'rack_loading': loading_result,
                           'aruco_markers': aruco_reports,
                           'exit_signs': exit_reports,
                           'time': time.monotonic()}
            elif mode == 'shutter':
                shutter_report = self._inspect_shutter(
                    frame, message.header, now, request_id, depth_record=shutter_depth_record)
                annotated = shutter_obstacle.draw_result(
                    frame, shutter_report, font_path=self.shutter_font_path, font_size=self.shutter_font_size)
                outcome = {'success': shutter_report['status'] != 'UNCONFIRMED',
                           'message': shutter_obstacle.STATUS_MESSAGES[shutter_report['status']],
                           'shutter': shutter_report, 'time': time.monotonic()}
                if shutter_report['status'] == 'ABNORMAL':
                    defects = [(InspectionResult.KIND_SHUTTER, json.dumps(
                        dict(shutter_obstacle.report_details(shutter_report),
                             mode='shutter', source='rgbd_floor_ransac'), ensure_ascii=False))]
            elif mode == 'rack':
                roi_frame, roi_box = self.crop_rack_roi(frame)
                rack_report = rack_conditioner.stabilize_result(
                    rack_conditioner.inspect_rack(
                        roi_frame, reference=self.reference_for_roi(roi_box),
                        **self.rack_inspect_options),
                    rack_state, self.rack_confirm_frames)
                x1, y1, x2, y2 = roi_box
                annotated[y1:y2, x1:x2] = rack_conditioner.draw_result(roi_frame, rack_report)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 1)
                verdict = rack_report['verdict']
                if verdict != InspectionResult.UNCONFIRMED:
                    normal = verdict == InspectionResult.NORMAL
                    outcome = {'success': normal,
                               'message': f"{'NORMAL' if normal else 'ABNORMAL'}: {rack_report['reason']}",
                               'time': time.monotonic()}
                if verdict == InspectionResult.ABNORMAL:
                    # ROI 안의 돌출 좌표를 전체 사진 좌표로 바꿔 YOLO와 같은 기준으로 보낸다.
                    protrusions = []
                    for hit in rack_report['protrusions']:
                        bx, by, bw, bh = hit['bbox']
                        protrusions.append({'side': hit['side'],
                                            'bbox': [bx + x1, by + y1, bw, bh],
                                            'area': float(hit['area'])})
                    defects = [(InspectionResult.KIND_RACK, json.dumps({
                        'mode': 'rack', 'reason': rack_report['reason'], 'protrusions': protrusions,
                    }, ensure_ascii=False))]
        except Exception as error:
            self.get_logger().error(f'{mode} inspection failed: {error}')
            annotated = frame.copy()
            defects = []
            aruco_reports = []
            if mode == 'shutter':
                shutter_report = shutter_obstacle.unconfirmed('inspection_error', str(error))
                annotated = shutter_obstacle.draw_result(
                    frame, shutter_report, font_path=self.shutter_font_path, font_size=self.shutter_font_size)
                with self._state_lock:
                    if mode == self._inspection_mode and request_id == self._inspection_id:
                        shutter_obstacle.reset_stability_state(self._shutter_state)
            with self._state_lock:
                if mode == 'yolo' and mode == self._inspection_mode and request_id == self._inspection_id:
                    extinguisher_pressure.reset_stability_state(self._pressure_state)
                    rack_loading.reset_stability_state(self._rack_loading_state)
                    exit_sign.reset_stability_state(self._exit_sign_state)
            outcome = {'success': False, 'message': f'{mode} inspection failed: {error}',
                       'time': time.monotonic()}

        with self._state_lock:
            if mode is not None and mode == self._inspection_mode and request_id == self._inspection_id:
                if mode == 'shutter':
                    # Depth 신선도는 짝을 고를 때 확인했으므로 계산 시간만큼 늦었다고 결과를 버리지 않는다.
                    expired = time.monotonic() - now > self.camera_timeout
                    alignment_changed = (shutter_report is not None
                                         and shutter_report.get('depth_alignment_epoch') != self._shutter_alignment_epoch)
                    if expired or alignment_changed:
                        shutter_obstacle.reset_stability_state(self._shutter_state)
                        if expired:
                            shutter_report = shutter_obstacle.unconfirmed('rgb_stale')
                        else:
                            shutter_report = shutter_obstacle.unconfirmed(
                                SHUTTER_ALIGNMENT_REASONS.get(
                                    self._shutter_alignment_status, 'depth_alignment_check_pending'),
                                self._shutter_alignment_detail)
                        shutter_report['depth_alignment_epoch'] = self._shutter_alignment_epoch
                        annotated = shutter_obstacle.draw_result(
                            frame, shutter_report, font_path=self.shutter_font_path, font_size=self.shutter_font_size)
                        defects, outcome = [], None
                    self._shutter_report = shutter_report
                if (mode == 'yolo' and (self.pressure_enabled or self.rack_loading_enabled
                                        or self.aruco_enabled or self.exit_sign_enabled)
                        and time.monotonic() - now > self.camera_timeout):
                    # 느린 추론 결과가 영상 단절 경고를 옛 정상 화면으로 덮지 않게 한다.
                    self._check_pressure_camera_timeout()
                    annotated = frame.copy()
                    cv2.putText(annotated, 'Inspection UNCONFIRMED: camera timeout', (12, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (192, 192, 192), 2, cv2.LINE_AA)
                    detections = ['Inspection UNCONFIRMED: camera frame expired']
                    defects = []
                    aruco_reports = []
                    outcome = None
                # 마지막 프레임이 미확인이면 과거의 확정 결과를 지워 옛 판정이 남지 않게 한다.
                self._inspection_result = outcome
            elif mode is not None:
                # 끝난 요청의 계산 결과를 다음 검사나 대기 화면에 섞지 않는다.
                annotated = frame.copy()
                detections = []
                defects = []
                rack_report = None
                aruco_reports = []
                shutter_report = None
                mode = None
            if self.show:
                self._display_frame = annotated

        # 불량 발행은 로그 주기나 서비스 종료를 기다리지 않는다. 중지 여부는 발행 직전에 재확인한다.
        self._publish_defects(defects, message.header.stamp, mode, request_id)

        try:
            ok, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                output = CompressedImage()
                output.header = message.header
                output.format = "jpeg"
                output.data = jpeg.tobytes()
                self.publisher.publish(output)
                self._last_jpeg = output.data
        except cv2.error as error:
            self.get_logger().error(f'Detection rendering failed: {error}')

        if mode is not None and now - self.last_log_time >= 1.0:
            if mode == 'yolo':
                summary = ', '.join(detections) if detections else 'none'
                self.get_logger().info(f'detections={len(detections)} [{summary}]')
                # ArUco 제조 정보는 로그 대신 IF-05 결과로 관제 DB(inspections.aruco)에 저장한다 (10/15)
            elif rack_report is not None:
                self.get_logger().info(f"rack verdict={rack_report['verdict']} reason={rack_report['reason']}")
            elif shutter_report is not None:
                self.get_logger().info(
                    f"shutter verdict={shutter_report['status']} reason={shutter_report['reason']} "
                    f"obstacles={len(shutter_report['obstacles'])} "
                    f"floor_inliers={shutter_report.get('floor_inlier_count', 0)} "
                    f"coverage={shutter_report.get('coverage_ratio')} "
                    f"floor_noise_m={shutter_report.get('floor_noise_m')} "
                    f"focal_px={shutter_report.get('focal_length_px')} "
                    f"camera_height_floor_m={shutter_report.get('camera_height_floor_m')} "
                    f"camera_height_tf_m={shutter_report.get('camera_height_tf_m')} "
                    f"camera_height_ratio={shutter_report.get('camera_height_ratio')} "
                    f"suggested_focal_px={shutter_report.get('suggested_focal_px')} "
                    f"plane={[round(v, 4) for v in shutter_report['plane']] if shutter_report.get('plane') else None} "
                    f"floor_diagnostics={shutter_report.get('floor_diagnostics')} "
                    f"ground_up_camera={shutter_report.get('ground_up_camera')} "
                    f"height_threshold_m={shutter_report.get('effective_height_m')} "
                    f"zone_tracking={shutter_report.get('zone_tracking')} "
                    f"confirm_frames={shutter_report.get('confirm_frames')} "
                    f"confirm_elapsed_s={shutter_report.get('confirm_elapsed_s')} "
                    f"obstacle_heights_m={[item.get('height_p90_m') for item in shutter_report['obstacles']]} "
                    f"rgb_depth_delta_s={shutter_report.get('rgb_depth_signed_diff_s')} "
                    f"sync_tolerance_s={self.shutter_sync_tolerance} "
                    f"depth_step_m={shutter_report.get('depth_step_m')} "
                    f"detail={shutter_report.get('detail', '')}")
            self.last_log_time = now
        if mode == 'shutter':
            self._log_shutter_timing(now)

    def destroy_node(self) -> bool:
        """영상 처리와 대기 중인 정렬 조회를 중지하고 영상 창·ROS 통신 자원을 정리한다."""
        self._stop_event.set()
        if self._shutter_alignment_future is not None:
            self._shutter_alignment_client.remove_pending_request(self._shutter_alignment_future)
            self._shutter_alignment_future.cancel()
            self._shutter_alignment_future = None
        if self.show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()


def main() -> None:
    """카메라를 실행하고 메인 스레드에서 영상 표시와 랙 기준 저장 키를 처리한 뒤 종료한다."""
    rclpy.init()
    node = None
    executor = None
    spin_thread = None
    try:
        node = ScvDetectionManager()
        # 영상 처리와 서비스 시작·중지 요청을 별도 워커에서 처리한다.
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        while rclpy.ok():
            if node.show:
                with node._state_lock:
                    frame = node._display_frame
                    node._display_frame = None
                try:
                    if frame is not None:
                        node.show_preview_frame(frame)
                    key = cv2.waitKey(30) & 0xFF
                    if key in (ord('n'), ord('N')):
                        node.capture_rack_reference()
                    elif key in (ord('r'), ord('R')):
                        node.reset_rack_reference()
                except cv2.error as error:
                    node.get_logger().warning(f"Display failed; disabling show: {error}")
                    node.show = False
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._stop_event.set()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
