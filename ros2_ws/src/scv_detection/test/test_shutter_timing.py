"""방화셔터 RGB·Depth 발행 주기 진단과 최신 짝 선택 시험."""

from collections import deque
from types import SimpleNamespace
import threading
import time

import numpy as np

from scv_detection import scv_detection_manager as module
from scv_detection import shutter_obstacle


def _samples(hz, count, offset_s=0.0, lag_s=0.0, clock_offset_s=100.0):
    """주기 hz로 촬영된 (촬영 시각 ns, 수신 시각 초) 표본을 만든다. 수신 시각은 다른 시계 기준이다."""
    stamps = (np.arange(count) / hz + 1000.0 + offset_s)
    return [(int(stamp * 1e9), stamp - clock_offset_s + lag_s) for stamp in stamps]


def test_stream_timing_reports_rates_phase_and_relative_lag():
    """30Hz RGB와 10Hz Depth가 같은 순간에 찍히면 위상차 0, Depth 추가 지연만 따로 나온다."""
    timing = shutter_obstacle.stream_timing(
        _samples(30, 90, lag_s=0.10), _samples(10, 30, lag_s=0.45), 0.03, (True, True, False, True))
    assert abs(timing['rgb_hz'] - 30) < 0.1 and abs(timing['depth_hz'] - 10) < 0.1
    assert timing['phase_median_ms'] < 1 and timing['matched_ratio'] == 1.0
    assert abs(timing['depth_extra_lag_s'] - 0.35) < 1e-6
    assert timing['pair_success_ratio'] == 0.75


def test_stream_timing_detects_phase_misalignment():
    """같은 10Hz라도 촬영 순간이 50ms 어긋나면 허용 30ms 안의 짝이 없다고 보고한다."""
    timing = shutter_obstacle.stream_timing(_samples(10, 30), _samples(10, 30, offset_s=0.05), 0.03)
    assert abs(timing['phase_median_ms'] - 50) < 1 and timing['matched_ratio'] == 0.0


def test_stream_timing_without_samples():
    """표본이 없으면 값을 만들지 않고 None으로 둔다."""
    timing = shutter_obstacle.stream_timing([], [], 0.03)
    assert timing['rgb_hz'] is None and timing['phase_median_ms'] is None and timing['depth_extra_lag_s'] is None


def _selector(rgb_stamps_s, depth_stamps_s, now):
    """짝 선택 함수가 읽는 속성만 가진 가짜 노드를 만든다. 모든 사진은 방금 받은 것으로 둔다."""
    rgb = deque({'message': f'rgb{stamp}', 'received': now, 'stamp_ns': int(stamp * 1e9)} for stamp in rgb_stamps_s)
    depth = deque({'stamp_ns': int(stamp * 1e9), 'received': now} for stamp in depth_stamps_s)
    return SimpleNamespace(
        _state_lock=threading.RLock(), _inspection_mode='shutter', _stop_event=threading.Event(),
        _inspection_id=7, _shutter_rgb_frames=rgb, _shutter_depth_frames=depth, _shutter_last_pair_stamp=0,
        _shutter_last_pair_time=None, _shutter_pair_results=deque(maxlen=50),
        camera_timeout=2.0, shutter_depth_timeout=0.5, shutter_sync_tolerance=0.03)


def test_pair_selection_prefers_newest_matching_depth():
    """시각 차이가 조금 더 작은 옛 짝보다 허용 범위 안의 최신 짝을 골라 결과가 뒤처지지 않게 한다."""
    now = time.monotonic()
    node = _selector([10.000, 10.033, 10.067, 10.100, 10.133], [10.001, 10.110], now)
    message, _, depth, request_id = module.ScvDetectionManager._select_shutter_pair(node, 'current', now)
    assert request_id == 7 and message == 'rgb10.1' and depth['stamp_ns'] == int(10.110 * 1e9)
    assert [item['stamp_ns'] for item in node._shutter_rgb_frames] == [int(10.133 * 1e9)]
    assert node._shutter_last_pair_time >= now and list(node._shutter_pair_results) == [True]
    # 같은 Depth는 다시 쓰지 않으며 새 Depth가 없으면 실패로 세지 않는다.
    message, _, depth, _ = module.ScvDetectionManager._select_shutter_pair(node, 'current', now)
    assert message == 'current' and depth is None and list(node._shutter_pair_results) == [True]


def test_pair_selection_rejects_out_of_tolerance_depth():
    """가장 가까운 RGB도 허용 시간 밖이면 짝을 만들지 않고 실패로 기록한다."""
    now = time.monotonic()
    node = _selector([10.000, 10.100], [10.050], now)
    message, _, depth, _ = module.ScvDetectionManager._select_shutter_pair(node, 'current', now)
    assert message == 'current' and depth is None and list(node._shutter_pair_results) == [False]


def _compressed_depth(depth_mm, encoding='16UC1', header=(0, 0.0, 0.0)):
    """compressedDepth png 형식(12바이트 압축 정보 + 16비트 PNG)의 가짜 메시지를 만든다."""
    from sensor_msgs.msg import CompressedImage
    import cv2
    message = CompressedImage()
    message.format = f'{encoding}; compressedDepth png'
    ok, png = cv2.imencode('.png', depth_mm.astype(np.uint16))
    assert ok
    message.data = np.array([header], dtype='<i4,<f4,<f4').tobytes() + png.tobytes()
    return message


def test_compressed_depth_16uc1_matches_raw_millimeters():
    """16UC1 압축 Depth는 원본 밀리미터 값을 손실 없이 미터로 돌려주고 0은 무효로 남긴다."""
    depth_mm = np.array([[0, 1000, 1234], [5000, 65535, 200]], dtype=np.uint16)
    depth = module.decode_compressed_depth(_compressed_depth(depth_mm))
    assert depth.dtype == np.float64 and np.array_equal(depth, depth_mm * 0.001)


def test_compressed_depth_32fc1_restores_quantized_distance():
    """32FC1 압축 Depth는 역수 양자화 값을 상수로 다시 거리(m)로 되돌린다."""
    quant_a, quant_b = 100.0, 10.0
    distance = np.array([[1.0, 2.0], [4.0, 0.0]])
    encoded = np.where(distance > 0, quant_a / np.where(distance > 0, distance, 1) + quant_b, 0)
    depth = module.decode_compressed_depth(
        _compressed_depth(np.round(encoded), '32FC1', (0, quant_a, quant_b)))
    assert np.allclose(depth, distance)


def test_compressed_depth_rejects_unknown_format():
    """rvl 같은 다른 압축 방식이나 컬러 형식은 잘못 해석하지 않고 오류로 알린다."""
    import pytest
    message = _compressed_depth(np.ones((2, 2)))
    message.format = '16UC1; compressedDepth rvl'
    with pytest.raises(ValueError):
        module.decode_compressed_depth(message)
