"""좌표 변환·깊이 단위·시간 대응·마우스 클릭 로그를 검증한다."""

from unittest.mock import Mock
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header

from scv_detection import depth_checker
from scv_detection import scv_detection_manager as module


@pytest.fixture
def make_depth_node(monkeypatch):
    """화면을 끈 시험 노드를 만들고 출력 로그를 기록하며 ROS 자원을 정리한다."""
    rclpy.init(args=[])
    nodes = []

    def create(**overrides):
        """원하는 깊이 설정을 넣은 노드와 로그 기록 객체를 반환한다."""
        params = dict(show=False, rack_reference_path='')
        params.update(overrides)
        node = module.ScvDetectionManager(namespace='depth_test', parameter_overrides=[
            Parameter(name, value=value) for name, value in params.items()])
        logger = Mock()
        monkeypatch.setattr(node, 'get_logger', Mock(return_value=logger))
        nodes.append(node)
        return node, logger

    yield create
    for node in nodes:
        node.show = False
        node.destroy_node()
    rclpy.shutdown()


def feed_depth(node, value=1420, encoding='16UC1', shape=(480, 640), stamp=100.0,
               frame_id='rgb_optical'):
    """지정한 깊이값·크기·촬영 시각을 가진 Image 메시지를 만들어 콜백에 전달한다."""
    dtype = np.uint16 if encoding == '16UC1' else np.float32
    message = node.bridge.cv2_to_imgmsg(np.full(shape, value, dtype), encoding=encoding)
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp - int(stamp)) * 1_000_000_000)
    message.header.frame_id = frame_id
    node.depth_callback(message)
    return message


def show_rgb(node, monkeypatch, stamp=100.0, frame_id='rgb_optical', shape=(480, 640, 3)):
    """실제 창 대신 화면 호출을 기록하고 클릭 기준이 되는 RGB 메타데이터를 설정한다."""
    monkeypatch.setattr(module.cv2, 'imshow', Mock())
    header = Header(frame_id=frame_id)
    header.stamp.sec = int(stamp)
    header.stamp.nanosec = round((stamp - int(stamp)) * 1_000_000_000)
    node.show_preview_frame(np.zeros(shape, np.uint8), header, time.monotonic())


@pytest.mark.parametrize('shape,display,point,expected', [
    ((480, 640, 3), (960, 540), (480, 270), (320, 240)),
    ((720, 1280, 3), (960, 540), (480, 270), (640, 360)),
    ((480, 640, 3), (960, 540), (959, 539), (639, 479)),
    ((1, 1, 3), (960, 540), (959, 539), (0, 0)),
    ((480, 640, 3), (0, 540), (0, 0), None),
    ((0, 640, 3), (960, 540), (0, 0), None),
    ((480, 640, 3), (960, 540), (-1, 0), None),
    ((480, 640, 3), (960, 540), (960, 0), None),
    ((480, 640, 3), (960, 540), (0, 540), None),
])
def test_display_coordinates(shape, display, point, expected):
    """화면 축소·확대와 가장자리에서도 원본 픽셀을 안전하게 고르는지 확인한다."""
    assert depth_checker.display_to_image_point(*point, shape, display) == expected


@pytest.mark.parametrize('encoding,value,expected', [
    ('16UC1', 1420, 1.42), ('16UC1', 0, None), ('16UC1', 200, None), ('16UC1', 5000, None),
    ('32FC1', 1.42, 1.42), ('32FC1', 0, None), ('32FC1', -1, None),
    ('32FC1', np.nan, None), ('32FC1', np.inf, None), ('32FC1', -np.inf, None),
])
def test_depth_unit_and_validity(encoding, value, expected):
    """밀리미터·미터 단위를 구분하고 유효하지 않은 센서 값을 제외한다."""
    dtype = np.uint16 if encoding == '16UC1' else np.float32
    actual = depth_checker.sample_depth_meters(np.full((2, 2), value, dtype), (1, 1), encoding)
    assert actual is None if expected is None else actual == pytest.approx(expected)


def test_invalid_depth_coordinates_and_encoding():
    """범위 밖 좌표·다채널 사진·지원하지 않는 인코딩을 잘못 읽지 않는다."""
    assert depth_checker.sample_depth_meters(np.ones((2, 2), np.uint16), (2, 0), '16UC1') is None
    assert depth_checker.sample_depth_meters(np.ones((2, 2, 3), np.uint16), (0, 0), '16UC1') is None
    with pytest.raises(ValueError, match='encoding'):
        depth_checker.sample_depth_meters(np.ones((2, 2)), (0, 0), 'mono16')


def test_depth_subscription_and_disable(make_depth_node):
    """기본 깊이 주소·BEST_EFFORT 설정·영상과 별도 콜백 그룹·비활성화를 확인한다."""
    node, _ = make_depth_node()
    assert node.depth_subscription.topic_name == '/depth_test/oakd/stereo/image_raw'
    assert node.depth_subscription.qos_profile.reliability == module.ReliabilityPolicy.BEST_EFFORT
    assert node.depth_subscription.callback_group is not node.subscription.callback_group
    disabled, _ = make_depth_node(depth_enabled=False)
    assert not hasattr(disabled, 'depth_subscription')


@pytest.mark.parametrize('options', [dict(depth_timeout=0.0), dict(depth_max_time_diff=0.0),
                                     dict(depth_min_m=-1.0), dict(depth_max_m=0.1),
                                     dict(depth_timeout=float('nan')), dict(depth_topic='')])
def test_invalid_depth_settings(make_depth_node, options):
    """단위 범위와 시간 제한·깊이 주소가 잘못되면 실행을 거절한다."""
    with pytest.raises(ValueError, match='depth'):
        make_depth_node(**options)


@pytest.mark.parametrize('encoding,value', [('16UC1', 1420), ('32FC1', 1.42)])
def test_mouse_click_logs_meters(make_depth_node, monkeypatch, encoding, value):
    """표시 창 중앙 클릭이 원본 중앙 깊이 1.420미터를 한 번 로그로 출력한다."""
    node, logger = make_depth_node()
    feed_depth(node, value=value, encoding=encoding)
    show_rgb(node, monkeypatch)
    node.mouse_callback(cv2.EVENT_LBUTTONDOWN, 480, 270, 0, None)
    logger.info.assert_called_once()
    assert 'Depth RGB=(320, 240): 1.420 m' in logger.info.call_args.args[0]
    assert node._depth_click['point'] == (320, 240)
    logger.warning.assert_not_called()


def test_depth_frame_nearest_to_shown_rgb(make_depth_node, monkeypatch):
    """최신 깊이만 쓰지 않고 표시 RGB의 촬영 시각에 가장 가까운 프레임을 고른다."""
    node, logger = make_depth_node()
    feed_depth(node, value=2300, stamp=99.9)
    feed_depth(node, value=1200, stamp=100.02)
    feed_depth(node, value=3200, stamp=100.15)
    show_rgb(node, monkeypatch, stamp=100.0)
    node.mouse_callback(cv2.EVENT_LBUTTONDOWN, 480, 270, 0, None)
    assert '1.200 m' in logger.info.call_args.args[0]
    assert 'dt=0.020 s' in logger.info.call_args.args[0]


def test_zero_stamp_uses_receipt_times(make_depth_node, monkeypatch):
    """촬영 시각이 0인 입력은 같은 프로세스의 수신 시각 차이를 사용한다."""
    node, logger = make_depth_node()
    feed_depth(node, stamp=0.0)
    show_rgb(node, monkeypatch, stamp=0.0)
    node.mouse_callback(cv2.EVENT_LBUTTONDOWN, 480, 270, 0, None)
    assert '1.420 m' in logger.info.call_args.args[0]


@pytest.mark.parametrize('failure,warning', [
    ('no_rgb', 'no displayed RGB'), ('no_depth', 'no depth image'),
    ('rgb_stale', 'RGB frame is stale'), ('depth_stale', 'depth image is stale'),
    ('time', 'time difference'), ('shape', 'size mismatch'), ('frame_id', 'frame_id mismatch'),
    ('zero', 'INVALID'), ('outside', 'outside'),
])
def test_click_rejects_invalid_or_unaligned_data(make_depth_node, monkeypatch, failure, warning):
    """영상 누락·오래된 값·시간차·픽셀 미정렬·무효 깊이는 거리 대신 경고를 낸다."""
    node, logger = make_depth_node()
    if failure != 'no_depth':
        feed_depth(node, value=0 if failure == 'zero' else 1420,
                   stamp=101.0 if failure == 'time' else 100.0,
                   shape=(480, 320) if failure == 'shape' else (480, 640),
                   frame_id='stereo_optical' if failure == 'frame_id' else 'rgb_optical')
    if failure != 'no_rgb':
        show_rgb(node, monkeypatch)
    if failure == 'rgb_stale':
        node._shown_frame_time -= 3.0
    if failure == 'depth_stale':
        node._depth_frames[-1]['received'] -= 2.0
    node.mouse_callback(cv2.EVENT_LBUTTONDOWN, 960 if failure == 'outside' else 480, 270, 0, None)
    logger.info.assert_not_called()
    assert warning in logger.warning.call_args.args[0]


def test_other_mouse_events_and_disabled_depth(make_depth_node, monkeypatch):
    """마우스 이동·우클릭·기능 비활성화·노드 종료에서는 깊이를 출력하지 않는다."""
    node, logger = make_depth_node()
    feed_depth(node)
    show_rgb(node, monkeypatch)
    for event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_RBUTTONDOWN):
        node.mouse_callback(event, 480, 270, 0, None)
    node.depth_enabled = False
    node.mouse_callback(cv2.EVENT_LBUTTONDOWN, 480, 270, 0, None)
    node.depth_enabled = True
    node._stop_event.set()
    node.mouse_callback(cv2.EVENT_LBUTTONDOWN, 480, 270, 0, None)
    logger.info.assert_not_called()
    logger.warning.assert_not_called()


def test_depth_cache_is_bounded_and_owns_image(make_depth_node):
    """프레임 캐시가 30개를 넘지 않고 원본 메시지 변경에 영향을 받지 않는다."""
    node, _ = make_depth_node()
    for index in range(35):
        message = feed_depth(node, value=1000 + index, shape=(4, 4), stamp=100 + index)
    assert len(node._depth_frames) == 30
    message.data[0] = 0
    assert node._depth_frames[-1]['image'][0, 0] == 1034


def test_depth_conversion_errors_do_not_stop_camera(make_depth_node):
    """지원하지 않는 인코딩·비어 있는 사진·깨진 깊이 입력은 저장하지 않고 경고한다."""
    node, logger = make_depth_node()
    from sensor_msgs.msg import Image
    for message in (Image(encoding='mono16'), Image(encoding='16UC1'),
                    Image(encoding='16UC1', height=480, width=640, step=1280, data=b'bad')):
        node.depth_callback(message)
    assert not node._depth_frames
    assert logger.warning.call_count == 3


def test_gui_registers_mouse_callback(make_depth_node, monkeypatch):
    """실제 검출 창을 준비할 때 기존 창 이름에 왼쪽 클릭 콜백을 연결한다."""
    monkeypatch.setenv('DISPLAY', ':test')
    for name in ('namedWindow', 'resizeWindow', 'imshow', 'waitKey'):
        monkeypatch.setattr(module.cv2, name, Mock())
    callback = Mock()
    monkeypatch.setattr(module.cv2, 'setMouseCallback', callback)
    node, _ = make_depth_node(show=True)
    callback.assert_called_once_with(node.window_name, node.mouse_callback)
