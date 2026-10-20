"""Configuration, headless operation, error handling and status-color tests."""

from unittest.mock import Mock
from pathlib import Path
import json
import threading
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
import torch
from ultralytics.engine.results import Results

from scv_detection import scv_detection_manager as module
import ultralytics
from ultralytics.utils import plotting
from test_extinguisher_pressure import make_gauge_image


@pytest.fixture
def ros():
    """각 시험을 시작할 때 ROS 통신을 준비하고 끝나면 종료한다."""
    rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def make_node(ros, tmp_path, monkeypatch):
    """시험용 모델을 사용해 검출 노드를 만들고 시험이 끝나면 정리한다."""
    weights = tmp_path / 'test.pt'
    weights.touch()
    monkeypatch.setattr(ultralytics, 'YOLO', Mock())
    monkeypatch.setattr(torch.cuda, 'is_available', lambda: False)
    nodes = []

    def create(namespace='robot3', **overrides):
        """화면과 파일 저장을 끈 시험 노드에 원하는 설정을 덮어쓴다."""
        params = dict(weights=str(weights), max_fps=0.0, show=False, rack_reference_path='')
        params.update(overrides)
        node = module.ScvDetectionManager(namespace=namespace, parameter_overrides=[
            Parameter(name, value=value) for name, value in params.items()
        ])
        nodes.append(node)
        return node

    yield create
    for node in nodes:
        node.destroy_node()


def test_required_configuration(make_node):
    """모델 경로 생략 시 자동 선택하고, 잘못된 경로는 YOLO 요청만 실패시키는지 확인한다."""
    node = make_node(weights='')
    assert node.weights.name == 'best.pt'
    assert node.weights.is_file()
    assert node.model is None
    bad_node = make_node(weights='/no-such-model.pt')
    bad_node.latest_frame = np.zeros((480, 640, 3), np.uint8)
    bad_node.last_frame_time = time.monotonic()
    response = bad_node._handle_inspect_yolo(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert 'Weights file not found' in response.message
    assert bad_node._inspection_mode is None
    assert not bad_node._inspection_lock.locked()


@pytest.mark.parametrize('namespace', ['robot3', '/robot3', 'inspector'])
def test_topics_and_qos(make_node, namespace):
    """로봇별 영상·불량 주소와 RELIABLE 통신 설정, 불량 큐 크기를 확인한다."""
    node = make_node(namespace=namespace)
    prefix = '/' + namespace.strip('/')
    assert node.get_name() == 'detection_alert'
    assert node.topic == 'oakd/rgb/image_raw/compressed'
    assert node.output_topic == 'fire_extinguisher/detections/compressed'
    assert node.subscription.topic_name == prefix + '/' + node.topic
    assert node.publisher.topic_name == prefix + '/' + node.output_topic
    assert node.subscription.qos_profile.depth == 1
    assert node.subscription.qos_profile.reliability == module.ReliabilityPolicy.RELIABLE
    assert node.result_topic == 'inspect/result'
    assert node.facility_result_publisher.topic_name == prefix + '/inspect/result'
    assert node.result_publisher.topic_name == prefix + '/inspect/defects'
    assert node.result_publisher.qos_profile.depth == 10
    assert node.result_publisher.qos_profile.reliability == module.ReliabilityPolicy.RELIABLE
    assert node.result_publisher.qos_profile.durability.name == 'VOLATILE'


def test_headless_override(make_node, monkeypatch):
    """show=false를 명시하면 영상 창을 만들지 않는지 확인한다."""
    window = Mock(side_effect=AssertionError('GUI must not be used'))
    monkeypatch.setattr(module.cv2, 'namedWindow', window)
    node = make_node()
    assert node.show is False
    window.assert_not_called()


def test_show_without_desktop(make_node, monkeypatch):
    """화면 표시를 요청해도 데스크톱이 없으면 창 기능만 꺼지는지 확인한다."""
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)
    window = Mock(side_effect=AssertionError('GUI must not be used'))
    monkeypatch.setattr(module.cv2, 'namedWindow', window)
    assert make_node(show=True).show is False
    window.assert_not_called()


def test_window_failure_falls_back(make_node, monkeypatch):
    """창 생성에 실패해도 검출 노드는 계속 사용할 수 있는지 확인한다."""
    monkeypatch.setenv('DISPLAY', ':test')
    monkeypatch.setattr(module.cv2, 'namedWindow', Mock(side_effect=cv2.error('no GUI')))
    assert make_node(show=True).show is False


def test_decode_and_inference_errors(make_node):
    """카메라 대기에서는 추론하지 않고, YOLO 오류가 나도 원본 영상을 계속 발행하는지 확인한다."""
    node = make_node()
    node.publisher = Mock()
    for payload in (b'', b'not a JPEG'):
        node.image_callback(CompressedImage(data=payload))
    assert node.model is None
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    _, encoded = cv2.imencode('.jpg', frame)
    message = CompressedImage(format='jpeg', data=encoded.tobytes())
    node.image_callback(message)
    assert node.model is None
    node._load_yolo_model()
    node._inspection_mode = 'yolo'
    node.model.predict.side_effect = RuntimeError('inference error')
    node.image_callback(message)
    assert 'inference error' in node._inspection_result['message']
    assert node.publisher.publish.call_count == 2
    node.model.predict.side_effect = None
    node.model.predict.return_value = [Results(frame, path='test.jpg',
                                             names={0: 'Normal'}, boxes=torch.empty((0, 6)))]
    node.image_callback(message)
    output = node.publisher.publish.call_args.args[0]
    assert output.format == 'jpeg'
    assert cv2.imdecode(np.frombuffer(output.data, dtype=np.uint8), cv2.IMREAD_COLOR).shape == frame.shape


@pytest.mark.parametrize('class_id,name,color', [
    (0, 'Abnormal_left', (0, 0, 255)), (1, 'Abnormal_right', (0, 0, 255)),
    (2, 'Fire Extinguisher', (192, 192, 192)), (3, 'Normal', (0, 255, 0)),
    (4, 'Exit_sign_on', (0, 255, 0)), (5, 'Exit_sign_off', (0, 0, 255)),
    (6, 'other', (192, 192, 192)),
])
def test_detection_colors(class_id, name, color):
    """현재 모델 ID별 박스 색상을 확인하고 원본 사진이 보존되는지 확인한다."""
    frame = np.zeros((180, 280, 3), dtype=np.uint8)
    cv2.circle(frame, (130, 100), 30, (255, 255, 255), 3)
    original = frame.copy()
    result = Results(frame, path='test.jpg', names={class_id: name},
                     boxes=torch.tensor([[60, 60, 200, 145, 0.8, class_id]]))
    annotated, _ = module.ScvDetectionManager.annotate_yolo_result(result)
    assert tuple(annotated[145, 130]) == color
    assert np.array_equal(frame, original)


def test_all_six_classes_keep_labels(monkeypatch):
    """실제 모델 순서대로 여섯 클래스의 이름·신뢰도·색상과 로그가 빠짐없이 나오는지 확인한다."""
    names = dict(enumerate(['Abnormal_left', 'Abnormal_right', 'Fire Extinguisher',
                            'Normal', 'Exit_sign_on', 'Exit_sign_off']))
    frame = np.zeros((480, 640, 3), np.uint8)
    original = frame.copy()
    boxes = torch.tensor([[20 + i * 50, 50, 60 + i * 50, 150, 0.9, i] for i in names])
    result = Results(frame, path='six.jpg', names=names, boxes=boxes)
    annotator = Mock()
    annotator.result.return_value = frame.copy()
    monkeypatch.setattr(plotting, 'Annotator', Mock(return_value=annotator))
    _, logs = module.ScvDetectionManager.annotate_yolo_result(result)
    labels = {call.args[1]: call.kwargs['color'] for call in annotator.box_label.call_args_list}
    assert labels == {
        'Fire Extinguisher 0.90': (192, 192, 192),
        'Normal 0.90': (0, 255, 0),
        'Abnormal_left 0.90': (0, 0, 255),
        'Abnormal_right 0.90': (0, 0, 255),
        'Exit_sign_on 0.90': (0, 255, 0),
        'Exit_sign_off 0.90': (0, 0, 255),
    }
    assert logs == [f'{names[i]}:0.90 verdict={state}' for i, state in enumerate(
        ['ABNORMAL', 'ABNORMAL', 'UNCONFIRMED', 'NORMAL', 'NORMAL', 'ABNORMAL'])]
    assert np.array_equal(frame, original)


def test_future_class_is_visible(monkeypatch):
    """색상표에 없는 새 ID도 회색 박스와 실제 이름으로 표시되는지 확인한다."""
    result = Results(np.zeros((100, 100, 3), np.uint8), path='new.jpg',
                     names={6: 'New_object'}, boxes=torch.tensor([[10, 10, 80, 80, 0.8, 6]]))
    annotator = Mock()
    monkeypatch.setattr(plotting, 'Annotator', Mock(return_value=annotator))
    module.ScvDetectionManager.annotate_yolo_result(result)
    assert annotator.box_label.call_args.args[1] == 'New_object 0.80'
    assert annotator.box_label.call_args.kwargs['color'] == (192, 192, 192)


def test_rack_only_without_weights(make_node):
    """랙 서비스만 준비하거나 카메라만 사용해도 모델 로딩 없이 원본 영상을 발행한다."""
    node = make_node(enable_yolo=False, rack_enabled=True, weights='')
    assert node.model is None
    node.publisher = Mock()
    _, jpeg = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
    node.image_callback(CompressedImage(format='jpeg', data=jpeg.tobytes()))
    node.publisher.publish.assert_called_once()


def test_rack_never_runs_yolo(make_node, monkeypatch):
    """랙 모드에서는 준비된 YOLO 모델도 호출하지 않고 랙 결과만 발행하는지 확인한다."""
    node = make_node(rack_enabled=True)
    node._load_yolo_model()
    node._inspection_mode = 'rack'
    node.publisher = Mock()
    node.model.predict.side_effect = RuntimeError('test model failure')
    inspect = Mock(return_value={'verdict': module.InspectionResult.UNCONFIRMED,
                                'rack': None, 'zones': [], 'protrusions': [], 'reason': 'test'})
    monkeypatch.setattr(module.rack_conditioner, 'inspect_rack', inspect)
    _, jpeg = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
    node.image_callback(CompressedImage(format='jpeg', data=jpeg.tobytes()))
    inspect.assert_called_once()
    node.model.predict.assert_not_called()
    node.publisher.publish.assert_called_once()


@pytest.mark.parametrize('settings', [dict(rack_confirm_frames=0), dict(rack_roi_x1=0.9),
                                    dict(camera_timeout=0.0)])
def test_invalid_rack_settings(make_node, settings):
    """연속 확인 횟수·영상 유효 시간·ROI가 잘못되면 실행을 막는지 확인한다."""
    with pytest.raises(ValueError):
        make_node(**settings)


def camera_message():
    """시험용 검은 카메라 영상을 JPEG 메시지로 만들어 반환한다."""
    _, encoded = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
    return CompressedImage(format='jpeg', data=encoded.tobytes())


def test_bare_node_defaults(ros, monkeypatch):
    """추가 인자 없이 robot4·화면 켜짐·두 서비스 준비 상태로 시작하고 모델을 로드하지 않는다."""
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)
    model_factory = Mock()
    monkeypatch.setattr(ultralytics, 'YOLO', model_factory)
    node = module.ScvDetectionManager()
    try:
        assert node.get_namespace() == '/robot4'
        assert node.subscription.topic_name == '/robot4/oakd/rgb/image_raw/compressed'
        assert node.get_parameter('show').value is True
        assert node.resolve_service_name(node.yolo_service.srv_name) == '/robot4/detection_alert/inspect_yolo'
        assert node.resolve_service_name(node.rack_service.srv_name) == '/robot4/detection_alert/inspect_rack'
        assert node.weights.is_file()
        assert node._inspection_mode is None
        model_factory.assert_not_called()
    finally:
        node.destroy_node()


def test_idle_camera_has_no_detection_or_roi(make_node, monkeypatch):
    """서비스 전에는 YOLO·랙 검사·ROI 표시 없이 원본 카메라만 발행하는지 확인한다."""
    node = make_node()
    node.publisher = Mock()
    inspect = Mock(side_effect=AssertionError('Rack must remain idle'))
    monkeypatch.setattr(module.rack_conditioner, 'inspect_rack', inspect)
    message = camera_message()
    node.image_callback(message)
    inspect.assert_not_called()
    assert node.model is None
    output = node.publisher.publish.call_args.args[0]
    image = cv2.imdecode(np.frombuffer(output.data, np.uint8), cv2.IMREAD_COLOR)
    assert not image.any()


@pytest.mark.parametrize('mode', ['yolo', 'rack'])
def test_services_toggle_continuously_and_return_to_camera(make_node, monkeypatch, mode):
    """5초가 지나도 요청한 검사만 계속하고, 같은 서비스 재호출로 중지되는지 확인한다."""
    node = make_node()
    node.publisher = Mock()
    message = camera_message()
    node.image_callback(message)
    node.rack_reference = {'frame': np.zeros((480, 640, 3), np.uint8),
                           'rack': {'left': 80, 'right': 180, 'top': 20, 'bottom': 150}}
    report = {'verdict': module.InspectionResult.NORMAL, 'rack': None,
              'zones': [], 'protrusions': [], 'reason': 'test normal'}
    inspect = Mock(return_value=report)
    monkeypatch.setattr(module.rack_conditioner, 'inspect_rack', inspect)
    ultralytics.YOLO.return_value.predict.return_value = [Results(
        np.zeros((480, 640, 3), np.uint8), path='test.jpg',
        names={3: 'Normal'}, boxes=torch.tensor([[30, 60, 130, 160, 0.9, 3]]))]
    clock = [time.monotonic()]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    handler = node._handle_inspect_yolo if mode == 'yolo' else node._handle_inspect_rack
    for _ in range(2):
        started = handler(Trigger.Request(), Trigger.Response())
        assert started.success and 'started' in started.message
        assert node._inspection_mode == mode
        assert not node._inspection_lock.locked()
        busy = node._handle_inspection('rack' if mode == 'yolo' else 'yolo', Trigger.Response())
        assert not busy.success and 'busy' in busy.message
        assert not node.capture_rack_reference()
        assert not node.reset_rack_reference()
        for _ in range(3):
            node.image_callback(message)
        clock[0] += 6.0
        node.image_callback(message)
        assert node._inspection_mode == mode
        stopped = handler(Trigger.Request(), Trigger.Response())
        assert stopped.success and 'stopped' in stopped.message
        assert node._inspection_mode is None
        node.image_callback(message)
        output = node.publisher.publish.call_args.args[0]
        assert not cv2.imdecode(np.frombuffer(output.data, np.uint8), cv2.IMREAD_COLOR).any()
    if mode == 'yolo':
        inspect.assert_not_called()
        assert node.model.predict.call_count == 8
        ultralytics.YOLO.assert_called_once()
    else:
        assert inspect.call_count == 8
        assert node.model is None


@pytest.mark.parametrize('mode', ['yolo', 'rack'])
def test_stop_works_after_camera_disconnection(make_node, mode):
    """영상이나 랙 기준이 없어져도 같은 서비스로 실행 중인 검사를 중지할 수 있다."""
    node = make_node()
    node._inspection_mode = mode
    node.last_frame_time = None
    node.rack_reference = None
    response = node._handle_inspection(mode, Trigger.Response())
    assert response.success and 'stopped' in response.message
    assert node._inspection_mode is None


def test_concurrent_start_is_rejected_during_model_loading(make_node, monkeypatch):
    """모델 로딩 중 겹친 시작 요청은 기다리지 않고 거절하여 모드가 뒤섞이지 않게 한다."""
    node = make_node()
    node.image_callback(camera_message())
    entered = threading.Event()
    release = threading.Event()
    responses = []

    def slow_load():
        """시험에서만 모델 준비를 잠시 멈춰 겹치는 서비스 요청을 보낼 수 있게 한다."""
        entered.set()
        assert release.wait(timeout=2)

    def start_service():
        """별도 스레드에서 YOLO 시작 요청을 처리한다."""
        responses.append(node._handle_inspect_yolo(Trigger.Request(), Trigger.Response()))

    monkeypatch.setattr(node, '_load_yolo_model', slow_load)
    thread = threading.Thread(target=start_service)
    thread.start()
    try:
        assert entered.wait(timeout=1)
        busy = node._handle_inspect_yolo(Trigger.Request(), Trigger.Response())
        assert not busy.success and 'busy' in busy.message
        release.set()
        thread.join(timeout=2)
        assert not thread.is_alive()
        assert responses[0].success
        assert node._inspection_mode == 'yolo'
    finally:
        release.set()
        thread.join(timeout=2)


def test_requests_without_camera_or_reference_fail(make_node):
    """영상·정상 기준이 없으면 즉시 실패하고 대기 모드를 유지하는지 확인한다."""
    node = make_node()
    response = node._handle_inspect_yolo(Trigger.Request(), Trigger.Response())
    assert not response.success and 'camera frame' in response.message
    node.image_callback(camera_message())
    response = node._handle_inspect_rack(Trigger.Request(), Trigger.Response())
    assert not response.success and 'reference' in response.message
    assert node._inspection_mode is None
    node.last_frame_time = time.monotonic() - 5
    response = node._handle_inspect_yolo(Trigger.Request(), Trigger.Response())
    assert not response.success and 'camera frame' in response.message


def test_camera_only_with_services_disabled(make_node):
    """두 검사 기능을 끄더라도 카메라 전용 실행을 허용하는지 확인한다."""
    node = make_node(enable_yolo=False, rack_enabled=False)
    node.publisher = Mock()
    node.image_callback(camera_message())
    node.publisher.publish.assert_called_once()
    response = node._handle_inspect_yolo(Trigger.Request(), Trigger.Response())
    assert not response.success and 'disabled' in response.message


def test_unconfirmed_frame_clears_old_rack_result(make_node, monkeypatch):
    """최근 랙 프레임이 미확인이면 앞선 정상 결과를 그대로 응답하지 않도록 지우는지 확인한다."""
    node = make_node()
    node._inspection_mode = 'rack'
    node._inspection_result = {'success': True, 'message': 'NORMAL', 'time': time.monotonic()}
    inspect = Mock(return_value={'verdict': module.InspectionResult.UNCONFIRMED,
                                'rack': None, 'zones': [], 'protrusions': [], 'reason': 'no rack'})
    monkeypatch.setattr(module.rack_conditioner, 'inspect_rack', inspect)
    node.image_callback(camera_message())
    assert node._inspection_result is None


def test_late_yolo_frame_does_not_reactivate_finished_request(make_node):
    """중지 전에 시작한 불량 검출을 화면이나 불량 토픽으로 내보내지 않는지 확인한다."""
    node = make_node()
    node.publisher = Mock()
    node.result_publisher = Mock()
    node._load_yolo_model()
    node._inspection_mode = 'yolo'

    def finish_during_predict(**kwargs):
        """추론 중 서비스가 끝나는 상황을 만들고 표시할 검출 결과를 반환한다."""
        node._inspection_mode = None
        node._inspection_id += 1
        return [Results(kwargs['source'], path='late.jpg', names={0: 'Abnormal_left'},
                        boxes=torch.tensor([[30, 60, 130, 160, 0.9, 0]]))]

    node.model.predict.side_effect = finish_during_predict
    node.image_callback(camera_message())
    assert node._inspection_result is None
    node.result_publisher.publish.assert_not_called()
    output = node.publisher.publish.call_args.args[0]
    assert not cv2.imdecode(np.frombuffer(output.data, np.uint8), cv2.IMREAD_COLOR).any()


def prepare_yolo_defect_node(make_node, class_ids):
    """원하는 클래스 목록을 예측하는 시험 노드를 만들고 불량 발행을 기록하게 한다."""
    node = make_node()
    node.publisher = Mock()
    node.result_publisher = Mock()
    node._load_yolo_model()
    node._inspection_mode = 'yolo'
    names = dict(enumerate(['Abnormal_left', 'Abnormal_right', 'Fire Extinguisher',
                            'Normal', 'Exit_sign_on', 'Exit_sign_off', 'other']))
    boxes = torch.tensor([[30, 60, 130, 160, 0.9, class_id] for class_id in class_ids]).reshape(-1, 6)
    node.model.predict.return_value = [Results(np.zeros((480, 640, 3), np.uint8),
                                             path='defects.jpg', names=names, boxes=boxes)]
    return node


def prepare_pressure_node(make_node, angle=90, gauge_class=3, *, crop_only=False,
                          green=True, yellow=True, exit_defect=False, **options):
    """소화기와 합성 압력계를 예측하는 노드·JPEG·원본 결과를 준비해 압력 연동을 시험한다."""
    node = make_node(**options)
    node.publisher = Mock()
    node.result_publisher = Mock()
    node._load_yolo_model()
    node._inspection_mode = 'yolo'
    gauge, _ = make_gauge_image(angle, green=green, yellow=yellow)
    frame = np.zeros((480, 640, 3), np.uint8)
    frame[80:260, 100:280] = gauge
    names = dict(enumerate(['Abnormal_left', 'Abnormal_right', 'Fire Extinguisher',
                            'Normal', 'Exit_sign_on', 'Exit_sign_off']))
    boxes = [[70, 40, 320, 450, 0.9, 2]]
    if not crop_only:
        boxes.append([105, 85, 275, 255, 0.8, gauge_class])
    if exit_defect:
        boxes.append([400, 40, 460, 100, 0.85, 5])
    full = Results(frame, path='pressure.jpg', names=names, boxes=torch.tensor(boxes))
    if crop_only:
        crop = frame[40:450, 70:320].copy()
        local = Results(crop, path='crop.jpg', names=names,
                        boxes=torch.tensor([[35, 45, 205, 215, 0.8, gauge_class]]))
        node.model.predict.side_effect = [[full], [local]]
    else:
        node.model.predict.return_value = [full]
    _, jpeg = cv2.imencode('.jpg', frame)
    message = CompressedImage(format='jpeg', data=jpeg.tobytes())
    message.header.stamp.sec = 123
    return node, message, full


@pytest.mark.parametrize('angle,gauge_class,status', [
    (90, 0, 'NORMAL'), (20, 3, 'ABNORMAL'), (160, 3, 'ABNORMAL'), (55, 0, 'UNCONFIRMED'),
])
def test_needle_verdict_overrides_nested_yolo_class(make_node, angle, gauge_class, status):
    """소화기 안에서는 모델의 정상·비정상 이름보다 바늘 각도 판정을 우선하고 불량만 발행한다."""
    node, message, _ = prepare_pressure_node(make_node, angle, gauge_class)
    node.image_callback(message)
    report = node._inspection_result['pressures'][0]
    assert report['status'] == status
    assert f'verdict={status}' in node._inspection_result['message']
    assert 'needle=' in node._inspection_result['message']
    assert node.model.predict.call_count == 1
    if status == 'ABNORMAL':
        node.result_publisher.publish.assert_called_once()
        event = node.result_publisher.publish.call_args.args[0]
        detail = json.loads(event.detail)
        assert event.kind == module.InspectionResult.KIND_EXTINGUISHER
        assert event.verdict == module.InspectionResult.ABNORMAL and event.stamp == message.header.stamp
        assert detail['source'] == 'yellow_needle_angle'
        assert detail['bbox'] == [70, 40, 320, 450]
        assert detail['gauge_bbox'] == [105, 85, 275, 255]
        assert detail['needle_angle_deg'] == pytest.approx(angle, abs=3)
        assert detail['range_source'] == 'green_pixels'
    else:
        node.result_publisher.publish.assert_not_called()


def test_second_inference_uses_extinguisher_crop_and_global_coordinates(make_node):
    """전체 검출에 압력계가 없으면 소화기 영역만 같은 모델에 넣고 좌표를 원본으로 되돌린다."""
    node, message, _ = prepare_pressure_node(make_node, 20, crop_only=True)
    node.image_callback(message)
    assert node.model.predict.call_count == 2
    assert node.model.predict.call_args_list[1].kwargs['source'].shape == (410, 250, 3)
    report = node._inspection_result['pressures'][0]
    assert report['status'] == 'ABNORMAL'
    assert report['gauge_bbox'] == (105, 85, 275, 255)
    assert report['dial_center'] == pytest.approx((190, 170), abs=3)
    node.result_publisher.publish.assert_called_once()


@pytest.mark.parametrize('green,yellow', [(False, True), (True, False)])
def test_unreadable_pressure_suppresses_nested_model_defect(make_node, green, yellow):
    """바늘·녹색을 읽지 못하면 압력계 비정상 클래스가 있어도 소화기 불량으로 발행하지 않는다."""
    node, message, _ = prepare_pressure_node(make_node, gauge_class=0, green=green, yellow=yellow)
    node.image_callback(message)
    assert node._inspection_result['pressures'][0]['status'] == 'UNCONFIRMED'
    node.result_publisher.publish.assert_not_called()


def test_no_gauge_detection_keeps_extinguisher_unconfirmed(make_node):
    """소화기를 찾았어도 재검출에서 압력계를 못 찾으면 정상으로 추정하지 않는다."""
    node, message, full = prepare_pressure_node(make_node, crop_only=True)
    crop = full.orig_img[40:450, 70:320]
    empty = Results(crop, path='empty.jpg', names=full.names, boxes=torch.empty((0, 6)))
    node.model.predict.side_effect = [[full], [empty]]
    node.image_callback(message)
    report = node._inspection_result['pressures'][0]
    assert report['status'] == 'UNCONFIRMED'
    assert report['reason'] == 'pressure gauge not detected'
    node.result_publisher.publish.assert_not_called()


def test_pressure_crop_failure_keeps_exit_defect(make_node):
    """소화기 재추론이 실패해도 전체 영상에서 이미 검출한 유도등 불량은 발행한다."""
    node, message, full = prepare_pressure_node(make_node, crop_only=True, exit_defect=True)
    node.model.predict.side_effect = [[full], RuntimeError('crop failed')]
    node.image_callback(message)
    assert node._inspection_result['success']
    report = node._inspection_result['pressures'][0]
    assert report['status'] == 'UNCONFIRMED' and 'inference failed' in report['reason']
    node.result_publisher.publish.assert_called_once()
    assert node.result_publisher.publish.call_args.args[0].kind == module.InspectionResult.KIND_EXIT_LIGHT


def test_pressure_can_be_disabled(make_node):
    """압력 기능을 끄면 추가 추론 없이 이전 YOLO 표시·불량 발행 동작을 유지한다."""
    node, message, _ = prepare_pressure_node(make_node, gauge_class=0, pressure_enabled=False)
    node.image_callback(message)
    assert node._inspection_result['pressures'] == []
    assert node.model.predict.call_count == 1
    node.result_publisher.publish.assert_called_once()
    assert json.loads(node.result_publisher.publish.call_args.args[0].detail)['class_id'] == 0


def test_overlapping_bodies_do_not_share_pressure_verdict(make_node):
    """압력계가 겹친 두 소화기에 모두 속하면 소속이 모호하다는 미확인으로 보류한다."""
    node, message, full = prepare_pressure_node(make_node, gauge_class=0)
    boxes = torch.cat((full.boxes.data, torch.tensor([[65, 35, 325, 455, 0.85, 2]])))
    node.model.predict.return_value = [Results(full.orig_img, path='overlap.jpg', names=full.names, boxes=boxes)]
    node.image_callback(message)
    reports = node._inspection_result['pressures']
    assert len(reports) == 2
    assert all(report['status'] == 'UNCONFIRMED' and 'multiple extinguishers' in report['reason']
               for report in reports)
    assert node.model.predict.call_count == 1
    node.result_publisher.publish.assert_not_called()


def test_pressure_measurement_finishing_after_stop_is_discarded(make_node, monkeypatch):
    """압력 판독 도중 중지한 요청의 불량을 나중에 발행하지 않는다."""
    node, message, _ = prepare_pressure_node(make_node, 20)
    original = module.extinguisher_pressure.inspect_pressure

    def stop_during_reading(*args, **kwargs):
        """판독을 마친 직후 서비스를 중지하여 늦은 결과 상황을 만든다."""
        report = original(*args, **kwargs)
        response = node._handle_inspect_yolo(Trigger.Request(), Trigger.Response())
        assert response.success
        return report

    monkeypatch.setattr(module.extinguisher_pressure, 'inspect_pressure', stop_during_reading)
    node.image_callback(message)
    assert node._inspection_result is None
    node.result_publisher.publish.assert_not_called()


@pytest.mark.parametrize('options', [
    {'pressure_min_gauge_diameter': 0}, {'pressure_boundary_margin_deg': -1.0},
    {'pressure_green_upper': [180, 255, 255]}, {'pressure_normal_start_deg': 60.0},
])
def test_invalid_pressure_settings(make_node, options):
    """ROS에서 잘못된 압력 판독 설정을 받으면 생성 단계에서 거절한다."""
    with pytest.raises(ValueError):
        make_node(**options)


def test_object_model_does_not_treat_box_or_exit_as_pressure_defects(make_node):
    """5개 객체 클래스 모델의 박스·비상구·압력판·랙을 이전 ID의 상태와 혼동하지 않는다."""
    node = prepare_yolo_defect_node(make_node, [])
    names = {0: 'box', 1: 'exit_sign', 2: 'fire_extinguisher', 3: 'pressure_gauge', 4: 'rack'}
    frame = np.zeros((480, 640, 3), np.uint8)
    boxes = torch.tensor([[20+i*100, 60, 80+i*100, 160, 0.9, i] for i in (0, 1, 3, 4)])
    result = Results(frame, path='objects.jpg', names=names, boxes=boxes)
    node.model.predict.return_value = [result]
    node.image_callback(camera_message())
    node.result_publisher.publish.assert_not_called()
    assert node._inspection_result['pressures'] == []
    assert node.model.predict.call_count == 1
    _, logs = node.annotate_yolo_result(result)
    assert all('verdict=UNCONFIRMED' in log for log in logs)


def test_pressure_roles_and_json_follow_model_class_names(make_node):
    """객체 모델의 클래스 ID가 바뀌어도 이름으로 소화기·압력계를 연결하고 실제 ID를 발행한다."""
    node, message, full = prepare_pressure_node(make_node, 20)
    names = {0: 'box', 1: 'pressure_gauge', 7: 'fire_extinguisher'}
    boxes = torch.tensor([[70, 40, 320, 450, 0.9, 7], [105, 85, 275, 255, 0.8, 1],
                          [400, 40, 460, 100, 0.9, 0]])
    node.model.predict.return_value = [Results(full.orig_img, path='names.jpg', names=names, boxes=boxes)]
    node.image_callback(message)
    assert node.model.predict.call_count == 1
    assert node._inspection_result['pressures'][0]['status'] == 'ABNORMAL'
    node.result_publisher.publish.assert_called_once()
    detail = json.loads(node.result_publisher.publish.call_args.args[0].detail)
    assert detail['class_id'] == 7 and detail['class_name'] == 'fire_extinguisher'


@pytest.mark.parametrize('jpeg_quality', [70, 85, 95])
def test_user_screen_pressure_through_jpeg_callback(make_node, jpeg_quality):
    """첨부 압력판을 실제 JPEG 수신 경로로 읽어 바늘 불량을 각도·범위와 함께 발행한다."""
    node, _, full = prepare_pressure_node(make_node)
    path = Path(__file__).parent / 'data' / 'pressure_screen_20261013.png'
    gauge = cv2.imread(str(path))
    assert gauge is not None
    frame = np.zeros((480, 640, 3), np.uint8)
    height, width = gauge.shape[:2]
    frame[100:100+height, 140:140+width] = gauge
    boxes = torch.tensor([[70, 40, 320, 450, 0.9, 2],
                          [140, 100, 140+width, 100+height, 0.9, 3]])

    def predict_received_frame(**kwargs):
        """사진 압축 때문에 색이 조금 바뀐 실제 수신 사진에 시험 검출 박스를 붙인다."""
        return [Results(kwargs['source'], path='user-screen.jpg', names=full.names, boxes=boxes)]

    node.model.predict.side_effect = predict_received_frame
    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    node.image_callback(CompressedImage(format='jpeg', data=jpeg.tobytes()))
    report = node._inspection_result['pressures'][0]
    assert report['status'] == 'ABNORMAL', report
    node.result_publisher.publish.assert_called_once()
    detail = json.loads(node.result_publisher.publish.call_args.args[0].detail)
    assert detail['source'] == 'yellow_needle_angle'
    assert detail['needle_angle_deg'] == pytest.approx(130, abs=8)
    assert detail['range_source'] == 'green_pixels'


@pytest.mark.parametrize('class_id,kind', [
    (0, module.InspectionResult.KIND_EXTINGUISHER),
    (1, module.InspectionResult.KIND_EXTINGUISHER),
    (2, None), (3, None), (4, None),
    (5, module.InspectionResult.KIND_EXIT_LIGHT), (6, None),
])
def test_yolo_defect_is_published_on_first_frame(make_node, class_id, kind):
    """첫 프레임에서 압력계 비정상·유도등 소등만 종류·시각·설명과 함께 발행한다."""
    node = prepare_yolo_defect_node(make_node, [class_id])
    message = camera_message()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    node.image_callback(message)
    if kind is None:
        node.result_publisher.publish.assert_not_called()
        return
    node.result_publisher.publish.assert_called_once()
    event = node.result_publisher.publish.call_args.args[0]
    assert isinstance(event, module.InspectionResult)
    assert event.robot_id == 'robot3'
    assert event.kind == kind and event.verdict == module.InspectionResult.ABNORMAL
    assert event.stamp == message.header.stamp
    assert event.run_id == 0 and event.task_id == '' and event.facility_id == ''
    assert event.image_ref == '' and not event.image.data and event.attempts == 1
    detail = json.loads(event.detail)
    assert detail['mode'] == 'yolo' and detail['class_id'] == class_id
    assert detail['class_name'] == node.model.predict.return_value[0].names[class_id]
    assert detail['confidence'] == pytest.approx(0.9)
    assert detail['bbox'] == [30.0, 60.0, 130.0, 160.0]


def test_mixed_yolo_results_publish_every_defect(make_node):
    """정상·전체 소화기와 함께 검출되어도 불량 물체를 각각 빠짐없이 발행한다."""
    node = prepare_yolo_defect_node(make_node, [0, 1, 2, 3, 4, 5, 0])
    node.image_callback(camera_message())
    events = [call.args[0] for call in node.result_publisher.publish.call_args_list]
    assert [json.loads(event.detail)['class_id'] for event in events] == [0, 1, 5, 0]
    assert all(event.verdict == module.InspectionResult.ABNORMAL for event in events)


def test_defect_repeats_per_frame_and_stops_with_service(make_node):
    """불량이 보이는 각 프레임에 발행하고 같은 서비스로 중지하면 더 이상 보내지 않는다."""
    node = prepare_yolo_defect_node(make_node, [5])
    message = camera_message()
    for _ in range(2):
        node.image_callback(message)
    assert node.result_publisher.publish.call_count == 2
    event = node.result_publisher.publish.call_args.args[0]
    assert event.stamp.sec or event.stamp.nanosec
    response = node._handle_inspect_yolo(Trigger.Request(), Trigger.Response())
    assert response.success and 'stopped' in response.message
    node.image_callback(message)
    assert node.result_publisher.publish.call_count == 2


@pytest.mark.parametrize('boxes', [None, torch.empty((0, 6))])
def test_empty_yolo_result_does_not_publish_defect(make_node, boxes):
    """박스 정보가 없거나 검출이 0건이면 불량이나 정상 이벤트를 만들지 않는다."""
    node = prepare_yolo_defect_node(make_node, [])
    result = node.model.predict.return_value[0]
    node.model.predict.return_value = [Results(result.orig_img, path='empty.jpg', names=result.names, boxes=boxes)]
    node.image_callback(camera_message())
    node.result_publisher.publish.assert_not_called()


def test_idle_and_inference_error_do_not_publish_defect(make_node):
    """카메라 대기나 추론 오류를 실제 설비 불량으로 잘못 발행하지 않는다."""
    node = prepare_yolo_defect_node(make_node, [0])
    node._inspection_mode = None
    node.image_callback(camera_message())
    node._inspection_mode = 'yolo'
    node.model.predict.side_effect = RuntimeError('test inference failure')
    node.image_callback(camera_message())
    node.result_publisher.publish.assert_not_called()


@pytest.mark.parametrize('confirm_frames', [1, 3])
def test_rack_defect_publishes_after_confirmation(make_node, monkeypatch, confirm_frames):
    """랙 돌출 확정 프레임부터 발행하고 ROI 좌표를 전체 사진 기준으로 변환한다."""
    node = make_node(rack_confirm_frames=confirm_frames)
    node._inspection_mode = 'rack'
    node.publisher = Mock()
    node.result_publisher = Mock()
    report = {'verdict': module.InspectionResult.ABNORMAL, 'rack': None, 'zones': [],
              'reason': 'object protruding outside rack boundary',
              'protrusions': [{'side': 'right', 'bbox': (20, 30, 40, 50), 'area': 1600}]}
    monkeypatch.setattr(module.rack_conditioner, 'inspect_rack', Mock(return_value=report))
    message = camera_message()
    for _ in range(confirm_frames - 1):
        node.image_callback(message)
        node.result_publisher.publish.assert_not_called()
    node.image_callback(message)
    node.result_publisher.publish.assert_called_once()
    event = node.result_publisher.publish.call_args.args[0]
    assert event.kind == module.InspectionResult.KIND_RACK
    assert event.verdict == module.InspectionResult.ABNORMAL
    detail = json.loads(event.detail)
    assert detail['mode'] == 'rack'
    assert detail['protrusions'] == [{'side': 'right', 'bbox': [180, 174, 40, 50], 'area': 1600.0}]
    node.image_callback(message)
    assert node.result_publisher.publish.call_count == 2
    report.update(verdict=module.InspectionResult.NORMAL, protrusions=[], reason='normal')
    node.image_callback(message)
    assert node.result_publisher.publish.call_count == 2
    assert node.model is None


@pytest.mark.parametrize('verdict', [module.InspectionResult.NORMAL, module.InspectionResult.UNCONFIRMED])
def test_rack_normal_or_unknown_does_not_publish_defect(make_node, monkeypatch, verdict):
    """랙 정상 또는 미확인 상태는 불량 이벤트로 발행하지 않는다."""
    node = make_node(rack_confirm_frames=1)
    node._inspection_mode = 'rack'
    node.result_publisher = Mock()
    report = {'verdict': verdict, 'rack': None, 'zones': [], 'protrusions': [], 'reason': 'test'}
    monkeypatch.setattr(module.rack_conditioner, 'inspect_rack', Mock(return_value=report))
    node.image_callback(camera_message())
    node.result_publisher.publish.assert_not_called()


@pytest.mark.parametrize('changed', ['mode', 'request_id', 'stop_event'])
def test_publication_rechecks_mode_and_request(make_node, changed):
    """프레임 처리 뒤 중지·새 요청·종료가 발생하면 발행 직전 다시 확인하여 버린다."""
    node = prepare_yolo_defect_node(make_node, [0])
    request_id = node._inspection_id
    if changed == 'mode':
        node._inspection_mode = 'rack'
    elif changed == 'request_id':
        node._inspection_id += 1
    else:
        node._stop_event.set()
    node._publish_defects([(module.InspectionResult.KIND_EXTINGUISHER, '{}')],
                          camera_message().header.stamp, 'yolo', request_id)
    node.result_publisher.publish.assert_not_called()
