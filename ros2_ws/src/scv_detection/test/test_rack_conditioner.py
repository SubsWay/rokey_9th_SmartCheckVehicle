"""학습 없이 랙 경계·돌출·보류를 확인하는 합성 사진 시험."""

import cv2
import numpy as np
import pytest
from scv_msgs.msg import InspectionResult
from scv_detection import rack_conditioner as rack


def make_rack_frame(protruding=False):
    """흰색 파이프와 안쪽 상자, 선택적으로 바깥 돌출 상자를 그린다."""
    frame = np.full((600, 800, 3), 120, np.uint8)
    # 왼쪽 수직 파이프 (흰색, 좁고 길게)
    cv2.rectangle(frame, (195, 50), (215, 550), (240, 240, 240), -1)
    # 오른쪽 수직 파이프
    cv2.rectangle(frame, (485, 50), (505, 550), (240, 240, 240), -1)
    # 수평 파이프 (선반)
    cv2.rectangle(frame, (195, 50), (505, 65), (240, 240, 240), -1)
    cv2.rectangle(frame, (195, 280), (505, 295), (240, 240, 240), -1)
    cv2.rectangle(frame, (195, 535), (505, 550), (240, 240, 240), -1)
    # 안쪽 상자 (어두운 갈색 — 배경 120과 충분한 명도 차)
    cv2.rectangle(frame, (220, 300), (480, 530), (30, 50, 80), -1)
    if protruding:
        # 오른쪽으로 삐져나온 상자
        cv2.rectangle(frame, (490, 350), (570, 480), (30, 50, 80), -1)
    return frame


def test_detect_rack_finds_pipes():
    """합성 사진에서 흰색 파이프 위치를 자동으로 찾는지 확인한다."""
    frame = make_rack_frame()
    result = rack.detect_rack(frame)
    assert result is not None
    assert 195 <= result['left'] <= 215
    assert 485 <= result['right'] <= 505


def test_detect_rack_returns_none_for_empty():
    """파이프가 없는 사진에서 None을 돌려주는지 확인한다."""
    assert rack.detect_rack(np.full((600, 800, 3), 50, np.uint8)) is None


@pytest.mark.parametrize('protruding,verdict', [
    (False, InspectionResult.NORMAL),
    (True, InspectionResult.ABNORMAL)])
def test_reference_comparison(protruding, verdict):
    """정상 기준과 비교해 돌출 상자를 찾는지 확인한다."""
    normal = make_rack_frame(False)
    detected = rack.detect_rack(normal)
    reference = rack.capture_reference(normal, detected)
    assert reference is not None
    report = rack.inspect_rack(make_rack_frame(protruding), reference=reference)
    assert report['verdict'] == verdict
    assert bool(report['protrusions']) is protruding


def test_missing_reference_holds():
    """기준 사진이 없으면 정상·이상 판정 없이 보류하는지 확인한다."""
    frame = make_rack_frame()
    report = rack.inspect_rack(frame, reference=None)
    assert report['verdict'] == InspectionResult.UNCONFIRMED


def test_no_rack_holds():
    """랙이 감지되지 않으면 보류하는지 확인한다."""
    frame = np.full((600, 800, 3), 50, np.uint8)
    report = rack.inspect_rack(frame)
    assert report['verdict'] == InspectionResult.UNCONFIRMED


def test_confirmation_and_reset():
    """연속 세 번 같은 판정 후 확정하고 미확인 시 이전 상태를 지우는지 확인한다."""
    state = {}
    normal = make_rack_frame(False)
    detected = rack.detect_rack(normal)
    reference = rack.capture_reference(normal, detected)
    abnormal_report = rack.inspect_rack(make_rack_frame(True), reference=reference)
    assert rack.stabilize_result(abnormal_report, state)['verdict'] == InspectionResult.UNCONFIRMED
    assert rack.stabilize_result(abnormal_report, state)['verdict'] == InspectionResult.UNCONFIRMED
    assert rack.stabilize_result(abnormal_report, state)['verdict'] == InspectionResult.ABNORMAL
    unknown = rack.inspect_rack(np.full((600, 800, 3), 50, np.uint8))
    assert rack.stabilize_result(unknown, state)['verdict'] == InspectionResult.UNCONFIRMED
    assert rack.stabilize_result(abnormal_report, state)['verdict'] == InspectionResult.UNCONFIRMED


def test_annotation_preserves_input():
    """표시용 사진을 만들 때 원본을 변경하지 않는지 확인한다."""
    frame = make_rack_frame(True)
    original = frame.copy()
    detected = rack.detect_rack(frame)
    reference = rack.capture_reference(make_rack_frame(False), detected)
    report = rack.inspect_rack(frame, reference=reference)
    output = rack.draw_result(frame, report)
    assert np.array_equal(frame, original)
    assert not np.array_equal(output, original)


@pytest.mark.parametrize('shape,click,expected', [
    ((480, 640, 3), (480, 270), [320, 240]),
    ((720, 1280, 3), (480, 270), [640, 360]),
])
def test_click_coordinate_scales_to_camera(shape, click, expected):
    """창 크기와 다른 카메라 해상도에서도 클릭이 원본 위치로 환산되는지 확인한다."""
    assert rack.scale_display_point(*click, shape) == expected


def test_save_and_load_reference(tmp_path):
    """기준을 파일로 저장했다가 다시 불러와도 같은 판정을 내리는지 확인한다."""
    normal = make_rack_frame(False)
    detected = rack.detect_rack(normal)
    reference = rack.capture_reference(normal, detected)
    path = tmp_path / 'ref.npz'
    rack.save_reference(reference, path)
    loaded = rack.load_reference(path)
    assert loaded is not None
    assert loaded['rack'] == reference['rack']
    assert np.array_equal(loaded['frame'], reference['frame'])
    report = rack.inspect_rack(make_rack_frame(True), reference=loaded)
    assert report['verdict'] == InspectionResult.ABNORMAL


def test_load_missing_file_returns_none(tmp_path):
    """존재하지 않는 파일을 불러오면 None을 돌려주는지 확인한다."""
    assert rack.load_reference(tmp_path / 'nonexistent.npz') is None


def test_normal_inside_change_not_flagged():
    """랙 안쪽 변화는 돌출로 감지하지 않는지 확인한다."""
    normal = make_rack_frame(False)
    detected = rack.detect_rack(normal)
    reference = rack.capture_reference(normal, detected)
    modified = make_rack_frame(False)
    cv2.rectangle(modified, (250, 350), (450, 500), (20, 40, 70), -1)
    report = rack.inspect_rack(modified, reference=reference)
    assert report['verdict'] == InspectionResult.NORMAL
