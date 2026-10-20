"""합성 압력계로 바늘 각도·녹색 정상 구간·판독 보류 조건을 확인한다."""

import cv2
import numpy as np
import pytest
from pathlib import Path

from scv_detection import extinguisher_pressure as pressure


def make_gauge_image(angle=90, start=55, end=125, *, green=True, yellow=True, radius=70):
    """흰 계기판에 녹색 부채꼴과 노란 바늘을 그려 (원본 사진, 압력계 박스)를 반환한다."""
    side = radius * 2 + 40
    image = np.full((side, side, 3), 230, np.uint8)
    center = np.array((side / 2, side / 2))
    cv2.circle(image, tuple(center.astype(int)), radius, (20, 20, 20), 2)
    if green:
        angles = np.radians(start + np.arange(int((end - start) % 360) + 1))
        unit = np.column_stack((np.cos(angles), -np.sin(angles)))
        outer = center + unit * radius * 0.83
        inner = center + unit[::-1] * radius * 0.52
        cv2.fillPoly(image, [np.vstack((outer, inner)).astype(np.int32)], (0, 150, 0))
    if yellow:
        tip = center + radius * 0.73 * np.array((np.cos(np.radians(angle)), -np.sin(np.radians(angle))))
        cv2.line(image, tuple(center.astype(int)), tuple(np.rint(tip).astype(int)), (0, 240, 240),
                 max(2, int(radius * 0.06)))
    return image, (5, 5, side - 5, side - 5)


@pytest.mark.parametrize('angle,status', [(90, 'NORMAL'), (70, 'NORMAL'), (110, 'NORMAL'),
                                         (20, 'ABNORMAL'), (160, 'ABNORMAL'), (270, 'ABNORMAL'),
                                         (55, 'UNCONFIRMED'), (125, 'UNCONFIRMED')])
def test_pressure_from_pixels(angle, status):
    """좌우 정상·비정상·경계 바늘을 색상과 각도로 판독하고 원본을 보존하는지 확인한다."""
    image, bbox = make_gauge_image(angle)
    original = image.copy()
    result = pressure.inspect_pressure(image, bbox)
    assert result['status'] == status, result
    assert abs((result['needle_angle_deg'] - angle + 180) % 360 - 180) < 3
    assert result['range_source'] == 'green_pixels'
    assert result['green_start_deg'] == pytest.approx(55, abs=5)
    assert result['green_end_deg'] == pytest.approx(125, abs=5)
    assert np.array_equal(image, original)


@pytest.mark.parametrize('angle,status', [(0, 'NORMAL'), (357, 'NORMAL'), (90, 'ABNORMAL'),
                                         (350, 'UNCONFIRMED'), (20, 'UNCONFIRMED')])
def test_sector_crosses_zero(angle, status):
    """정상 구간이 350도에서 20도로 이어져도 경계와 안팎을 올바르게 판단한다."""
    assert pressure.classify_angle(angle, 350, 20) == status
    image, bbox = make_gauge_image(angle, 350, 20)
    result = pressure.inspect_pressure(image, bbox)
    assert result['status'] == status, result


@pytest.mark.parametrize('green,yellow,reason', [
    (False, True, 'green normal sector'), (True, False, 'yellow needle'),
])
def test_missing_evidence_is_unconfirmed(green, yellow, reason):
    """녹색이나 노란 바늘이 없으면 정상·비정상 대신 이유를 담은 미확인을 반환한다."""
    image, bbox = make_gauge_image(green=green, yellow=yellow)
    result = pressure.inspect_pressure(image, bbox)
    assert result['status'] == 'UNCONFIRMED'
    assert reason in result['reason']


def test_calibrated_sector_only_used_when_green_missing():
    """보정 각도는 녹색 검출 실패 때만 사용하고, 기본 설정에는 임의 정상 범위가 없음을 확인한다."""
    image, bbox = make_gauge_image(green=False)
    assert pressure.inspect_pressure(image, bbox)['status'] == 'UNCONFIRMED'
    options = dict(normal_start_deg=60.0, normal_end_deg=120.0)
    result = pressure.inspect_pressure(image, bbox, **options)
    assert result['status'] == 'NORMAL'
    assert result['range_source'] == 'configured_angles'
    image, bbox = make_gauge_image()
    result = pressure.inspect_pressure(image, bbox, normal_start_deg=150.0, normal_end_deg=190.0)
    assert result['status'] == 'NORMAL'
    assert result['range_source'] == 'green_pixels'


def test_small_gauge_is_not_made_readable_by_resizing():
    """원본 계기판이 너무 작으면 확대해서 정상으로 판정하지 않는지 확인한다."""
    image, _ = make_gauge_image(radius=8)
    result = pressure.inspect_pressure(image, (19, 19, 37, 37))
    assert result['status'] == 'UNCONFIRMED'
    assert 'too small' in result['reason']


def test_blurred_or_oblique_dial_is_unconfirmed():
    """테두리가 없거나 심하게 찌그러진 압력계는 둥근 계기판으로 인정하지 않는다."""
    image = np.full((180, 180, 3), 230, np.uint8)
    for ellipse in (False, True):
        if ellipse:
            cv2.ellipse(image, (90, 90), (70, 32), 0, 0, 360, (20, 20, 20), 2)
        result = pressure.inspect_pressure(image, (5, 5, 175, 175))
        assert result['status'] == 'UNCONFIRMED'
        assert 'round dial' in result['reason']


def test_yellow_decorations_and_two_ended_line_are_not_needles():
    """중심에서 떨어진 노란 장식과 머리·꼬리를 구별할 수 없는 선을 바늘로 오인하지 않는다."""
    mask = np.zeros((180, 180), np.uint8)
    cv2.line(mask, (130, 40), (150, 60), 255, 4)
    assert pressure._find_yellow_needle(mask, (90, 90), 70) is None
    mask[:] = 0
    cv2.line(mask, (40, 90), (140, 90), 255, 4)
    assert pressure._find_yellow_needle(mask, (90, 90), 70) is None


def test_two_green_sectors_are_ambiguous():
    """서로 떨어진 녹색 구간 두 개를 임의로 하나 선택해 정상 판정하지 않는다."""
    image, bbox = make_gauge_image(90, 55, 100)
    second, _ = make_gauge_image(90, 200, 245, yellow=False)
    green_pixels = (second[:, :, 1] == 150) & (second[:, :, 0] == 0)
    image[green_pixels] = second[green_pixels]
    report = pressure.inspect_pressure(image, bbox)
    assert report['status'] == 'UNCONFIRMED'
    assert 'green normal sector' in report['reason']


def test_coordinates_and_large_image_scaling():
    """부분 사진을 원본에 넣었을 때 중심·바늘 끝이 전체 사진 좌표로 돌아오는지 확인한다."""
    gauge, _ = make_gauge_image(radius=200)
    h, w = gauge.shape[:2]
    frame = np.zeros((h + 100, w + 130, 3), np.uint8)
    frame[40:40+h, 70:70+w] = gauge
    report = pressure.inspect_pressure(frame, (70, 40, 70+w, 40+h))
    assert report['status'] == 'NORMAL'
    assert report['dial_center'] == pytest.approx((70+w/2, 40+h/2), abs=3)
    assert report['needle_tip'][0] == pytest.approx(70+w/2, abs=3)


@pytest.mark.parametrize('bbox', [(40, 40, 20, 20), (0, 0, 0, 10),
                                 (np.nan, 0, 30, 30), (100, 100, 150, 150)])
def test_invalid_bbox_does_not_crash(bbox):
    """역방향·빈 영역·숫자가 아닌 좌표·화면 밖 압력계 박스는 미확인으로 처리한다."""
    report = pressure.inspect_pressure(np.zeros((80, 80, 3), np.uint8), bbox)
    assert report['status'] == 'UNCONFIRMED'
    assert report['gauge_bbox'] is None


@pytest.mark.parametrize('options', [
    {'min_gauge_diameter': 0}, {'min_gauge_diameter': np.nan},
    {'boundary_margin_deg': -1}, {'boundary_margin_deg': 30},
    {'yellow_lower': (0, 255)}, {'yellow_upper': (180, 255, 255)},
    {'green_lower': (38, 60, 280)}, {'green_upper': (30, 50, 20)},
    {'yellow_lower': (15.5, 80, 70)}, {'normal_start_deg': 60.0},
    {'normal_start_deg': 120.0, 'normal_end_deg': 60.0},
    {'normal_start_deg': np.nan, 'normal_end_deg': 120.0},
])
def test_invalid_options_fail_early(options):
    """잘못된 크기·각도·색 범위는 노드를 실행하기 전에 거절할 수 있도록 오류를 낸다."""
    with pytest.raises(ValueError):
        pressure.validate_options(**options)


def test_gauge_must_belong_to_one_body():
    """작은 내부 압력계만 연결하고 소화기 크기와 같은 박스·바깥 박스는 제외한다."""
    body = (10, 10, 210, 400)
    assert pressure.gauge_belongs_to_extinguisher((80, 30, 140, 90), body)
    assert not pressure.gauge_belongs_to_extinguisher(body, body)
    assert not pressure.gauge_belongs_to_extinguisher((190, 30, 250, 90), body)


def test_multiple_dials_are_not_arbitrarily_selected():
    """같은 계기의 중복 후보는 합치고 서로 다른 계기가 두 개면 소화기 판정을 보류한다."""
    image, bbox = make_gauge_image()
    first = pressure.inspect_pressure(image, bbox)
    duplicate = first.copy()
    assert pressure.select_pressure_report([first, duplicate])['status'] == 'NORMAL'
    second = first.copy()
    second['dial_center'] = (300.0, 300.0)
    result = pressure.select_pressure_report([first, second])
    assert result['status'] == 'UNCONFIRMED'
    assert 'multiple pressure dials' in result['reason']


def test_draw_result_includes_pressure_measurements():
    """판독 결과의 계기판과 바늘을 표시용 사진에 덧그릴 수 있는지 확인한다."""
    image, bbox = make_gauge_image(20)
    report = pressure.inspect_pressure(image, bbox)
    report['bbox'] = (0, 30, image.shape[1], image.shape[0])
    annotated = image.copy()
    pressure.draw_result(annotated, report)
    assert not np.array_equal(image, annotated)


def test_duplicate_readings_use_better_evidence_and_reject_conflicts():
    """같은 계기의 좋은 근거를 우선하되 정상과 비정상이 충돌하면 미확인으로 처리한다."""
    image, bbox = make_gauge_image()
    normal = pressure.inspect_pressure(image, bbox)
    unknown = normal.copy()
    unknown.update(status='UNCONFIRMED', needle_angle_deg=None, green_start_deg=None)
    assert pressure.select_pressure_report([unknown, normal])['status'] == 'NORMAL'
    abnormal = normal.copy()
    abnormal['status'] = 'ABNORMAL'
    report = pressure.select_pressure_report([normal, abnormal])
    assert report['status'] == 'UNCONFIRMED'
    assert 'inconsistent readings' in report['reason']
    assert normal['status'] == 'NORMAL'


def make_printed_gauge(angle=90, *, rotation=0, normal_sector=True):
    """붉은 눈금·어두운 청록색 구간·검은 회전축 때문에 짧게 보이는 주황빛 바늘을 만든다."""
    image = np.full((84, 84, 3), 225, np.uint8)
    center, radius = np.array((42, 42)), 28
    cv2.circle(image, tuple(center), radius, (20, 20, 20), 2)
    red_angles = np.radians(np.arange(rotation, rotation + 181))
    red_points = center + radius * 0.83 * np.column_stack((np.cos(red_angles), -np.sin(red_angles)))
    cv2.fillPoly(image, [np.vstack((center, red_points)).astype(np.int32)], (35, 40, 130))
    if normal_sector:
        green_angles = np.radians(np.arange(78 + rotation, 103 + rotation))
        green_points = center + radius * 0.83 * np.column_stack((np.cos(green_angles), -np.sin(green_angles)))
        green_color = tuple(int(value) for value in cv2.cvtColor(
            np.array([[[100, 90, 45]]], np.uint8), cv2.COLOR_HSV2BGR)[0, 0])
        cv2.fillPoly(image, [np.vstack((center, green_points)).astype(np.int32)], green_color)
    # 청록색 범위를 넓히더라도 아래쪽 파란 인쇄 영역이 정상 눈금이 되면 안 된다.
    blue_angles = np.radians(np.arange(200 + rotation, 301 + rotation))
    blue_points = center + radius * 0.70 * np.column_stack((np.cos(blue_angles), -np.sin(blue_angles)))
    cv2.fillPoly(image, [np.vstack((center, blue_points)).astype(np.int32)], (170, 100, 50))
    cv2.circle(image, tuple(center), int(radius * 0.32), (25, 25, 25), -1)
    direction = np.array((np.cos(np.radians(angle + rotation)), -np.sin(np.radians(angle + rotation))))
    start, end = center + direction * radius * 0.35, center + direction * radius * 0.73
    needle_color = tuple(int(value) for value in cv2.cvtColor(
        np.array([[[8, 105, 220]]], np.uint8), cv2.COLOR_HSV2BGR)[0, 0])
    cv2.line(image, tuple(np.rint(start).astype(int)), tuple(np.rint(end).astype(int)), needle_color, 2)
    return image, (8, 8, 76, 76)


@pytest.mark.parametrize('angle,status', [(90, 'NORMAL'), (130, 'ABNORMAL'), (40, 'ABNORMAL')])
@pytest.mark.parametrize('rotation', [0, 90, 180, 270])
def test_short_warm_needle_and_dark_cyan_scale(angle, status, rotation):
    """짧은 주황빛 바늘과 어두운 청록색 눈금을 회전한 계기판에서도 읽는다."""
    image, bbox = make_printed_gauge(angle, rotation=rotation)
    report = pressure.inspect_pressure(image, bbox)
    assert report['status'] == status, report
    assert abs((report['needle_angle_deg'] - angle - rotation + 180) % 360 - 180) < 6
    assert report['range_source'] == 'green_pixels'


def test_blue_print_is_not_a_normal_sector():
    """녹색 눈금이 없으면 아래쪽 파란 글씨·도형만 보고 압력 정상 범위를 만들지 않는다."""
    image, bbox = make_printed_gauge(normal_sector=False)
    report = pressure.inspect_pressure(image, bbox)
    assert report['needle_angle_deg'] is not None
    assert report['status'] == 'UNCONFIRMED'
    assert report['green_start_deg'] is None
    assert 'green normal sector' in report['reason']


def test_cyan_requires_red_scale_context():
    """붉은 눈금이 없는 청색 장식은 일반 녹색 구간으로 확장해서 읽지 않는다."""
    image, bbox = make_gauge_image(green=False)
    cv2.ellipse(image, (90, 90), (50, 50), 0, 235, 305, (170, 100, 50), 12)
    report = pressure.inspect_pressure(image, bbox)
    assert report['status'] == 'UNCONFIRMED'
    assert 'green normal sector' in report['reason']


def test_user_screen_needle_is_detected_without_manual_normal_angles():
    """사용자가 올린 화면의 주황빛 짧은 바늘·청록색 구간을 기본 설정으로 재현한다."""
    path = Path(__file__).parent / 'data' / 'pressure_screen_20261013.png'
    image = cv2.imread(str(path))
    assert image is not None
    bbox = (0, 0, image.shape[1], image.shape[0])
    old = pressure.inspect_pressure(image, bbox, yellow_lower=(15, 80, 70),
                                    green_lower=(38, 60, 40), green_upper=(90, 255, 255))
    assert old['status'] == 'UNCONFIRMED'
    report = pressure.inspect_pressure(image, bbox)
    assert report['status'] == 'ABNORMAL', report
    assert report['needle_angle_deg'] == pytest.approx(130, abs=5)
    assert report['green_start_deg'] == pytest.approx(71, abs=5)
    assert report['green_end_deg'] == pytest.approx(96, abs=5)
    assert report['range_source'] == 'green_pixels'


@pytest.mark.parametrize('shape', ['round_spot', 'symmetric_line', 'two_tips'])
def test_short_tip_fallback_rejects_reflections_and_ambiguity(shape):
    """짧은 바늘 보완 경로도 둥근 반사점·대칭인 선·서로 다른 두 끝점은 보류한다."""
    mask = np.zeros((180, 180), np.uint8)
    center, radius = np.array((90, 90)), 70
    if shape == 'round_spot':
        cv2.circle(mask, (90, 55), 8, 255, -1)
    elif shape == 'symmetric_line':
        cv2.line(mask, (40, 90), (140, 90), 255, 4)
    else:
        for angle in (50, 130):
            direction = np.array((np.cos(np.radians(angle)), -np.sin(np.radians(angle))))
            start, end = center + radius * 0.35 * direction, center + radius * 0.75 * direction
            cv2.line(mask, tuple(np.rint(start).astype(int)), tuple(np.rint(end).astype(int)), 255, 3)
    assert pressure._find_short_needle_tip(mask, center, radius) is None
