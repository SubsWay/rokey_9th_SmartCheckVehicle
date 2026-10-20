"""압력계 사진에서 노란 바늘과 녹색 정상 구간을 비교하는 함수 모음.

영상 수신, YOLO 실행, ROS 통신은 매니저에서 담당한다. 기존 색 범위로 정상 영역에
바늘색 픽셀이 있는지 확인하고, 동일 대상의 정상 복귀를 연속 관측으로 확인한다.
바늘 각도는 측정하지 않는다. 반환 좌표는 입력 원본 사진 기준이다.
"""

from __future__ import annotations

from operator import itemgetter

import cv2
import numpy as np


def clip_bbox(bbox, image_shape):
    """실수 박스를 사진 안의 정수 (x1, y1, x2, y2)로 바꾼다. 빈 박스는 None이다."""
    values = np.asarray(bbox, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        return None
    height, width = image_shape[:2]
    x1, y1 = np.floor(values[:2]).astype(int)
    x2, y2 = np.ceil(values[2:]).astype(int)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width, x2), min(height, y2)
    return tuple(int(value) for value in (x1, y1, x2, y2)) if x2 > x1 and y2 > y1 else None


def gauge_belongs_to_extinguisher(gauge_box, extinguisher_box):
    """압력계 박스의 90% 이상이 소화기 안에 있고 면적이 절반 이하인지 확인한다."""
    gx1, gy1, gx2, gy2 = gauge_box
    ex1, ey1, ex2, ey2 = extinguisher_box
    gauge_area = max(0, gx2 - gx1) * max(0, gy2 - gy1)
    body_area = max(0, ex2 - ex1) * max(0, ey2 - ey1)
    overlap = max(0, min(gx2, ex2) - max(gx1, ex1)) * max(0, min(gy2, ey2) - max(gy1, ey1))
    return (gauge_area > 0 and gauge_area <= body_area * 0.5
            and overlap >= gauge_area * 0.9
            and ex1 <= (gx1 + gx2) / 2 <= ex2 and ey1 <= (gy1 + gy2) / 2 <= ey2)


def validate_options(*, min_gauge_diameter=24, boundary_margin_deg=5.0,
                     yellow_lower=(8, 40, 70), yellow_upper=(35, 255, 255),
                     green_lower=(38, 25, 20), green_upper=(120, 255, 255),
                     normal_start_deg=-1.0, normal_end_deg=-1.0, needle_min_pixels=3, normal_margin_deg=0.0):
    """크기·HSV·정상 범위·최소 바늘색 픽셀 수를 검사해 설정 사전을 반환한다.

    정상 범위 두 값이 -1이면 녹색만 사용한다. 보정 각도는 녹색 검출 실패 때만 사용한다.
    boundary_margin_deg는 이전 설정과의 호환을 위해 받지만 새 존재 판정에는 사용하지 않는다.
    needle_min_pixels는 정상 영역에서 붙어 있는 바늘색 픽셀이 몇 개 이상이어야 하는지 뜻한다.
    normal_margin_deg는 찾은 정상 구간을 양옆으로 넓히는 각도다. 바늘이 눈금판보다 떠 있어 비스듬히 보면
    정상 구간 바로 옆으로 밀려 보이는 시차를 흡수한다 (10/16 실기, 0이면 넓히지 않음).
    """
    if not np.isfinite(min_gauge_diameter) or min_gauge_diameter < 16:
        raise ValueError('pressure_min_gauge_diameter must be at least 16 pixels')
    if not np.isfinite(boundary_margin_deg) or not 0 <= boundary_margin_deg < 30:
        raise ValueError('pressure_boundary_margin_deg must satisfy 0 <= margin < 30')
    if not np.isfinite(normal_margin_deg) or not 0 <= normal_margin_deg <= 30:
        raise ValueError('pressure_normal_margin_deg must satisfy 0 <= margin <= 30')
    if (not np.isfinite(needle_min_pixels) or needle_min_pixels < 1
            or needle_min_pixels != int(needle_min_pixels)):
        raise ValueError('pressure_needle_min_pixels must be a positive integer')
    options = {'needle_min_pixels': int(needle_min_pixels),
               'min_gauge_diameter': int(min_gauge_diameter),
               'boundary_margin_deg': float(boundary_margin_deg),
               'normal_margin_deg': float(normal_margin_deg),
               'normal_start_deg': float(normal_start_deg), 'normal_end_deg': float(normal_end_deg)}
    if (normal_start_deg, normal_end_deg) != (-1, -1):
        if (not np.isfinite((normal_start_deg, normal_end_deg)).all()
                or not 0 <= normal_start_deg < 360 or not 0 <= normal_end_deg < 360
                or not 10 <= (normal_end_deg - normal_start_deg) % 360 <= 160):
            raise ValueError('pressure normal bounds must both be -1 or define a 10..160 degree sector')
    for color, lower, upper in (('yellow', yellow_lower, yellow_upper),
                                 ('green', green_lower, green_upper)):
        lo, hi = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
        if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite(lo).all()
                or not np.isfinite(hi).all() or np.any(lo != np.floor(lo))
                or np.any(hi != np.floor(hi)) or np.any(lo < 0) or np.any(lo > hi)
                or np.any(hi > (179, 255, 255))):
            raise ValueError(f'pressure_{color} HSV bounds must be integer H/S/V triples in range')
        options[color + '_lower'] = tuple(int(value) for value in lo)
        options[color + '_upper'] = tuple(int(value) for value in hi)
    return options


def _find_dial(image, min_diameter):
    """압력계 후보 사진의 둥근 테두리를 찾아 (중심 x, 중심 y, 반지름)을 반환한다.

    너무 작은 원, 잘린 원, 테두리 근거가 약한 원은 제외한다. 못 찾으면 None이다.
    """
    height, width = image.shape[:2]
    side = min(height, width)
    if side < min_diameter:
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 40, 120)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    candidates = []
    for contour in contours:
        if len(contour) < 20:
            continue
        (cx, cy), (minor, major), _ = cv2.fitEllipse(contour)
        smaller, larger = min(minor, major), max(minor, major)
        if larger > 0 and smaller / larger >= 0.85:
            candidates.append((cx, cy, (smaller + larger) / 4))
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.0, minDist=max(8, side / 3),
                               param1=120, param2=22,
                               minRadius=max(int(min_diameter / 2), int(side * 0.30)),
                               maxRadius=int(side * 0.55))
    if circles is not None:
        candidates.extend(circles[0].tolist())
    # 테두리에서 1~2픽셀 떨어진 점도 같은 테두리로 인정한다.
    edge_band = cv2.dilate(edges, np.ones((3, 3), np.uint8))
    angles = np.linspace(0, 2 * np.pi, 180, endpoint=False)
    best, best_score = None, 0.0
    for cx, cy, radius in candidates:
        if (radius * 2 < max(min_diameter, side * 0.60)
                or radius > side * 0.55 or min(cx, cy, width - 1 - cx, height - 1 - cy) < radius * 0.94):
            continue
        xs = np.rint(cx + radius * np.cos(angles)).astype(int)
        ys = np.rint(cy + radius * np.sin(angles)).astype(int)
        xs, ys = np.clip(xs, 0, width - 1), np.clip(ys, 0, height - 1)
        coverage = float(np.mean(edge_band[ys, xs] > 0))
        if coverage < 0.65:
            continue
        offset = np.hypot(cx - width / 2, cy - height / 2) / side
        score = coverage + 0.15 * radius / side - 0.2 * offset
        if score > best_score:
            best, best_score = (float(cx), float(cy), float(radius)), score
    return best


def _needle_color_mask(hsv, radius, lower, upper):
    """기존 바늘색 범위로 픽셀을 추출한다. 바늘의 각도·길이·방향은 계산하지 않는다.

    입력은 HSV 사진과 계기판 반지름이다. 반환 사진에서 바늘색 픽셀은 255, 나머지는 0이다.
    JPEG에서 주황빛으로 번진 부분은 주변보다 밝기 V가 15 이상 높은 경우만 보완한다.
    """
    mask = cv2.inRange(hsv, lower, upper)
    warm_lower = (max(0, lower[0] - 3), int(lower[1] * 0.65), max(90, lower[2]))
    if warm_lower[2] <= upper[2]:
        warm = cv2.inRange(hsv, warm_lower, upper)
        kernel = max(5, int(round(radius * 0.35)) | 1)
        brightness = hsv[:, :, 2].astype(float)
        background = cv2.GaussianBlur(hsv[:, :, 2], (kernel, kernel), 0).astype(float)
        warm[brightness - background < 15] = 0
        mask = cv2.bitwise_or(mask, warm)
    return mask


def _normal_region_mask(frame, center, radius, sector, needle_mask):
    """녹색 범위로 정상 영역을 채우고 RGB 색이 빨간 비정상 영역은 빼서 검사 마스크를 만든다.

    중심 축과 유리 테두리는 빼고, 기존 녹색 검사의 반지름 40~90% 부분을 사용한다.
    바늘이 녹색 배경을 가린 자리는 채우되 R이 G·B보다 뚜렷하게 높은 빨간 픽셀은 제외한다.
    needle_mask에 있는 주황빛 바늘색은 빨간 배경으로 오해해 제외하지 않는다.
    각도는 정상 영역의 위치를 그리는 데만 사용하며 바늘 각도는 측정하지 않는다.
    """
    ys, xs = np.indices(frame.shape[:2])
    cx, cy = center
    distance = np.hypot(xs - cx, ys - cy)
    positions = (np.degrees(np.arctan2(cy - ys, xs - cx)) - sector[0]) % 360
    width = (sector[1] - sector[0]) % 360
    inside = ((distance >= radius * 0.40) & (distance <= radius * 0.90)
              & (positions <= width))
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.int16)
    red, green, blue = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    abnormal_red = ((red >= 35) & (red > green + 20) & (red > blue + 20)
                    & (needle_mask == 0))
    inside &= ~abnormal_red
    return (inside.astype(np.uint8) * 255)


def _needle_in_normal_region(needle_mask, normal_mask, min_pixels):
    """정상 영역 안에 붙어 있는 바늘색 픽셀이 충분한지 확인한다.

    작은 색 잡음은 제외한다. (충분한지 여부, 가장 큰 픽셀 덩어리의 크기, 그 덩어리 중심)을 반환한다.
    정상 영역 밖의 바늘은 세지 않으며, 덩어리가 없으면 크기는 0이고 중심은 None이다.
    """
    overlap = cv2.bitwise_and(needle_mask, normal_mask)
    count, _, stats, centers = cv2.connectedComponentsWithStats(overlap)
    if count <= 1:
        return False, 0, None
    label = int(np.argmax(stats[1:, cv2.CC_STAT_AREA])) + 1
    pixels = int(stats[label, cv2.CC_STAT_AREA])
    present = pixels >= min_pixels
    point = tuple(float(value) for value in centers[label]) if present else None
    return present, pixels, point


def _circular_runs(active):
    """360개 각도 표시에서 이어진 참 구간을 (시작 각도, 길이) 목록으로 반환한다."""
    if not np.any(active) or np.all(active):
        return []
    start = int(np.flatnonzero(~active)[0])
    ordered = np.roll(active, -start)
    changes = np.diff(np.r_[False, ordered, False].astype(int))
    return [((int(left) + start) % 360, int(right - left))
            for left, right in zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1))]


def _find_red_scale_arc(hsv, center, radius):
    """붉은 눈금의 각도 띠를 찾아 픽셀별 참·거짓 마스크를 반환한다. 근거가 없으면 None이다.

    빨간 눈금 사이의 작은 정상 구간도 같은 띠로 묶는다. 회전한 계기판도 같은 방식으로 처리한다.
    """
    ys, xs = np.indices(hsv.shape[:2])
    cx, cy = center
    distance = np.hypot(xs - cx, ys - cy)
    red = (((hsv[:, :, 0] <= 10) | (hsv[:, :, 0] >= 170))
           & (hsv[:, :, 1] >= 55) & (hsv[:, :, 2] >= 35)
           & (distance >= radius * 0.40) & (distance <= radius * 0.85))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(red.astype(np.uint8))
    retained = np.zeros(red.shape, dtype=bool)
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] >= max(8, radius * radius * 0.018):
            retained |= labels == label
    red_y, red_x = np.nonzero(retained)
    if len(red_x) < max(20, radius * radius * 0.04):
        return None
    angles = np.degrees(np.arctan2(cy - red_y, red_x - cx)) % 360
    histogram = np.bincount(np.floor(angles).astype(int), minlength=360).astype(float)
    smooth = sum(np.roll(histogram, shift) for shift in range(-4, 5)) / 9
    active = smooth >= max(0.25, float(smooth.max()) * 0.20)
    for start, length in _circular_runs(~active):
        if length <= 45:
            active[(np.arange(length) + start) % 360] = True
    runs = sorted(_circular_runs(active), key=itemgetter(1), reverse=True)
    if not runs or not 45 <= runs[0][1] <= 230:
        return None
    # 떨어진 두 눈금 띠가 비슷한 크기이면 어느 띠가 압력 눈금인지 선택하지 않는다.
    if len(runs) > 1 and runs[1][1] >= max(30, runs[0][1] * 0.4):
        return None
    start, length = runs[0]
    arc = np.zeros(360, dtype=bool)
    # 띠를 붉은 눈금 밖까지 늘리면 JPEG에서 번진 아래쪽 파란 글씨가 다시 포함된다.
    arc[(start + np.arange(length)) % 360] = True
    pixel_angles = np.floor(np.degrees(np.arctan2(cy - ys, xs - cx)) % 360).astype(int)
    return arc[pixel_angles]


def _green_scale_mask(hsv, center, radius, lower, upper):
    """녹색·어두운 청록색을 추출하고 붉은 눈금 밖의 파란 글씨를 제외한 마스크를 반환한다.

    카메라에서 녹색이 청록·청색으로 보일 때는 붉은 눈금 띠가 확인된 부분만 사용한다.
    붉은 눈금이 없으면 기본 녹색 계열(H 90 이하)만 사용하여 파란 글씨를 정상 범위로 읽지 않는다.
    """
    mask = cv2.inRange(hsv, lower, upper)
    arc = _find_red_scale_arc(hsv, center, radius)
    if arc is None:
        mask[hsv[:, :, 0] > 90] = 0
    else:
        mask[~arc] = 0
    return mask


def _find_green_sector(mask, center, radius):
    """원 안쪽 녹색 픽셀의 각도 분포에서 단일 정상 구간을 찾는다. 못 찾으면 None이다."""
    ys, xs = np.nonzero(mask)
    distances = np.hypot(xs - center[0], ys - center[1])
    keep = (distances >= radius * 0.40) & (distances <= radius * 0.90)
    if np.count_nonzero(keep) < max(12, radius * radius * 0.015):
        return None
    angles = np.degrees(np.arctan2(center[1] - ys[keep], xs[keep] - center[0])) % 360
    histogram = np.bincount(np.floor(angles).astype(int), minlength=360).astype(float)
    smooth = sum(np.roll(histogram, shift) for shift in range(-3, 4)) / 7
    active = smooth >= max(0.5, float(smooth.max()) * 0.18)
    # 바늘이나 글자가 가린 8도 이하의 작은 빈틈만 메운다.
    for start, length in _circular_runs(~active):
        if length <= 8:
            active[(np.arange(length) + start) % 360] = True
    runs = sorted(_circular_runs(active), key=itemgetter(1), reverse=True)
    if not runs or not 11 <= runs[0][1] <= 160:
        return None
    if len(runs) > 1 and runs[1][1] >= max(10, runs[0][1] * 0.3):
        return None
    start, length = runs[0]
    return float(start), float((start + length - 1) % 360)


# 넓힌 띠 안에서 바늘로 인정하는 순수 노랑 (HSV). robot4 실사진(good · badleft · badright 63장)으로 확인한 값 (10/16)
BAND_NEEDLE_LOWER = (18, 80, 70)
BAND_NEEDLE_UPPER = (35, 255, 255)


def widen_sector(sector, margin_deg):
    """정상 구간 (시작 각도, 끝 각도)를 양옆으로 margin_deg 씩 넓힌 구간을 반환한다 (0~360 범위로 감음).

    판정에 쓰는 검사 영역만 넓히고(그 띠 안에서는 순수 노랑만 바늘), 화면에 그리는 정상 구간과 보고값은 원래 구간을 쓴다.
    """
    start, end = sector
    if margin_deg <= 0 or (end - start) % 360 + 2 * margin_deg >= 360:   # 넓히면 한 바퀴를 넘는 구간은 그대로
        return float(start), float(end)
    return float((start - margin_deg) % 360), float((end + margin_deg) % 360)


def classify_angle(angle, green_start, green_end, boundary_margin_deg=5.0):
    """이전 호출과의 호환을 위한 각도 비교 함수다. 현재 inspect_pressure에서는 사용하지 않는다.

    입력 각도를 녹색 범위와 비교한다. 새 판정은 정상 영역 안의 바늘색 존재 여부를 사용한다.
    """
    if (not np.isfinite((angle, green_start, green_end, boundary_margin_deg)).all()
            or not 0 <= boundary_margin_deg < 30):
        raise ValueError('angles must be finite and boundary margin must satisfy 0 <= margin < 30')
    width = (green_end - green_start) % 360
    if not 10 <= width <= 160:
        raise ValueError('normal angle sector must span 10..160 degrees')
    position = (angle - green_start) % 360
    start_distance = abs((angle - green_start + 180) % 360 - 180)
    end_distance = abs((angle - green_end + 180) % 360 - 180)
    if min(start_distance, end_distance) <= boundary_margin_deg:
        return 'UNCONFIRMED'
    return 'NORMAL' if position < width else 'ABNORMAL'


def inspect_pressure(frame, gauge_bbox, **options):
    """원본 BGR 사진과 압력계 박스를 받아 정상 영역 안의 바늘색 존재 여부를 판정한다.

    정상 영역에 최소 수 이상의 붙어 있는 바늘색 픽셀이 있으면 NORMAL, 없으면 ABNORMAL이다.
    이 반환값은 이번 사진의 판정이며 정상 복귀의 연속 확인은 별도 함수에서 한다.
    바늘 각도와 끝점 방향은 계산하지 않는다. 원이나 정상 영역을 못 찾으면 UNCONFIRMED이다.
    계산량을 제한하기 위해 큰 사진만 축소하며 최소 크기는 원본 픽셀로 확인한다.
    """
    settings = validate_options(**options)
    bbox = clip_bbox(gauge_bbox, frame.shape)
    report = {'status': 'UNCONFIRMED', 'reason': 'invalid gauge bbox', 'gauge_bbox': bbox,
              'dial_center': None, 'dial_radius': None, 'needle_tip': None,
              'needle_angle_deg': None, 'green_start_deg': None, 'green_end_deg': None,
              'range_source': None, 'needle_in_normal_range': None, 'needle_in_normal_pixels': 0,
              'needle_in_normal_point': None}
    if bbox is None:
        return report
    x1, y1, x2, y2 = bbox
    crop = frame[y1:y2, x1:x2]
    if min(crop.shape[:2]) < settings['min_gauge_diameter']:
        report['reason'] = 'gauge too small in original image'
        return report
    scale = min(1.0, 360 / max(crop.shape[:2]))
    if scale < 1:
        crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    dial = _find_dial(crop, settings['min_gauge_diameter'] * scale)
    if dial is None:
        report['reason'] = 'round dial not found; blurred, cropped or oblique gauge'
        return report
    cx, cy, radius = dial
    report['dial_center'] = (cx / scale + x1, cy / scale + y1)
    report['dial_radius'] = radius / scale
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    green = _green_scale_mask(hsv, (cx, cy), radius, settings['green_lower'], settings['green_upper'])
    sector = _find_green_sector(green, (cx, cy), radius)
    source = 'green_pixels'
    if sector is None:
        if settings['normal_start_deg'] < 0:
            report['reason'] = 'green normal sector missing or ambiguous'
            return report
        sector = (settings['normal_start_deg'], settings['normal_end_deg'])
        source = 'configured_angles'
    report['green_start_deg'], report['green_end_deg'] = sector
    report['range_source'] = source
    needle_mask = _needle_color_mask(hsv, radius, settings['yellow_lower'], settings['yellow_upper'])
    normal_mask = _normal_region_mask(crop, (cx, cy), radius, sector, needle_mask)
    if not np.any(normal_mask):
        report['reason'] = 'normal region empty after excluding abnormal colors'
        return report
    if settings['normal_margin_deg'] > 0:
        # 넓힌 띠(정상 구간 양옆)는 빨간 눈금판 위라, 그 안에서는 주황 보완 없이 순수 노랑만 바늘로 인정한다.
        # (warm 보완을 쓰면 빨간 판의 밝은 부분이 바늘로 잡혀 실제 비정상이 정상으로 뒤집힘 — 10/16 robot4 실사진 QA)
        wide = _normal_region_mask(crop, (cx, cy), radius, widen_sector(sector, settings['normal_margin_deg']),
                                   needle_mask)
        band = cv2.bitwise_and(wide, cv2.bitwise_not(normal_mask))
        strict = cv2.inRange(hsv, BAND_NEEDLE_LOWER, BAND_NEEDLE_UPPER)
        needle_mask = cv2.bitwise_or(cv2.bitwise_and(needle_mask, normal_mask), cv2.bitwise_and(strict, band))
        normal_mask = cv2.bitwise_or(normal_mask, band)
    present, pixels, point = _needle_in_normal_region(
        needle_mask, normal_mask, settings['needle_min_pixels'])
    report['needle_in_normal_range'] = present
    report['needle_in_normal_pixels'] = pixels
    # 이전 키는 호환용으로 남긴다. point는 실제 바늘 끝이 아닌 영역 안 색상 덩어리의 중심이다.
    report['needle_angle_deg'] = None
    report['needle_tip'] = None
    report['needle_in_normal_point'] = (
        (point[0] / scale + x1, point[1] / scale + y1) if point is not None else None)
    report['status'] = 'NORMAL' if present else 'ABNORMAL'
    report['reason'] = ('needle color present inside normal region' if present
                        else 'needle color absent inside normal region')
    return report


def _reading_quality(report):
    """중복 후보 중 확정 판정·정상 범위·바늘 근거가 더 많이 있는 결과를 우선하도록 점수를 준다."""
    return (report['status'] != 'UNCONFIRMED', report['green_start_deg'] is not None,
            report.get('needle_in_normal_range') is True)


def select_pressure_report(reports):
    """중복 계기의 좋은 근거를 선택한다. 서로 다른 계기나 서로 모순되는 판정은 보류한다."""
    found = []
    for report in sorted(reports, key=_reading_quality, reverse=True):
        if report['dial_center'] is None:
            continue
        duplicates = [other for other in found
                      if np.linalg.norm(np.subtract(report['dial_center'], other['dial_center']))
                      < min(report['dial_radius'], other['dial_radius']) * 0.5]
        if duplicates:
            other = duplicates[0]
            if {report['status'], other['status']} == {'NORMAL', 'ABNORMAL'}:
                other.update(status='UNCONFIRMED', reason='inconsistent readings of the same pressure dial')
            continue
        found.append(report.copy())
    if len(found) == 1:
        return found[0].copy()
    if len(found) > 1:
        report = found[0].copy()
        report.update(status='UNCONFIRMED', reason='multiple pressure dials inside extinguisher')
        return report
    if reports:
        return reports[0].copy()
    return {'status': 'UNCONFIRMED', 'reason': 'pressure gauge not detected',
            'gauge_bbox': None, 'dial_center': None, 'dial_radius': None,
            'needle_tip': None, 'needle_angle_deg': None, 'green_start_deg': None, 'green_end_deg': None,
            'range_source': None, 'needle_in_normal_range': None, 'needle_in_normal_pixels': 0,
            'needle_in_normal_point': None}


def reset_stability_state(state):
    """압력 판정 이력을 지우되 다음 대상 번호는 보존해 새 대상과 예전 대상을 구별한다."""
    next_id = state.get('next_id', 1)
    state.clear()
    state['next_id'] = next_id


def _same_pressure_target(current, previous):
    """소화기 박스와 계기 중심·크기·정상 구간이 비슷한 경우만 같은 관측 대상으로 연결한다.

    YOLO 목록의 순서는 사용하지 않는다. 계기 위치 정보가 없거나 큰 이동·크기/정상 영역 변화가
    있으면 확인 이력을 이어 쓰지 않는다. 계기 중심·크기는 계기 크기의 비율로 비교하고,
    정상 구간 양 끝은 각도 차이, 소화기 박스는 겹침 비율로 비교한다.
    """
    if any(report.get('dial_center') is None or report.get('dial_radius') is None
           or report.get('gauge_bbox') is None or report.get('bbox') is None
           or report.get('green_start_deg') is None or report.get('green_end_deg') is None
           for report in (current, previous)):
        return False
    radius = min(current['dial_radius'], previous['dial_radius'])
    if radius <= 0:
        return False
    if (np.linalg.norm(np.subtract(current['dial_center'], previous['dial_center'])) > radius * 0.35
            or abs(current['dial_radius'] - previous['dial_radius']) > radius * 0.25
            or current.get('range_source') != previous.get('range_source')):
        return False
    for key in ('green_start_deg', 'green_end_deg'):
        if abs((current[key] - previous[key] + 180) % 360 - 180) > 12:
            return False
    a, b = current['bbox'], previous['bbox']
    overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    area_a, area_b = (a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - overlap
    return union > 0 and overlap / union >= 0.30


def _confirm_pressure_report(report, track, frame_time, confirm_seconds, confirm_frames):
    """유효 비정상은 즉시 반영하고 정상은 같은 대상에서 시간과 프레임 수를 함께 확인한다.

    미확인은 확인 횟수와 확정 상태를 지운다. 정상 확인 중에는 직전 비정상만 유지하며,
    최초 관측이면 미확인이다. 원래 프레임 판정과 확인 진행 상황도 함께 반환한다.
    """
    output = dict(report)
    raw_status = report['status']
    output.update(raw_status=raw_status, raw_reason=report['reason'])
    if raw_status == 'ABNORMAL':
        track.update(confirmed_status='ABNORMAL', normal_since=None, normal_count=0)
    elif raw_status != 'NORMAL':
        track.update(confirmed_status=None, normal_since=None, normal_count=0)
    else:
        if track.get('normal_since') is None:
            track['normal_since'] = frame_time
        track['normal_count'] = track.get('normal_count', 0) + 1
        elapsed = frame_time - track['normal_since']
        if elapsed >= confirm_seconds and track['normal_count'] >= confirm_frames:
            track['confirmed_status'] = 'NORMAL'
        elif track.get('confirmed_status') != 'NORMAL':
            output.update(status=track.get('confirmed_status') or 'UNCONFIRMED',
                          reason='normal recovery pending')
    output.update(pressure_track_id=track['track_id'],
                  normal_confirm_frames=track.get('normal_count', 0),
                  normal_confirm_elapsed_s=(frame_time - track['normal_since']
                                            if track.get('normal_since') is not None else 0.0),
                  normal_confirm_required_s=confirm_seconds)
    track['geometry'] = dict(report)
    return output


def stabilize_pressure_reports(reports, state, frame_time, *, observation_stamp=None,
                               normal_confirm_seconds=0.5, normal_confirm_frames=3,
                               max_gap_seconds=2.0):
    """같은 압력계에서 새 정상 관측이 연속될 때만 정상 복귀를 확정한다.

    reports는 이번 사진의 판독 목록이고 state는 갱신할 이전 관측 기록이다.
    frame_time은 단조 증가하는 수신 시각(초), observation_stamp는 촬영 시각이다.
    반환한 확정 목록을 화면과 ROS 발행에 함께 사용한다. 대상이 사라지거나 영상이
    오래 끊기면 이력을 버린다. 중복·역순 촬영 시각은 정상 확인에 세지 않는다.
    """
    if (not np.isfinite((frame_time, normal_confirm_seconds, normal_confirm_frames, max_gap_seconds)).all()
            or normal_confirm_seconds <= 0 or max_gap_seconds <= 0
            or normal_confirm_frames < 2 or normal_confirm_frames != int(normal_confirm_frames)):
        raise ValueError('positive confirmation times and at least two normal frames required')
    last_time, last_stamp = state.get('last_time'), state.get('last_stamp')
    if last_time is not None and frame_time - last_time > max_gap_seconds:
        reset_stability_state(state)
        last_time, last_stamp = None, None
    if ((last_time is not None and frame_time <= last_time)
            or (observation_stamp is not None and last_stamp is not None
                and observation_stamp <= last_stamp)):
        reset_stability_state(state)
        # 확인 이력은 지워도 마지막 촬영 시각은 남겨 반복된 옛 사진을 계속 거른다.
        state.update(last_time=max(frame_time, last_time) if last_time is not None else frame_time,
                     last_stamp=last_stamp)
        return [dict(report, raw_status=report['status'], raw_reason=report['reason'],
                     status='UNCONFIRMED', reason='duplicate or older pressure frame') for report in reports]
    previous = state.get('tracks', {})
    matches = {index: [track_id for track_id, track in previous.items()
                       if _same_pressure_target(report, track['geometry'])]
               for index, report in enumerate(reports)}
    uses = {track_id: sum(track_id in candidates for candidates in matches.values())
            for track_id in previous}
    tracks, outputs = {}, []
    for index, report in enumerate(reports):
        candidates = matches[index]
        if len(candidates) == 1 and uses[candidates[0]] == 1:
            track_id = candidates[0]
            track = dict(previous[track_id])
        else:
            track_id = state.get('next_id', 1)
            state['next_id'] = track_id + 1
            track = {'track_id': track_id}
        outputs.append(_confirm_pressure_report(report, track, frame_time,
                                                normal_confirm_seconds, normal_confirm_frames))
        tracks[track_id] = track
    # 안 보인 대상의 이력을 즉시 버려 다른 대상이나 다음 관측으로 이어 붙이지 않는다.
    state.update(tracks=tracks, last_time=frame_time,
                 last_stamp=observation_stamp if observation_stamp is not None else last_stamp)
    return outputs


def draw_result(frame, report):
    """확정 상태로 계기판을 표시하고 정상 확인 대기·미확인 사유를 함께 그린다."""
    color = {'NORMAL': (0, 255, 0), 'ABNORMAL': (0, 0, 255),
             'UNCONFIRMED': (192, 192, 192)}[report['status']]
    bbox = report['gauge_bbox']
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
    if report['dial_center'] is not None:
        center = tuple(int(round(value)) for value in report['dial_center'])
        radius = int(round(report['dial_radius']))
        cv2.circle(frame, center, radius, color, 1)
        if report['green_start_deg'] is not None:
            start, end = report['green_start_deg'], report['green_end_deg']
            angles = np.radians(start + np.arange(int((end - start) % 360) + 1))
            points = np.column_stack((center[0] + radius * 0.85 * np.cos(angles),
                                      center[1] - radius * 0.85 * np.sin(angles))).astype(np.int32)
            cv2.polylines(frame, [points], False, (0, 255, 0), 2)
        if report.get('needle_in_normal_point') is not None:
            point = tuple(int(round(value)) for value in report['needle_in_normal_point'])
            cv2.circle(frame, point, 3, (0, 255, 255), 1)
    text = f"Pressure {report['status']}"
    if report['reason'] == 'normal recovery pending':
        text += f" (NORMAL pending {report.get('normal_confirm_elapsed_s', 0):.1f}s)"
    x1, y1, _, y2 = report['bbox']
    cv2.putText(frame, text, (x1, max(14, y1 - 24)), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, color, 1, cv2.LINE_AA)
    if report['status'] == 'UNCONFIRMED':
        reasons = {
            'gauge too small in original image': 'Gauge too small',
            'round dial not found; blurred, cropped or oblique gauge': 'Dial outline not found',
            'yellow needle missing or ambiguous': 'Needle not found or ambiguous',
            'normal recovery pending': 'Confirming normal pressure',
            'duplicate or older pressure frame': 'Pressure frame not fresh',
            'pressure camera frame expired': 'Camera frame expired',
            'green normal sector missing or ambiguous': 'Normal sector not found or ambiguous',
            'normal region empty after excluding abnormal colors': 'Normal region not found',
            'yellow needle near normal boundary': 'Needle near normal boundary',
            'pressure gauge not detected': 'Pressure gauge not detected',
            'pressure gauge crop inference failed': 'Gauge detection failed',
            'pressure gauge belongs to multiple extinguishers': 'Gauge ownership ambiguous',
            'multiple pressure dials inside extinguisher': 'Multiple pressure dials',
            'inconsistent readings of the same pressure dial': 'Pressure readings disagree',
        }
        reason = reasons.get(report['reason'], report['reason'])
        cv2.putText(frame, reason, (x1 + 4, min(y2 - 6, frame.shape[0] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv2.LINE_AA)
