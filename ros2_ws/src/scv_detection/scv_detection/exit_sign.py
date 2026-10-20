"""검출된 비상구 표지의 밝기·녹색 분포를 읽고 점등 상태를 연속 확인하는 함수 모음.

영상 수신·YOLO 추론·ROS 실행·바운딩 박스 그리기는 매니저에서 담당한다.
아래 기준값은 실촬영으로 보정하지 않은 초기값이다. 실제 조명과 카메라 노출에 맞춰 조정한다.
표시 면이나 비교할 주변을 찾지 못하면 소등으로 단정하지 않고 UNCONFIRMED를 반환한다.
"""

from __future__ import annotations

import cv2
import numpy as np


DEFAULT_OPTIONS = {
    'min_face_pixels': 256,
    'min_face_side': 16,
    'min_face_area_ratio': 0.04,
    'max_face_area_ratio': 0.85,
    'green_lower': (35, 30, 20),
    'green_upper': (100, 255, 255),
    'min_green_ratio': 0.015,
    'on_y_median': 115.0,
    'off_y_median': 95.0,
    'on_y_p90': 155.0,
    'off_y_p90': 130.0,
    'bright_y': 145.0,
    'on_bright_ratio': 0.20,
    'off_bright_ratio': 0.08,
    'on_contrast_min': -0.15,
    'off_contrast_max': -0.25,
    'reference_margin_ratio': 0.15,
    'min_reference_pixels': 64,
    'min_sharpness': 5.0,
    'max_clipped_ratio': 0.80,
    'confirm_seconds': 0.5,
    'confirm_frames': 3,
    'match_iou': 0.30,
}


def validate_options(**overrides):
    """판독할 크기·색·밝기·확인 시간 설정을 검사하고 정리된 설정 사전을 반환한다."""
    unknown = set(overrides) - set(DEFAULT_OPTIONS)
    if unknown:
        raise ValueError(f'unknown exit sign options: {sorted(unknown)}')
    settings = dict(DEFAULT_OPTIONS, **overrides)
    for key in set(settings) - {'green_lower', 'green_upper'}:
        if not np.isfinite(settings[key]):
            raise ValueError(f'exit_sign_{key} must be finite')
    for key in ('min_face_pixels', 'min_face_side', 'min_reference_pixels', 'confirm_frames'):
        value = settings[key]
        if value < 1 or value != int(value):
            raise ValueError(f'exit_sign_{key} must be a positive integer')
        settings[key] = int(value)
    if settings['min_face_side'] < 4 or settings['confirm_frames'] < 2:
        raise ValueError('exit sign requires at least 4-pixel face sides and 2 confirmation frames')
    for key in ('min_face_area_ratio', 'max_face_area_ratio', 'min_green_ratio',
                'on_bright_ratio', 'off_bright_ratio', 'max_clipped_ratio', 'match_iou'):
        if not 0 < settings[key] < 1:
            raise ValueError(f'exit_sign_{key} must be between 0 and 1')
    if (settings['min_face_area_ratio'] >= settings['max_face_area_ratio']
            or settings['off_bright_ratio'] >= settings['on_bright_ratio']):
        raise ValueError('exit sign area ratios and OFF/ON brightness ratios must be ordered')
    for key in ('on_y_median', 'off_y_median', 'on_y_p90', 'off_y_p90', 'bright_y'):
        if not 0 <= settings[key] <= 255:
            raise ValueError(f'exit_sign_{key} must be between 0 and 255')
    if (settings['off_y_median'] >= settings['on_y_median']
            or settings['off_y_p90'] >= settings['on_y_p90']):
        raise ValueError('exit sign OFF brightness must be lower than ON brightness')
    if not -1 <= settings['off_contrast_max'] < settings['on_contrast_min'] <= 1:
        raise ValueError('exit sign OFF/ON contrast thresholds must be ordered within -1..1')
    if (settings['reference_margin_ratio'] <= 0 or settings['min_sharpness'] < 0
            or settings['confirm_seconds'] <= 0):
        raise ValueError('exit sign reference margin/time must be positive; sharpness must be nonnegative')
    lower, upper = np.asarray(settings['green_lower'], dtype=float), np.asarray(settings['green_upper'], dtype=float)
    if (lower.shape != (3,) or upper.shape != (3,) or not np.isfinite((lower, upper)).all()
            or np.any(lower != np.floor(lower)) or np.any(upper != np.floor(upper))
            or np.any(lower < 0) or np.any(lower > upper) or np.any(upper > (179, 255, 255))):
        raise ValueError('exit sign green bounds must be integer HSV triples within H=0..179, S/V=0..255')
    settings['green_lower'], settings['green_upper'] = tuple(map(int, lower)), tuple(map(int, upper))
    return settings


def clip_bbox(bbox, image_shape):
    """전체 영상의 좌표를 사진 안의 정수 xyxy로 바꾸고 잘못됐거나 빈 영역이면 None을 반환한다."""
    values = np.asarray(bbox, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        return None
    height, width = image_shape[:2]
    x1, y1 = max(0, int(np.floor(values[0]))), max(0, int(np.floor(values[1])))
    x2, y2 = min(width, int(np.ceil(values[2]))), min(height, int(np.ceil(values[3])))
    return (x1, y1, x2, y2) if x1 < x2 and y1 < y2 else None


def _find_face_mask(gray, settings):
    """표지 영상의 사각형 테두리를 찾아 (표시 면 마스크, 꼭짓점)을 반환하며 실패하면 None이다.

    녹색이 없을 수 있는 소등 표지도 찾도록 색 대신 테두리 형태를 사용한다.
    검출 박스 전체에 가까운 외곽은 제외하고 안쪽의 가장 큰 사각형을 선택한다.
    """
    height, width = gray.shape
    smooth = cv2.GaussianBlur(gray, (3, 3), 0)
    lower = max(8, int(np.percentile(smooth, 60) * 0.5))
    upper = max(lower + 10, int(np.percentile(smooth, 90) * 1.2))
    edges = cv2.Canny(smooth, lower, upper)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        polygon = cv2.approxPolyDP(contour, perimeter * 0.03, True)
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            continue
        area = float(cv2.contourArea(polygon))
        ratio = area / (height * width)
        if not settings['min_face_area_ratio'] <= ratio <= settings['max_face_area_ratio']:
            continue
        x, y, face_width, face_height = cv2.boundingRect(polygon)
        if (min(face_width, face_height) < settings['min_face_side']
                or x <= 0 or y <= 0 or x + face_width >= width or y + face_height >= height):
            continue
        candidates.append((area, polygon, min(face_width, face_height)))
    if not candidates:
        return None
    _, polygon, side = max(candidates, key=lambda item: item[0])
    mask = np.zeros_like(gray)
    cv2.fillConvexPoly(mask, polygon, 255)
    # 테두리 반사광이 밝기 통계에 섞이지 않도록 표시 면의 가장자리를 조금 제외한다.
    radius = max(1, int(round(side * 0.025)))
    mask = cv2.erode(mask, np.ones((radius * 2 + 1, radius * 2 + 1), dtype=np.uint8))
    return mask, polygon.reshape(4, 2)


def _reference_brightness(frame, bbox, excluded_boxes, settings):
    """표지 밖의 주변 영역에서 다른 검출 물체를 빼고 밝기 중앙값을 반환하며 부족하면 None이다."""
    x1, y1, x2, y2 = bbox
    margin_x = max(1, int(round((x2 - x1) * settings['reference_margin_ratio'])))
    margin_y = max(1, int(round((y2 - y1) * settings['reference_margin_ratio'])))
    region = clip_bbox((x1 - margin_x, y1 - margin_y, x2 + margin_x, y2 + margin_y), frame.shape)
    rx1, ry1, rx2, ry2 = region
    mask = np.ones((ry2 - ry1, rx2 - rx1), dtype=bool)
    for excluded in [bbox, *excluded_boxes]:
        box = clip_bbox(excluded, frame.shape)
        if box is None:
            continue
        ex1, ey1 = max(rx1, box[0]), max(ry1, box[1])
        ex2, ey2 = min(rx2, box[2]), min(ry2, box[3])
        if ex1 < ex2 and ey1 < ey2:
            mask[ey1 - ry1:ey2 - ry1, ex1 - rx1:ex2 - rx1] = False
    if np.count_nonzero(mask) < settings['min_reference_pixels']:
        return None
    gray = cv2.cvtColor(frame[ry1:ry2, rx1:rx2], cv2.COLOR_BGR2GRAY)
    return float(np.median(gray[mask]))


def inspect(frame, bbox, *, excluded_boxes=(), **options):
    """원본 BGR 영상과 비상구 xyxy 좌표를 받아 점등·소등·미확인 판독과 수치 근거를 반환한다.

    녹색은 보조 근거다. 표시 면 밝기 Y·밝은 면적·주변 대비를 함께 사용한다.
    초기 기준값은 ROS 파라미터로 조정할 수 있으며 색만으로 실제 전원을 확인하지는 않는다.
    """
    settings = validate_options(**options)
    report = {'bbox': None, 'status': 'UNCONFIRMED', 'reason': 'invalid exit sign image or box',
              'face_polygon': None, 'y_median': None, 'y_p90': None, 'v_median': None,
              'v_p90': None, 'green_ratio': None, 'bright_ratio': None,
              'reference_y': None, 'relative_contrast': None, 'sharpness': None, 'clipped_ratio': None}
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8
            or frame.ndim != 3 or frame.shape[2] != 3):
        return report
    box = clip_bbox(bbox, frame.shape)
    if box is None:
        return report
    report['bbox'] = box
    x1, y1, x2, y2 = box
    if min(x2 - x1, y2 - y1) < settings['min_face_side']:
        report['reason'] = 'exit sign image too small'
        return report
    crop = frame[y1:y2, x1:x2]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    face = _find_face_mask(gray, settings)
    if face is None:
        report['reason'] = 'exit sign display face not found'
        return report
    mask, polygon = face
    selected = mask > 0
    report['face_polygon'] = (polygon + np.array([x1, y1])).tolist()
    if np.count_nonzero(selected) < settings['min_face_pixels']:
        report['reason'] = 'exit sign display face has too few pixels'
        return report
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv, settings['green_lower'], settings['green_upper'])
    values, brightness_v = gray[selected], hsv[:, :, 2][selected]
    reference = _reference_brightness(frame, box, excluded_boxes, settings)
    median = float(np.median(values))
    report.update(y_median=median, y_p90=float(np.percentile(values, 90)),
                  v_median=float(np.median(brightness_v)), v_p90=float(np.percentile(brightness_v, 90)),
                  green_ratio=float(np.mean(green[selected] > 0)),
                  bright_ratio=float(np.mean(values >= settings['bright_y'])),
                  sharpness=float(np.var(cv2.Laplacian(gray, cv2.CV_64F)[selected])),
                  clipped_ratio=float(np.mean(values >= 250)), reference_y=reference)
    if reference is None:
        report['reason'] = 'not enough surrounding pixels for brightness comparison'
        return report
    report['relative_contrast'] = (median - reference) / (median + reference + 1.0)
    if report['sharpness'] < settings['min_sharpness']:
        report['reason'] = 'exit sign display face too blurred or flat'
        return report
    if report['clipped_ratio'] > settings['max_clipped_ratio']:
        report['reason'] = 'exit sign display face largely overexposed'
        return report
    if (median >= settings['on_y_median'] and report['y_p90'] >= settings['on_y_p90']
            and report['bright_ratio'] >= settings['on_bright_ratio']
            and report['green_ratio'] >= settings['min_green_ratio']
            and report['relative_contrast'] >= settings['on_contrast_min']):
        report.update(status='NORMAL', reason='exit sign ON brightness and green-area evidence')
    elif (median <= settings['off_y_median'] and report['y_p90'] <= settings['off_y_p90']
          and report['bright_ratio'] <= settings['off_bright_ratio']
          and report['relative_contrast'] <= settings['off_contrast_max']):
        report.update(status='ABNORMAL', reason='exit sign OFF brightness evidence')
    else:
        report['reason'] = 'exit sign brightness evidence between ON/OFF thresholds'
    return report


def reset_stability_state(state):
    """비상구 확인 이력을 지우되 다음 대상 번호는 보존해 새 표지와 이전 표지를 구별한다."""
    next_id = state.get('next_id', 1)
    state.clear()
    state['next_id'] = next_id


def _bbox_iou(first, second):
    """두 xyxy 박스의 겹침 비율을 반환한다. 좌표가 없거나 넓이가 없으면 0이다."""
    if first is None or second is None:
        return 0.0
    overlap = max(0, min(first[2], second[2]) - max(first[0], second[0])) * max(
        0, min(first[3], second[3]) - max(first[1], second[1]))
    union = ((first[2] - first[0]) * (first[3] - first[1])
             + (second[2] - second[0]) * (second[3] - second[1]) - overlap)
    return overlap / union if union > 0 else 0.0


def unconfirmed_reports(reports, reason):
    """판독 목록과 미확인 사유를 받아 원래 판독은 보존하고 표시 상태를 미확인으로 바꾼 복사본을 반환한다."""
    return [dict(report, raw_status=report['status'], raw_reason=report['reason'],
                 status='UNCONFIRMED', reason=reason, confirm_frames=0, confirm_elapsed_s=0.0)
            for report in reports]


def stabilize_reports(reports, state, frame_time, *, observation_stamp=None,
                      max_gap_seconds=2.0, **options):
    """새 사진의 비상구 판독 목록과 이력 사전을 받아 같은 표지의 연속 확인 결과를 반환한다.

    점등·소등 모두 시간과 유효 프레임 수를 만족해야 확정한다. 확인 중이거나 품질이 나쁘면 회색이다.
    박스 순서 대신 일대일 겹침으로 연결하고 사라진 표지·중복 사진의 과거 판정은 이어 쓰지 않는다.
    frame_time은 단조 증가하는 수신 시각(초), observation_stamp는 촬영 시각(나노초)다.
    """
    settings = validate_options(**options)
    last_time, last_stamp = state.get('last_time'), state.get('last_stamp')
    if not np.isfinite(frame_time) or not np.isfinite(max_gap_seconds) or max_gap_seconds <= 0:
        raise ValueError('exit sign observation time/gap must be finite; gap must be positive')
    if ((last_time is not None and frame_time <= last_time)
            or (observation_stamp is not None and last_stamp is not None and observation_stamp <= last_stamp)):
        reset_stability_state(state)
        state.update(last_time=frame_time, last_stamp=observation_stamp)
        return unconfirmed_reports(reports, 'exit sign duplicate or old camera frame')
    if last_time is not None and frame_time - last_time > max_gap_seconds:
        reset_stability_state(state)
    previous = state.get('tracks', {})
    matches = {index: [track_id for track_id, track in previous.items()
                       if _bbox_iou(report['bbox'], track['bbox']) >= settings['match_iou']]
               for index, report in enumerate(reports)}
    next_id = state.get('next_id', 1)
    tracks, output = {}, []
    for index, report in enumerate(reports):
        candidates = matches[index]
        track_id = candidates[0] if (len(candidates) == 1
                                    and sum(candidates[0] in values for values in matches.values()) == 1) else None
        if track_id is None:
            track_id, next_id = next_id, next_id + 1
            track = {'count': 0, 'since': frame_time, 'raw_status': None}
        else:
            track = dict(previous[track_id])
        raw_status = report['status']
        confirmed = dict(report, raw_status=raw_status, raw_reason=report['reason'], exit_track_id=track_id)
        if raw_status not in ('NORMAL', 'ABNORMAL'):
            track.update(count=0, since=frame_time, raw_status=None)
        elif track['raw_status'] == raw_status:
            track['count'] += 1
        else:
            track.update(count=1, since=frame_time, raw_status=raw_status)
        elapsed = frame_time - track['since'] if track['count'] else 0.0
        confirmed.update(confirm_frames=track['count'], confirm_elapsed_s=elapsed,
                         required_frames=settings['confirm_frames'], required_seconds=settings['confirm_seconds'])
        if (raw_status in ('NORMAL', 'ABNORMAL')
                and (track['count'] < settings['confirm_frames'] or elapsed < settings['confirm_seconds'])):
            confirmed.update(status='UNCONFIRMED', reason=f'exit sign {raw_status} confirmation pending')
        track['bbox'] = report['bbox']
        tracks[track_id] = track
        output.append(confirmed)
    state.update(tracks=tracks, next_id=next_id, last_time=frame_time,
                 last_stamp=observation_stamp if observation_stamp is not None else last_stamp)
    return output
