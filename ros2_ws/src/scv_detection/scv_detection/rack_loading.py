"""YOLO가 찾은 랙과 박스의 경계를 비교하고 같은 대상의 연속 판정을 확인한다.

좌표는 디코딩한 원본 영상의 [x1, y1, x2, y2]다. 좌우·위아래를 독립적으로
검사한다. 카메라 수신·모델 추론·ROS 발행은 매니저가 담당한다.
"""

from __future__ import annotations

import math

import cv2


def validate_options(*, tolerance_ratio=0.0, association_margin=0.10,
                     required_frames=3, match_iou=0.30, match_margin=0.10):
    """경계 여유·소속 모호성·연속 확인·대상 연결 기준을 검사해 설정 사전을 반환한다.

    tolerance_ratio는 랙 너비/높이에 대한 허용 초과 비율이다. 0이면 경계 초과를
    모두 후보로 잡는다. association_margin은 두 랙의 박스 겹침 비율 차이 기준이다.
    match_margin은 이전 대상을 연결할 때 가장 좋은 IoU와 두 번째 IoU의 최소 차이다.
    """
    if not math.isfinite(tolerance_ratio) or not 0 <= tolerance_ratio < 0.5:
        raise ValueError('rack_loading_tolerance_ratio must satisfy 0 <= ratio < 0.5')
    if not math.isfinite(association_margin) or not 0 <= association_margin < 1:
        raise ValueError('rack_loading_association_margin must satisfy 0 <= margin < 1')
    if not math.isfinite(required_frames) or required_frames < 1 or required_frames != int(required_frames):
        raise ValueError('rack_loading_confirm_frames must be a positive integer')
    if not math.isfinite(match_iou) or not 0 < match_iou <= 1:
        raise ValueError('rack_loading_match_iou must satisfy 0 < IoU <= 1')
    if not math.isfinite(match_margin) or not 0 <= match_margin < 1:
        raise ValueError('rack_loading_match_margin must satisfy 0 <= margin < 1')
    return {'tolerance_ratio': float(tolerance_ratio), 'association_margin': float(association_margin),
            'required_frames': int(required_frames), 'match_iou': float(match_iou),
            'match_margin': float(match_margin)}


def _detection_record(detection, image_shape):
    """검출 좌표를 원본 영상 안으로 제한하고, 가장자리에서 잘릴 가능성도 기록한다.

    입력 검출에는 box_index, bbox, confidence가 있다. 잘못된 좌표는 None으로 반환한다.
    소수점 좌표를 유지하므로 정수 반올림 때문에 경계를 넘었다고 판단하지 않는다.
    """
    values = detection['bbox']
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        return None
    height, width = image_shape[:2]
    x1, y1, x2, y2 = (float(value) for value in values)
    clipped = [max(0.0, x1), max(0.0, y1), min(float(width), x2), min(float(height), y2)]
    if clipped[0] >= clipped[2] or clipped[1] >= clipped[3]:
        return None
    return {'box_index': int(detection['box_index']), 'bbox': clipped,
            'confidence': float(detection['confidence']),
            'touches_image_edge': x1 <= 0 or y1 <= 0 or x2 >= width or y2 >= height}


def _intersection_area(a, b):
    """두 [x1,y1,x2,y2] 박스가 겹치는 면적을 반환한다. 겹치지 않으면 0이다."""
    return max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))


def _bbox_iou(a, b):
    """두 박스의 겹침 면적을 전체 합친 면적으로 나눠 같은 대상 연결에 쓸 비율을 구한다."""
    overlap = _intersection_area(a, b)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - overlap
    return overlap / union if union > 0 else 0.0


def _overflow(box, rack, tolerance_ratio):
    """박스가 랙의 네 경계를 넘는 픽셀 수·랙 크기 대비 비율·초과 방향을 반환한다."""
    width, height = rack[2] - rack[0], rack[3] - rack[1]
    pixels = {'left': max(0.0, rack[0] - box[0]), 'right': max(0.0, box[2] - rack[2]),
              'top': max(0.0, rack[1] - box[1]), 'bottom': max(0.0, box[3] - rack[3])}
    ratios = {side: value / (width if side in ('left', 'right') else height)
              for side, value in pixels.items()}
    return pixels, ratios, [side for side, ratio in ratios.items() if ratio > tolerance_ratio]


def _rack_verdict(report):
    """확인된 불량 박스가 하나라도 있으면 랙 불량, 모든 연결 박스가 정상이면 랙 정상이다.

    랙이 잘렸거나 소속이 모호하거나 연결 박스가 없으면 미확인이다.
    미확인 박스와 확실한 불량이 함께 있으면 알려진 불량을 우선한다.
    불량이 없고 미확인 박스가 있으면 그 박스의 실제 보류 이유를 반환한다.
    """
    if report['touches_image_edge']:
        return 'UNCONFIRMED', 'rack touches image edge; loading boundary incomplete'
    if any(box['status'] == 'ABNORMAL' for box in report['boxes']):
        return 'ABNORMAL', 'box extends outside rack boundary'
    if report['ambiguous_box_indices']:
        return 'UNCONFIRMED', 'box belongs to multiple rack candidates'
    if not report['boxes']:
        return 'UNCONFIRMED', 'no associated box detected'
    pending = next((box for box in report['boxes'] if box['status'] == 'UNCONFIRMED'), None)
    if pending is not None:
        return 'UNCONFIRMED', pending['reason']
    return 'NORMAL', 'all associated boxes inside rack boundary'


def inspect_loading(racks, boxes, image_shape, **options):
    """랙·박스 검출 목록을 받아 소속과 경계 초과를 판정한 racks/boxes 사전을 반환한다.

    박스 면적 중 랙에 겹친 비율이 가장 높은 랙을 고른다. 상위 후보의 차이가
    작으면 소속을 보류한다. 겹침이 전혀 없는 박스는 다른 랙의 불량으로 처리하지 않는다.
    반환 상태는 이번 사진만의 후보이며 연속 확인은 stabilize_result에서 처리한다.
    """
    settings = validate_options(**options)
    rack_reports = []
    for detection in racks:
        record = _detection_record(detection, image_shape)
        if record is not None:
            rack_reports.append(dict(record, status='UNCONFIRMED', reason='no associated box detected',
                                     boxes=[], ambiguous_box_indices=[],
                                     tolerance_ratio=settings['tolerance_ratio']))
    box_reports = []
    for detection in boxes:
        record = _detection_record(detection, image_shape)
        if record is None:
            continue
        box = dict(record, status='UNCONFIRMED', reason='box not associated with a detected rack',
                   rack_box_index=None, rack_bbox=None, candidate_rack_indices=[],
                   association_overlap_ratio=None, overflow_px={}, overflow_ratio={}, directions=[])
        box_reports.append(box)
        bbox = box['bbox']
        area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        candidates = sorted(((overlap / area, index) for index, rack in enumerate(rack_reports)
                             if (overlap := _intersection_area(bbox, rack['bbox'])) > 0), reverse=True)
        if not candidates:
            continue
        score, rack_index = candidates[0]
        if len(candidates) > 1 and score - candidates[1][0] <= settings['association_margin']:
            box['reason'] = 'box belongs to multiple rack candidates'
            for other_score, index in candidates:
                if score - other_score <= settings['association_margin']:
                    rack_reports[index]['ambiguous_box_indices'].append(box['box_index'])
                    box['candidate_rack_indices'].append(rack_reports[index]['box_index'])
            continue
        rack = rack_reports[rack_index]
        box.update(rack_box_index=rack['box_index'], rack_bbox=rack['bbox'],
                   association_overlap_ratio=score)
        pixels, ratios, directions = _overflow(bbox, rack['bbox'], settings['tolerance_ratio'])
        box.update(overflow_px=pixels, overflow_ratio=ratios, directions=directions)
        if rack['touches_image_edge']:
            box['reason'] = 'rack touches image edge; loading boundary incomplete'
        elif box['touches_image_edge']:
            box['reason'] = 'box touches image edge; loading boundary incomplete'
        else:
            box.update(status='ABNORMAL' if directions else 'NORMAL',
                       reason='box extends outside rack boundary' if directions else 'box inside rack boundary')
        rack['boxes'].append(box)
    for rack in rack_reports:
        rack['status'], rack['reason'] = _rack_verdict(rack)
    return {'racks': rack_reports, 'boxes': box_reports}


def reset_stability_state(state):
    """연속 확인 이력을 지우고 다음 랙·박스 번호는 보존해 예전 대상과 새 대상을 구별한다."""
    next_rack_id, next_box_id = state.get('next_rack_id', 1), state.get('next_box_id', 1)
    state.clear()
    state.update(next_rack_id=next_rack_id, next_box_id=next_box_id)


def _best_match(candidates, match_margin):
    """(겹침 점수, 대상 번호) 후보에서 가장 좋은 번호를 고른다. 비슷한 후보가 있으면 None이다."""
    ranked = sorted(candidates, reverse=True)
    if not ranked or (len(ranked) > 1 and ranked[0][0] - ranked[1][0] <= match_margin):
        return None
    return ranked[0][1]


def _unique_matches(records, previous, match_iou, match_margin):
    """현재와 이전 대상이 서로 가장 잘 맞는 경우만 일대일로 연결한다.

    다른 박스와 조금 겹쳐도 가장 좋은 IoU가 충분히 뚜렷하면 연결을 유지한다.
    상위 두 점수가 비슷하거나 여러 현재 대상이 같은 이전 대상을 원하면 확실한 쌍만 남긴다.
    """
    scores = {index: {track_id: score for track_id, track in previous.items()
                      if (score := _bbox_iou(record['bbox'], track['bbox'])) >= match_iou}
              for index, record in enumerate(records)}
    current_best = {index: _best_match(((score, track_id) for track_id, score in candidates.items()), match_margin)
                    for index, candidates in scores.items()}
    previous_best = {track_id: _best_match(((candidates[track_id], index) for index, candidates in scores.items()
                                          if track_id in candidates), match_margin) for track_id in previous}
    return {index: track_id for index, track_id in current_best.items()
            if track_id is not None and previous_best[track_id] == index}


def _confirm_box(box, track, required_frames):
    """같은 박스의 정상 또는 불량 후보가 필요한 횟수만큼 연속될 때 확정한 복사본을 반환한다."""
    output = dict(box, raw_status=box['status'], raw_reason=box['reason'])
    candidate = box['status']
    if candidate == 'UNCONFIRMED':
        count = 0
    else:
        count = track.get('count', 0) + 1 if track.get('candidate') == candidate else 1
        if count < required_frames:
            output.update(status='UNCONFIRMED', reason=f'confirming loading {count}/{required_frames}')
    track.update(bbox=box['bbox'], candidate=candidate, count=count)
    output.update(box_track_id=track['track_id'], confirm_frames=count, required_frames=required_frames)
    return output


def _confirm_rack(rack, track, state, settings):
    """계속 연결되는 박스의 확인 횟수는 유지하고 신규·모호·누락 대상만 이력을 새로 센다.

    다른 박스가 나타나거나 사라져도 확실한 불량 박스의 확인을 막지 않는다.
    랙을 정상으로 바꾸려면 같은 구성의 박스들이 연속으로 정상이어야 한다.
    """
    previous = track.get('boxes', {})
    matches = _unique_matches(rack['boxes'], previous, settings['match_iou'], settings['match_margin'])
    box_tracks, outputs = {}, []
    for index, box in enumerate(rack['boxes']):
        if index in matches:
            box_track_id = matches[index]
            box_track = dict(previous[box_track_id])
        else:
            box_track_id = state.get('next_box_id', 1)
            state['next_box_id'] = box_track_id + 1
            box_track = {'track_id': box_track_id}
        confirmed = _confirm_box(box, box_track, settings['required_frames'])
        confirmed['tracking_reason'] = 'matched' if index in matches else 'new or ambiguous target'
        outputs.append(confirmed)
        box_tracks[box_track_id] = box_track
    output = dict(rack, raw_status=rack['status'], raw_reason=rack['reason'],
                  rack_track_id=track['track_id'], boxes=outputs)
    output['status'], output['reason'] = _rack_verdict(output)
    members = tuple(sorted(box_tracks))
    if rack['status'] == 'NORMAL':
        normal_count = track.get('normal_count', 0) + 1 if track.get('normal_members') == members else 1
    else:
        normal_count = 0
    if output['status'] == 'NORMAL' and normal_count < settings['required_frames']:
        output.update(status='UNCONFIRMED',
                      reason=f'confirming rack normal membership {normal_count}/{settings["required_frames"]}')
    output.update(normal_confirm_frames=normal_count, required_frames=settings['required_frames'])
    track.update(bbox=rack['bbox'], boxes=box_tracks, normal_count=normal_count,
                 normal_members=members if normal_count else None)
    return output


def stabilize_result(reading, state, frame_time, *, observation_stamp=None,
                     max_gap_seconds=2.0, **options):
    """새 사진에서 같은 랙·박스가 연속 확인된 결과를 반환하고 입력 state를 갱신한다.

    reading은 inspect_loading의 반환값, frame_time은 단조 증가하는 수신 시각(초)다.
    사라진 대상의 이력은 버린다. 영상 단절·중복·역순 촬영 시각도 정상 확인에 이어 쓰지 않는다.
    """
    settings = validate_options(**options)
    if not math.isfinite(frame_time) or not math.isfinite(max_gap_seconds) or max_gap_seconds <= 0:
        raise ValueError('finite frame time and positive camera gap required')
    last_time, last_stamp = state.get('last_time'), state.get('last_stamp')
    if last_time is not None and frame_time - last_time > max_gap_seconds:
        reset_stability_state(state)
        last_time, last_stamp = None, None
    if ((last_time is not None and frame_time <= last_time)
            or (observation_stamp is not None and last_stamp is not None and observation_stamp <= last_stamp)):
        reset_stability_state(state)
        state.update(last_time=max(frame_time, last_time) if last_time is not None else frame_time,
                     last_stamp=last_stamp)
        return unconfirmed_result(reading, 'duplicate or older loading frame')
    previous = state.get('racks', {})
    matches = _unique_matches(reading['racks'], previous, settings['match_iou'], settings['match_margin'])
    tracks, racks = {}, []
    for index, rack in enumerate(reading['racks']):
        if index in matches:
            track_id = matches[index]
            track = dict(previous[track_id])
        else:
            track_id = state.get('next_rack_id', 1)
            state['next_rack_id'] = track_id + 1
            track = {'track_id': track_id}
        racks.append(_confirm_rack(rack, track, state, settings))
        tracks[track_id] = track
    boxes = {box['box_index']: box for rack in racks for box in rack['boxes']}
    result = {'racks': racks, 'boxes': [boxes.get(box['box_index'], dict(box, raw_status=box['status'],
                                                                             raw_reason=box['reason']))
                                      for box in reading['boxes']]}
    state.update(racks=tracks, last_time=frame_time,
                 last_stamp=observation_stamp if observation_stamp is not None else last_stamp)
    return result


def unconfirmed_result(reading, reason):
    """오래된 사진의 랙·박스 판정을 미확인 복사본으로 바꾸고 후보 판정은 raw_status에 남긴다.

    reading은 racks/boxes 목록이고 reason은 보류 이유다. 내부 박스도 복사해 원래 판독을 보존한다.
    """
    boxes = {box['box_index']: dict(box, raw_status=box['status'], raw_reason=box['reason'],
                                   status='UNCONFIRMED', reason=reason) for box in reading['boxes']}
    racks = [dict(rack, raw_status=rack['status'], raw_reason=rack['reason'],
                  status='UNCONFIRMED', reason=reason,
                  boxes=[boxes[box['box_index']] for box in rack['boxes']]) for rack in reading['racks']]
    return {'racks': racks, 'boxes': list(boxes.values())}


def draw_result(frame, reading):
    """원본 표시 사진에 경계 초과 띠와 픽셀 수를 그린다. 입력 frame을 직접 수정한다.

    좌우·위아래 초과를 모두 표시하며 미확인은 회색, 확정 불량은 빨간색이다.
    연속 확인 중인 박스에는 횟수와 연결 상태를 표시해 보류되는 이유를 알 수 있게 한다.
    반환값은 없고 reading의 확정 상태는 화면과 불량 발행에서 함께 사용한다.
    """
    colors = {'NORMAL': (0, 255, 0), 'ABNORMAL': (0, 0, 255), 'UNCONFIRMED': (192, 192, 192)}
    for box in reading['boxes']:
        x1, y1, x2, y2 = box['bbox']
        color = colors[box['status']]
        details = []
        if box['rack_bbox'] is not None and box['directions']:
            rx1, ry1, rx2, ry2 = box['rack_bbox']
            strips = {'left': (x1, y1, min(x2, rx1), y2), 'right': (max(x1, rx2), y1, x2, y2),
                      'top': (x1, y1, x2, min(y2, ry1)), 'bottom': (x1, max(y1, ry2), x2, y2)}
            for side in box['directions']:
                sx1, sy1, sx2, sy2 = (int(round(value)) for value in strips[side])
                cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), color, 2)
                details.append(f"{side.upper()}={box['overflow_px'][side]:.1f}px")
        text = 'Overflow ' + ' '.join(details) if details else ''
        if box['status'] == 'UNCONFIRMED' and 'required_frames' in box:
            if box.get('raw_status') in ('NORMAL', 'ABNORMAL'):
                progress = f"confirm {box['confirm_frames']}/{box['required_frames']}"
                progress += ' matched' if box.get('tracking_reason') == 'matched' else ' new/ambiguous'
            else:
                progress = box['reason']
            text += (' | ' if text else 'Loading ') + progress
        if text:
            cv2.putText(frame, text, (int(x1), min(frame.shape[0] - 6, int(y2) + 16)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv2.LINE_AA)
    for rack in reading['racks']:
        if rack['status'] == 'UNCONFIRMED':
            x1, y1, _, y2 = rack['bbox']
            cv2.putText(frame, rack['reason'], (int(x1) + 4, min(int(y2) - 6, int(y1) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors['UNCONFIRMED'], 1, cv2.LINE_AA)
