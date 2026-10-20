"""랙 파이프 자동 감지와 기준 사진 비교로 상자 돌출을 검사하는 함수 모음."""

from pathlib import Path

import cv2
import numpy as np
from scv_msgs.msg import InspectionResult


def _longest_run_per_column(mask):
    """각 열에서 가장 긴 연속 흰 구간의 길이와 시작 행을 구한다."""
    h, w = mask.shape
    lengths = np.zeros(w, dtype=np.int32)
    starts = np.zeros(w, dtype=np.int32)
    for col in range(w):
        column = mask[:, col]
        best_len = 0
        best_start = 0
        run_start = -1
        for row in range(h):
            if column[row] > 0:
                if run_start < 0:
                    run_start = row
            else:
                if run_start >= 0:
                    run_len = row - run_start
                    if run_len > best_len:
                        best_len = run_len
                        best_start = run_start
                    run_start = -1
        if run_start >= 0:
            run_len = h - run_start
            if run_len > best_len:
                best_len = run_len
                best_start = run_start
        lengths[col] = best_len
        starts[col] = best_start
    return lengths, starts


def detect_rack(frame, min_run_ratio=0.30, pipe_max_width=40):
    """흰색 파이프의 수직 연속성으로 랙 좌우 경계를 자동 검출한다. 못 찾으면 None."""
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 170), (180, 65, 255))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_RECT, (1, 25)))
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((7, 3), np.uint8))

    run_lengths, run_starts = _longest_run_per_column(white)

    pipe_threshold = h * min_run_ratio
    pipe_cols = np.where(run_lengths >= pipe_threshold)[0]
    margin = int(w * 0.03)
    pipe_cols = pipe_cols[(pipe_cols > margin) & (pipe_cols < w - margin)]
    if len(pipe_cols) < 2:
        return None

    groups = []
    current = [pipe_cols[0]]
    for i in range(1, len(pipe_cols)):
        if pipe_cols[i] - pipe_cols[i - 1] <= 8:
            current.append(pipe_cols[i])
        else:
            groups.append(np.array(current))
            current = [pipe_cols[i]]
    groups.append(np.array(current))

    scored = []
    for g in groups:
        width = g[-1] - g[0] + 1
        if width > pipe_max_width:
            continue
        mean_run = float(run_lengths[g].mean())
        scored.append({
            'group': g,
            'center': float(np.mean(g)),
            'width': width,
            'max_run': int(run_lengths[g].max()),
            'score': mean_run * (1.0 if 8 <= width <= 30 else 0.5),
            'start': int(np.median(run_starts[g])),
            'end': int(np.median(run_starts[g] + run_lengths[g])),
        })

    best_pair = None
    best_score = 0
    for i in range(len(scored)):
        for j in range(i + 1, len(scored)):
            gi, gj = scored[i], scored[j]
            sep = abs(gj['center'] - gi['center'])
            if sep < 80 or sep > w * 0.65:
                continue
            ratio = min(gi['max_run'], gj['max_run']) / max(gi['max_run'], gj['max_run'])
            if ratio < 0.5 or abs(gi['start'] - gj['start']) > h * 0.15:
                continue
            score = (gi['score'] + gj['score']) * ratio
            if score > best_score:
                best_score = score
                best_pair = (gi, gj)

    if best_pair is None:
        return None

    left = best_pair[0] if best_pair[0]['center'] < best_pair[1]['center'] else best_pair[1]
    right = best_pair[1] if best_pair[0]['center'] < best_pair[1]['center'] else best_pair[0]

    return {
        'left': int(left['center']),
        'right': int(right['center']),
        'top': min(left['start'], right['start']),
        'bottom': max(left['end'], right['end']),
    }


def capture_reference(frame, rack):
    """정상 상태 사진과 랙 위치를 기준으로 저장한다. 랙이 없거나 내부가 비면 None."""
    if rack is None:
        return None
    left, right = rack['left'], rack['right']
    top, bottom = rack['top'], rack['bottom']
    inside = cv2.cvtColor(frame[top:bottom, left:right], cv2.COLOR_BGR2GRAY)
    if float(np.std(inside)) < 8.0:
        return None
    return {'frame': frame.copy(), 'rack': dict(rack)}


def save_reference(reference, path):
    """기준 사진과 랙 좌표를 npz 파일로 저장한다. 다음 실행 때 자동 로드할 수 있다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rack = reference['rack']
    np.savez(str(path), frame=reference['frame'],
             rack_left=rack['left'], rack_right=rack['right'],
             rack_top=rack['top'], rack_bottom=rack['bottom'])


def load_reference(path):
    """npz 파일에서 기준 사진과 랙 좌표를 불러온다. 파일이 없거나 깨지면 None."""
    path = Path(path)
    if not path.is_file():
        return None
    try:
        data = np.load(str(path))
        return {
            'frame': data['frame'],
            'rack': {
                'left': int(data['rack_left']),
                'right': int(data['rack_right']),
                'top': int(data['rack_top']),
                'bottom': int(data['rack_bottom']),
            }
        }
    except Exception:
        return None


def _check_zone(ref_zone, cur_zone, diff_threshold=25, min_area=300):
    """기준과 현재 영역의 BGR 채널별 차이 중 최대값으로 바뀐 윤곽을 찾는다."""
    diff = cv2.absdiff(ref_zone, cur_zone)
    if len(diff.shape) == 3:
        diff = diff.max(axis=2)
    changed = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)[1]
    changed = cv2.morphologyEx(changed, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    changed = cv2.morphologyEx(changed, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(changed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        if w < 8 or h < 15:
            continue
        results.append({'bbox': (x, y, w, h), 'area': area})
    return results


def inspect_rack(frame, rack=None, reference=None, margin=10, zone_width=70,
                 diff_threshold=25, min_area=300):
    """기준 사진과 비교해 랙 좌우 바깥 영역이 바뀌었는지 검사한다."""
    report = {'verdict': InspectionResult.UNCONFIRMED,
              'reason': 'rack not detected',
              'rack': None, 'protrusions': [], 'zones': []}

    if rack is None:
        rack = detect_rack(frame)
    if rack is None:
        return report

    report['rack'] = rack
    if reference is None:
        report['reason'] = 'normal reference missing; press N to save'
        return report

    ref_frame = reference['frame']
    ref_rack = reference['rack']
    if ref_frame.shape != frame.shape:
        report['reason'] = 'frame size changed; press R and re-capture'
        return report

    left_x, right_x = rack['left'], rack['right']
    top, bottom = rack['top'], rack['bottom']
    rack_h = bottom - top
    check_top = top + int(rack_h * 0.12)
    check_bottom = bottom - int(rack_h * 0.04)
    h, w = frame.shape[:2]

    ref_left = ref_rack['left']
    ref_right = ref_rack['right']
    ref_top = ref_rack['top']
    ref_bottom = ref_rack['bottom']
    ref_rack_h = ref_bottom - ref_top
    ref_check_top = ref_top + int(ref_rack_h * 0.12)
    ref_check_bottom = ref_bottom - int(ref_rack_h * 0.04)

    for side in ('left', 'right'):
        if side == 'left':
            cx_s = max(0, left_x - zone_width)
            cx_e = max(0, left_x - margin)
            rx_s = max(0, ref_left - zone_width)
            rx_e = max(0, ref_left - margin)
        else:
            cx_s = min(w, right_x + margin)
            cx_e = min(w, right_x + zone_width)
            rx_s = min(w, ref_right + margin)
            rx_e = min(w, ref_right + zone_width)

        zone_w = min(cx_e - cx_s, rx_e - rx_s)
        zone_h = min(check_bottom - check_top, ref_check_bottom - ref_check_top)
        if zone_w < 10 or zone_h < 30:
            continue

        report['zones'].append((cx_s, check_top, zone_w, zone_h))
        cur_zone = frame[check_top:check_top + zone_h, cx_s:cx_s + zone_w]
        ref_zone = ref_frame[ref_check_top:ref_check_top + zone_h, rx_s:rx_s + zone_w]
        if cur_zone.shape != ref_zone.shape:
            continue

        hits = _check_zone(ref_zone, cur_zone, diff_threshold, min_area)
        for hit in hits:
            bx, by, bw, bh = hit['bbox']
            report['protrusions'].append({
                'side': side,
                'bbox': (bx + cx_s, by + check_top, bw, bh),
                'area': hit['area'],
            })

    if report['protrusions']:
        report.update(verdict=InspectionResult.ABNORMAL,
                      reason='object protruding outside rack boundary')
    else:
        report.update(verdict=InspectionResult.NORMAL,
                      reason='outside area matches normal reference')
    return report


def stabilize_result(report, state, required_frames=3):
    """같은 판정이 여러 번 연속될 때만 확정한다."""
    rack = report.get('rack')
    prev_rack = state.get('rack')
    moved = (rack is not None and prev_rack is not None and
             (abs(rack['left'] - prev_rack['left']) > 15 or
              abs(rack['right'] - prev_rack['right']) > 15))
    verdict = report['verdict']
    if moved or verdict == InspectionResult.UNCONFIRMED:
        state.clear()
    count = state.get('count', 0) + 1 if state.get('verdict') == verdict else 1
    state.update(verdict=verdict, count=count, rack=rack)
    output = dict(report)
    if verdict != InspectionResult.UNCONFIRMED and count < required_frames:
        output.update(verdict=InspectionResult.UNCONFIRMED,
                      reason=f'confirming {count}/{required_frames}')
    return output


def draw_result(frame, report):
    """검사 범위(랙 + 좌우 검사 구역 + 여유)를 노란 사각형으로, 돌출 영역을 빨간 사각형으로 그리고 판정 텍스트를 표시한다."""
    output = frame.copy()
    colors = {InspectionResult.NORMAL: (0, 255, 0), InspectionResult.ABNORMAL: (0, 0, 255)}
    color = colors.get(report['verdict'], (192, 192, 192))
    rack = report.get('rack')
    if rack is not None:
        xs = [rack['left'], rack['right']]
        ys = [rack['top'], rack['bottom']]
        for zx, zy, zw, zh in report.get('zones', []):
            xs += [zx, zx + zw]
            ys += [zy, zy + zh]
        pad = 15
        h, w = output.shape[:2]
        cv2.rectangle(output, (max(0, min(xs) - pad), max(0, min(ys) - pad)),
                      (min(w - 1, max(xs) + pad), min(h - 1, max(ys) + pad)), (0, 255, 255), 2)
    for p in report['protrusions']:
        x, y, pw, ph = p['bbox']
        cv2.rectangle(output, (x, y), (x + pw, y + ph), (0, 0, 255), 3)
    label = {InspectionResult.NORMAL: 'NORMAL', InspectionResult.ABNORMAL: 'ABNORMAL'}.get(
        report['verdict'], 'UNCONFIRMED')
    cv2.putText(output, 'RACK: ' + label, (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    return output


def scale_display_point(x, y, frame_shape, display_size=(960, 540)):
    """창에서 누른 좌표를 원본 카메라 사진 좌표로 환산한다."""
    height, width = frame_shape[:2]
    display_width, display_height = display_size
    if display_width <= 0 or display_height <= 0:
        return None
    point = [round(x * width / display_width), round(y * height / display_height)]
    if point[0] < 0 or point[0] >= width or point[1] < 0 or point[1] >= height:
        return None
    return point
