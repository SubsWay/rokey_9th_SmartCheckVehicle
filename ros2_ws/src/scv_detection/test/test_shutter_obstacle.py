"""방화셔터 구역의 기본 장애물 높이(7cm)가 10cm 물체는 잡고 판지 바닥 오차는 무시하는지 확인하는 시험."""

import cv2
import numpy as np

from scv_detection import shutter_obstacle


def _scene(box_height_m=None, noise_m=0.012, seed=0, box_rows=(280, 330), box_cols=(280, 360)):
    """카메라가 바닥을 비스듬히 보는 가상 장면을 만든다.

    반환값은 (BGR 영상, 미터 Depth, 픽셀 방향, 카메라 기준 위쪽 방향)이다.
    box_height_m을 주면 box_rows·box_cols(화면 행·열 범위, 기본은 구역 가운데)에 그 높이의 상자 윗면을 Depth로 넣는다.
    실제 상자처럼 윗면의 가까운 모서리에서 바닥까지 내려오는 세로 앞면도 넣는다(윗면만 있으면 공중에 뜬 판이 된다).
    """
    height, width, focal = 480, 640, 518.0
    matrix = (focal, 0.0, 320.0, 0.0, focal, 240.0, 0.0, 0.0, 1.0)
    calibration = {'width': width, 'height': height, 'k': matrix, 'p': matrix[:3] + (0.0,) + matrix[3:6] + (0.0,)
                   + matrix[6:] + (0.0,), 'r': (1, 0, 0, 0, 1, 0, 0, 0, 1), 'd': (0, 0, 0, 0, 0),
                   'distortion_model': 'plumb_bob', 'frame_id': 'camera', 'cropped_or_binned': False}
    rays = shutter_obstacle.camera_rays((height, width), calibration)
    tilt = np.deg2rad(20)
    up = np.array([0.0, -np.cos(tilt), -np.sin(tilt)])
    camera_height = 0.25
    along_down = rays @ -up
    depth = np.where(along_down > 1e-3, camera_height / np.maximum(along_down, 1e-6), 0.0)
    frame = np.full((height, width, 3), (150, 170, 180), np.uint8)
    corners = np.array([[150, 250], [490, 250], [600, 460], [40, 460]])
    cv2.polylines(frame, [corners], True, (30, 30, 200), 8)
    if box_height_m is not None:
        box = np.zeros((height, width), bool)
        # 윗면의 바닥 투영이 가까운 변의 안쪽 띠(obstacle_boundary_inset_m)에 걸리지 않게 조금 멀리 둔다.
        box[box_rows[0]:box_rows[1], box_cols[0]:box_cols[1]] = True
        top = camera_height - box_height_m
        depth[box] = top / along_down[box]
        # 앞면: 윗면 가까운 모서리(box_rows 마지막 행)를 지나고 앞쪽 수평 방향을 법선으로 하는 세로 면이다.
        forward = np.array([0.0, 0.0, 1.0]) - up * up[2]
        forward /= np.linalg.norm(forward)
        edge_row, columns = box_rows[1] - 1, slice(*box_cols)
        edge = rays[edge_row, columns] * depth[edge_row, columns, None]
        face_t = (edge @ forward)[None, :] / np.maximum(rays[edge_row + 1:, columns] @ forward, 1e-6)
        face_height = camera_height + face_t * (rays[edge_row + 1:, columns] @ up)
        below = depth[edge_row + 1:, columns]
        on_face = (face_height >= 0) & (face_t < below)
        below[on_face] = face_t[on_face]
    depth = depth + np.random.default_rng(seed).normal(0, noise_m, depth.shape)
    return frame, depth, rays, up


def test_default_height_detects_ten_centimeter_box():
    """요구사항 크기인 10cm 상자는 기본 설정에서 장애물로 판정한다."""
    report = shutter_obstacle.inspect(*_scene(box_height_m=0.10))
    assert report['status'] == 'ABNORMAL' and report['obstacles']
    assert max(item['height_p90_m'] for item in report['obstacles']) > 0.08


def test_default_height_ignores_floor_depth_error():
    """판지 바닥 녹화에서 본 12mm 수준 Depth 오차만 있는 바닥은 장애물로 판정하지 않는다."""
    report = shutter_obstacle.inspect(*_scene())
    assert report['obstacles'] == [] and report['status'] == 'NORMAL'
    assert report['floor_pixel_ratio'] > 0.9


def test_thick_near_tape_is_kept_as_zone_boundary():
    """카메라 가까이에서 약 25px 폭으로 굵게 보이는 테두리 테이프도 넓은 빨간 면으로 지우지 않는다."""
    frame = np.full((480, 640, 3), (150, 170, 180), np.uint8)
    corners = np.array([[230, 300], [600, 300], [630, 470], [60, 470]])
    cv2.polylines(frame, [corners[:2]], False, (30, 30, 200), 6)
    cv2.polylines(frame, [corners[1:3]], False, (30, 30, 200), 10)
    cv2.polylines(frame, [corners[2:]], False, (30, 30, 200), 18)
    cv2.line(frame, tuple(corners[3]), tuple(corners[0]), (30, 30, 200), 25)
    target = cv2.contourArea(corners.astype(np.float32))
    zone = shutter_obstacle._find_zone(frame, shutter_obstacle.validate_options())
    old_zone = shutter_obstacle._find_zone(frame, shutter_obstacle.validate_options(max_tape_width_ratio=0.035))
    # 기본값에서는 왼쪽 굵은 선까지 구역에 들어가고, 예전 17px 기준에서는 그 선이 지워져 구역이 작아진다.
    assert zone is not None and cv2.contourArea(zone[0].astype(np.float32)) > 0.8 * target
    assert old_zone is None or cv2.contourArea(old_zone[0].astype(np.float32)) < 0.6 * target


def test_unpaired_reading_keeps_confirmation_history():
    """짝 없는 판독이 사이에 끼어도 같은 구역의 ABNORMAL 확인 횟수가 이어져 확정되는지 확인한다."""
    polygon = [[0, 0], [100, 0], [100, 100], [0, 100]]
    state = {}

    def abnormal():
        """구역 가운데 장애물 하나가 있는 ABNORMAL 판독을 만든다."""
        return dict(shutter_obstacle.unconfirmed('x'), status='ABNORMAL', reason='obstacle_above_floor_inside_zone',
                    zone_polygon=polygon, obstacles=[{'bbox': [40, 40, 60, 60]}], obstacles_evaluated=True)

    shutter_obstacle.stabilize_report(abnormal(), state, 0.0, 1)
    skipped = shutter_obstacle.stabilize_report(shutter_obstacle.unconfirmed('rgb_depth_time_mismatch'), state, 0.1, 0)
    assert skipped['status'] == 'UNCONFIRMED' and state['count'] == 1
    geometry_fail = dict(shutter_obstacle.unconfirmed('zone_plane_intersection_invalid'), zone_polygon=polygon)
    shutter_obstacle.stabilize_report(geometry_fail, state, 0.15, 0)
    assert state['count'] == 1
    shutter_obstacle.stabilize_report(abnormal(), state, 0.3, 2)
    final = shutter_obstacle.stabilize_report(abnormal(), state, 0.6, 3)
    assert final['status'] == 'ABNORMAL' and final['confirm_frames'] == 3


def test_slightly_changed_zone_keeps_history():
    """잘린 구역의 모양이 조금 바뀌어(겹침 약 40%) 도 기본 기준 30%에서는 이력이 유지되는지 확인한다."""
    first = np.array([[0, 0], [100, 0], [100, 100], [0, 100]])
    second = np.array([[0, 0], [100, 0], [100, 40], [0, 40]])
    assert shutter_obstacle._same_zone(first, second, 0.30)
    assert not shutter_obstacle._same_zone(first, second, 0.50)


def test_obstacle_candidates_exclude_boundary_band():
    """경계에서 안쪽 띠 안에 있는 점은 음수 허용값으로 구역 밖 취급되는지 확인한다."""
    polygon = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    points = np.array([[0.5, 0.98], [0.5, 0.5]])
    assert shutter_obstacle._inside_polygon(points, polygon, 0.0).tolist() == [True, True]
    assert shutter_obstacle._inside_polygon(points, polygon, -0.05).tolist() == [False, True]


def test_scale_depth_calibration_moves_focal_and_center():
    """화면 배율·이동이 K와 P의 초점 거리·주점에 반영되고, 1·0 이면 원본을 그대로 돌려주는지 확인한다."""
    calibration = {'k': (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0),
                   'p': (500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0), 'frame_id': 'cam'}
    assert shutter_obstacle.scale_depth_calibration(calibration) is calibration
    scaled = shutter_obstacle.scale_depth_calibration(calibration, 0.5, 100.0, 2.0, -10.0)
    assert scaled['k'][0] == 250.0 and scaled['k'][2] == 260.0 and scaled['k'][4] == 1000.0 and scaled['k'][5] == 470.0
    assert scaled['p'][0] == 250.0 and scaled['p'][2] == 260.0 and scaled['p'][5] == 1000.0 and scaled['p'][6] == 470.0
    assert scaled['frame_id'] == 'cam' and calibration['k'][0] == 500.0


def test_fixed_plane_skips_ransac_but_still_detects_box():
    """미리 잰 바닥 평면을 주면 RANSAC 없이도 10cm 상자를 잡고 빈 바닥은 정상으로 판정하며, 틀린 평면은 거부한다."""
    frame, depth, rays, up = _scene(box_height_m=0.10)
    true_plane = [*up, 0.25]
    report = shutter_obstacle.inspect(frame, depth, rays, up, fixed_plane=true_plane)
    assert report['status'] == 'ABNORMAL' and report['floor_diagnostics']['plane_source'] == 'fixed_plane'
    empty = shutter_obstacle.inspect(*_scene(), fixed_plane=true_plane)
    assert empty['status'] == 'NORMAL' and empty['obstacles'] == []
    wrong = shutter_obstacle.inspect(*_scene(), fixed_plane=[*up, 0.50])
    assert wrong['status'] == 'UNCONFIRMED' and wrong['reason'] == 'floor_not_confirmed'
    assert wrong['floor_diagnostics']['plane_source'] == 'fixed_plane_rejected'


def test_display_bbox_covers_object_outside_zone():
    """구역 밖으로 이어진 상자 윗부분까지 표시용 사각형에 포함되고, 판정용 bbox 는 구역 근처에 머무는지 확인한다.

    판정은 선 위에 선 물체도 잡도록 벽이 아닌 변에서 테이프 폭(짧은 변의 2%, 이 장면 10px)만큼 바깥까지 허용한다.
    먼 쪽 선은 y=250 에 두께 8px 로 그려 바깥 가장자리가 약 246 이므로 판정 bbox 위 끝은 246 - 10 = 236 이상이다.
    """
    frame, depth, rays, up = _scene(box_height_m=0.10)
    # 같은 높이·거리의 상자 면을 구역 위쪽(빨간 테두리 밖, 행 200~280)까지 이어 붙인다.
    box_rows = slice(200, 330)
    along_down = rays @ -up
    extended = np.zeros(depth.shape, bool)
    extended[box_rows, 280:360] = True
    depth[extended] = (0.25 - 0.10) / along_down[extended]
    report = shutter_obstacle.inspect(frame, depth, rays, up)
    assert report['status'] == 'ABNORMAL' and report['obstacles']
    obstacle = report['obstacles'][0]
    assert obstacle['bbox'][1] >= 236 and obstacle['display_bbox'][1] < obstacle['bbox'][1]
    assert obstacle['display_bbox'][2] >= obstacle['bbox'][2] and obstacle['display_bbox'][3] >= obstacle['bbox'][3]


def test_vertical_consistency_drops_thin_band_but_keeps_surface():
    """6줄짜리 가까운 깊이 띠는 걸러지고, 20줄짜리 실제 면과 평탄한 바닥은 통과하는지 확인한다."""
    depth = np.full((120, 40), 2.0, dtype=np.float32)
    depth[50:56, :] = 1.3   # 상자 밑단 가장자리 잡음 같은 얇은 띠
    depth[80:100, :] = 1.5  # 실제 물체 면
    depth[10:14, 5] = np.nan  # 없는 깊이는 판단에서 제외
    mask = shutter_obstacle._vertically_consistent(depth, 0.10)
    assert not mask[50:56, 20].any()
    assert mask[82:98, 20].all() and mask[20:45, 20].all()
    assert mask[10:14, 5].all()


def _reference_vertical_check(depth, tolerance, rows=21):
    """속도 개선 전 방식(np.nanmedian)으로 세로 중앙값 검사를 그대로 계산한 기준 답을 반환한다."""
    import warnings
    values = np.where(np.isfinite(depth) & (depth > 0), depth, np.nan).astype(np.float32)
    half = rows // 2
    padded = np.pad(values, ((half, half), (0, 0)), constant_values=np.nan)
    stack = np.stack([padded[offset:offset + values.shape[0]] for offset in range(rows)])
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        median = np.nanmedian(stack, axis=0)
    enough = np.count_nonzero(np.isfinite(stack), axis=0) >= (rows + 1) // 2
    return ~np.isfinite(values) | ~enough | ~np.isfinite(median) | (np.abs(values - median) <= tolerance)


def test_fast_vertical_check_matches_nanmedian_with_holes():
    """빠르게 바꾼 세로 중앙값 검사가 빈 Depth가 섞여도 예전 nanmedian 방식과 픽셀마다 같은지 확인한다.

    where로 일부 픽셀만 검사할 때는 그 픽셀의 답이 같고, 나머지는 통과(참)여야 한다.
    """
    rng = np.random.default_rng(3)
    depth = 1.0 + rng.normal(0, 0.05, (120, 90))
    depth[rng.random(depth.shape) < 0.25] = 0.0  # 빈 값을 많이 섞어 짝수 개 중앙값 경우도 만든다.
    depth[30:36, 10:30] = 0.6  # 가까운 얇은 띠
    depth[:, 50] = np.nan
    expected = _reference_vertical_check(depth, 0.10)
    assert np.array_equal(shutter_obstacle._vertically_consistent(depth, 0.10), expected)
    where = rng.random(depth.shape) < 0.3
    partial = shutter_obstacle._vertically_consistent(depth, 0.10, where=where)
    assert np.array_equal(partial[where], expected[where]) and partial[~where].all()


def test_focal_override_changes_only_focal_length():
    """초점 거리 덮어쓰기는 K·P의 fx·fy만 바꾸고 주점·왜곡은 그대로 두며, 0이면 원래 보정값을 그대로 돌려준다."""
    calibration = {'k': (569.95, 0, 353.4, 0, 569.95, 354.1, 0, 0, 1),
                   'p': (569.95, 0, 353.4, 0, 0, 569.95, 354.1, 0, 0, 0, 1, 0),
                   'd': (0.1, 0.2, 0, 0, 0), 'width': 704, 'height': 704}
    assert shutter_obstacle.override_focal_length(calibration, 0) is calibration
    changed = shutter_obstacle.override_focal_length(calibration, 1036.0)
    k = np.reshape(changed['k'], (3, 3)); p = np.reshape(changed['p'], (3, 4))
    assert k[0, 0] == k[1, 1] == p[0, 0] == p[1, 1] == 1036.0
    assert (k[0, 2], k[1, 2], p[0, 2], p[1, 2]) == (353.4, 354.1, 353.4, 354.1)
    assert changed['d'] == calibration['d'] and calibration['k'][0] == 569.95


def test_camera_height_check_suggests_focal_from_tf_height():
    """바닥 평면 높이가 TF 높이의 1.77배면 비율 1.77과 그만큼 큰 제안 초점 거리를 돌려준다."""
    check = shutter_obstacle.camera_height_check([0.0, -1.0, 0.0, 0.432], 0.244, 569.95)
    assert abs(check['camera_height_ratio'] - 0.432 / 0.244) < 1e-9
    assert abs(check['suggested_focal_px'] - 569.95 * 0.432 / 0.244) < 1e-6
    empty = shutter_obstacle.camera_height_check(None, 0.244, 569.95)
    assert empty['camera_height_ratio'] is None and empty['suggested_focal_px'] is None


def test_expected_red_state_separates_seen_occluded_and_missing():
    """이전에 보이던 빨간 선 픽셀을 '지금도 보임 / 물체에 가려짐 / 바닥이 보이는데 사라짐'으로 나눈다.

    카메라가 바닥 0.25m 위에서 수평으로 보는 가상 장면이다. 선의 왼쪽 1/3은 그대로 보이고, 가운데 1/3은
    바닥(그 행에서 1.0m)보다 가까운 물체(Depth 0.6m)가 덮고, 오른쪽 1/3은 선만 지워져 바닥 Depth가 그대로 보인다.
    """
    height, width, focal = 200, 300, 200.0
    ys, xs = np.indices((height, width), dtype=float)
    rays = np.dstack(((xs - 150) / focal, (ys - 100) / focal, np.ones((height, width))))
    plane = np.array([0.0, -1.0, 0.0, 0.25])  # 높이 = 0.25 - y, 바닥은 카메라 아래 0.25m
    floor_depth = np.where(rays[..., 1] > 1e-6, 0.25 / np.maximum(rays[..., 1], 1e-6), 0.0)
    expected = np.zeros((height, width), bool)
    expected[150:154, 30:270] = True  # 이전에 보이던 가로 테이프
    red = np.zeros((height, width), np.uint8)
    red[150:154, 30:110] = 255  # 왼쪽 1/3만 지금도 빨갛게 보임
    depth = floor_depth.copy()
    depth[120:160, 110:190] = 0.6  # 가운데 1/3을 덮는 가까운 물체(그 자리 바닥은 약 1.0m)
    settings = shutter_obstacle.validate_options()
    state = shutter_obstacle._expected_red_state(expected, red, settings, depth, rays, plane)
    assert 0.25 < state['seen'] < 0.45 and 0.25 < state['occluded'] < 0.40 and 0.25 < state['missing'] < 0.40
    # 바닥 평면을 모르면 가림을 판단하지 않으므로 보이지 않는 선은 모두 사라짐으로 센다.
    unknown = shutter_obstacle._expected_red_state(expected, red, settings)
    assert unknown['occluded'] == 0 and abs(unknown['seen'] + unknown['missing'] - 1) < 1e-9


def test_one_side_line_fills_hidden_opposite_side():
    """좌우 긴 선 중 오른쪽 선만 보이면 띠 폭(0.8m)만큼 떨어진 왼쪽 선 위치까지 구역을 채운다.

    카메라가 바닥 0.25m 위에서 수평으로 보는 가상 장면이다. 바닥 좌표(옆, 앞)로 오른쪽 선(옆 +0.4m)과
    띠 안쪽 사선 두 개를 화면에 그린다. 왼쪽 선(옆 -0.4m)은 물체에 가려졌다고 보고 그리지 않는다.
    """
    height, width, focal = 240, 320, 300.0
    ys, xs = np.indices((height, width), dtype=float)
    rays = np.dstack(((xs - 160) / focal, (ys - 120) / focal, np.ones((height, width))))
    plane = np.array([0.0, -1.0, 0.0, 0.25])
    red = np.zeros((height, width), np.uint8)

    def draw(start, end):
        """바닥 좌표 두 점 사이 선을 화면 빨간 마스크에 굵게 그린다."""
        pts = shutter_obstacle._project_floor_points(np.linspace(start, end, 60), plane, rays)
        cv2.polylines(red, [np.rint(pts).astype(np.int32)], False, 255, 4)

    draw((0.4, 1.2), (0.4, 3.0))       # 오른쪽 긴 선
    draw((-0.35, 1.6), (0.35, 2.0))     # 띠 안쪽 사선
    draw((-0.35, 2.2), (0.35, 2.6))
    polygon = cv2.convexHull(np.column_stack(np.nonzero(red)[::-1]).astype(np.int32)).reshape(-1, 2)
    settings = shutter_obstacle.validate_options()
    completed, info = shutter_obstacle._complete_band(polygon, red, rays, plane, settings, {})
    assert completed is not None and info['zone_band_completion'] == 'interpolated_from_one_side'
    hidden = shutter_obstacle._project_floor_points(np.array([[-0.38, 1.4], [-0.38, 2.0], [-0.38, 2.6]]), plane, rays)
    for x, y in hidden:
        assert cv2.pointPolygonTest(polygon.astype(np.float32), (float(x), float(y)), False) < 0  # 원래는 구역 밖
        assert cv2.pointPolygonTest(completed.astype(np.float32), (float(x), float(y)), True) >= -3  # 보간 뒤 구역 안


def test_box_standing_on_side_line_is_obstacle():
    """오른쪽 테두리 선 위에 걸쳐 놓인 10cm 상자도 장애물로 판정한다.

    오른쪽 변은 화면에서 (490,250)→(600,460)이라 행 330~380에서 x≈532~558 이다. 상자 열 525~570이 선을 가운데에 둔다.
    이 변은 좌우 긴 선(띠 방향과 나란한 변)이라 벽으로 보지 않고, 테이프 폭만큼 바깥까지 장애물 후보로 인정한다.
    """
    report = shutter_obstacle.inspect(*_scene(box_height_m=0.10, box_rows=(330, 380), box_cols=(525, 570)))
    assert report['status'] == 'ABNORMAL' and report['obstacles']
    assert report['zone_wall_edges'] == 0


def test_reported_height_uses_whole_object_above_zone():
    """구역 윗변 위로 이어진 세로 상자 면은 물체 전체 높이로 보고하고, 판정용 bbox 는 구역 근처에 머문다.

    상자 앞면(행 200~330, 열 280~360)을 바닥과 닿는 행 330의 바닥 거리 하나로 둔다(세로로 선 면).
    구역 먼 쪽 선은 y=250 이라 윗부분(행 200~250)은 화면상 구역 밖이다.
    """
    frame, depth, rays, up = _scene(noise_m=0.003)
    along_down = rays @ -up
    face_depth = 0.25 / along_down[330, 320]
    depth[200:330, 280:360] = face_depth + np.random.default_rng(1).normal(0, 0.003, (130, 80))
    report = shutter_obstacle.inspect(frame, depth, rays, up)
    assert report['status'] == 'ABNORMAL' and report['obstacles']
    obstacle = max(report['obstacles'], key=lambda item: item['point_count'])
    assert obstacle['height_whole_p90_m'] > obstacle['height_zone_p90_m'] + 0.02
    assert obstacle['height_p90_m'] == max(obstacle['height_whole_p90_m'], obstacle['height_zone_p90_m'])
    assert obstacle['bbox'][1] >= 236


def test_supported_from_below_separates_floating_band_from_standing_face():
    """바로 아래가 바닥인 떠 있는 점은 거짓, 아래로 물체 면이 이어지거나 빈 Depth뿐인 점은 참인지 확인한다."""
    floor = np.ones((100, 3), dtype=bool)
    floor[20:60, 1] = False  # 가운데 열: 20~59행이 물체 앞면
    floor[20:60, 2] = False
    floor[30:50, 2] = False  # 오른쪽 열: 물체 면 중간이 빈 Depth(바닥으로 세지 않음)
    xs = np.array([0, 1, 2])
    ys = np.array([20, 20, 20])
    heights = np.full(3, 0.09)
    foot_rows = np.full(3, 70.0)  # 발은 70행, 바닥 분류 거리 0.03m 이면 20 + 50×(1-1/3) ≈ 53행까지 본다
    supported = shutter_obstacle._supported_from_below(xs, ys, heights, foot_rows, floor, 0.03, 0.5)
    assert supported.tolist() == [False, True, True]
    # 비율 1이면 확인을 끈다.
    assert shutter_obstacle._supported_from_below(xs, ys, heights, foot_rows, floor, 0.03, 1.0).all()


def test_floating_stripe_on_tape_is_not_obstacle():
    """가로 테이프 위 스테레오 오차처럼 바닥 위 9cm 로 뜬 15줄 띠는 장애물이 아니고, 확인을 끄면 장애물이 된다."""
    frame, depth, rays, up = _scene(noise_m=0.004)
    along_down = rays @ -up
    stripe = (slice(330, 345), slice(200, 440))
    depth[stripe] = (0.25 - 0.09) / along_down[stripe]
    report = shutter_obstacle.inspect(frame, depth, rays, up)
    assert report['status'] == 'NORMAL' and report['obstacles'] == []
    assert report['unsupported_pixel_ratio'] > 0
    report = shutter_obstacle.inspect(frame, depth, rays, up, unsupported_floor_ratio=1.0)
    assert report['status'] == 'ABNORMAL'


def _hatch_red_mask(with_side_line):
    """704px·초점 1036px 카메라(바닥 0.24m 위, 수평)에서 띠 가운데 끊긴 가로 테이프 조각 7개를 그린 빨간 마스크를 만든다.

    조각들은 앞쪽으로 일정 간격이라 앞쪽 방향 가짜 직선으로 이어질 수 있다. with_side_line이 참이면 옆 0.4m에
    끊김 없는 실제 좌우 선(폭 5cm)도 그린다. 반환값은 (빨간 마스크, 픽셀 방향, 바닥 평면)이다.
    """
    height, width, focal = 704, 704, 1036.0
    ys, xs = np.indices((height, width), dtype=float)
    rays = np.dstack(((xs - 352) / focal, (ys - 352) / focal, np.ones((height, width))))
    plane = np.array([0.0, -1.0, 0.0, 0.24])
    red = np.zeros((height, width), np.uint8)

    def band(l0, l1, a0, a1):
        """바닥 좌표(옆 l0~l1, 앞 a0~a1) 사각형을 화면에 채운다."""
        pts = shutter_obstacle._project_floor_points(np.array([[l0, a0], [l1, a0], [l1, a1], [l0, a1]], float), plane, rays)
        cv2.fillConvexPoly(red, np.rint(pts).astype(np.int32), 255)

    for k in range(7):
        band(-0.15, 0.15, 1.0 + 0.3 * k, 1.06 + 0.3 * k)
    if with_side_line:
        band(0.38, 0.43, 0.8, 2.9)
    return red, rays, plane


def test_side_lines_ignore_chain_of_hatch_pieces():
    """끊긴 빗금 조각이 이어진 가짜 앞쪽 선은 버리고, 끊김 없는 실제 좌우 선만 남기는지 확인한다."""
    polygon = np.array([[0, 703], [703, 703], [703, 380], [0, 380]])
    axis = np.array([0.0, 1.0])
    red, rays, plane = _hatch_red_mask(False)
    loose = shutter_obstacle.validate_options(side_line_min_coverage=0.0, side_line_max_two_sided_ratio=1e9)
    assert shutter_obstacle._floor_side_lines(polygon, red, rays, plane, loose, axis)[0]  # 거르지 않으면 가짜 선이 생긴다
    settings = shutter_obstacle.validate_options()
    assert shutter_obstacle._floor_side_lines(polygon, red, rays, plane, settings, axis)[0] == []
    red, rays, plane = _hatch_red_mask(True)
    lines, _ = shutter_obstacle._floor_side_lines(polygon, red, rays, plane, settings, axis)
    assert len(lines) == 1 and abs(abs(lines[0]['offset']) - 0.405) < 0.03 and lines[0]['coverage'] >= 0.9


def test_far_end_edge_is_top_flat_edge_only():
    """잘린 변을 뺀 변 중 화면 가장 위의 누운 변(먼 끝, 벽 쪽)만 고르고 비스듬한 좌우 선 쪽 변은 고르지 않는다.

    2026-10-15 robot3 녹화에서 구역 꼭짓점이 벽 위에 걸려 바닥 좌표 비교로는 먼 변을 좌우 선으로 잘못 분류했던 모양이다.
    """
    polygon = np.array([[703, 688], [0, 702], [0, 486], [175, 418], [539, 434], [703, 488]])
    border = shutter_obstacle._edge_on_image_border(polygon, (704, 704))
    far = shutter_obstacle._far_end_edges(polygon, (704, 704), border)
    assert far.tolist() == [False, False, False, True, False, False]
