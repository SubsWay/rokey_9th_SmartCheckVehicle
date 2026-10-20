import pytest

from scv_survey.heatmap import (
    KEEPOUT, MISS, MISS_COLOR, NORMAL, SHADOW, CellStats, Sample, SurveyGrid,
    cell_center, cell_color, cell_index, cell_status, keepout_cells, keepout_mask,
    pose_usable, rssi_to_color)

RED, YELLOW, GREEN, CYAN, BLUE = (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1)

# test_map.yaml 의 원점
OX, OY = -3.107, -4.823


def s(seq, x=0.0, y=0.0, rssi=-50, detected=True, pose_ok=True, robot='laptop'):
    return Sample(robot, seq, x, y, pose_ok, detected, rssi)


def approx_color(c):
    return pytest.approx(c, abs=1e-6)


# 기본 범위 -35(빨강) ~ -75(파랑): 강할수록 빨강, 음영으로 갈수록 파랑
@pytest.mark.parametrize('rssi, color', [
    (-20, RED), (-35, RED),            # 강한 쪽 끝과 그 위는 빨강
    (-45, YELLOW),                     # 1/4 지점
    (-55, GREEN),                      # 가운데
    (-65, CYAN),                       # 3/4 지점
    (-75, BLUE), (-95, BLUE),          # 약한 쪽 끝과 그 아래는 파랑
])
def test_color_gradient(rssi, color):
    assert rssi_to_color(rssi) == approx_color(color)


def test_color_gradient_is_monotonic():
    # 신호가 약해질수록 빨강 성분은 줄고(같거나), 파랑 성분은 늘어난다(같거나)
    cols = [rssi_to_color(v) for v in range(-35, -76, -1)]
    assert all(a[0] >= b[0] for a, b in zip(cols, cols[1:]))
    assert all(a[2] <= b[2] for a, b in zip(cols, cols[1:]))


def test_color_range_param():
    assert rssi_to_color(-68, strong=-35, weak=-68) == approx_color(BLUE)
    assert rssi_to_color(-50, strong=-50, weak=-90) == approx_color(RED)


def test_cell_index_on_test_map():
    # map 원점 (0,0) = SLAM 시작점 → (3.107/0.5, 4.823/0.5) = (6.2, 9.6) → 칸 (6, 9)
    assert cell_index(0.0, 0.0, OX, OY, 0.5) == (6, 9)
    assert cell_index(OX, OY, OX, OY, 0.5) == (0, 0)
    assert cell_index(OX + 0.4999, OY + 0.5, OX, OY, 0.5) == (0, 1)
    assert cell_index(OX - 0.01, OY, OX, OY, 0.5) == (-1, 0)


def test_cell_center_roundtrip():
    cx, cy = cell_center(6, 9, OX, OY, 0.5)
    assert cx == pytest.approx(OX + 3.25) and cy == pytest.approx(OY + 4.75)
    assert cell_index(cx, cy, OX, OY, 0.5) == (6, 9)


def test_median_and_miss_ratio():
    st = CellStats(rssi=[-40, -42, -60], n_missed=1)
    assert st.median == -42
    assert st.n_total == 4
    assert st.miss_ratio == pytest.approx(0.25)
    assert CellStats().median is None and CellStats().miss_ratio == 0.0


def test_cell_color_rules():
    assert cell_color(CellStats(rssi=[-55, -55])) == approx_color(GREEN)
    assert cell_color(CellStats(rssi=[], n_missed=3)) == MISS_COLOR
    # 미검출 비율이 기준 이상이면 회색 (약한 신호의 파랑과 다름)
    assert cell_color(CellStats(rssi=[-55], n_missed=1), miss_ratio_thresh=0.5) == MISS_COLOR
    assert cell_color(CellStats(rssi=[-55, -55], n_missed=1), miss_ratio_thresh=0.5) == approx_color(GREEN)
    assert cell_color(CellStats(rssi=[-80])) == approx_color(BLUE)


def test_grid_dedupe_and_pose_filter():
    g = SurveyGrid(0.5)
    assert g.add(s(0))
    assert not g.add(s(0))                     # 같은 (robot_id, seq)
    assert g.add(s(0, robot='robot4'))         # 다른 로봇은 별개
    assert not g.add(s(1, pose_ok=False))      # 위치 불확실
    cells = g.cells()
    assert list(cells) == [(0, 0)] and cells[(0, 0)].n_detected == 2


def test_grid_missed_samples_not_in_stats():
    g = SurveyGrid(0.5)
    g.add(s(0, rssi=-40))
    g.add(s(1, rssi=0, detected=False))
    st = g.cells()[(0, 0)]
    assert st.rssi == [-40] and st.n_missed == 1


def test_grid_uses_map_origin_and_bounds():
    g = SurveyGrid(0.5)
    g.add(s(0, x=0.0, y=0.0))
    g.add(s(1, x=-10.0, y=0.0))                # test_map 밖
    assert len(g.cells()) == 2                 # /map 받기 전에는 범위 검사 없음
    g.set_map(OX, OY, 94 * 0.05, 230 * 0.05)
    assert list(g.cells()) == [(6, 9)]         # 받은 뒤 원점 기준으로 다시 계산
    assert not g.in_bounds(-10.0, 0.0) and g.in_bounds(0.0, 0.0)


def test_five_samples_at_start_point():
    # 초기위치에서 5개를 재면 한 칸, 중앙값으로 색
    g = SurveyGrid(0.5)
    g.set_map(OX, OY, 94 * 0.05, 230 * 0.05)
    for i, v in enumerate([-41, -43, -40, -52, -42]):
        g.add(s(i, rssi=v))
    st = g.cells()[(6, 9)]
    r, gr, b = cell_color(st)
    assert st.median == -42 and r == 1.0 and b == 0.0   # 핫스팟 근처 → 빨강~주황


def test_grid_samples_keeps_accepted_only():
    g = SurveyGrid(0.5)
    g.add(s(0, x=1.0))
    g.add(s(1, pose_ok=False))
    g.add(s(0, x=9.0))                         # 중복
    assert [(x.seq, x.x) for x in g.samples] == [(0, 1.0)]


@pytest.mark.parametrize('age, cov, ok', [
    (0.0, None, True),          # 측정 시각의 TF, amcl_pose 아직 없음 → 공분산 검사 생략
    (0.8, 0.05, True),
    (-0.8, 0.05, True),         # TF 가 측정보다 늦어도 차이만 본다
    (None, 0.05, False),        # TF 없음
    (1.5, 0.05, False),         # 너무 오래된 TF
    (0.1, 0.30, False),         # AMCL 이 아직 수렴 안 함
])
def test_pose_usable(age, cov, ok):
    assert pose_usable(age, cov, max_age=1.0, max_cov=0.25)[0] is ok


@pytest.mark.parametrize('lag, ok', [
    (0.1, True),                # 보통의 Wi-Fi 지연
    (-0.1, True),
    (2.0, False),               # 노트북 시계가 로봇보다 2 s 빠름 (또는 TF 가 2 s 끊김)
    (-2.0, False),              # 노트북 시계가 2 s 느림 → 측정 시각 TF 가 "찾아져도"(age 0) 과거 위치
])
def test_pose_usable_lag(lag, ok):
    assert pose_usable(0.0, 0.05, max_age=0.5, lag_s=lag)[0] is ok


def test_pose_usable_age_check_off():
    # max_age <= 0 이면 시각 차는 보지 않는다 (노트북·로봇 시계가 안 맞을 때)
    assert pose_usable(30.0, 0.05, max_age=0.0, lag_s=30.0)[0]


def test_teleop_path_fills_cells_along_route():
    # 로봇이 map 원점에서 y 방향으로 0.2 m/s 로 가며 1 Hz 측정 → 지나간 칸만 칠해짐
    g = SurveyGrid(0.5)
    g.set_map(OX, OY, 94 * 0.05, 230 * 0.05)
    for i in range(10):
        g.add(s(i, x=0.0, y=0.2 * i, rssi=-45 - i))
    cells = g.cells()
    assert sorted(cells) == [(6, 9), (6, 10), (6, 11), (6, 12), (6, 13)]
    assert sum(st.n_total for st in cells.values()) == 10


# ---------------- 회피 마스크 ----------------

# testbed_v1.yaml (주행 팀 기준 지도): 69 x 138 px, 0.05 m, 원점 (-0.15, -0.25)
TB_O, TB_RES, TB_W, TB_H = (-0.15, -0.25), 0.05, 69, 138


def mask_px(data, r, c, w=TB_W):
    return data[r * w + c]


def test_cell_status_rules():
    assert cell_status(CellStats(rssi=[-50, -52])) == NORMAL
    assert cell_status(CellStats(rssi=[-72, -72])) == NORMAL        # 기준 -72 는 음영 아님 (미만만)
    assert cell_status(CellStats(rssi=[-73, -80])) == SHADOW
    assert cell_status(CellStats(rssi=[], n_missed=3)) == MISS
    assert cell_status(CellStats(rssi=[-50], n_missed=1)) == MISS   # 미검출 비율 0.5 = 회색과 같은 조건
    assert cell_status(CellStats(rssi=[-75, -75]), shadow_dbm=-80) == NORMAL


def test_keepout_cells_picks_shadow_and_miss_only():
    g = SurveyGrid(0.5)
    g.set_map(*TB_O, TB_W * TB_RES, TB_H * TB_RES)
    seq = iter(range(100))
    for x, y, vals in [(0.1, 0.0, [-45, -46, -44]),            # 정상
                       (0.6, 0.0, [-80, -78, -79]),            # 음영
                       (1.1, 0.0, [None, None, None]),         # 미검출
                       (1.6, 0.0, [-90])]:                     # 음영이지만 샘플 1 개 → 제외
        for v in vals:
            g.add(s(next(seq), x=x, y=y, rssi=v or 0, detected=v is not None))
    ko = keepout_cells(g.cells(), min_samples=2)
    assert ko == {(1, 0): SHADOW, (2, 0): MISS}


def test_keepout_mask_empty_when_no_cells():
    # 측정 중·새 측정 시작: 전부 0 → 코스트맵에 회피 칸이 남지 않는다
    data = keepout_mask({}, 0.5, TB_O, TB_RES, TB_W, TB_H)
    assert len(data) == TB_W * TB_H and not any(data)


def test_keepout_mask_one_cell_with_margin():
    # 칸 (1, 2) = x -0.15+0.5 ~ +1.0, y -0.25+1.0 ~ +1.5 → 픽셀 열 10~19, 행 20~29
    data = keepout_mask({(1, 2): SHADOW}, 0.5, TB_O, TB_RES, TB_W, TB_H, margin=0.2)
    assert mask_px(data, 20, 10) == KEEPOUT and mask_px(data, 29, 19) == KEEPOUT
    # 0.2 m = 4 픽셀까지 넓힘, 5 번째는 아님
    assert mask_px(data, 25, 6) == KEEPOUT and mask_px(data, 25, 5) == 0
    assert mask_px(data, 25, 23) == KEEPOUT and mask_px(data, 25, 24) == 0
    assert mask_px(data, 16, 15) == KEEPOUT and mask_px(data, 15, 15) == 0
    assert mask_px(data, 33, 15) == KEEPOUT and mask_px(data, 34, 15) == 0
    # 모서리는 둥글게: 대각선으로 4 번째 픽셀은 0.247 m 라 제외, 3 번째는 0.177 m 라 포함
    assert mask_px(data, 16, 6) == 0 and mask_px(data, 17, 7) == KEEPOUT
    # 칸 10x10 + 변 4x10x4 + 모서리 13x4
    assert sum(1 for v in data if v) == 100 + 160 + 52
    assert set(data) == {0, KEEPOUT}


def test_keepout_mask_default_is_cell_only():
    # 기본(margin 0): 0.5 m 칸 그대로 10x10 픽셀만, 옆 칸은 0
    data = keepout_mask({(1, 2): MISS}, 0.5, TB_O, TB_RES, TB_W, TB_H)
    on = {(i // TB_W, i % TB_W) for i, v in enumerate(data) if v}
    assert on == {(r, c) for r in range(20, 30) for c in range(10, 20)}


def test_keepout_mask_clipped_at_map_edge():
    # 지도 끝 칸 (마지막 열 칸 6 은 x 2.85 ~ 3.35 인데 지도는 3.30 까지) — 넘치지 않고 잘림
    data = keepout_mask({(6, 0), (0, 0)}, 0.5, TB_O, TB_RES, TB_W, TB_H, margin=0.2)
    assert len(data) == TB_W * TB_H
    assert mask_px(data, 0, 68) == KEEPOUT and mask_px(data, 0, 0) == KEEPOUT


def test_keepout_mask_matches_grid_cells():
    # 엔진과 같은 흐름: 샘플 → 칸 → 회피 칸 → 마스크. 정상 칸 픽셀은 0, 음영 칸 가운데 픽셀은 100
    g = SurveyGrid(0.5)
    g.set_map(*TB_O, TB_W * TB_RES, TB_H * TB_RES)
    for k in range(3):
        g.add(s(k, x=0.0, y=0.0, rssi=-45))          # 칸 (0, 0) 정상
        g.add(s(10 + k, x=1.5, y=2.0, rssi=-85))     # 칸 (3, 4) 음영
    data = keepout_mask(keepout_cells(g.cells()), g.cell_size, g.origin, TB_RES, TB_W, TB_H)
    c, r = int((0.0 - TB_O[0]) / TB_RES), int((0.0 - TB_O[1]) / TB_RES)
    assert mask_px(data, r, c) == 0
    c, r = int((1.5 - TB_O[0]) / TB_RES), int((2.0 - TB_O[1]) / TB_RES)
    assert mask_px(data, r, c) == KEEPOUT


def test_grid_clear_for_new_survey():
    g = SurveyGrid(0.5)
    g.set_map(*TB_O, TB_W * TB_RES, TB_H * TB_RES)
    g.add(s(0, rssi=-85))
    g.clear()
    assert g.cells() == {} and g.samples == []
    assert g.add(s(0, rssi=-50))                     # 비운 뒤엔 같은 seq 도 새 샘플
    assert g.bounds is not None                      # 지도 정보는 그대로
