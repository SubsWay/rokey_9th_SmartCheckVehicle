"""묶음 시험에서 더한 순수 로직: 벽 여유 경로, 설비 섹터 배정, 재측정 칸 선택, 회피 칸."""

import math

import pytest
from pathlib import Path
from types import SimpleNamespace

from scv_survey.cellmap_store import CellRow

from scv_manager.manager_core import Waypoint, blocked_cells, mask_cells, remeasure_targets
from scv_manager.route_builder import RouteBuilder, wall_points

ZONES = """
zones:
  - id: Z1
    dock: {x: 3.184, y: 0.995}
    polygon: [[0.08, -0.05], [3.18, -0.05], [3.18, 3.29], [0.08, 3.29]]
    spacing: 0.5
  - id: Z2
    dock: {x: 3.176, y: 4.597}
    polygon: [[0.08, 3.29], [3.18, 3.29], [3.18, 6.48], [0.08, 6.48]]
    spacing: 0.5
"""
FACILITIES = """
facilities:
  - id: SHT-01
    view: {x: 2.70, y: 3.00, yaw: 3.142}
  - id: EXT-01
    view: {x: 0.0, y: 0.0, yaw: 0.0}
"""
ORIGIN = (-0.15, -0.25)


def builder(tmp_path: Path, **kwargs) -> RouteBuilder:
    (tmp_path / "zones.yaml").write_text(ZONES, encoding="utf-8")
    (tmp_path / "facilities.yaml").write_text(FACILITIES, encoding="utf-8")
    return RouteBuilder(
        str(tmp_path / "zones.yaml"),
        str(tmp_path / "facilities.yaml"),
        wall_bounds=(0.08, -0.05, 3.18, 6.48),
        wall_clearance=0.45,
        origin=ORIGIN,
        **kwargs,
    )


def test_wall_points_reads_occupied_pixels_in_map_coordinates(tmp_path):
    # 3x2, 8비트. 윗줄 가운데와 아랫줄 왼쪽이 검은(점유) 픽셀
    (tmp_path / "m.pgm").write_bytes(b"P5\n3 2\n255\n" + bytes([254, 0, 254, 0, 205, 254]))
    info = SimpleNamespace(pgm=tmp_path / "m.pgm", origin=(1.0, 2.0), resolution=0.5)

    assert wall_points(info) == [(1.75, 2.75), (1.25, 2.25)]


def room_walls(extra=()):
    """ZONES 의 방 둘레 벽 (5 cm 픽셀) + 더할 벽."""
    xs = [0.08 + 0.05 * k for k in range(63)]
    ys = [-0.05 + 0.05 * k for k in range(132)]
    return (
        [(x, y) for x in xs for y in (-0.05, 6.48)]
        + [(x, y) for y in ys for x in (0.08, 3.18)]
        + list(extra)
    )


def cell_routes(tmp_path, extra=()):
    rb = builder(tmp_path, walls=room_walls(extra), obstacle_clearance=0.30)
    return rb, rb.build_survey_routes()


def rows(rb, route):
    """줄(연속한 두 점이 같은 높이이고 직선으로 이어짐)의 (y, x 작은 쪽, x 큰 쪽)."""
    w = route.waypoints
    return sorted(
        (round(a.y, 2), round(min(a.x, b.x), 2), round(max(a.x, b.x), 2))
        for a, b in zip(w, w[1:])
        if abs(a.y - b.y) < 1e-6 and rb.line_clear((a.x, a.y), (b.x, b.y))
    )


def test_cell_route_in_open_room_is_the_same_zigzag(tmp_path):
    rb, (z1, z2) = cell_routes(tmp_path)

    # 가장자리 열(x 0.1 · 3.1)은 빠지고 줄은 칸 중심을 지난다
    assert rows(rb, z1) == [(y, 0.6, 2.6) for y in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)]
    assert rows(rb, z2) == [(y, 0.6, 2.6) for y in (3.5, 4.0, 4.5, 5.0, 5.5, 6.0)]
    assert all(rb.clear_of_walls(p.x, p.y) for p in z1.waypoints + z2.waypoints)
    assert [len(s) for s in z1.segments] == [12]                    # 막힌 곳이 없으면 한 구간
    assert (z1.waypoints[0].x, z1.waypoints[0].y) == (2.6, 0.5)     # Dock(오른쪽 아래) 쪽


def test_cell_route_splits_a_row_at_a_crossing_wall(tmp_path):
    wall = [(1.6, 5.75 + 0.05 * k) for k in range(15)]             # Z2 맨 윗줄을 가로지름
    rb, (_, z2) = cell_routes(tmp_path, wall)

    assert (6.0, 0.6, 1.1) in rows(rb, z2) and (6.0, 2.1, 2.6) in rows(rb, z2)
    # 벽(y 5.75 부터)을 가로지르는 줄은 없다. 그 아래 줄은 높이를 낮춰 벽 끝 밑으로 지나간다
    assert not any(y >= 5.45 and left < 1.6 < right for y, left, right in rows(rb, z2))
    assert any(y < 5.45 and left < 1.6 < right for y, left, right in rows(rb, z2))
    assert all(rb.clear_of_walls(p.x, p.y) for p in z2.waypoints)


def test_cell_route_leaves_out_a_closed_room(tmp_path):
    box = (
        [(1.05 + 0.05 * k, y) for k in range(23) for y in (0.75, 1.85)]
        + [(x, 0.75 + 0.05 * k) for k in range(23) for x in (1.05, 2.15)]
    )
    rb, (z1, _) = cell_routes(tmp_path, box)

    assert not any(1.05 < p.x < 2.15 and 0.75 < p.y < 1.85 for p in z1.waypoints)
    assert (2.5, 0.6, 2.6) in rows(rb, z1)


def test_cell_route_ends_a_segment_in_a_dead_end(tmp_path):
    stub = [(0.08 + 0.05 * k, 4.8) for k in range(23)]             # 실제 지도의 Z2 벽 조각 (x 0.08–1.18)
    rb, (_, z2) = cell_routes(tmp_path, stub)

    # 벽 조각 위 줄은 높이를 옮겨 벽 옆 칸까지 지나간다 (아래 줄은 딱 0.30 m 라 그대로)
    assert (5.15, 0.6, 2.6) in rows(rb, z2) and (4.5, 0.6, 2.6) in rows(rb, z2)
    # 벽 조각 위 막다른 끝 (0.6, 5.15) 에서 구간이 끝난다
    ends = [(round(s[-1].x, 2), round(s[-1].y, 2)) for s in z2.segments]
    assert (0.6, 5.15) in ends
    assert [p for s in z2.segments for p in s] == list(z2.waypoints)
    # 구간 안의 이웃한 두 점은 직선으로 갈 수 있다
    assert all(
        rb.line_clear((a.x, a.y), (b.x, b.y))
        for s in z2.segments
        for a, b in zip(s, s[1:])
    )


def test_two_lines_per_cell_row_every_other_line_first(tmp_path):
    rb = builder(tmp_path, walls=room_walls(), obstacle_clearance=0.30, cell_size=0.5, line_spacing=0.25,
                 passed_goal_radius=0.35)
    z1, _ = rb.build_survey_routes()

    lines = sorted({y for y, _, _ in rows(rb, z1)})
    # 칸(0.25–0.75, …)마다 1/4 · 3/4 높이에 한 줄씩, 칸 경계에서 0.125 m 안쪽
    assert lines == [round(0.25 + 0.5 * j + d, 2) for j in range(6) for d in (0.125, 0.375)]
    # 1/4 높이 줄을 먼저 다 돈 뒤 3/4 높이 줄로 돌아온다 → 줄 사이 이동은 0.5 m, 바꿀 때 한 번만 0.25 m
    w = z1.waypoints
    steps = [round(abs(b.y - a.y), 3) for a, b in zip(w, w[1:]) if abs(a.y - b.y) > 1e-6]
    assert steps.count(0.25) == 1 and set(steps) == {0.25, 0.5}
    first_band = [round(p.y - 0.25, 3) % 0.5 for p in w[: steps.index(0.25) * 2 + 2]]
    assert set(first_band) == {0.125}
    # 0.35 m 보다 가까운 다음 점이 있으면 거기서 구간이 끝난다 (함께 지워지지 않게)
    assert all(
        math.dist((a.x, a.y), (b.x, b.y)) >= 0.35
        for s in z1.segments
        for a, b in zip(s, s[1:])
    )
    # 출발한 쪽(Dock, 오른쪽 아래) 근처에서 끝난다
    assert (w[0].x, w[-1].x) == (2.6, 2.6) and w[-1].y < 1.0


def test_corner_mode_angle_ends_segments_at_sharp_corners(tmp_path):
    def segments(mode, break_deg=90.0):
        rb = builder(tmp_path, walls=room_walls(), obstacle_clearance=0.30,
                     corner_mode=mode, corner_break_deg=break_deg)
        return rb.build_survey_routes()[0].segments

    # none: 90도 모서리는 구간 가운데 점으로 지나간다 → 한 구간
    assert [len(s) for s in segments("none")] == [12]
    # angle: 90도 이상 꺾는 점마다 구간이 끝나 줄(과 줄 바꿈) 하나씩
    assert all(len(s) <= 2 for s in segments("angle"))
    # 기준을 90도보다 크게 잡으면 90도 모서리는 끊지 않는다
    assert [len(s) for s in segments("angle", 100.0)] == [12]


def test_split_puts_points_every_cell_on_straight_lines(tmp_path):
    whole = builder(tmp_path, walls=room_walls(), obstacle_clearance=0.30, corner_mode="angle")
    split = builder(tmp_path, walls=room_walls(), obstacle_clearance=0.30, corner_mode="angle", split=0.5)
    a, b = whole.build_survey_routes()[0], split.build_survey_routes()[0]

    # 구간 수 · 구간 끝점은 그대로, 사이에 점이 늘어난다
    assert len(a.segments) == len(b.segments)
    assert [s[-1] for s in a.segments] == [s[-1] for s in b.segments]
    assert len(b.waypoints) > len(a.waypoints)
    assert [p for s in b.segments for p in s] == list(b.waypoints)
    for s in b.segments:
        steps = [math.dist((p.x, p.y), (q.x, q.y)) for p, q in zip(s, s[1:])]
        # 0.5 m 마다, 끝점 앞 마지막 간격은 지움 반경(0.35) 이상
        assert all(d <= 0.5 + 0.35 + 1e-6 for d in steps) and all(d >= 0.35 - 1e-6 for d in steps)
        # 중간 점은 그 구간의 직선 위에 있다
        for p in s[1:-1]:
            ax, ay, bx, by = s[0].x, s[0].y, s[-1].x, s[-1].y
            assert abs((bx - ax) * (p.y - ay) - (by - ay) * (p.x - ax)) < 1e-2


def test_unknown_corner_mode_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        builder(tmp_path, corner_mode="guide")


def test_survey_route_keeps_wall_clearance_and_starts_at_dock_side(tmp_path):
    z1, z2 = builder(tmp_path).build_survey_routes()

    assert len(z1.waypoints) == 12
    assert {round(p.x, 2) for p in z1.waypoints} == {0.53, 2.73}
    assert sorted({round(p.y, 2) for p in z1.waypoints}) == [
        0.5, 1.0, 1.5, 2.0, 2.5, 3.0,
    ]
    assert (round(z1.waypoints[0].x, 2), z1.waypoints[0].y) == (2.73, 0.5)
    assert sorted({round(p.y, 2) for p in z2.waypoints}) == [
        3.5, 4.0, 4.5, 5.0, 5.5, 6.0,
    ]


def test_facility_gets_zone_and_zero_view_is_skipped(tmp_path):
    rb = builder(tmp_path)
    routes = rb.build_facility_routes()

    assert [(r.facility_id, r.zone_id) for r in routes] == [("SHT-01", "Z1")]
    assert rb.skipped_facilities == ["EXT-01"]


def test_reachable_cells_are_inside_zone_and_clearance(tmp_path):
    grid = [
        (i, j, ORIGIN[0] + (i + 0.5) * 0.5, ORIGIN[1] + (j + 0.5) * 0.5)
        for i in range(7)
        for j in range(14)
    ]
    rb = builder(tmp_path)

    assert len(rb.reachable_cells("Z1", grid)) == 30
    assert len(rb.reachable_cells("Z2", grid)) == 30
    assert not rb.reachable_cells("Z1", grid) & rb.reachable_cells("Z2", grid)


def test_remeasure_targets_order_limit_and_reachable():
    cells = {
        (1, 1): CellRow("normal", -50.0, 3, "ok"),
        (2, 1): CellRow("shadow", -75.0, 3, "ok"),
        (3, 1): CellRow("shadow", -80.0, 3, "ok"),
        (4, 1): CellRow("miss", None, 3, "ok"),
        (9, 9): CellRow("miss", None, 3, "ok"),
    }
    reachable = {(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)}

    # 재측정은 1차 측정에서 미검출된 칸만 (약한 신호 칸은 넣지 않음, 10/15 PM). 갈 수 없는 (9, 9) 는 뺀다
    assert remeasure_targets(cells, reachable, 10) == [(4, 1), (5, 1)]
    assert remeasure_targets(cells, reachable, 1) == [(4, 1)]
    assert remeasure_targets(cells, reachable, 10, False) == [(4, 1)]
    # 측정 중 장애물로 못 간 미측정 칸 (5, 1) 은 채우러 가지 않는다
    assert remeasure_targets(cells, reachable, 10, True, {(5, 1)}) == [(4, 1)]


def test_blocked_cells_from_executor_reason():
    w = tuple(Waypoint(x=0.6 + 0.5 * k, y=5.375, yaw=0.0) for k in range(5))   # 0.6 … 2.6
    origin = (-0.15, -0.25)

    def ix(x):
        return math.floor((x - origin[0]) / 0.5)

    iy = math.floor((5.375 - origin[1]) / 0.5)
    # 중간 점 2 (1.6) 를 건너뜀, 골 4 (2.6) 를 줄임 → 1.6 칸 + 2.1 → 2.6 구간이 지나는 칸
    cells = blocked_cells(w, "BLOCKED skipped=[2] shortened=[4]", origin, 0.5)
    assert cells == {(ix(1.6), iy)} | {(ix(x), iy) for x in (2.2, 2.4, 2.6)}
    assert blocked_cells(w, "BLOCKED skipped=[] shortened=[]", origin, 0.5) == set()
    assert blocked_cells(w, "CANCELED", origin, 0.5) == set()


def test_mask_cells_skip_normal_low_confidence_and_dock():
    cells = {
        (1, 1): CellRow("normal", -50.0, 3, "ok"),
        (2, 1): CellRow("shadow", -75.0, 3, "ok"),
        (3, 1): CellRow("miss", None, 1, "low"),
        (6, 2): CellRow("shadow", -75.0, 3, "ok"),
    }

    assert mask_cells(cells, 0.5, ORIGIN, [(3.184, 0.995)], 0.6) == {(2, 1)}


def test_candidate_filter_preserves_low_sample_vs_unmeasured_and_all_candidates():
    cells = {(1,1): CellRow('miss', None, 1, 'low'),
             (2,1): CellRow('miss', None, 3, 'ok'),
             (3,1): CellRow('shadow', -80, 3, 'ok')}
    reachable = {(1,1),(2,1),(3,1),(4,1)}
    # 표본 부족 미검출 (1,1) 은 후보가 아니지만 미측정으로 바뀌지도 않는다. 약한 신호 (3,1) 은 후보 아님
    assert remeasure_targets(cells,reachable,0,False,min_samples=3) == [(2,1)]
    assert remeasure_targets(cells,reachable,0,True,min_samples=3) == [(2,1),(4,1)]
    many = {(i,0): CellRow('miss',None,3,'ok') for i in range(7)}
    assert len(remeasure_targets(many,set(many),0,False,min_samples=3)) == 7
