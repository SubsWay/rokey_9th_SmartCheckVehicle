"""관제(inspection_manager)가 로봇에 보낼 경로를 만든다: 구역(zones.yaml)의 RSSI 측정 경로와 설비(facilities.yaml) 점검 자리.

측정 경로는 지도(testbed_v1)의 벽 픽셀을 기준으로 RSSI 칸(0.5 m)마다 측정 줄을 놓고, Dock 에서 갈 수 있는 줄만
지그재그로 이어, 직선으로 못 가거나 크게 꺾는 점에서 작업(구간)으로 나눈다 (docs/control/PLAN.md N-1 ~ N-11).
지도가 없으면 예전 방식(구역 사각형에 가로줄)을 쓴다.
"""
import math
import re
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import yaml

from scv_survey.cellmap_store import in_polygon

from .manager_core import Waypoint


# 주행 설정과 맞추는 값 (기본값). 바뀌면 관제 파라미터로 같이 바꾼다
PASSED_GOAL_RADIUS = 0.35   # navigate_through_poses_scv.xml 의 RemovePassedGoals radius (주행 10/12 값)
CORNER_BREAK_DEG = 90.0     # corner_mode=angle: 이만큼 이상 꺾는 점에서 구간을 끊는다


@dataclass(frozen=True)
class SurveyRoute:
    """한 구역의 측정 경로. waypoints 는 모든 점을 순서대로, segments 는 그 점들을 작업(ExecuteTask 하나) 단위로 나눈 것.

    segments 를 이어 붙이면 waypoints 와 같다 (다음 구간이 앞 구간 끝점에서 시작하면 그 점이 두 번 들어감).
    지도 없이 만든 예전 경로는 segments 가 비어 있다.
    """

    zone_id: str
    waypoints: tuple[Waypoint, ...]
    # 작업 구간. 구간의 마지막 점은 Nav2 가 끝까지 도착한다 (막다른 자리용)
    segments: tuple[tuple[Waypoint, ...], ...] = ()


@dataclass(frozen=True)
class FacilityRoute:
    """설비 점검 경로: 점검 자리(view) 하나와 그 자리가 든 구역 id (어느 구역에도 없으면 "")."""

    facility_id: str
    waypoints: tuple[Waypoint, ...]
    zone_id: str = ""


def wall_points(info, occupied_thresh: float = 0.65) -> list[tuple[float, float]]:
    """지도(P5 pgm)의 점유 픽셀 중심 [(x, y)] (map 좌표). info 는 read_map_yaml 결과."""
    raw = Path(info.pgm).read_bytes()
    head = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s", raw)
    if head is None:
        raise ValueError(f"{info.pgm}: P5 pgm 이 아닙니다")
    width, height, maxval = map(int, head.groups())
    step = 2 if maxval > 255 else 1     # testbed_v1.pgm 은 16비트
    data = raw[head.end():]
    return [
        (
            info.origin[0] + (i % width + 0.5) * info.resolution,
            info.origin[1] + (height - 1 - i // width + 0.5) * info.resolution,
        )
        for i in range(width * height)
        if 1.0 - int.from_bytes(data[i * step:(i + 1) * step], "big") / maxval
        > occupied_thresh
    ]


class RouteBuilder:
    """구역 · 설비 설정 파일과 지도 벽으로 측정 · 설비 경로를 만든다.

    zones_file · facilities_file: 구역 polygon · Dock, 설비 view 좌표 (yaml)
    wall_bounds · wall_clearance: 시험장 벽 안쪽 사각형과 그 안쪽 여유 (재측정 칸 · 예전 줄 방식이 씀)
    origin: 지도 origin (칸 격자의 원점)
    walls · obstacle_clearance: 지도 점유 픽셀(wall_points)과, 경로 점 · 직선을 벽에서 띄울 거리 (0.30 = 로봇 0.19 + 0.10)
    cell_size · line_spacing: RSSI 칸 크기와 측정 줄 간격 (0.5 · 0.25 면 칸 한 줄에 두 줄)
    corner_mode · corner_break_deg: angle 이면 이 각도 이상 꺾는 점에서 작업을 끊음
    passed_goal_radius: 주행 BT 의 지나간 점 지움 반경 — 이보다 가까운 이웃 점은 같은 작업에 두지 않음
    split: 작업 직선을 이 간격의 중간 점으로 나눔 (None 이면 안 나눔)
    """

    def __init__(
        self,
        zones_file: str,
        facilities_file: str,
        wall_bounds: tuple[float, ...] = (),
        wall_clearance: float = 0.0,
        origin: tuple[float, float] = (0.0, 0.0),
        walls=(),
        obstacle_clearance: float = 0.0,
        cell_size: float | None = None,
        line_spacing: float | None = None,
        corner_mode: str = "none",
        passed_goal_radius: float = PASSED_GOAL_RADIUS,
        corner_break_deg: float = CORNER_BREAK_DEG,
        split: float | None = None,
    ):
        self.zones_file = Path(zones_file)
        # RSSI 칸 크기와 측정 줄 간격. 줄 간격이 칸보다 작으면 칸 한 줄을 여러 번 지난다
        self.cell_size = cell_size
        self.line_spacing = line_spacing
        # 모서리 처리: none · angle(corner_break_deg 이상 꺾는 점을 구간 끝으로 → 직선 하나씩 멈췄다 정렬,
        # 10/14 비교 시험으로 채택 — PLAN N-10)
        if corner_mode not in ("none", "angle"):
            raise ValueError(f"corner_mode 는 none · angle 중 하나: {corner_mode}")
        self.corner_mode = corner_mode
        self.passed_goal_radius = passed_goal_radius
        self.corner_break_deg = corner_break_deg
        # 구간 직선을 이 간격의 중간 점으로 나눔 (None·0 이면 안 나눔). 막힌 점만 건너뛰는 단위 (10/14)
        self.split = split
        self.facilities_file = Path(facilities_file)
        # 벽 안쪽 사각형 (xmin, ymin, xmax, ymax). 주면 경로를 벽에서
        # wall_clearance 만큼 안쪽으로 자르고 y 줄을 칸 중심에 맞춘다.
        self.wall_bounds = tuple(wall_bounds)
        self.wall_clearance = wall_clearance
        self.origin = origin
        # 지도의 점유 픽셀. 경로 점과 재측정 칸은 여기서 obstacle_clearance
        # 이상 떨어뜨린다 (사각형 wall_bounds 는 안쪽 벽 조각을 모른다).
        self.walls = list(walls)
        self.obstacle_clearance = obstacle_clearance
        # 가까운 벽만 보도록 0.5 m 통에 나눠 담는다
        self._wall_buckets: dict[tuple[int, int], list] = {}
        for wx, wy in self.walls:
            self._wall_buckets.setdefault(
                (math.floor(wx / 0.5), math.floor(wy / 0.5)), []
            ).append((wx, wy))
        self.skipped_facilities: list[str] = []

    def zones(self) -> dict[str, dict]:
        """zones.yaml 의 구역을 {id: 구역 설정} 으로."""
        data = self._load_yaml(self.zones_file)
        return {str(zone["id"]): zone for zone in data.get("zones", [])}

    def inner_bounds(self) -> tuple[float, float, float, float]:
        """벽 안쪽 사각형(wall_bounds)을 wall_clearance 만큼 줄인 (xmin, ymin, xmax, ymax)."""
        xmin, ymin, xmax, ymax = self.wall_bounds
        c = self.wall_clearance
        return (xmin + c, ymin + c, xmax - c, ymax - c)

    def wall_distance(self, x: float, y: float) -> float:
        """가장 가까운 벽 픽셀까지 거리. 0.5 m 안에 없으면 inf."""
        bx, by = math.floor(x / 0.5), math.floor(y / 0.5)
        return min(
            (
                math.hypot(x - wx, y - wy)
                for i in (bx - 1, bx, bx + 1)
                for j in (by - 1, by, by + 1)
                for wx, wy in self._wall_buckets.get((i, j), ())
            ),
            default=math.inf,
        )

    def clear_of_walls(self, x: float, y: float) -> bool:
        """(x, y) 가 벽에서 obstacle_clearance 이상 떨어져 있는가."""
        # 벽 픽셀과 후보 점이 같은 5 cm 격자라 딱 0.30 인 점이 흔하다
        return self.wall_distance(x, y) >= self.obstacle_clearance - 1e-6

    def line_clear(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        """a–b 직선이 벽에서 obstacle_clearance 이상 떨어져 있는가 (5 cm 마다)."""
        steps = max(math.ceil(math.dist(a, b) / 0.05), 1)
        return all(
            self.clear_of_walls(
                a[0] + (b[0] - a[0]) * k / steps,
                a[1] + (b[1] - a[1]) * k / steps,
            )
            for k in range(steps + 1)
        )

    def reachable_cells(
        self,
        zone_id: str,
        grid_cells,
    ) -> set[tuple[int, int]]:
        """중심이 zone 안이고 벽에서 wall_clearance 안쪽, 점유 픽셀에서 떨어진 칸."""
        polygon = self.zones()[zone_id]["polygon"]
        xmin, ymin, xmax, ymax = self.inner_bounds()
        return {
            (ix, iy)
            for ix, iy, cx, cy in grid_cells
            if in_polygon(cx, cy, polygon)
            and xmin <= cx <= xmax
            and ymin <= cy <= ymax
            and self.clear_of_walls(cx, cy)
        }

    def build_survey_routes(
        self,
    ) -> list[SurveyRoute]:
        """구역마다 측정 경로. 지도 벽이 있으면 칸 기준 경로(_build_cell_route → _split_segment), 없으면 예전 줄 방식.

        점이 하나도 안 나오는 구역(좌표 미입력)은 ValueError.
        """
        data = self._load_yaml(self.zones_file)
        routes = []

        for zone in data.get("zones", []):
            polygon = [
                (float(point[0]), float(point[1]))
                for point in zone["polygon"]
            ]
            spacing = float(zone.get("spacing", 1.0))

            dock = zone.get("dock")
            segments = ()
            if self.walls:
                cell = self.cell_size or spacing
                segments = self._build_cell_route(
                    polygon,
                    cell,
                    (float(dock["x"]), float(dock["y"])) if dock else None,
                    lines=max(round(cell / self.line_spacing), 1) if self.line_spacing else 1,
                )
                if self.split:
                    segments = [self._split_segment(s) for s in segments]
                waypoints = [p for segment in segments for p in segment]
            else:
                # shortcut: 지도 없이 쓰는 예전 줄 방식, 칸 방식이 실기로 확인되면 지운다
                waypoints = self._build_scanline_route(
                    polygon,
                    spacing,
                    bounds=self.inner_bounds() if self.wall_bounds else None,
                    origin_y=self.origin[1],
                    start_x=float(dock["x"]) if dock else None,
                )

            if not waypoints:
                raise ValueError(
                    f"{zone['id']}의 polygon 좌표가 "
                    "아직 입력되지 않았습니다."
                )

            routes.append(
                SurveyRoute(
                    zone_id=str(zone["id"]),
                    waypoints=tuple(waypoints),
                    segments=tuple(tuple(s) for s in segments),
                )
            )

        return routes

    def build_facility_routes(
        self,
    ) -> list[FacilityRoute]:
        """설비마다 점검 자리 경로. 좌표가 (0, 0) 인 설비는 건너뛰고 skipped_facilities 에 남긴다."""
        data = self._load_yaml(self.facilities_file)
        zones = self.zones()
        routes = []
        self.skipped_facilities = []

        for facility in data.get("facilities", []):
            view = facility["view"]

            # 좌표를 아직 넣지 않은 설비 (0, 0) 는 건너뛴다
            if float(view["x"]) == 0.0 and float(view["y"]) == 0.0:
                self.skipped_facilities.append(str(facility["id"]))
                continue

            waypoint = Waypoint(
                x=float(view["x"]),
                y=float(view["y"]),
                yaw=float(view["yaw"]),
            )

            routes.append(
                FacilityRoute(
                    facility_id=str(facility["id"]),
                    waypoints=(waypoint,),
                    zone_id=next(
                        (
                            zone_id
                            for zone_id, zone in zones.items()
                            if in_polygon(
                                waypoint.x,
                                waypoint.y,
                                zone["polygon"],
                            )
                        ),
                        "",
                    ),
                )
            )

        return routes

    @staticmethod
    def _load_yaml(path: Path) -> dict:
        """yaml 파일을 읽는다 (비어 있으면 {})."""
        with path.open(
            "r",
            encoding="utf-8",
        ) as file:
            return yaml.safe_load(file) or {}

    def _split_segment(self, segment: list[Waypoint]) -> list[Waypoint]:
        """구간의 직선을 split 마다 중간 점으로 나눈다 (점 하나가 막혀도 그 점만 건너뛰고 줄의 나머지를 재게).

        일직선 위의 점이라 RPP 가 그대로 지나가며 지움 반경 안에 들어간다. 끝점까지 지움 반경보다 가까운 중간 점은 두지 않는다.
        """
        out = segment[:1]
        for a, b in zip(segment, segment[1:]):
            length = math.dist((a.x, a.y), (b.x, b.y))
            yaw = math.atan2(b.y - a.y, b.x - a.x)
            k = 1
            while k * self.split < length - self.passed_goal_radius:
                s = k * self.split / length
                out.append(Waypoint(x=round(a.x + (b.x - a.x) * s, 3), y=round(a.y + (b.y - a.y) * s, 3), yaw=yaw))
                k += 1
            out.append(b)
        return out

    def _build_cell_route(
        self,
        polygon: list[tuple[float, float]],
        cell: float,
        dock: tuple[float, float] | None,
        lines: int = 1,
    ) -> list[list[Waypoint]]:
        """칸 기준 측정 경로. 구간(작업)의 목록을 돌려준다.

        칸 한 줄을 lines 개의 띠로 나눠 띠마다 측정 줄을 하나씩 둔다 (0.5 m 칸 · 0.25 m 간격이면 칸의
        1/4 · 3/4 높이). 첫 번째 띠의 줄을 지그재그로 다 돈 뒤 다음 띠의 줄을 반대 방향으로 돈다
        (한 줄 건너 돌기 — U턴 폭이 칸 크기 그대로라 RPP 가 줄 끝에서 일찍 돌아서지 않는다).
        직선으로 못 가거나 되돌아가야 하는 점에서는 구간을 끊어 Nav2 가 그 점에 끝까지 도착하게 한다.
        """
        if cell <= 0.0 or len(polygon) < 3 or lines < 1:
            return []

        oy = self.origin[1]
        route: list[tuple[tuple[float, float], float]] = []     # (점, yaw)

        def add(run: list[tuple[float, float]]) -> None:
            """측정 줄(양 끝, 칸 하나면 한 점)을 route 에 붙인다. yaw 는 줄 방향, 점 하나면 앞 점에서 오는 방향."""
            start, end = run[0], run[-1]
            origin = start if len(run) > 1 or not route else route[-1][0]
            target = end if len(run) > 1 else start
            yaw = math.atan2(target[1] - origin[1], target[0] - origin[0])
            route.append((start, yaw))
            if len(run) > 1:
                route.append((end, yaw))

        for run in self._cell_lines(polygon, cell, dock, 0, lines):
            add(run)

        for band in range(1, lines):
            runs = self._cell_lines(polygon, cell, dock, band, lines)
            if not runs or not route:
                continue
            # 앞 띠가 올라갔으면 이번 띠는 내려온다. 칸 줄마다 앞 줄이 끝난 쪽 끝에서 시작
            row_of = lambda r: math.floor((r[0][1] - oy) / cell)
            went_up = route[-1][0][1] > route[0][0][1]
            for j in sorted({row_of(r) for r in runs}, reverse=went_up):
                here = sorted((r for r in runs if row_of(r) == j), key=lambda r: min(r[0][0], r[-1][0]))
                here = [sorted(r) for r in here]
                last = route[-1][0]
                if abs(here[-1][-1][0] - last[0]) < abs(here[0][0][0] - last[0]):
                    here = [r[::-1] for r in here[::-1]]
                for run in here:
                    add(run)

        if not route:
            return []

        def turns_back(a, b, c) -> bool:
            """a → b → c 가 b 에서 120도 넘게 되돌아가는가 (막다른 자리에서 돌아 나옴)."""
            turn = math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0])
            return abs(math.remainder(turn, math.tau)) > math.radians(120.0)

        def sharp_corner(a, b, c) -> bool:
            """b 에서 corner_break_deg 이상 꺾는가 (corner_mode=angle).

            구간 가운데 모서리는 RPP 가 lookahead 0.4 m 앞을 쫓느라 안쪽으로 질러가고, 한 번 지움 반경 밖으로
            지나치면 lookahead 가 반경보다 길어 되돌아가도 못 지운다 (10/14 course_run 반경 0.15 2회차 진동).
            그래서 크게 꺾는 점은 구간 끝(도착 허용오차로 도착)으로 보낸다.
            """
            turn = abs(math.remainder(
                math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0]),
                math.tau,
            ))
            return turn >= math.radians(self.corner_break_deg) - 1e-6

        # 구간을 끊는 점: 앞이나 뒤 직선이 벽에 걸리거나, 120도 넘게 되돌아가거나, 다음 점이 0.35 m
        # 안인 점. 경로 가운데 점은 0.35 m 안에 들어오면 지워지는데(주행 BT 의 RemovePassedGoals),
        # 이런 점은 RPP 가 닿기 전에 돌아서거나 다음 점과 함께 지워져 줄 끝을 덜 간다
        radius = self.passed_goal_radius
        segments: list[list[Waypoint]] = [[]]
        for index, (point, yaw) in enumerate(route):
            before = route[index - 1][0] if index else dock
            after = route[index + 1][0] if index + 1 < len(route) else None
            segments[-1].append(Waypoint(x=point[0], y=point[1], yaw=yaw))
            if after is not None and (
                # Dock 은 벽에 붙어 있어 첫 점까지의 직선은 따지지 않는다
                (index and not self.line_clear(before, point))
                or not self.line_clear(point, after)
                or (before is not None and turns_back(before, point, after))
                or math.dist(point, after) < radius
                or (index and math.dist(before, point) < radius)
                or (
                    self.corner_mode == "angle"
                    and before is not None
                    and sharp_corner(before, point, after)
                )
            ):
                # 다음 점까지 곧게 갈 수 있으면 다음 구간도 이 점에서 시작한다 (그 줄이 구간 안에 들게).
                # 다음 점이 가까우면 그 점을 혼자 한 구간으로 끝까지 가게 한다
                segments.append(
                    [Waypoint(x=point[0], y=point[1], yaw=route[index + 1][1])]
                    if self.line_clear(point, after)
                    and math.dist(point, after) >= radius
                    else []
                )
        return segments

    def _cell_lines(
        self,
        polygon: list[tuple[float, float]],
        cell: float,
        dock: tuple[float, float] | None,
        band: int,
        bands: int,
    ) -> list[list[tuple[float, float]]]:
        """칸 한 줄의 band 번째 띠에 놓을 측정 줄(양 끝 두 점)을 Dock 쪽에서부터 지그재그 순서로.

        줄마다 벽을 피하는 높이를 띠 안에서 골라 이어진 칸을 직선 하나로 묶고, Dock 에서 갈 수 있는 줄만
        남겨 가까운 줄 끝으로 잇는다.
        """
        ox, oy = self.origin
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        # 줄 높이 후보 (칸 아래에서부터): 띠 가운데, 그다음 띠 안의 5 cm 격자.
        # 칸 가장자리 10 cm 는 빼서 위치 오차가 있어도 샘플이 그 칸에 찍히게
        lo, hi = band * cell / bands, (band + 1) * cell / bands
        middle = (lo + hi) / 2.0
        offsets = sorted(
            {round(middle, 6)} | {
                round(k * 0.05, 6) for k in range(round(cell / 0.05) + 1)
                if max(lo, 0.10) - 1e-9 <= k * 0.05 <= min(hi, cell - 0.10) + 1e-9
            },
            key=lambda d: abs(d - middle),
        )

        def point_at(i: int, dy: float):
            """칸 i 를 높이 dy(칸 아래에서부터)로 지나갈 점. x 는 칸 중심 (벽에 붙은 가장자리 열은 뺀다)."""
            x, y = ox + (i + 0.5) * cell, oy + j * cell + dy
            # 높이를 옮긴 점은 벽 픽셀과 같은 5 cm 격자라 딱 0.30 이 흔하다 → 2 cm 더
            need = self.obstacle_clearance + (0.0 if abs(dy - middle) < 1e-9 else 0.02)
            if in_polygon(x, y, polygon) and self.wall_distance(x, y) >= need - 1e-6:
                return (x, y)
            return None

        columns = range(
            math.floor((min(xs) - ox) / cell),
            math.floor((max(xs) - ox) / cell) + 1,
        )
        points: dict[tuple[int, int], tuple[float, float]] = {}
        runs: list[list[tuple[int, int]]] = []
        for j in range(
            math.floor((min(ys) - oy) / cell),
            math.floor((max(ys) - oy) / cell) + 1,
        ):
            i = columns.start
            while i < columns.stop:
                # 이 칸에서 시작해 가장 길게 이어지는 높이를 고른다
                best: list[tuple[float, float]] = []
                for dy in offsets:
                    line: list[tuple[float, float]] = []
                    for k in range(i, columns.stop):
                        point = point_at(k, dy)
                        if point is None or (line and not self.line_clear(line[-1], point)):
                            break
                        line.append(point)
                    if len(line) > len(best):
                        best = line
                if not best:
                    i += 1
                    continue
                run = [(i + k, j) for k in range(len(best))]
                points.update(zip(run, best))
                runs.append(run)
                i += len(best)

        if not runs:
            return []

        adjacent = {
            key: [
                near for near in (
                    (key[0] + 1, key[1]), (key[0] - 1, key[1]),
                    (key[0], key[1] + 1), (key[0], key[1] - 1),
                )
                if near in points and self.line_clear(points[key], points[near])
            ]
            for key in points
        }

        def hops(source):
            """source 칸에서 이웃 그래프(adjacent)로 몇 칸 만에 가는지 {칸: 칸 수} (BFS, 못 가는 칸은 없음)."""
            distance = {source: 0}
            queue = deque([source])
            while queue:
                key = queue.popleft()
                for near in adjacent[key]:
                    if near not in distance:
                        distance[near] = distance[key] + 1
                        queue.append(near)
            return distance

        home = min(
            points,
            key=lambda key: math.dist(points[key], dock) if dock else (key[1], key[0]),
        )
        from_home = hops(home)
        # 줄 안의 칸은 서로 이어져 있으므로 줄은 통째로 남거나 빠진다
        runs = [run for run in runs if run[0] in from_home]

        # 맨 아래 줄과 맨 위 줄 중 Dock 에서 가까운 쪽 끝에서 시작
        edge_rows = {runs[0][0][1], runs[-1][0][1]}
        current = min(
            (key for run in runs if run[0][1] in edge_rows for key in (run[0], run[-1])),
            key=lambda key: from_home[key],
        )
        ordered: list[list[tuple[float, float]]] = []
        remaining = list(runs)
        first_row = current[1]
        while remaining:
            distance = hops(current)
            run = min(
                remaining,
                key=lambda r: (
                    min(distance[r[0]], distance[r[-1]]),
                    abs(r[0][1] - current[1]),
                    abs(r[0][1] - first_row),       # 같으면 지나온 쪽에 남은 줄부터
                ),
            )
            remaining.remove(run)
            if distance[run[-1]] < distance[run[0]]:
                run = run[::-1]
            ordered.append([points[key] for key in (run[0], run[-1])] if len(run) > 1 else [points[run[0]]])
            current = run[-1]
        return ordered

    @staticmethod
    def _build_scanline_route(
        polygon: list[tuple[float, float]],
        spacing: float,
        bounds: tuple[float, float, float, float] | None = None,
        origin_y: float = 0.0,
        start_x: float | None = None,
    ) -> list[Waypoint]:
        """예전 측정 경로 (지도 없이): polygon 을 spacing 간격의 가로줄로 잘라 줄 양 끝을 지그재그로.

        bounds 를 주면 줄 끝을 그 사각형 안으로 자르고 줄 높이를 칸 중심(origin_y 기준)에 맞춘다.
        start_x(Dock x)가 있으면 그쪽 끝에서 시작한다. 안쪽 벽 조각은 모른다 — 칸 방식이 실기로 확인되면 지운다.
        """
        if len(polygon) < 3:
            return []

        ys = [point[1] for point in polygon]
        min_y = min(ys)
        max_y = max(ys)

        if max_y <= min_y or spacing <= 0.0:
            return []

        first_y = min_y + spacing / 2.0

        if bounds is not None:
            # 칸 중심 줄 중 polygon 과 벽 여유 안쪽에 드는 것만
            low = max(min_y, bounds[1])
            high = min(max_y, bounds[3])
            index = math.ceil((low - origin_y) / spacing - 0.5)
            scanline_ys = []

            while origin_y + (index + 0.5) * spacing < high:
                scanline_ys.append(origin_y + (index + 0.5) * spacing)
                index += 1
        elif first_y >= max_y:
            scanline_ys = [
                (min_y + max_y) / 2.0
            ]
        else:
            scanline_ys = []
            y = first_y

            while y < max_y:
                scanline_ys.append(y)
                y += spacing

        waypoints = []
        xs = [point[0] for point in polygon]
        # Dock 이 있는 쪽 끝에서 시작한다
        forward = start_x is None or start_x < (min(xs) + max(xs)) / 2.0

        for y in scanline_ys:
            intersections = []

            for index in range(len(polygon)):
                x1, y1 = polygon[index]
                x2, y2 = polygon[
                    (index + 1) % len(polygon)
                ]

                crosses = (
                    (y1 <= y < y2)
                    or (y2 <= y < y1)
                )

                if not crosses:
                    continue

                ratio = (y - y1) / (y2 - y1)
                x = x1 + ratio * (x2 - x1)
                intersections.append(x)

            intersections.sort()

            for index in range(
                0,
                len(intersections) - 1,
                2,
            ):
                left_x = intersections[index]
                right_x = intersections[index + 1]

                if bounds is not None:
                    left_x = max(left_x, bounds[0])
                    right_x = min(right_x, bounds[2])
                    if left_x >= right_x:
                        continue

                if forward:
                    start_x = left_x
                    end_x = right_x
                    yaw = 0.0
                else:
                    start_x = right_x
                    end_x = left_x
                    yaw = math.pi

                waypoints.append(
                    Waypoint(
                        x=start_x,
                        y=y,
                        yaw=yaw,
                    )
                )
                waypoints.append(
                    Waypoint(
                        x=end_x,
                        y=y,
                        yaw=yaw,
                    )
                )

                forward = not forward

        return waypoints
