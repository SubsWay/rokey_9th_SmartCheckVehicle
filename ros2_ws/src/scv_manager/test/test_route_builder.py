from pathlib import Path

from scv_manager.route_builder import RouteBuilder


def write_yaml(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


def test_builds_scanline_survey_route(tmp_path):
    zones_file = tmp_path / "zones.yaml"
    facilities_file = tmp_path / "facilities.yaml"
    write_yaml(
        zones_file,
        """
zones:
  - id: Z1
    polygon: [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
    spacing: 1.0
""",
    )
    write_yaml(facilities_file, "facilities: []\n")

    routes = RouteBuilder(
        str(zones_file),
        str(facilities_file),
    ).build_survey_routes()

    assert len(routes) == 1
    assert routes[0].zone_id == "Z1"
    assert [(point.x, point.y) for point in routes[0].waypoints] == [
        (0.0, 0.5),
        (2.0, 0.5),
        (2.0, 1.5),
        (0.0, 1.5),
    ]


def test_builds_facility_view_routes(tmp_path):
    zones_file = tmp_path / "zones.yaml"
    facilities_file = tmp_path / "facilities.yaml"
    write_yaml(zones_file, "zones: []\n")
    write_yaml(
        facilities_file,
        """
facilities:
  - id: EXT-01
    view: {x: 1.0, y: 2.0, yaw: 1.57}
""",
    )

    routes = RouteBuilder(
        str(zones_file),
        str(facilities_file),
    ).build_facility_routes()

    assert len(routes) == 1
    assert routes[0].facility_id == "EXT-01"
    assert routes[0].waypoints[0].x == 1.0
    assert routes[0].waypoints[0].y == 2.0
    assert routes[0].waypoints[0].yaw == 1.57


def test_rejects_zero_area_zone(tmp_path):
    zones_file = tmp_path / "zones.yaml"
    facilities_file = tmp_path / "facilities.yaml"
    write_yaml(
        zones_file,
        """
zones:
  - id: Z1
    polygon: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    spacing: 1.0
""",
    )
    write_yaml(facilities_file, "facilities: []\n")

    builder = RouteBuilder(str(zones_file), str(facilities_file))

    try:
        builder.build_survey_routes()
    except ValueError as exc:
        assert "polygon 좌표" in str(exc)
    else:
        raise AssertionError("면적이 0인 구역은 거절해야 합니다.")
