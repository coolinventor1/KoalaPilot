import pytest

from openpilot.selfdrive.koalanav.planner import (
  Coordinate, build_shadow_path, coordinate_to_car_frame, route_offset,
)


ORIGIN = Coordinate(41.0, -88.0)


def test_coordinate_transform_uses_openpilot_forward_left_axes():
  north = coordinate_to_car_frame(ORIGIN, 0.0, Coordinate(41.001, -88.0))
  east = coordinate_to_car_frame(ORIGIN, 0.0, Coordinate(41.0, -87.999))

  assert north.forward == pytest.approx(111.19, abs=0.2)
  assert north.left == pytest.approx(0.0, abs=0.01)
  assert east.forward == pytest.approx(0.0, abs=0.01)
  assert east.left < -80.0


def test_shadow_path_is_capped_at_150_meters_and_marks_maneuver():
  maneuver = Coordinate(41.001, -88.0)
  route = (ORIGIN, maneuver, Coordinate(41.002, -88.0))

  preview = build_shadow_path(ORIGIN, 0.0, route, maneuver)

  assert preview.length_m == pytest.approx(150.0, abs=0.01)
  assert len(preview.points) == 3
  assert preview.points[-1].forward == pytest.approx(150.0, abs=0.2)
  assert preview.points[-1].left == pytest.approx(0.0, abs=0.01)
  assert preview.maneuver_point is not None
  assert preview.maneuver_point.forward == pytest.approx(111.19, abs=0.2)


def test_maneuver_outside_preview_is_not_marked():
  route = (ORIGIN, Coordinate(41.001, -88.0), Coordinate(41.002, -88.0))

  preview = build_shadow_path(ORIGIN, 0.0, route, Coordinate(41.003, -88.0))

  assert len(preview.points) >= 2
  assert preview.maneuver_point is None


def test_preview_starts_at_projection_on_sparse_route_segment():
  position = Coordinate(41.0005, -88.0)
  route = (ORIGIN, Coordinate(41.001, -88.0))

  preview = build_shadow_path(position, 0.0, route)

  assert len(preview.points) == 2
  assert preview.points[0].forward == pytest.approx(0.0, abs=0.05)
  assert preview.points[0].left == pytest.approx(0.0, abs=0.05)
  assert preview.points[1].forward == pytest.approx(55.60, abs=0.2)
  assert route_offset(position, route) == pytest.approx(0.0, abs=0.05)
