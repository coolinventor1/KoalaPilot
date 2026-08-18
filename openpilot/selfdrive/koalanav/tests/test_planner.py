from openpilot.selfdrive.koalanav.planner import (
  Coordinate, GpsFix, KoalaNavPlanner, Maneuver, NavInstruction, NavRoute, PlanState,
  haversine_distance, signed_bearing_delta,
)


ROUTE_ID = "test-route"
TARGET = Coordinate(41.00045, -88.0)
ROUTE = NavRoute(ROUTE_ID, 1, (
  Coordinate(41.0, -88.0),
  TARGET,
  Coordinate(41.00045, -87.9995),
))


def gps(latitude: float = 41.0, longitude: float = -88.0, accuracy: float = 3.0) -> GpsFix:
  return GpsFix(latitude, longitude, 10.0, 0.0, accuracy, True, 100)


def instruction(route_id: str = ROUTE_ID, distance: float = 50.0) -> NavInstruction:
  return NavInstruction(True, route_id, Maneuver.RIGHT, distance, TARGET, 90.0, 200, "Test Road")


def test_disabled_is_off_and_never_allows_control():
  plan = KoalaNavPlanner().update(False, "shadow", gps(), ROUTE, instruction())
  assert plan.state == PlanState.OFF
  assert not plan.control_allowed


def test_requires_accurate_gps():
  plan = KoalaNavPlanner().update(True, "shadow", gps(accuracy=50.0), ROUTE, instruction())
  assert plan.state == PlanState.WAITING_FOR_GPS
  assert not plan.control_allowed


def test_rejects_route_id_mismatch():
  plan = KoalaNavPlanner().update(True, "shadow", gps(), ROUTE, instruction("another-route"))
  assert plan.state == PlanState.ABORTED
  assert not plan.route_matched


def test_gps_and_route_produce_approach_plan():
  expected_distance = haversine_distance(Coordinate(41.0, -88.0), TARGET)
  plan = KoalaNavPlanner().update(True, "shadow", gps(), ROUTE, instruction(distance=expected_distance))
  assert plan.state == PlanState.APPROACH
  assert plan.route_matched
  assert abs(plan.distance_to_maneuver - expected_distance) < 0.01
  assert plan.turn_angle_deg == 90.0
  assert plan.confidence > 0.8
  assert len(plan.shadow_path) >= 2
  assert plan.shadow_path[0].forward == 0.0
  assert plan.shadow_path[0].left == 0.0
  assert plan.road_name == "Test Road"
  assert plan.maneuver_point is not None
  assert not plan.control_allowed


def test_close_target_becomes_ready_without_actuation():
  close_gps = gps(latitude=41.00040)
  distance = haversine_distance(Coordinate(close_gps.latitude, close_gps.longitude), TARGET)
  plan = KoalaNavPlanner().update(True, "simulator", close_gps, ROUTE, instruction(distance=distance))
  assert plan.state == PlanState.READY
  assert plan.mode == "simulator"
  assert not plan.control_allowed


def test_signed_bearing_delta_wraps():
  assert signed_bearing_delta(10.0, 350.0) == 20.0
  assert signed_bearing_delta(350.0, 10.0) == -20.0


def test_maneuver_must_agree_with_bearings():
  wrong_direction = NavInstruction(True, ROUTE_ID, Maneuver.LEFT, 50.0, TARGET, 90.0)
  plan = KoalaNavPlanner().update(True, "shadow", gps(), ROUTE, wrong_direction)
  assert plan.state == PlanState.ABORTED
  assert "disagrees" in plan.reason
  assert not plan.control_allowed
