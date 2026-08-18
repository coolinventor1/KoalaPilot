from openpilot.selfdrive.koalanav.mapbox import MapboxRoute, MapboxStep
from openpilot.selfdrive.koalanav.planner import Coordinate, Maneuver
from openpilot.selfdrive.koalanav.session import NavigationSession


def test_navigation_session_advances_after_passing_maneuver():
  coordinates = (
    Coordinate(41.0, -88.0),
    Coordinate(41.0001, -88.0),
    Coordinate(41.0002, -88.0),
    Coordinate(41.0002, -87.9999),
    Coordinate(41.0002, -87.9998),
  )
  steps = (
    MapboxStep("first", Maneuver.RIGHT, coordinates[2], 90.0, "Main Street", 2, "Turn right"),
    MapboxStep("second", Maneuver.LEFT, coordinates[4], 0.0, "Pine Street", 4, "Turn left"),
  )
  session = NavigationSession(MapboxRoute("route", coordinates, steps, 100.0, 20.0))

  assert session.update(coordinates[0]).step_id == "first"
  assert session.update(coordinates[2]).step_id == "first"
  assert session.update(coordinates[4]).step_id == "second"
  assert session.route_progress == 4
