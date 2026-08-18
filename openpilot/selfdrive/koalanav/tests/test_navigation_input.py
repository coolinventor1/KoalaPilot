from openpilot.selfdrive.koalanav.koalanavd import _publish
from openpilot.selfdrive.koalanav.navigation_input import (
  instruction_from_message, instruction_message, route_from_message, route_message,
)
from openpilot.selfdrive.koalanav.planner import (
  Coordinate, GpsFix, KoalaNavPlanner, Maneuver, NavInstruction, NavRoute,
)


class RecordingPubMaster:
  def __init__(self):
    self.service = None
    self.message = None

  def send(self, service, message):
    self.service = service
    self.message = message


def test_navigation_messages_round_trip():
  route = NavRoute("route-42", 7, (Coordinate(41.0, -88.0), Coordinate(41.001, -88.0)), 123)
  instruction = NavInstruction(True, "route-42", Maneuver.LEFT, 111.0,
                               Coordinate(41.001, -88.0), 270.0, 456, "Oak Street")

  decoded_route = route_from_message(route_message(
    route.route_id, route.revision, route.coordinates, route.source_mono_time,
  ).koalaNavRoute)
  decoded_instruction = instruction_from_message(instruction_message(instruction).koalaNavInstruction)

  assert decoded_route == route
  assert decoded_instruction == instruction


def test_published_shadow_plan_has_no_control_authority():
  route = NavRoute("route-42", 7, (Coordinate(41.0, -88.0), Coordinate(41.001, -88.0)), 123)
  instruction = NavInstruction(True, "route-42", Maneuver.RIGHT, 111.0,
                               Coordinate(41.001, -88.0), 90.0, 456)
  gps = GpsFix(41.0, -88.0, 9.0, 0.0, 3.0, True, 789)
  plan = KoalaNavPlanner().update(True, "shadow", gps, route, instruction)
  pm = RecordingPubMaster()

  _publish(pm, plan)

  assert pm.service == "koalaNavPlan"
  assert pm.message.which() == "koalaNavPlan"
  assert pm.message.koalaNavPlan.mode == "shadow"
  assert pm.message.koalaNavPlan.routeId == "route-42"
  assert pm.message.koalaNavPlan.shadowPathValid
  assert len(pm.message.koalaNavPlan.shadowPath) >= 2
  assert pm.message.koalaNavPlan.roadName == ""
  assert pm.message.koalaNavPlan.maneuverPointValid
  assert not pm.message.koalaNavPlan.controlAllowed
