from openpilot.selfdrive.koalanav.planner import Maneuver
from openpilot.selfdrive.koalanav.turn_desire import derive_turn_desire


def test_turn_desire_classifies_direction_without_actuation():
  left = derive_turn_desire(Maneuver.LEFT, 0.0, 270.0)
  right = derive_turn_desire(Maneuver.RIGHT, 350.0, 80.0)
  straight = derive_turn_desire(Maneuver.STRAIGHT, 350.0, 10.0)

  assert left.signed_angle_deg == -90.0 and left.direction_consistent
  assert right.signed_angle_deg == 90.0 and right.direction_consistent
  assert straight.signed_angle_deg == 20.0 and straight.direction_consistent
