from __future__ import annotations

from dataclasses import dataclass

from openpilot.selfdrive.koalanav.planner import Maneuver, signed_bearing_delta


@dataclass(frozen=True)
class TurnDesire:
  """Navigation intent only; this contains no curvature, torque, or actuator request."""

  maneuver: Maneuver
  signed_angle_deg: float
  direction_consistent: bool


def derive_turn_desire(maneuver: Maneuver, current_bearing_deg: float,
                       outgoing_bearing_deg: float) -> TurnDesire:
  angle = signed_bearing_delta(outgoing_bearing_deg, current_bearing_deg)
  if maneuver == Maneuver.LEFT:
    consistent = angle < -10.0
  elif maneuver == Maneuver.RIGHT:
    consistent = angle > 10.0
  elif maneuver == Maneuver.STRAIGHT:
    consistent = abs(angle) <= 35.0
  elif maneuver == Maneuver.U_TURN:
    consistent = abs(angle) >= 120.0
  else:
    consistent = False
  return TurnDesire(maneuver, angle, consistent)
