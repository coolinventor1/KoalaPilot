from __future__ import annotations

from collections.abc import Sequence
import time

from openpilot.cereal import messaging

from openpilot.selfdrive.koalanav.planner import Coordinate, Maneuver, NavInstruction, NavRoute


def route_message(route_id: str, revision: int, coordinates: Sequence[Coordinate],
                  source_mono_time: int | None = None):
  """Build the route message expected from a future KoalaNav navigation provider."""
  dat = messaging.new_message("koalaNavRoute", valid=True)
  msg = dat.koalaNavRoute
  msg.routeId = route_id
  msg.revision = revision
  msg.coordinates = [{"latitude": point.latitude, "longitude": point.longitude} for point in coordinates]
  msg.sourceMonoTime = source_mono_time or time.monotonic_ns()
  return dat


def instruction_message(instruction: NavInstruction):
  """Build one maneuver update; navigation providers should publish this at 1 Hz."""
  dat = messaging.new_message("koalaNavInstruction", valid=instruction.valid)
  msg = dat.koalaNavInstruction
  msg.valid = instruction.valid
  msg.routeId = instruction.route_id
  msg.maneuver = instruction.maneuver.value
  msg.distanceToManeuver = instruction.distance_to_maneuver
  msg.targetLatitude = instruction.target.latitude
  msg.targetLongitude = instruction.target.longitude
  msg.targetBearingDeg = instruction.target_bearing_deg
  msg.sourceMonoTime = instruction.source_mono_time or time.monotonic_ns()
  msg.roadName = instruction.road_name
  return dat


def route_from_message(msg) -> NavRoute:
  return NavRoute(
    route_id=msg.routeId,
    revision=msg.revision,
    coordinates=tuple(Coordinate(point.latitude, point.longitude) for point in msg.coordinates),
    source_mono_time=msg.sourceMonoTime,
  )


def instruction_from_message(msg) -> NavInstruction:
  return NavInstruction(
    valid=msg.valid,
    route_id=msg.routeId,
    maneuver=Maneuver(str(msg.maneuver)),
    distance_to_maneuver=msg.distanceToManeuver,
    target=Coordinate(msg.targetLatitude, msg.targetLongitude),
    target_bearing_deg=msg.targetBearingDeg,
    source_mono_time=msg.sourceMonoTime,
    road_name=msg.roadName,
  )
