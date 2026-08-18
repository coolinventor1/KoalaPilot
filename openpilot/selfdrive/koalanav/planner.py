from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
import math


EARTH_RADIUS_M = 6_371_000.0
MAX_GPS_ACCURACY_M = 25.0
MAX_ROUTE_OFFSET_M = 40.0
APPROACH_DISTANCE_M = 80.0
READY_DISTANCE_M = 20.0


class Maneuver(StrEnum):
  NONE = "none"
  LEFT = "left"
  RIGHT = "right"
  STRAIGHT = "straight"
  U_TURN = "uTurn"


class PlanState(StrEnum):
  OFF = "off"
  WAITING_FOR_GPS = "waitingForGps"
  WAITING_FOR_ROUTE = "waitingForRoute"
  WAITING_FOR_INSTRUCTION = "waitingForInstruction"
  MONITORING = "monitoring"
  APPROACH = "approach"
  READY = "ready"
  COMPLETE = "complete"
  ABORTED = "aborted"


@dataclass(frozen=True)
class Coordinate:
  latitude: float
  longitude: float


@dataclass(frozen=True)
class GpsFix:
  latitude: float
  longitude: float
  speed: float
  bearing_deg: float
  horizontal_accuracy: float
  has_fix: bool
  source_mono_time: int = 0


@dataclass(frozen=True)
class NavInstruction:
  valid: bool
  route_id: str
  maneuver: Maneuver
  distance_to_maneuver: float
  target: Coordinate
  target_bearing_deg: float
  source_mono_time: int = 0
  road_name: str = ""


@dataclass(frozen=True)
class NavRoute:
  route_id: str
  revision: int
  coordinates: tuple[Coordinate, ...]
  source_mono_time: int = 0


@dataclass(frozen=True)
class KoalaNavPlan:
  enabled: bool
  mode: str
  state: PlanState
  maneuver: Maneuver
  gps_valid: bool
  route_valid: bool
  instruction_valid: bool
  route_matched: bool
  control_allowed: bool
  latitude: float
  longitude: float
  horizontal_accuracy: float
  speed: float
  bearing_deg: float
  distance_to_maneuver: float
  target_latitude: float
  target_longitude: float
  target_bearing_deg: float
  turn_angle_deg: float
  confidence: float
  reason: str
  source_mono_time: int
  route_id: str


def valid_coordinate(point: Coordinate) -> bool:
  return (math.isfinite(point.latitude) and math.isfinite(point.longitude)
          and -90.0 <= point.latitude <= 90.0
          and -180.0 <= point.longitude <= 180.0)


def haversine_distance(a: Coordinate, b: Coordinate) -> float:
  lat1 = math.radians(a.latitude)
  lat2 = math.radians(b.latitude)
  d_lat = lat2 - lat1
  d_lon = math.radians(b.longitude - a.longitude)
  h = math.sin(d_lat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) ** 2
  return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def signed_bearing_delta(target_deg: float, current_deg: float) -> float:
  return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def _gps_valid(gps: GpsFix | None) -> bool:
  if gps is None:
    return False
  return (gps.has_fix and valid_coordinate(Coordinate(gps.latitude, gps.longitude))
          and math.isfinite(gps.horizontal_accuracy)
          and 0.0 < gps.horizontal_accuracy <= MAX_GPS_ACCURACY_M
          and math.isfinite(gps.speed) and gps.speed >= 0.0
          and math.isfinite(gps.bearing_deg))


def _route_valid(route: NavRoute | None) -> bool:
  return route is not None and bool(route.route_id) and len(route.coordinates) >= 2 and all(
    valid_coordinate(point) for point in route.coordinates
  )


def _instruction_valid(instruction: NavInstruction | None) -> bool:
  return (instruction is not None and instruction.valid and bool(instruction.route_id)
          and instruction.maneuver != Maneuver.NONE and valid_coordinate(instruction.target)
          and math.isfinite(instruction.distance_to_maneuver) and instruction.distance_to_maneuver >= 0.0
          and math.isfinite(instruction.target_bearing_deg))


def _empty_plan(enabled: bool, mode: str, state: PlanState, reason: str,
                gps: GpsFix | None = None, route: NavRoute | None = None,
                instruction: NavInstruction | None = None) -> KoalaNavPlan:
  return KoalaNavPlan(
    enabled=enabled,
    mode=mode,
    state=state,
    maneuver=instruction.maneuver if instruction is not None else Maneuver.NONE,
    gps_valid=_gps_valid(gps),
    route_valid=_route_valid(route),
    instruction_valid=_instruction_valid(instruction),
    route_matched=False,
    control_allowed=False,
    latitude=gps.latitude if gps is not None else 0.0,
    longitude=gps.longitude if gps is not None else 0.0,
    horizontal_accuracy=gps.horizontal_accuracy if gps is not None else 0.0,
    speed=gps.speed if gps is not None else 0.0,
    bearing_deg=gps.bearing_deg if gps is not None else 0.0,
    distance_to_maneuver=instruction.distance_to_maneuver if instruction is not None else 0.0,
    target_latitude=instruction.target.latitude if instruction is not None else 0.0,
    target_longitude=instruction.target.longitude if instruction is not None else 0.0,
    target_bearing_deg=instruction.target_bearing_deg if instruction is not None else 0.0,
    turn_angle_deg=0.0,
    confidence=0.0,
    reason=reason,
    source_mono_time=max(gps.source_mono_time if gps is not None else 0,
                         route.source_mono_time if route is not None else 0,
                         instruction.source_mono_time if instruction is not None else 0),
    route_id=instruction.route_id if instruction is not None else (route.route_id if route is not None else ""),
  )


class KoalaNavPlanner:
  """Fuses GPS and navigation data without producing vehicle-control commands."""

  def update(self, enabled: bool, mode: str, gps: GpsFix | None,
             route: NavRoute | None, instruction: NavInstruction | None) -> KoalaNavPlan:
    normalized_mode = mode if mode in ("shadow", "simulator") else "shadow"
    if not enabled:
      return _empty_plan(False, "off", PlanState.OFF, "KoalaNav is disabled", gps, route, instruction)
    if not _gps_valid(gps):
      return _empty_plan(True, normalized_mode, PlanState.WAITING_FOR_GPS,
                         "waiting for an accurate GPS fix", gps, route, instruction)
    if not _route_valid(route):
      return _empty_plan(True, normalized_mode, PlanState.WAITING_FOR_ROUTE,
                         "waiting for a valid navigation route", gps, route, instruction)
    if not _instruction_valid(instruction):
      return _empty_plan(True, normalized_mode, PlanState.WAITING_FOR_INSTRUCTION,
                         "waiting for a valid turn instruction", gps, route, instruction)

    assert gps is not None and route is not None and instruction is not None
    if route.route_id != instruction.route_id:
      return _empty_plan(True, normalized_mode, PlanState.ABORTED,
                         "route and instruction identifiers do not match", gps, route, instruction)

    current = Coordinate(gps.latitude, gps.longitude)
    distance_to_target = haversine_distance(current, instruction.target)
    current_route_offset = min(haversine_distance(current, point) for point in route.coordinates)
    target_route_offset = min(haversine_distance(instruction.target, point) for point in route.coordinates)
    route_matched = current_route_offset <= MAX_ROUTE_OFFSET_M and target_route_offset <= MAX_ROUTE_OFFSET_M
    # Local import avoids a module cycle while keeping turn intent isolated from the route/GPS fusion code.
    from openpilot.selfdrive.koalanav.turn_desire import derive_turn_desire
    turn_desire = derive_turn_desire(instruction.maneuver, gps.bearing_deg, instruction.target_bearing_deg)
    turn_angle = turn_desire.signed_angle_deg

    gps_score = max(0.0, 1.0 - gps.horizontal_accuracy / MAX_GPS_ACCURACY_M)
    route_score = max(0.0, 1.0 - max(current_route_offset, target_route_offset) / MAX_ROUTE_OFFSET_M)
    distance_error = abs(distance_to_target - instruction.distance_to_maneuver)
    distance_tolerance = max(20.0, distance_to_target * 0.35)
    distance_score = max(0.0, 1.0 - distance_error / distance_tolerance)
    direction_score = 1.0 if turn_desire.direction_consistent else 0.0
    confidence = max(0.0, min(1.0, 0.40 * gps_score + 0.30 * route_score
                              + 0.20 * distance_score + 0.10 * direction_score))

    if not route_matched:
      state = PlanState.ABORTED
      reason = "GPS position or maneuver target is outside the supplied route"
    elif not turn_desire.direction_consistent:
      state = PlanState.ABORTED
      reason = "navigation maneuver disagrees with the GPS and outgoing-road bearings"
    elif distance_to_target <= READY_DISTANCE_M:
      state = PlanState.READY
      reason = "maneuver target is within the ready distance"
    elif distance_to_target <= APPROACH_DISTANCE_M:
      state = PlanState.APPROACH
      reason = "approaching the navigation maneuver"
    else:
      state = PlanState.MONITORING
      reason = "route matched; monitoring the upcoming maneuver"

    return KoalaNavPlan(
      enabled=True,
      mode=normalized_mode,
      state=state,
      maneuver=instruction.maneuver,
      gps_valid=True,
      route_valid=True,
      instruction_valid=True,
      route_matched=route_matched,
      control_allowed=False,
      latitude=gps.latitude,
      longitude=gps.longitude,
      horizontal_accuracy=gps.horizontal_accuracy,
      speed=gps.speed,
      bearing_deg=gps.bearing_deg,
      distance_to_maneuver=distance_to_target,
      target_latitude=instruction.target.latitude,
      target_longitude=instruction.target.longitude,
      target_bearing_deg=instruction.target_bearing_deg,
      turn_angle_deg=turn_angle,
      confidence=confidence,
      reason=reason,
      source_mono_time=max(gps.source_mono_time, route.source_mono_time, instruction.source_mono_time),
      route_id=route.route_id,
    )
