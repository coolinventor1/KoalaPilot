from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
import math


EARTH_RADIUS_M = 6_371_000.0
MAX_GPS_ACCURACY_M = 25.0
MAX_ROUTE_OFFSET_M = 40.0
APPROACH_DISTANCE_M = 80.0
READY_DISTANCE_M = 20.0
SHADOW_PATH_DISTANCE_M = 150.0
SHADOW_PATH_STEP_M = 5.0


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
class RelativePathPoint:
  forward: float
  left: float


@dataclass(frozen=True)
class ShadowPathPreview:
  points: tuple[RelativePathPoint, ...]
  length_m: float
  maneuver_point: RelativePathPoint | None


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
  shadow_path: tuple[RelativePathPoint, ...] = ()
  shadow_path_length: float = 0.0
  road_name: str = ""
  maneuver_point: RelativePathPoint | None = None


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


def coordinate_to_car_frame(origin: Coordinate, bearing_deg: float, point: Coordinate) -> RelativePathPoint:
  """Convert a nearby WGS84 point to openpilot car space: x forward, y left."""
  latitude = math.radians(origin.latitude)
  north = EARTH_RADIUS_M * math.radians(point.latitude - origin.latitude)
  east = EARTH_RADIUS_M * math.cos(latitude) * math.radians(point.longitude - origin.longitude)
  bearing = math.radians(bearing_deg)
  forward = east * math.sin(bearing) + north * math.cos(bearing)
  left = -east * math.cos(bearing) + north * math.sin(bearing)
  return RelativePathPoint(forward, left)


def _interpolate_coordinate(start: Coordinate, end: Coordinate, fraction: float) -> Coordinate:
  return Coordinate(
    start.latitude + (end.latitude - start.latitude) * fraction,
    start.longitude + (end.longitude - start.longitude) * fraction,
  )


def _nearest_route_position(position: Coordinate, bearing_deg: float,
                            route: tuple[Coordinate, ...]) -> tuple[int, Coordinate]:
  best_distance = math.inf
  best_segment_index = 0
  best_position = route[0]

  for index, (start, end) in enumerate(zip(route, route[1:], strict=False)):
    start_relative = coordinate_to_car_frame(position, bearing_deg, start)
    end_relative = coordinate_to_car_frame(position, bearing_deg, end)
    delta_forward = end_relative.forward - start_relative.forward
    delta_left = end_relative.left - start_relative.left
    length_squared = delta_forward ** 2 + delta_left ** 2
    if length_squared <= 1e-6:
      continue

    fraction = max(0.0, min(1.0, -(
      start_relative.forward * delta_forward + start_relative.left * delta_left
    ) / length_squared))
    closest_forward = start_relative.forward + fraction * delta_forward
    closest_left = start_relative.left + fraction * delta_left
    distance = math.hypot(closest_forward, closest_left)
    if distance < best_distance:
      best_distance = distance
      best_segment_index = index
      best_position = _interpolate_coordinate(start, end, fraction)

  return best_segment_index, best_position


def route_offset(position: Coordinate, route: tuple[Coordinate, ...]) -> float:
  if len(route) < 2:
    return math.inf
  _, nearest_position = _nearest_route_position(position, 0.0, route)
  return haversine_distance(position, nearest_position)


def _resample_route(route: list[Coordinate], step_m: float = SHADOW_PATH_STEP_M) -> tuple[Coordinate, ...]:
  if len(route) < 2 or step_m <= 0.0:
    return tuple(route)

  resampled = [route[0]]
  for start, end in zip(route, route[1:], strict=False):
    segment_length = haversine_distance(start, end)
    distance = step_m
    while distance < segment_length:
      resampled.append(_interpolate_coordinate(start, end, distance / segment_length))
      distance += step_m
    if haversine_distance(resampled[-1], end) > 1e-3:
      resampled.append(end)
  return tuple(resampled)


def build_shadow_path(position: Coordinate, bearing_deg: float, route: tuple[Coordinate, ...],
                      maneuver_target: Coordinate | None = None,
                      max_distance_m: float = SHADOW_PATH_DISTANCE_M) -> ShadowPathPreview:
  """Extract and transform the next route segment without producing control commands."""
  if len(route) < 2 or not math.isfinite(bearing_deg) or max_distance_m <= 0.0:
    return ShadowPathPreview((), 0.0, None)

  segment_index, route_position = _nearest_route_position(position, bearing_deg, route)
  selected = [route_position]
  length_m = 0.0

  for next_point in route[segment_index + 1:]:
    segment_start = selected[-1]
    segment_length = haversine_distance(segment_start, next_point)
    if segment_length <= 1e-3:
      continue

    remaining = max_distance_m - length_m
    if segment_length > remaining:
      selected.append(_interpolate_coordinate(segment_start, next_point, remaining / segment_length))
      length_m = max_distance_m
      break

    selected.append(next_point)
    length_m += segment_length
    if length_m >= max_distance_m:
      break

  preview_route = _resample_route(selected)
  points = tuple(coordinate_to_car_frame(position, bearing_deg, point) for point in preview_route)
  if len(points) < 2:
    return ShadowPathPreview((), 0.0, None)

  maneuver_point = None
  if maneuver_target is not None:
    relative_target = coordinate_to_car_frame(position, bearing_deg, maneuver_target)
    target_on_preview = route_offset(maneuver_target, tuple(selected)) <= 1.0
    if relative_target.forward >= -5.0 and target_on_preview:
      maneuver_point = relative_target

  return ShadowPathPreview(points, length_m, maneuver_point)


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
    current_route_offset = route_offset(current, route.coordinates)
    target_route_offset = route_offset(instruction.target, route.coordinates)
    route_matched = current_route_offset <= MAX_ROUTE_OFFSET_M and target_route_offset <= MAX_ROUTE_OFFSET_M
    shadow_path = build_shadow_path(current, gps.bearing_deg, route.coordinates, instruction.target) if route_matched \
      else ShadowPathPreview((), 0.0, None)
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
      shadow_path=shadow_path.points,
      shadow_path_length=shadow_path.length_m,
      road_name=instruction.road_name,
      maneuver_point=shadow_path.maneuver_point,
    )
