from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import urllib.error
import urllib.parse
import urllib.request

from openpilot.selfdrive.koalanav.planner import Coordinate, Maneuver, haversine_distance, signed_bearing_delta


MAPBOX_DIRECTIONS_URL = "https://api.mapbox.com/directions/v5/mapbox/driving-traffic"


class MapboxError(RuntimeError):
  pass


@dataclass(frozen=True)
class MapboxStep:
  step_id: str
  maneuver: Maneuver
  target: Coordinate
  target_bearing_deg: float
  road_name: str
  route_index: int
  provider_instruction: str


@dataclass(frozen=True)
class MapboxRoute:
  route_id: str
  coordinates: tuple[Coordinate, ...]
  steps: tuple[MapboxStep, ...]
  distance_m: float
  duration_s: float


def _maneuver_for_step(step: dict) -> Maneuver | None:
  maneuver = step.get("maneuver", {})
  maneuver_type = str(maneuver.get("type", "")).lower()
  if maneuver_type in ("depart", "arrive", "notification"):
    return None

  modifier = str(maneuver.get("modifier", "")).lower()
  if "uturn" in modifier or "u-turn" in modifier:
    return Maneuver.U_TURN
  if "left" in modifier:
    return Maneuver.LEFT
  if "right" in modifier:
    return Maneuver.RIGHT
  if modifier == "straight":
    return Maneuver.STRAIGHT

  before = float(maneuver.get("bearing_before", 0.0))
  after = float(maneuver.get("bearing_after", before))
  delta = signed_bearing_delta(after, before)
  if delta < -35.0:
    return Maneuver.LEFT
  if delta > 35.0:
    return Maneuver.RIGHT
  return Maneuver.STRAIGHT


def _nearest_route_index(coordinates: tuple[Coordinate, ...], target: Coordinate) -> int:
  return min(range(len(coordinates)), key=lambda index: haversine_distance(coordinates[index], target))


def parse_mapbox_route(payload: dict) -> MapboxRoute:
  if payload.get("code") != "Ok" or not payload.get("routes"):
    raise MapboxError(f"Mapbox returned {payload.get('code', 'an invalid response')}")

  route = payload["routes"][0]
  raw_coordinates = route.get("geometry", {}).get("coordinates", [])
  coordinates = tuple(Coordinate(float(latitude), float(longitude)) for longitude, latitude in raw_coordinates)
  if len(coordinates) < 2:
    raise MapboxError("Mapbox route contains no usable geometry")

  raw_steps = [step for leg in route.get("legs", []) for step in leg.get("steps", [])]
  steps: list[MapboxStep] = []
  for sequence, step in enumerate(raw_steps):
    mapped_maneuver = _maneuver_for_step(step)
    maneuver = step.get("maneuver", {})
    location = maneuver.get("location", [])
    if mapped_maneuver is None or len(location) != 2:
      continue
    target = Coordinate(float(location[1]), float(location[0]))
    road_name = str(step.get("name") or step.get("rotary_name") or "").strip()
    route_index = _nearest_route_index(coordinates, target)
    step_id = f"{sequence}:{route_index}:{mapped_maneuver.value}"
    steps.append(MapboxStep(
      step_id=step_id,
      maneuver=mapped_maneuver,
      target=target,
      target_bearing_deg=float(maneuver.get("bearing_after", 0.0)) % 360.0,
      road_name=road_name,
      route_index=route_index,
      provider_instruction=str(maneuver.get("instruction", "")),
    ))

  identity = json.dumps({
    "coordinates": raw_coordinates,
    "steps": [(step.step_id, step.road_name) for step in steps],
  }, separators=(",", ":"), sort_keys=True).encode()
  route_id = hashlib.sha256(identity).hexdigest()
  return MapboxRoute(route_id, coordinates, tuple(steps), float(route.get("distance", 0.0)), float(route.get("duration", 0.0)))


class MapboxClient:
  def __init__(self, access_token: str, timeout: float = 20.0):
    if not access_token:
      raise ValueError("a Mapbox access token is required")
    self.access_token = access_token
    self.timeout = timeout

  def route(self, origin: Coordinate, destination: Coordinate, bearing_deg: float | None = None) -> MapboxRoute:
    coordinates = f"{origin.longitude:.7f},{origin.latitude:.7f};{destination.longitude:.7f},{destination.latitude:.7f}"
    query: dict[str, str] = {
      "access_token": self.access_token,
      "alternatives": "false",
      "geometries": "geojson",
      "language": "en",
      "overview": "full",
      "steps": "true",
    }
    if bearing_deg is not None and math.isfinite(bearing_deg):
      query["bearings"] = f"{bearing_deg % 360.0:.0f},45;"
    url = f"{MAPBOX_DIRECTIONS_URL}/{coordinates}?{urllib.parse.urlencode(query)}"
    request = urllib.request.Request(url, headers={"Accept": "application/json", "User-Agent": "KoalaPilot/KoalaNav"})
    try:
      with urllib.request.urlopen(request, timeout=self.timeout) as response:
        payload = json.load(response)
    except urllib.error.HTTPError as error:
      detail = error.read().decode(errors="replace")[:300]
      raise MapboxError(f"Mapbox request failed ({error.code}): {detail}") from error
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as error:
      raise MapboxError(f"Mapbox request failed: {error}") from error
    return parse_mapbox_route(payload)
