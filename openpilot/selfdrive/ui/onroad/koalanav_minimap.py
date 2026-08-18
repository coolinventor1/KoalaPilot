from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import math
import os
import time
import urllib.error
import urllib.parse
import urllib.request

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.koalanav.planner import EARTH_RADIUS_M
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached


MAPBOX_STATIC_URL = "https://api.mapbox.com/styles/v1/mapbox/streets-v12/static"
MAP_REQUEST_SIZE = 320
MAP_ZOOM = 15.8
MAP_REFRESH_DISTANCE_M = 120.0
MAP_REFRESH_BEARING_DEG = 22.0
MAP_MIN_REFRESH_S = 5.0
MAP_MAX_REFRESH_S = 20.0
MAP_RETRY_S = 30.0
MAP_MAX_BYTES = 5 * 1024 * 1024
MAP_PANEL_SIZE = 290.0
MAP_PANEL_MARGIN = 30.0
MAP_BORDER_COLOR = rl.Color(255, 255, 255, 215)
MAP_ROUTE_COLOR = rl.Color(36, 198, 255, 225)
MAP_DOT_COLOR = rl.Color(36, 198, 255, 255)
MAP_ARROW_COLOR = rl.Color(255, 255, 255, 245)
MAP_ARROW_OUTLINE_COLOR = rl.Color(11, 34, 46, 210)
MAP_LOADING_COLOR = rl.Color(220, 235, 240, 220)


@dataclass(frozen=True)
class MapAnchor:
  latitude: float
  longitude: float
  bearing_deg: float


@dataclass(frozen=True)
class MapDownload:
  anchor: MapAnchor
  png: bytes


def signed_bearing_delta(target_deg: float, current_deg: float) -> float:
  return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def geographic_offset_m(anchor: MapAnchor, latitude: float, longitude: float) -> tuple[float, float]:
  """Return east and north offset from an anchor using a local tangent plane."""
  latitude_rad = math.radians(anchor.latitude)
  north = EARTH_RADIUS_M * math.radians(latitude - anchor.latitude)
  east = EARTH_RADIUS_M * math.cos(latitude_rad) * math.radians(longitude - anchor.longitude)
  return east, north


def geographic_distance_m(anchor: MapAnchor, latitude: float, longitude: float) -> float:
  east, north = geographic_offset_m(anchor, latitude, longitude)
  return math.hypot(east, north)


def meters_per_pixel(latitude: float, zoom: float = MAP_ZOOM) -> float:
  latitude = max(-85.0, min(85.0, latitude))
  return math.cos(math.radians(latitude)) * 2.0 * math.pi * EARTH_RADIUS_M / (256.0 * 2.0 ** zoom)


def mapbox_static_url(token: str, anchor: MapAnchor) -> str:
  center = f"{anchor.longitude:.7f},{anchor.latitude:.7f},{MAP_ZOOM:.2f},{anchor.bearing_deg % 360.0:.1f},0"
  query = urllib.parse.urlencode({
    "access_token": token,
    "logo": "true",
    "attribution": "true",
  })
  return f"{MAPBOX_STATIC_URL}/{center}/{MAP_REQUEST_SIZE}x{MAP_REQUEST_SIZE}@2x?{query}"


def download_mapbox_static(token: str, anchor: MapAnchor) -> MapDownload:
  request = urllib.request.Request(mapbox_static_url(token, anchor), headers={
    "Accept": "image/png",
    "User-Agent": "KoalaPilot-MiniMap/1.0",
  })
  try:
    with urllib.request.urlopen(request, timeout=10.0) as response:
      content_type = response.headers.get_content_type()
      png = response.read(MAP_MAX_BYTES + 1)
  except urllib.error.HTTPError as error:
    raise RuntimeError(f"Mapbox mini-map request failed with HTTP {error.code}") from error
  except (urllib.error.URLError, TimeoutError) as error:
    raise RuntimeError("Mapbox mini-map request failed") from error
  if content_type != "image/png" or not png.startswith(b"\x89PNG\r\n\x1a\n"):
    raise RuntimeError("Mapbox mini-map returned an unsupported image")
  if len(png) > MAP_MAX_BYTES:
    raise RuntimeError("Mapbox mini-map image exceeded the size limit")
  return MapDownload(anchor, png)


def map_screen_offset(anchor: MapAnchor, east_m: float, north_m: float, panel_scale: float) -> tuple[float, float]:
  bearing = math.radians(anchor.bearing_deg)
  right = east_m * math.cos(bearing) - north_m * math.sin(bearing)
  forward = east_m * math.sin(bearing) + north_m * math.cos(bearing)
  scale = panel_scale / meters_per_pixel(anchor.latitude)
  return right * scale, -forward * scale


def car_path_screen_offset(anchor: MapAnchor, current_bearing_deg: float, forward_m: float, left_m: float,
                           panel_scale: float) -> tuple[float, float]:
  delta = math.radians(current_bearing_deg - anchor.bearing_deg)
  right = forward_m * math.sin(delta) - left_m * math.cos(delta)
  map_forward = forward_m * math.cos(delta) + left_m * math.sin(delta)
  scale = panel_scale / meters_per_pixel(anchor.latitude)
  return right * scale, -map_forward * scale


class KoalaNavMiniMap:
  def __init__(self):
    params = Params()
    self._token = os.environ.get("MAPBOX_ACCESS_TOKEN") or params.get("MapboxAccessToken") or ""
    self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="koalanav-map")
    self._future: Future[MapDownload] | None = None
    self._texture: rl.Texture | None = None
    self._anchor: MapAnchor | None = None
    self._requested_at = 0.0
    self._retry_at = 0.0
    self._font = gui_app.font(FontWeight.SEMI_BOLD)

  @property
  def ready(self) -> bool:
    return self._texture is not None and self._anchor is not None

  def _rounded_texture(self, png: bytes) -> rl.Texture:
    image = rl.load_image_from_memory(".png", png, len(png))
    if not rl.is_image_valid(image):
      raise RuntimeError("Mapbox mini-map PNG could not be decoded")

    radius = max(1, round(min(image.width, image.height) * 0.085))
    mask = rl.gen_image_color(image.width, image.height, rl.BLACK)
    rl.image_draw_rectangle(mask, radius, 0, image.width - 2 * radius, image.height, rl.WHITE)
    rl.image_draw_rectangle(mask, 0, radius, image.width, image.height - 2 * radius, rl.WHITE)
    for x, y in ((radius, radius), (image.width - radius - 1, radius),
                 (radius, image.height - radius - 1), (image.width - radius - 1, image.height - radius - 1)):
      rl.image_draw_circle(mask, x, y, radius, rl.WHITE)
    rl.image_alpha_mask(image, mask)
    rl.unload_image(mask)
    return gui_app._load_texture_from_image(image)

  def _finish_download(self, now: float) -> None:
    if self._future is None or not self._future.done():
      return
    try:
      result = self._future.result()
      texture = self._rounded_texture(result.png)
      if self._texture is not None:
        rl.unload_texture(self._texture)
      self._texture = texture
      self._anchor = result.anchor
    except Exception:
      self._retry_at = now + MAP_RETRY_S
    finally:
      self._future = None

  def _request_due(self, nav, now: float) -> bool:
    if not self._token or self._future is not None or now < self._retry_at:
      return False
    if self._anchor is None:
      return True
    elapsed = now - self._requested_at
    moved = geographic_distance_m(self._anchor, float(nav.latitude), float(nav.longitude))
    turned = abs(signed_bearing_delta(float(nav.bearingDeg), self._anchor.bearing_deg))
    return elapsed >= MAP_MAX_REFRESH_S or (elapsed >= MAP_MIN_REFRESH_S and (
      moved >= MAP_REFRESH_DISTANCE_M or turned >= MAP_REFRESH_BEARING_DEG
    ))

  def update(self, nav) -> None:
    now = time.monotonic()
    self._finish_download(now)
    if not self._request_due(nav, now):
      return
    latitude = float(nav.latitude)
    longitude = float(nav.longitude)
    bearing = float(nav.bearingDeg)
    if not (math.isfinite(latitude) and math.isfinite(longitude) and math.isfinite(bearing)
            and -85.0 <= latitude <= 85.0 and -180.0 <= longitude <= 180.0):
      return
    anchor = MapAnchor(latitude, longitude, bearing % 360.0)
    self._requested_at = now
    self._future = self._executor.submit(download_mapbox_static, self._token, anchor)

  @staticmethod
  def _draw_turn_arrow(points: list[rl.Vector2], maneuver_index: int) -> None:
    if len(points) < 2:
      return
    tip_index = min(len(points) - 1, maneuver_index + 2)
    base_index = max(0, maneuver_index - 2)
    if tip_index == base_index:
      base_index = max(0, tip_index - 1)
    base = points[base_index]
    tip = points[tip_index]
    dx, dy = tip.x - base.x, tip.y - base.y
    length = math.hypot(dx, dy)
    if length < 2.0:
      return
    unit_x, unit_y = dx / length, dy / length
    normal_x, normal_y = -unit_y, unit_x
    arrow_length = 17.0
    wing = 11.0
    wing_center = rl.Vector2(tip.x - unit_x * arrow_length, tip.y - unit_y * arrow_length)
    wing_a = rl.Vector2(wing_center.x + normal_x * wing, wing_center.y + normal_y * wing)
    wing_b = rl.Vector2(wing_center.x - normal_x * wing, wing_center.y - normal_y * wing)
    for width, color in ((8.0, MAP_ARROW_OUTLINE_COLOR), (4.0, MAP_ARROW_COLOR)):
      rl.draw_line_ex(wing_a, tip, width, color)
      rl.draw_line_ex(wing_b, tip, width, color)

  def _draw_route(self, panel: rl.Rectangle, nav) -> None:
    if self._anchor is None:
      return
    panel_scale = panel.width / MAP_REQUEST_SIZE
    east, north = geographic_offset_m(self._anchor, float(nav.latitude), float(nav.longitude))
    current_dx, current_dy = map_screen_offset(self._anchor, east, north, panel_scale)
    current = rl.Vector2(panel.x + panel.width / 2.0 + current_dx, panel.y + panel.height / 2.0 + current_dy)

    route_points: list[rl.Vector2] = []
    maneuver_index = 0
    maneuver_error = float("inf")
    for index, point in enumerate(nav.shadowPath):
      forward, left = float(point.forward), float(point.left)
      dx, dy = car_path_screen_offset(self._anchor, float(nav.bearingDeg), forward, left, panel_scale)
      route_points.append(rl.Vector2(current.x + dx, current.y + dy))
      if nav.maneuverPointValid:
        error = math.hypot(forward - float(nav.maneuverForward), left - float(nav.maneuverLeft))
        if error < maneuver_error:
          maneuver_error = error
          maneuver_index = index

    for start, end in zip(route_points, route_points[1:], strict=False):
      rl.draw_line_ex(start, end, 7.0, MAP_ROUTE_COLOR)
    if nav.maneuverPointValid:
      self._draw_turn_arrow(route_points, maneuver_index)

    heading_delta = math.radians(float(nav.bearingDeg) - self._anchor.bearing_deg)
    heading = rl.Vector2(current.x + math.sin(heading_delta) * 15.0,
                         current.y - math.cos(heading_delta) * 15.0)
    rl.draw_line_ex(current, heading, 7.0, rl.WHITE)
    rl.draw_circle(int(current.x), int(current.y), 11.0, rl.WHITE)
    rl.draw_circle(int(current.x), int(current.y), 7.0, MAP_DOT_COLOR)

  def render(self, rect: rl.Rectangle, nav) -> None:
    self.update(nav)
    size = min(MAP_PANEL_SIZE, rect.width * 0.22, rect.height * 0.30)
    panel = rl.Rectangle(rect.x + rect.width - size - MAP_PANEL_MARGIN,
                         rect.y + rect.height - size - MAP_PANEL_MARGIN, size, size)
    rl.draw_rectangle_rounded(panel, 0.16, 12, rl.Color(0, 0, 0, 205))

    if self._texture is not None:
      source = rl.Rectangle(0, 0, self._texture.width, self._texture.height)
      rl.draw_texture_pro(self._texture, source, panel, rl.Vector2(0, 0), 0.0, rl.WHITE)
    else:
      label = "MAP LOADING" if self._token else "MAP TOKEN MISSING"
      measured = measure_text_cached(self._font, label, 25)
      rl.draw_text_ex(self._font, label,
                      rl.Vector2(panel.x + (panel.width - measured.x) / 2.0,
                                 panel.y + (panel.height - measured.y) / 2.0), 25, 0, MAP_LOADING_COLOR)

    rl.begin_scissor_mode(int(panel.x), int(panel.y), int(panel.width), int(panel.height))
    self._draw_route(panel, nav)
    rl.end_scissor_mode()
    rl.draw_rectangle_rounded_lines_ex(panel, 0.16, 12, 3.0, MAP_BORDER_COLOR)
