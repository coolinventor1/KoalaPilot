from __future__ import annotations

import numpy as np
import pyray as rl

from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.onroad.koalanav_minimap import KoalaNavMiniMap
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets import Widget


PATH_COLOR = rl.Color(36, 198, 255, 145)
ARROW_TAIL_ALPHA = 70
ARROW_BASE_ALPHA = 135
ARROW_TIP_ALPHA = 165
ROUTE_NEAR_ALPHA = 120
ROUTE_FAR_ALPHA = 80
PATH_WIDTH_M = 0.42
ARROW_BODY_WIDTH_M = 0.90
ARROW_TAIL_FORWARD_M = 1.5
ARROW_TIP_DISTANCE_M = 17.0
ARROW_LENGTH_M = 8.0
ARROW_WIDTH_M = 2.45
TURN_ARROW_AFTER_MANEUVER_M = 10.0
TURN_ARROW_LENGTH_M = 10.0
TURN_ARROW_WIDTH_M = 12.0
TURN_ARROW_MIN_SCREEN_WIDTH = 28.0
PANEL_COLOR = rl.Color(0, 0, 0, 185)
PANEL_BORDER_COLOR = rl.Color(55, 205, 255, 210)
TEXT_COLOR = rl.Color(255, 255, 255, 255)
SECONDARY_TEXT_COLOR = rl.Color(220, 235, 240, 230)


class KoalaNavRenderer(Widget):
  """Draw the route preview only; this renderer has no control or messaging outputs."""

  def __init__(self):
    super().__init__()
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)
    self._mini_map = KoalaNavMiniMap()

  def set_transform(self, transform: np.ndarray) -> None:
    self._car_space_transform = transform.astype(np.float32)

  @staticmethod
  def _distance_text(distance_m: float) -> str:
    if ui_state.is_metric:
      return f"{round(distance_m):.0f} m" if distance_m < 1000.0 else f"{distance_m / 1000.0:.1f} km"

    distance_ft = distance_m * 3.28084
    return f"{round(distance_ft):.0f} ft" if distance_ft < 1000.0 else f"{distance_m / 1609.344:.1f} mi"

  def _project(self, forward: float, left: float, height: float, rect: rl.Rectangle) -> rl.Vector2 | None:
    if forward < -2.0:
      return None

    projected = self._car_space_transform @ np.array([forward, left, height], dtype=np.float32)
    if not np.all(np.isfinite(projected)) or abs(projected[2]) < 1e-6:
      return None

    x = float(projected[0] / projected[2])
    y = float(projected[1] / projected[2])
    margin = 250.0
    if not (rect.x - margin <= x <= rect.x + rect.width + margin and
            rect.y - margin <= y <= rect.y + rect.height + margin):
      return None
    return rl.Vector2(x, y)

  @staticmethod
  def _offset_path(points: list[tuple[float, float]], width_m: float) -> tuple[list[tuple[float, float]],
                                                                                 list[tuple[float, float]]]:
    """Return both edges of a constant-width path in car space."""
    edge_a: list[tuple[float, float]] = []
    edge_b: list[tuple[float, float]] = []
    half_width = width_m / 2.0

    for i, (forward, left) in enumerate(points):
      previous = points[max(0, i - 1)]
      following = points[min(len(points) - 1, i + 1)]
      tangent_forward = following[0] - previous[0]
      tangent_left = following[1] - previous[1]
      tangent_length = float(np.hypot(tangent_forward, tangent_left))
      if tangent_length < 1e-3:
        normal_forward, normal_left = 0.0, 1.0
      else:
        normal_forward = -tangent_left / tangent_length
        normal_left = tangent_forward / tangent_length

      edge_a.append((forward + normal_forward * half_width, left + normal_left * half_width))
      edge_b.append((forward - normal_forward * half_width, left - normal_left * half_width))

    return edge_a, edge_b

  @staticmethod
  def _split_for_arrow(points: list[tuple[float, float]], arrow_length_m: float) -> tuple[
      list[tuple[float, float]], tuple[float, float], tuple[float, float]]:
    """Split a path at a point measured back from its tip."""
    if len(points) < 2:
      return points, points[-1], points[-1]

    remaining = arrow_length_m
    for i in range(len(points) - 1, 0, -1):
      start = points[i - 1]
      end = points[i]
      segment_length = float(np.hypot(end[0] - start[0], end[1] - start[1]))
      if segment_length < 1e-3:
        continue
      if segment_length >= remaining:
        fraction = (segment_length - remaining) / segment_length
        base = (start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction)
        return [*points[:i], base], base, points[-1]
      remaining -= segment_length

    return [points[0]], points[0], points[-1]

  @staticmethod
  def _path_prefix(points: list[tuple[float, float]], distance_m: float) -> list[tuple[float, float]]:
    """Clip a path at a distance measured from its first point."""
    if len(points) < 2:
      return points
    if distance_m <= 0.0:
      return [points[0]]

    prefix = [points[0]]
    remaining = distance_m
    for start, end in zip(points, points[1:], strict=False):
      segment_length = float(np.hypot(end[0] - start[0], end[1] - start[1]))
      if segment_length < 1e-3:
        continue
      if segment_length >= remaining:
        fraction = remaining / segment_length
        prefix.append((start[0] + (end[0] - start[0]) * fraction,
                       start[1] + (end[1] - start[1]) * fraction))
        return prefix
      prefix.append(end)
      remaining -= segment_length

    return prefix

  @staticmethod
  def _path_suffix(points: list[tuple[float, float]], distance_m: float) -> list[tuple[float, float]]:
    """Clip the beginning of a path at a distance measured from its first point."""
    if len(points) < 2 or distance_m <= 0.0:
      return points

    remaining = distance_m
    for i, (start, end) in enumerate(zip(points, points[1:], strict=False)):
      segment_length = float(np.hypot(end[0] - start[0], end[1] - start[1]))
      if segment_length < 1e-3:
        continue
      if segment_length >= remaining:
        fraction = remaining / segment_length
        split = (start[0] + (end[0] - start[0]) * fraction,
                 start[1] + (end[1] - start[1]) * fraction)
        return [split, *points[i + 1:]]
      remaining -= segment_length

    return [points[-1]]

  @staticmethod
  def _extend_tail_to_camera(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Start the arrow below the camera viewport so its tail has no visible endpoint."""
    if len(points) < 2 or points[0][0] >= ARROW_TAIL_FORWARD_M:
      return points

    first, following = points[0], points[1]
    forward_delta = following[0] - first[0]
    if abs(forward_delta) < 1e-3:
      tail_left = first[1]
    else:
      fraction = (ARROW_TAIL_FORWARD_M - first[0]) / forward_delta
      tail_left = first[1] + (following[1] - first[1]) * fraction
    return [(ARROW_TAIL_FORWARD_M, tail_left), *points[1:]]

  @staticmethod
  def _distance_along_path_nearest(points: list[tuple[float, float]], target: tuple[float, float]) -> float:
    """Return path distance to the sampled point closest to a car-space target."""
    nearest_distance = 0.0
    nearest_error = float("inf")
    distance_along = 0.0
    previous = points[0]
    for point in points:
      distance_along += float(np.hypot(point[0] - previous[0], point[1] - previous[1]))
      error = float(np.hypot(point[0] - target[0], point[1] - target[1]))
      if error < nearest_error:
        nearest_error = error
        nearest_distance = distance_along
      previous = point
    return nearest_distance

  def _draw_road_ribbon(self, rect: rl.Rectangle, points: list[tuple[float, float]], height: float,
                        width_m: float, color: rl.Color, start_alpha: int | None = None,
                        end_alpha: int | None = None) -> None:
    if len(points) < 2:
      return

    edge_a, edge_b = self._offset_path(points, width_m)
    projected_a = [self._project(forward, left, height, rect) for forward, left in edge_a]
    projected_b = [self._project(forward, left, height, rect) for forward, left in edge_b]

    distances = [0.0]
    for start, end in zip(points, points[1:], strict=False):
      distances.append(distances[-1] + float(np.hypot(end[0] - start[0], end[1] - start[1])))
    total_distance = max(distances[-1], 1e-3)
    alpha_start = color.a if start_alpha is None else start_alpha
    alpha_end = color.a if end_alpha is None else end_alpha
    alphas = [round(alpha_start + (alpha_end - alpha_start) * distance / total_distance) for distance in distances]

    texture = rl.get_shapes_texture()
    source = rl.get_shapes_texture_rectangle()
    texture_u = (source.x + source.width / 2.0) / texture.width
    texture_v = (source.y + source.height / 2.0) / texture.height

    def vertex(point: rl.Vector2, alpha: int) -> None:
      rl.rl_color4ub(color.r, color.g, color.b, alpha)
      rl.rl_tex_coord2f(texture_u, texture_v)
      rl.rl_vertex2f(point.x, point.y)

    # Paint far-to-near so translucent sections overlap like one continuous road marking.
    rl.rl_set_texture(texture.id)
    rl.rl_begin(rl.RL_TRIANGLES)
    for i in range(len(points) - 2, -1, -1):
      a0, a1 = projected_a[i], projected_a[i + 1]
      b0, b1 = projected_b[i], projected_b[i + 1]
      if a0 is None or a1 is None or b0 is None or b1 is None:
        continue
      vertex(a0, alphas[i])
      vertex(b0, alphas[i])
      vertex(a1, alphas[i + 1])
      vertex(a1, alphas[i + 1])
      vertex(b0, alphas[i])
      vertex(b1, alphas[i + 1])
    rl.rl_end()
    rl.rl_set_texture(0)

  def _draw_arrow_head(self, rect: rl.Rectangle, base: tuple[float, float], tip: tuple[float, float],
                       height: float, width_m: float, color: rl.Color, min_screen_width: float = 0.0,
                       base_alpha: int | None = None, tip_alpha: int | None = None) -> None:
    tangent_forward = tip[0] - base[0]
    tangent_left = tip[1] - base[1]
    tangent_length = float(np.hypot(tangent_forward, tangent_left))
    if tangent_length < 1e-3:
      return

    normal_forward = -tangent_left / tangent_length
    normal_left = tangent_forward / tangent_length
    half_width = width_m / 2.0
    base_a = self._project(base[0] + normal_forward * half_width,
                           base[1] + normal_left * half_width, height, rect)
    base_b = self._project(base[0] - normal_forward * half_width,
                           base[1] - normal_left * half_width, height, rect)
    projected_tip = self._project(tip[0], tip[1], height, rect)
    if base_a is not None and base_b is not None and projected_tip is not None:
      screen_width = float(np.hypot(base_a.x - base_b.x, base_a.y - base_b.y))
      if 1e-3 < screen_width < min_screen_width:
        center_x = (base_a.x + base_b.x) / 2.0
        center_y = (base_a.y + base_b.y) / 2.0
        scale = min_screen_width / screen_width
        base_a = rl.Vector2(center_x + (base_a.x - center_x) * scale,
                            center_y + (base_a.y - center_y) * scale)
        base_b = rl.Vector2(center_x + (base_b.x - center_x) * scale,
                            center_y + (base_b.y - center_y) * scale)
      texture = rl.get_shapes_texture()
      source = rl.get_shapes_texture_rectangle()
      texture_u = (source.x + source.width / 2.0) / texture.width
      texture_v = (source.y + source.height / 2.0) / texture.height
      rl.rl_set_texture(texture.id)
      rl.rl_begin(rl.RL_TRIANGLES)
      for point, alpha in ((base_a, color.a if base_alpha is None else base_alpha),
                           (base_b, color.a if base_alpha is None else base_alpha),
                           (projected_tip, color.a if tip_alpha is None else tip_alpha)):
        rl.rl_color4ub(color.r, color.g, color.b, alpha)
        rl.rl_tex_coord2f(texture_u, texture_v)
        rl.rl_vertex2f(point.x, point.y)
      rl.rl_end()
      rl.rl_set_texture(0)

  def _draw_path(self, rect: rl.Rectangle, nav, height: float) -> None:
    points = [(float(point.forward), float(point.left)) for point in nav.shadowPath]
    arrow_path = self._path_prefix(points, ARROW_TIP_DISTANCE_M)
    arrow_body, arrow_base, arrow_tip = self._split_for_arrow(arrow_path, ARROW_LENGTH_M)
    arrow_body = self._extend_tail_to_camera(arrow_body)

    # Draw each translucent region once so overlapping alpha does not create dark seams.
    self._draw_road_ribbon(rect, arrow_body, height, ARROW_BODY_WIDTH_M, PATH_COLOR,
                           ARROW_TAIL_ALPHA, ARROW_BASE_ALPHA)
    self._draw_arrow_head(rect, arrow_base, arrow_tip, height, ARROW_WIDTH_M, PATH_COLOR,
                          base_alpha=ARROW_BASE_ALPHA, tip_alpha=ARROW_TIP_ALPHA)

    if nav.maneuverPointValid:
      maneuver = (float(nav.maneuverForward), float(nav.maneuverLeft))
      maneuver_distance = self._distance_along_path_nearest(points, maneuver)
      turn_tip_distance = maneuver_distance + TURN_ARROW_AFTER_MANEUVER_M
      turn_base_distance = turn_tip_distance - TURN_ARROW_LENGTH_M
      route_before_turn = self._path_prefix(self._path_suffix(points, ARROW_TIP_DISTANCE_M),
                                            max(0.0, turn_base_distance - ARROW_TIP_DISTANCE_M))
      self._draw_road_ribbon(rect, route_before_turn, height, PATH_WIDTH_M, PATH_COLOR,
                             ROUTE_NEAR_ALPHA, ROUTE_FAR_ALPHA)

      turn_arrow_path = self._path_prefix(points, turn_tip_distance)
      _, turn_arrow_base, turn_arrow_tip = self._split_for_arrow(turn_arrow_path, TURN_ARROW_LENGTH_M)
      self._draw_arrow_head(rect, turn_arrow_base, turn_arrow_tip, height,
                            TURN_ARROW_WIDTH_M, PATH_COLOR, TURN_ARROW_MIN_SCREEN_WIDTH,
                            ARROW_BASE_ALPHA, ARROW_TIP_ALPHA)
      self._draw_road_ribbon(rect, self._path_suffix(points, turn_tip_distance), height,
                             PATH_WIDTH_M, PATH_COLOR, ROUTE_NEAR_ALPHA, ROUTE_FAR_ALPHA)
    else:
      self._draw_road_ribbon(rect, self._path_suffix(points, ARROW_TIP_DISTANCE_M), height,
                             PATH_WIDTH_M, PATH_COLOR, ROUTE_NEAR_ALPHA, ROUTE_FAR_ALPHA)

  def _draw_instruction(self, rect: rl.Rectangle, nav) -> None:
    panel_width = min(460.0, rect.width - 60.0)
    panel_height = 150.0
    panel = rl.Rectangle(rect.x + rect.width - panel_width - 30.0, rect.y + 280.0, panel_width, panel_height)
    rl.draw_rectangle_rounded(panel, 0.18, 10, PANEL_COLOR)
    rl.draw_rectangle_rounded_lines_ex(panel, 0.18, 10, 3.0, PANEL_BORDER_COLOR)

    maneuver = str(nav.maneuver).replace("uTurn", "U-TURN").upper()
    title = f"{maneuver} IN {self._distance_text(float(nav.distanceToManeuver))}"
    road_name = str(nav.roadName).strip() or "Unnamed road"
    if len(road_name) > 31:
      road_name = road_name[:30] + "…"

    rl.draw_text_ex(self._font_semi_bold, title, rl.Vector2(panel.x + 24.0, panel.y + 22.0), 42, 0, TEXT_COLOR)
    rl.draw_text_ex(self._font_medium, road_name, rl.Vector2(panel.x + 24.0, panel.y + 82.0), 34, 0,
                    SECONDARY_TEXT_COLOR)

  def _render(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    if (sm.recv_frame["koalaNavPlan"] < ui_state.started_frame or not sm.valid["koalaNavPlan"]):
      return

    nav = sm["koalaNavPlan"]
    if not (nav.enabled and nav.shadowPathValid and nav.routeMatched and len(nav.shadowPath) >= 2):
      return

    calibration = sm["extrinsicsCalibration"]
    height = float(calibration.height[0]) if calibration.height else float(HEIGHT_INIT[0])
    self._draw_path(rect, nav, height)
    self._draw_instruction(rect, nav)
    self._mini_map.render(rect, nav)
