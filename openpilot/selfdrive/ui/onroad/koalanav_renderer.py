from __future__ import annotations

import numpy as np
import pyray as rl

from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets import Widget


PATH_COLOR = rl.Color(55, 205, 255, 205)
PATH_GLOW_COLOR = rl.Color(20, 120, 180, 100)
MARKER_COLOR = rl.Color(255, 191, 64, 255)
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

  def _draw_path(self, rect: rl.Rectangle, nav, height: float) -> None:
    points = [(float(point.forward), float(point.left)) for point in nav.shadowPath]
    for (forward_a, left_a), (forward_b, left_b) in zip(points, points[1:], strict=False):
      start = self._project(forward_a, left_a, height, rect)
      end = self._project(forward_b, left_b, height, rect)
      if start is None or end is None:
        continue
      rl.draw_line_ex(start, end, 18.0, PATH_GLOW_COLOR)
      rl.draw_line_ex(start, end, 8.0, PATH_COLOR)

    if nav.maneuverPointValid:
      marker = self._project(float(nav.maneuverForward), float(nav.maneuverLeft), height, rect)
      if marker is not None:
        rl.draw_circle(int(marker.x), int(marker.y), 22.0, rl.Color(0, 0, 0, 180))
        rl.draw_circle(int(marker.x), int(marker.y), 14.0, MARKER_COLOR)

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
