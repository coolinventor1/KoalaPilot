from __future__ import annotations

from dataclasses import dataclass

from openpilot.selfdrive.koalanav.mapbox import MapboxRoute, MapboxStep
from openpilot.selfdrive.koalanav.planner import Coordinate, haversine_distance


@dataclass
class NavigationSession:
  route: MapboxRoute
  step_index: int = 0
  route_progress: int = 0
  reached_step: bool = False

  @property
  def current_step(self) -> MapboxStep | None:
    return self.route.steps[self.step_index] if self.step_index < len(self.route.steps) else None

  def update(self, position: Coordinate) -> MapboxStep | None:
    search_start = max(0, self.route_progress - 10)
    nearest = min(range(search_start, len(self.route.coordinates)),
                  key=lambda index: haversine_distance(self.route.coordinates[index], position))
    self.route_progress = max(self.route_progress, nearest)

    step = self.current_step
    while step is not None:
      distance = haversine_distance(position, step.target)
      if distance <= 10.0:
        self.reached_step = True
      passed_by_geometry = self.route_progress > step.route_index + 1
      passed_by_distance = self.reached_step and distance > 25.0
      if not (passed_by_geometry or passed_by_distance):
        break
      self.step_index += 1
      self.reached_step = False
      step = self.current_step
    return step

  def route_offset(self, position: Coordinate) -> float:
    return min(haversine_distance(position, point) for point in self.route.coordinates)
