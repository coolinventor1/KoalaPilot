from urllib.parse import parse_qs, urlparse

import pytest

from openpilot.selfdrive.ui.onroad.koalanav_minimap import (
  MAP_REQUEST_SIZE,
  MapAnchor,
  car_path_screen_offset,
  geographic_distance_m,
  geographic_offset_m,
  map_screen_offset,
  mapbox_static_url,
  meters_per_pixel,
  signed_bearing_delta,
)


ANCHOR = MapAnchor(41.0, -88.0, 0.0)


def test_mapbox_static_url_keeps_attribution_and_retina_resolution():
  parsed = urlparse(mapbox_static_url("token with spaces", ANCHOR))
  query = parse_qs(parsed.query)

  assert parsed.netloc == "api.mapbox.com"
  assert f"/{MAP_REQUEST_SIZE}x{MAP_REQUEST_SIZE}@2x" in parsed.path
  assert query["access_token"] == ["token with spaces"]
  assert query["logo"] == ["true"]
  assert query["attribution"] == ["true"]


def test_geographic_offset_uses_east_north_meters():
  east, north = geographic_offset_m(ANCHOR, 41.001, -87.999)

  assert east == pytest.approx(83.92, abs=0.2)
  assert north == pytest.approx(111.19, abs=0.2)
  assert geographic_distance_m(ANCHOR, 41.001, -87.999) == pytest.approx(139.3, abs=0.3)


def test_map_rotation_keeps_anchor_heading_at_top():
  scale = 1.0
  north = map_screen_offset(ANCHOR, 0.0, 20.0, scale)
  east = map_screen_offset(ANCHOR, 20.0, 0.0, scale)

  assert north[0] == pytest.approx(0.0)
  assert north[1] < 0.0
  assert east[0] > 0.0
  assert east[1] == pytest.approx(0.0)

  east_up_anchor = MapAnchor(41.0, -88.0, 90.0)
  east_up = map_screen_offset(east_up_anchor, 20.0, 0.0, scale)
  assert east_up[0] == pytest.approx(0.0, abs=1e-9)
  assert east_up[1] < 0.0


def test_car_path_tracks_current_heading_relative_to_map_anchor():
  forward = car_path_screen_offset(ANCHOR, 0.0, 20.0, 0.0, 1.0)
  left = car_path_screen_offset(ANCHOR, 0.0, 0.0, 20.0, 1.0)

  assert forward[0] == pytest.approx(0.0)
  assert forward[1] < 0.0
  assert left[0] < 0.0
  assert left[1] == pytest.approx(0.0)


def test_map_scale_and_bearing_wrap_are_stable():
  assert meters_per_pixel(41.0) > 0.0
  assert signed_bearing_delta(5.0, 355.0) == pytest.approx(10.0)
  assert signed_bearing_delta(355.0, 5.0) == pytest.approx(-10.0)
