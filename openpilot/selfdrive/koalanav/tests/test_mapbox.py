import io
import json

from openpilot.selfdrive.koalanav.mapbox import MapboxClient, MapboxError, parse_mapbox_route
from openpilot.selfdrive.koalanav.planner import Coordinate, Maneuver


MAPBOX_RESPONSE = {
  "code": "Ok",
  "routes": [{
    "distance": 250.0,
    "duration": 30.0,
    "geometry": {"coordinates": [[-88.0, 41.0], [-88.0, 41.00045], [-87.9995, 41.00045]]},
    "legs": [{"steps": [
      {"name": "Oak Street", "maneuver": {
        "type": "depart", "modifier": "straight", "location": [-88.0, 41.0],
        "bearing_before": 0, "bearing_after": 0,
      }},
      {"name": "Main Street", "maneuver": {
        "type": "turn", "modifier": "right", "location": [-88.0, 41.00045],
        "bearing_before": 0, "bearing_after": 90, "instruction": "Turn right onto Main Street",
      }},
      {"name": "Main Street", "maneuver": {
        "type": "arrive", "location": [-87.9995, 41.00045], "bearing_before": 90, "bearing_after": 90,
      }},
    ]}],
  }],
}


def test_parse_mapbox_geometry_and_maneuvers():
  route = parse_mapbox_route(MAPBOX_RESPONSE)
  assert len(route.coordinates) == 3
  assert len(route.steps) == 1
  assert route.steps[0].maneuver == Maneuver.RIGHT
  assert route.steps[0].road_name == "Main Street"
  assert route.steps[0].target == Coordinate(41.00045, -88.0)
  assert route.steps[0].target_bearing_deg == 90.0
  assert route.steps[0].route_index == 1
  assert len(route.route_id) == 64


def test_mapbox_client_requests_full_geojson_steps(mocker):
  response = io.BytesIO(json.dumps(MAPBOX_RESPONSE).encode())
  urlopen = mocker.patch("openpilot.selfdrive.koalanav.mapbox.urllib.request.urlopen", return_value=response)
  route = MapboxClient("test-token").route(Coordinate(41.0, -88.0), Coordinate(41.1, -87.9), 45.0)

  requested_url = urlopen.call_args.args[0].full_url
  assert route.steps[0].maneuver == Maneuver.RIGHT
  assert "geometries=geojson" in requested_url
  assert "overview=full" in requested_url
  assert "steps=true" in requested_url
  assert "bearings=45%2C45%3B" in requested_url


def test_mapbox_rejects_empty_response():
  try:
    parse_mapbox_route({"code": "NoRoute", "routes": []})
  except MapboxError as error:
    assert "NoRoute" in str(error)
  else:
    raise AssertionError("invalid Mapbox response was accepted")


def test_route_with_no_turns_still_preserves_geometry():
  response = json.loads(json.dumps(MAPBOX_RESPONSE))
  response["routes"][0]["legs"][0]["steps"] = [response["routes"][0]["legs"][0]["steps"][0]]
  route = parse_mapbox_route(response)
  assert len(route.coordinates) == 3
  assert not route.steps
