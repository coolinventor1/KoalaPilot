from __future__ import annotations

import os
import time

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.koalanav.mapbox import MapboxClient, MapboxError
from openpilot.selfdrive.koalanav.navigation_input import instruction_message, route_message
from openpilot.selfdrive.koalanav.planner import Coordinate, NavInstruction, haversine_distance, valid_coordinate
from openpilot.selfdrive.koalanav.session import NavigationSession
from openpilot.selfdrive.koalanav.voice import ElevenLabsSpeechCache, VoiceAnnouncementScheduler, direction_prompts


GPS_SERVICES = ("gpsLocationExternal",)
ROUTE_REPUBLISH_S = 5.0
ROUTE_RETRY_S = 30.0
OFF_ROUTE_DISTANCE_M = 80.0
OFF_ROUTE_TIME_S = 8.0


def _secret(params: Params, environment_name: str, param_name: str) -> str:
  return os.environ.get(environment_name) or params.get(param_name) or ""


def _destination(params: Params) -> Coordinate | None:
  value = params.get("KoalaNavDestination")
  if not isinstance(value, dict):
    return None
  try:
    destination = Coordinate(float(value["latitude"]), float(value["longitude"]))
  except (KeyError, TypeError, ValueError):
    return None
  return destination if valid_coordinate(destination) else None


def _gps(sm: messaging.SubMaster) -> tuple[Coordinate, float, float] | None:
  service = next((name for name in GPS_SERVICES if sm.seen[name] and sm.valid[name]), None)
  if service is None:
    return None
  msg = sm[service]
  position = Coordinate(msg.latitude, msg.longitude)
  if not (bool(msg.hasFix or msg.flags) and valid_coordinate(position)):
    return None
  return position, max(0.0, float(msg.speed)), float(msg.bearingDeg)


def _prepare_upcoming_speech(speech: ElevenLabsSpeechCache, session: NavigationSession) -> None:
  for step in session.route.steps[session.step_index:session.step_index + 2]:
    for prompt in direction_prompts(step.maneuver, step.road_name, step.provider_instruction):
      speech.prepare(prompt)


def _publish_voice(params: Params, scheduler: VoiceAnnouncementScheduler,
                   speech: ElevenLabsSpeechCache, cue) -> bool:
  cache_key = speech.ready_cache_key(cue.text)
  if cache_key is None:
    return False
  params.put("KoalaNavVoicePrompt", {
    "id": time.monotonic_ns(),
    "cacheKey": cache_key,
    "text": cue.text,
    "phase": cue.phase,
    "priority": cue.priority,
  })
  scheduler.mark_announced(cue)
  return True


def main() -> None:
  params = Params()
  mapbox_token = _secret(params, "MAPBOX_ACCESS_TOKEN", "MapboxAccessToken")
  elevenlabs_key = _secret(params, "ELEVENLABS_API_KEY", "ElevenLabsApiKey")
  elevenlabs_voice = _secret(params, "ELEVENLABS_VOICE_ID", "ElevenLabsVoiceId")
  mapbox = MapboxClient(mapbox_token) if mapbox_token else None
  speech = ElevenLabsSpeechCache(elevenlabs_key, elevenlabs_voice)
  scheduler = VoiceAnnouncementScheduler()
  sm = messaging.SubMaster(list(GPS_SERVICES), poll="gpsLocationExternal")
  pm = messaging.PubMaster(["koalaNavRoute", "koalaNavInstruction"])
  rk = Ratekeeper(10, print_delay_threshold=None)

  session: NavigationSession | None = None
  active_destination: Coordinate | None = None
  next_route_attempt = 0.0
  last_route_publish = 0.0
  last_instruction_publish = 0.0
  off_route_since: float | None = None
  revision = 0

  if mapbox is None:
    cloudlog.warning("KoalaNav Mapbox provider is waiting for MAPBOX_ACCESS_TOKEN or MapboxAccessToken")
  if not speech.enabled:
    cloudlog.warning("KoalaNav voice is waiting for ElevenLabs credentials")

  while True:
    sm.update(100)
    now = time.monotonic()
    gps = _gps(sm)
    destination = _destination(params)

    if destination != active_destination:
      active_destination = destination
      session = None
      scheduler = VoiceAnnouncementScheduler()
      next_route_attempt = 0.0

    if gps is not None and destination is not None and session is None and mapbox is not None and now >= next_route_attempt:
      position, _, bearing = gps
      try:
        route = mapbox.route(position, destination, bearing)
        session = NavigationSession(route)
        revision += 1
        last_route_publish = 0.0
        cloudlog.info(f"KoalaNav loaded route {route.route_id[:12]} with {len(route.steps)} maneuvers")
      except MapboxError as error:
        cloudlog.warning(f"KoalaNav route request failed: {error}")
        next_route_attempt = now + ROUTE_RETRY_S

    if session is not None and gps is not None:
      position, speed, _ = gps
      step = session.update(position)
      if session.route_offset(position) > OFF_ROUTE_DISTANCE_M:
        off_route_since = off_route_since or now
        if now - off_route_since >= OFF_ROUTE_TIME_S:
          session = None
          scheduler = VoiceAnnouncementScheduler()
          next_route_attempt = now
          off_route_since = None
      else:
        off_route_since = None

      if session is not None:
        if now - last_route_publish >= ROUTE_REPUBLISH_S:
          pm.send("koalaNavRoute", route_message(session.route.route_id, revision, session.route.coordinates))
          last_route_publish = now

        if step is not None:
          distance = haversine_distance(position, step.target)
          if now - last_instruction_publish >= 1.0:
            instruction = NavInstruction(True, session.route.route_id, step.maneuver, distance, step.target,
                                         step.target_bearing_deg, time.monotonic_ns(), step.road_name)
            pm.send("koalaNavInstruction", instruction_message(instruction))
            last_instruction_publish = now
          _prepare_upcoming_speech(speech, session)
          cue = scheduler.candidate(step.step_id, step.maneuver, step.road_name, distance, speed, step.provider_instruction)
          if cue is not None:
            _publish_voice(params, scheduler, speech, cue)

    rk.keep_time()


if __name__ == "__main__":
  main()
