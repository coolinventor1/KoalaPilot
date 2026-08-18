from __future__ import annotations

import time

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.koalanav.navigation_input import instruction_from_message, route_from_message
from openpilot.selfdrive.koalanav.planner import GpsFix, KoalaNavPlanner


GPS_SERVICES = ("gpsLocationExternal",)
NAV_SERVICES = ("koalaNavRoute", "koalaNavInstruction")
GPS_MAX_AGE_S = 2.5
INSTRUCTION_MAX_AGE_S = 5.0


def _fresh(sm: messaging.SubMaster, service: str, max_age_s: float) -> bool:
  return sm.seen[service] and sm.valid[service] and time.monotonic() - sm.recv_time[service] <= max_age_s


def _read_gps(sm: messaging.SubMaster) -> GpsFix | None:
  service = next((name for name in GPS_SERVICES if _fresh(sm, name, GPS_MAX_AGE_S)), None)
  if service is None:
    return None
  msg = sm[service]
  return GpsFix(
    latitude=msg.latitude,
    longitude=msg.longitude,
    speed=msg.speed,
    bearing_deg=msg.bearingDeg,
    horizontal_accuracy=msg.horizontalAccuracy,
    has_fix=bool(msg.hasFix or msg.flags),
    source_mono_time=sm.logMonoTime[service],
  )


def _publish(pm: messaging.PubMaster, plan) -> None:
  dat = messaging.new_message("koalaNavPlan", valid=True)
  msg = dat.koalaNavPlan
  msg.enabled = plan.enabled
  msg.mode = plan.mode
  msg.state = plan.state.value
  msg.maneuver = plan.maneuver.value
  msg.gpsValid = plan.gps_valid
  msg.routeValid = plan.route_valid
  msg.instructionValid = plan.instruction_valid
  msg.routeMatched = plan.route_matched
  msg.controlAllowed = False
  msg.latitude = plan.latitude
  msg.longitude = plan.longitude
  msg.horizontalAccuracy = plan.horizontal_accuracy
  msg.speed = plan.speed
  msg.bearingDeg = plan.bearing_deg
  msg.distanceToManeuver = plan.distance_to_maneuver
  msg.targetLatitude = plan.target_latitude
  msg.targetLongitude = plan.target_longitude
  msg.targetBearingDeg = plan.target_bearing_deg
  msg.turnAngleDeg = plan.turn_angle_deg
  msg.confidence = plan.confidence
  msg.reason = plan.reason
  msg.sourceMonoTime = plan.source_mono_time
  msg.routeId = plan.route_id
  pm.send("koalaNavPlan", dat)


def main() -> None:
  params = Params()
  sm = messaging.SubMaster([*GPS_SERVICES, *NAV_SERVICES], poll="gpsLocationExternal",
                           ignore_alive=[*NAV_SERVICES])
  pm = messaging.PubMaster(["koalaNavPlan"])
  planner = KoalaNavPlanner()
  rk = Ratekeeper(10, print_delay_threshold=None)
  route = None
  instruction = None

  while True:
    sm.update(100)
    if sm.updated["koalaNavRoute"] and sm.valid["koalaNavRoute"]:
      route = route_from_message(sm["koalaNavRoute"])
    if sm.updated["koalaNavInstruction"] and sm.valid["koalaNavInstruction"]:
      instruction = instruction_from_message(sm["koalaNavInstruction"])
    elif instruction is not None and not _fresh(sm, "koalaNavInstruction", INSTRUCTION_MAX_AGE_S):
      instruction = None

    enabled = params.get_bool("KoalaNavEnabled")
    mode = params.get("KoalaNavMode", return_default=True) or "shadow"
    plan = planner.update(enabled, mode, _read_gps(sm), route, instruction)
    _publish(pm, plan)
    rk.keep_time()


if __name__ == "__main__":
  main()
