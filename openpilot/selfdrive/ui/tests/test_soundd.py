import threading
from types import SimpleNamespace
import wave

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.basedir import BASEDIR
from openpilot.cereal import log, messaging
from openpilot.cereal.messaging import SubMaster, PubMaster
from opendbc.car import structs
from openpilot.selfdrive.ui.soundd import (
  ALERT_DUCK_GAIN,
  SAMPLE_RATE,
  SELFDRIVE_STATE_TIMEOUT,
  Soundd,
  check_selfdrive_timeout_alert,
  voice_prompt_for_alert,
  voice_prompt_for_gear,
  voice_prompt_for_lane_change,
  voice_prompt_list,
)

AudibleAlert = log.SelfdriveState.AudibleAlert
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
GearShifter = structs.CarState.GearShifter


class TestSoundd(OpenpilotTestCase):
  def test_voice_prompt_alert_transitions(self):
    assert voice_prompt_for_alert(AudibleAlert.none, AudibleAlert.warningImmediate) == "take_over_immediately"
    assert voice_prompt_for_alert(AudibleAlert.none, AudibleAlert.warningSoft) == "take_control"
    assert voice_prompt_for_alert(AudibleAlert.none, AudibleAlert.engage) == "openpilot_engaged"
    assert voice_prompt_for_alert(AudibleAlert.warningImmediate, AudibleAlert.warningImmediate) is None

  def test_voice_prompt_lane_change_transitions(self):
    assert voice_prompt_for_lane_change(LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting,
                                        LaneChangeDirection.left) == "lane_change_left"
    assert voice_prompt_for_lane_change(LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting,
                                        LaneChangeDirection.right) == "lane_change_right"
    assert voice_prompt_for_lane_change(LaneChangeState.laneChangeFinishing, LaneChangeState.off,
                                        LaneChangeDirection.none) == "lane_change_complete"
    assert voice_prompt_for_lane_change(LaneChangeState.off, LaneChangeState.off, LaneChangeDirection.none) is None

  def test_gear_prompt_transitions(self):
    assert voice_prompt_for_gear(None, GearShifter.park) is None
    assert voice_prompt_for_gear(GearShifter.park, GearShifter.park) is None
    assert voice_prompt_for_gear(GearShifter.park, GearShifter.reverse) == "gear_reverse"
    assert voice_prompt_for_gear(GearShifter.reverse, GearShifter.neutral) == "gear_neutral"
    assert voice_prompt_for_gear(GearShifter.neutral, GearShifter.drive) == "gear_drive"
    assert voice_prompt_for_gear(GearShifter.drive, GearShifter.park) == "gear_park"
    assert voice_prompt_for_gear(GearShifter.drive, GearShifter.sport) is None

  def test_first_runtime_gear_reading_is_silent(self):
    class FakeSubMaster(dict):
      updated = {'selfdriveState': False, 'modelV2': False, 'carState': True}

    soundd = Soundd()
    sm = FakeSubMaster(carState=SimpleNamespace(gearShifter=SimpleNamespace(raw=GearShifter.park)))
    soundd.get_voice_prompt(sm)
    assert soundd.current_voice_prompt is None

    sm['carState'].gearShifter.raw = GearShifter.reverse
    soundd.get_voice_prompt(sm)
    assert soundd.current_voice_prompt == "gear_reverse"

    sm['carState'].gearShifter.raw = GearShifter.unknown
    soundd.get_voice_prompt(sm)
    assert soundd.previous_gear == GearShifter.reverse

  def test_voice_assets_are_soundd_compatible(self):
    for filename, _ in voice_prompt_list.values():
      with wave.open(f"{BASEDIR}/openpilot/selfdrive/assets/sounds/{filename}", "r") as wavefile:
        assert wavefile.getnchannels() == 1
        assert wavefile.getsampwidth() == 2
        assert wavefile.getframerate() == SAMPLE_RATE
        assert wavefile.getnframes() > 0

  def test_urgent_prompt_interrupts_lower_priority_speech(self):
    soundd = Soundd()
    soundd.update_voice_prompt("lane_change_left")
    soundd.update_voice_prompt("take_over_immediately")
    soundd.update_voice_prompt("openpilot_engaged")
    assert soundd.current_voice_prompt == "take_over_immediately"

  def test_critical_tone_remains_mixed_with_takeover_speech(self):
    mixed_soundd = Soundd()
    mixed_soundd.current_volume = 1.0
    mixed_soundd.update_alert(AudibleAlert.warningImmediate)
    mixed_soundd.update_voice_prompt("take_over_immediately")
    mixed = mixed_soundd.get_sound_data(4096)

    alert_soundd = Soundd()
    alert_soundd.current_volume = 1.0
    alert_soundd.update_alert(AudibleAlert.warningImmediate)
    alert_only = alert_soundd.get_sound_data(4096)

    voice_soundd = Soundd()
    voice_soundd.current_volume = 1.0
    voice_soundd.update_voice_prompt("take_over_immediately")
    voice_only = voice_soundd.get_sound_data(4096)

    expected = np.clip(alert_only * ALERT_DUCK_GAIN + voice_only, -1.0, 1.0)
    assert np.allclose(mixed, expected)
    assert not np.allclose(mixed, alert_only)
    assert not np.allclose(mixed, voice_only)

  def test_check_selfdrive_timeout_alert(self, mocker):
    sm = SubMaster(['selfdriveState'])
    pm = PubMaster(['selfdriveState'])

    cs = messaging.new_message('selfdriveState')
    cs.selfdriveState.enabled = True
    threading.Timer(0.01, pm.send, args=("selfdriveState", cs)).start()
    sm.update(100)
    assert sm.updated['selfdriveState']

    sm.recv_time['selfdriveState'] = 0
    clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=SELFDRIVE_STATE_TIMEOUT)
    assert not check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 0.1
    assert check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 10
    assert not check_selfdrive_timeout_alert(sm)

  # TODO: add test with micd for checking that soundd actually outputs sounds
