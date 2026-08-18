import math
import numpy as np
import time
import wave


from openpilot.cereal import log, messaging
from opendbc.car import structs
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog

from openpilot.system import micd
from openpilot.common.hardware import HARDWARE
from openpilot.selfdrive.koalanav.voice import navigation_voice_cache_path

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1
ALERT_RAMP_TIME = 4 # seconds to ramp to max volume for warningImmediate
SELFDRIVE_STATE_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)
VOICE_GAIN = 0.85
ALERT_DUCK_GAIN = 0.72
NAVIGATION_VOICE_POLL_S = 0.2

AMBIENT_DB = 26 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

VOLUME_BASE = 20
if HARDWARE.get_device_type() == "tizi":
  AMBIENT_DB = 30
  VOLUME_BASE = 10

AudibleAlert = log.SelfdriveState.AudibleAlert
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
GearShifter = structs.CarState.GearShifter


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("warning.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("warning.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("dm_warning.wav", None, MAX_VOLUME),

  AudibleAlert.preAlert: ("pre_alert.wav", 1, MAX_VOLUME),

  AudibleAlert.warningSoft: ("critical.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("dm_critical.wav", None, MAX_VOLUME),
}

voice_prompt_list: dict[str, tuple[str, int]] = {
  # Prompt name: file name, interruption priority
  "openpilot_engaged": ("voice/openpilot_engaged.wav", 10),
  "openpilot_disengaged": ("voice/openpilot_disengaged.wav", 20),
  "gear_park": ("gear/park.wav", 35),
  "gear_reverse": ("gear/reverse.wav", 35),
  "gear_neutral": ("gear/neutral.wav", 35),
  "gear_drive": ("gear/drive.wav", 35),
  "lane_change_complete": ("voice/lane_change_complete.wav", 30),
  "lane_change_left": ("voice/lane_change_left.wav", 40),
  "lane_change_right": ("voice/lane_change_right.wav", 40),
  "take_control": ("voice/take_control.wav", 80),
  "take_over_immediately": ("voice/take_over_immediately.wav", 100),
}


def voice_prompt_for_alert(previous_alert: int, new_alert: int) -> str | None:
  if previous_alert == new_alert:
    return None
  return {
    AudibleAlert.engage: "openpilot_engaged",
    AudibleAlert.disengage: "openpilot_disengaged",
    AudibleAlert.warningSoft: "take_control",
    AudibleAlert.warningImmediate: "take_over_immediately",
  }.get(new_alert)


def voice_prompt_for_lane_change(previous_state: int, new_state: int, direction: int) -> str | None:
  if previous_state != LaneChangeState.laneChangeStarting and new_state == LaneChangeState.laneChangeStarting:
    if direction == LaneChangeDirection.left:
      return "lane_change_left"
    if direction == LaneChangeDirection.right:
      return "lane_change_right"
  if previous_state == LaneChangeState.laneChangeFinishing and new_state == LaneChangeState.off:
    return "lane_change_complete"
  return None


def voice_prompt_for_gear(previous_gear: int | None, new_gear: int) -> str | None:
  if previous_gear is None or previous_gear == new_gear:
    return None
  return {
    GearShifter.park: "gear_park",
    GearShifter.reverse: "gear_reverse",
    GearShifter.neutral: "gear_neutral",
    GearShifter.drive: "gear_drive",
  }.get(new_gear)

def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time['selfdriveState']

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if sm['selfdriveState'].enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10:
      return True

  return False


def load_mono_wav(path: str) -> np.ndarray:
  with wave.open(path, 'r') as wavefile:
    assert wavefile.getnchannels() == 1
    assert wavefile.getsampwidth() == 2
    assert wavefile.getframerate() == SAMPLE_RATE
    length = wavefile.getnframes()
    return np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)


class Soundd:
  def __init__(self):
    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.ramp_start_volume = MIN_VOLUME
    self.ramp_start_time = 0.

    self.selfdrive_timeout_alert = False
    self.pending_stop = False

    self.current_voice_prompt: str | None = None
    self.current_voice_frame = 0
    self.current_voice_priority = -1
    self.previous_alert = AudibleAlert.none
    self.previous_lane_change_state = LaneChangeState.off
    self.previous_gear: int | None = None
    self.params = Params()
    self.last_navigation_voice_check = 0.0
    self.last_navigation_voice_id: int | None = None

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}
    self.loaded_voice_prompts: dict[str, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      self.loaded_sounds[sound] = load_mono_wav(BASEDIR + "/openpilot/selfdrive/assets/sounds/" + filename)

    for prompt, (filename, _) in voice_prompt_list.items():
      self.loaded_voice_prompts[prompt] = load_mono_wav(BASEDIR + "/openpilot/selfdrive/assets/sounds/" + filename)

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      num_loops = sound_list[self.current_alert][1]
      sound_data = self.loaded_sounds[self.current_alert]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write
        current_sound_frame = self.current_sound_frame % len(sound_data)
        loops = self.current_sound_frame // len(sound_data)
        if self.pending_stop and current_sound_frame == 0:
          self.current_alert = AudibleAlert.none
          self.pending_stop = False
          break

    voice_data = self.get_voice_data(frames)
    alert_gain = ALERT_DUCK_GAIN if np.any(voice_data) else 1.0
    return np.clip((ret * alert_gain + voice_data * VOICE_GAIN) * self.current_volume, -1.0, 1.0)

  def get_voice_data(self, frames: int) -> np.ndarray:
    ret = np.zeros(frames, dtype=np.float32)
    if self.current_voice_prompt is None:
      return ret

    voice_data = self.loaded_voice_prompts[self.current_voice_prompt]
    frames_to_write = min(frames, len(voice_data) - self.current_voice_frame)
    ret[:frames_to_write] = voice_data[self.current_voice_frame:self.current_voice_frame + frames_to_write]
    self.current_voice_frame += frames_to_write

    if self.current_voice_frame >= len(voice_data):
      self.current_voice_prompt = None
      self.current_voice_frame = 0
      self.current_voice_priority = -1

    return ret

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    current_alert_played_once = self.current_alert == AudibleAlert.none or self.current_sound_frame >= len(self.loaded_sounds[self.current_alert])
    # let looping sounds finish the current loop instead of cutting off mid tone
    if new_alert == AudibleAlert.none and self.current_alert != AudibleAlert.none and sound_list[self.current_alert][1] is None:
      if current_alert_played_once:
        self.pending_stop = True
      else:
        self.current_alert = AudibleAlert.none
        self.current_sound_frame = 0
      return
    self.pending_stop = False
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
      if new_alert == AudibleAlert.warningImmediate:
        self.ramp_start_volume = self.current_volume
        self.ramp_start_time = time.monotonic()
      self.current_alert = new_alert
      self.current_sound_frame = 0

  def update_voice_prompt(self, prompt: str | None, priority: int | None = None) -> None:
    if prompt is None:
      return

    priority = voice_prompt_list[prompt][1] if priority is None else priority
    if self.current_voice_prompt is None or priority >= self.current_voice_priority:
      self.current_voice_prompt = prompt
      self.current_voice_frame = 0
      self.current_voice_priority = priority

  def handle_navigation_voice_prompt(self, payload: object) -> None:
    if not isinstance(payload, dict):
      return
    try:
      prompt_id = int(payload["id"])
      cache_key = str(payload["cacheKey"])
      priority = min(79, max(0, int(payload.get("priority", 15))))
    except (KeyError, TypeError, ValueError):
      return
    if prompt_id == self.last_navigation_voice_id:
      return
    self.last_navigation_voice_id = prompt_id

    try:
      path = navigation_voice_cache_path(cache_key)
      prompt = f"koalanav:{cache_key}"
      for loaded_prompt in list(self.loaded_voice_prompts):
        if loaded_prompt.startswith("koalanav:") and loaded_prompt != self.current_voice_prompt:
          del self.loaded_voice_prompts[loaded_prompt]
      self.loaded_voice_prompts[prompt] = load_mono_wav(str(path))
      self.update_voice_prompt(prompt, priority)
    except (AssertionError, OSError, ValueError) as error:
      cloudlog.warning(f"KoalaNav navigation voice could not be loaded: {error}")

  def poll_navigation_voice_prompt(self) -> None:
    now = time.monotonic()
    if now - self.last_navigation_voice_check < NAVIGATION_VOICE_POLL_S:
      return
    self.last_navigation_voice_check = now
    self.handle_navigation_voice_prompt(self.params.get("KoalaNavVoicePrompt"))

  def get_voice_prompt(self, sm) -> None:
    prompts: list[str] = []

    self.poll_navigation_voice_prompt()

    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      prompt = voice_prompt_for_alert(self.previous_alert, new_alert)
      if prompt is not None:
        prompts.append(prompt)
      self.previous_alert = new_alert

    if sm.updated['modelV2']:
      meta = sm['modelV2'].meta
      new_state = meta.laneChangeState.raw
      prompt = voice_prompt_for_lane_change(self.previous_lane_change_state, new_state, meta.laneChangeDirection.raw)
      if prompt is not None:
        prompts.append(prompt)
      self.previous_lane_change_state = new_state

    if sm.updated['carState']:
      new_gear = sm['carState'].gearShifter.raw
      if new_gear != GearShifter.unknown:
        prompt = voice_prompt_for_gear(self.previous_gear, new_gear)
        if prompt is not None:
          prompts.append(prompt)
        self.previous_gear = new_gear

    if prompts:
      self.update_voice_prompt(max(prompts, key=lambda name: voice_prompt_list[name][1]))

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      self.update_alert(new_alert)
    elif check_selfdrive_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      if not self.selfdrive_timeout_alert:
        self.update_voice_prompt("take_over_immediately")
      self.selfdrive_timeout_alert = True
    elif self.selfdrive_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.selfdrive_timeout_alert = False

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(VOLUME_BASE, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd
    micd.patch_sounddevice(sd)

    sm = messaging.SubMaster(['selfdriveState', 'soundPressure', 'modelV2', 'carState'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)

        # freeze volume during alerts to avoid mic feedback increasing volume
        if sm.updated['soundPressure']:
          self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          if self.current_alert == AudibleAlert.none and self.current_voice_prompt is None:
            self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x))

        self.get_audible_alert(sm)
        self.get_voice_prompt(sm)

        # Ramp up immediate warning sound over 4s
        if self.current_alert == AudibleAlert.warningImmediate:
          elapsed = time.monotonic() - self.ramp_start_time
          ramp_vol = float(np.interp(elapsed, [0, ALERT_RAMP_TIME], [self.ramp_start_volume, MAX_VOLUME]))
          self.current_volume = max(self.current_volume, ramp_vol)

        rk.keep_time()

        assert stream.active


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
