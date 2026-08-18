from pathlib import Path
import wave

import numpy as np

from openpilot.selfdrive.koalanav.planner import Maneuver
from openpilot.selfdrive.koalanav.voice import (
  ElevenLabsSpeechCache, VoiceAnnouncementScheduler, direction_prompts, navigation_voice_cache_path,
)


def test_direction_prompt_wording():
  early, now = direction_prompts(Maneuver.LEFT, "Main Street")
  assert early == "Turn left ahead onto Main Street."
  assert now == "Now turn left onto Main Street."


def test_mapbox_instruction_preserves_roundabout_details():
  early, now = direction_prompts(Maneuver.RIGHT, "State Street", "At the roundabout, take the second exit onto State Street")
  assert early == "At the roundabout, take the second exit onto State Street."
  assert now == "Now, at the roundabout, take the second exit onto State Street."


def test_scheduler_announces_early_then_now_once():
  scheduler = VoiceAnnouncementScheduler()
  early = scheduler.candidate("step-1", Maneuver.RIGHT, "Oak Road", 100.0, 10.0)
  assert early is not None and early.phase == "early"
  scheduler.mark_announced(early)
  assert scheduler.candidate("step-1", Maneuver.RIGHT, "Oak Road", 90.0, 10.0) is None

  now = scheduler.candidate("step-1", Maneuver.RIGHT, "Oak Road", 20.0, 10.0)
  assert now is not None and now.phase == "now"
  assert now.text == "Now turn right onto Oak Road."
  scheduler.mark_announced(now)
  assert scheduler.candidate("step-1", Maneuver.RIGHT, "Oak Road", 5.0, 10.0) is None


def test_scheduler_skips_early_prompt_when_route_starts_near_turn():
  cue = VoiceAnnouncementScheduler().candidate("step-1", Maneuver.LEFT, "Pine Avenue", 10.0, 5.0)
  assert cue is not None and cue.phase == "now"


def test_disabled_speech_cache_never_generates():
  speech = ElevenLabsSpeechCache(None, None)
  speech.prepare("Now turn left.")
  assert not speech.enabled
  assert speech.ready_cache_key("Now turn left.") is None
  assert not speech.futures


def test_speech_cache_converts_and_reuses_wav(mocker, monkeypatch, tmp_path):
  monkeypatch.setenv("COMMA_CACHE", str(tmp_path))
  mocker.patch("openpilot.selfdrive.koalanav.voice.shutil.which", return_value="/usr/bin/ffmpeg")
  mocker.patch("openpilot.selfdrive.koalanav.voice._generate_mp3", return_value=b"test-mp3")

  def fake_ffmpeg(args, check):
    assert check
    destination = Path(args[-1])
    with wave.open(str(destination), "w") as wavefile:
      wavefile.setnchannels(1)
      wavefile.setsampwidth(2)
      wavefile.setframerate(48000)
      wavefile.writeframes(np.zeros(4800, dtype=np.int16).tobytes())

  converter = mocker.patch("openpilot.selfdrive.koalanav.voice.subprocess.run", side_effect=fake_ffmpeg)
  speech = ElevenLabsSpeechCache("api-key", "voice-id")
  text = "Now turn left onto Main Street."
  cache_key = speech.cache_key(text)

  generated = speech._generate(cache_key, text)

  assert generated == navigation_voice_cache_path(cache_key)
  assert generated.is_file()
  speech.prepare(text)
  assert speech.ready_cache_key(text) == cache_key
  assert converter.call_count == 1
