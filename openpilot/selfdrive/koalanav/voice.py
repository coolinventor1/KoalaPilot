from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.koalanav.planner import Maneuver


MODEL_ID = "eleven_multilingual_v2"
OUTPUT_FORMAT = "mp3_44100_128"
CACHE_KEY_PATTERN = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class VoiceCue:
  phase: str
  text: str
  priority: int


def navigation_voice_cache_dir() -> Path:
  return Path(Paths.download_cache_root()) / "koalanav_voice"


def navigation_voice_cache_path(cache_key: str) -> Path:
  if CACHE_KEY_PATTERN.fullmatch(cache_key) is None:
    raise ValueError("invalid KoalaNav voice cache key")
  return navigation_voice_cache_dir() / f"{cache_key}.wav"


def maneuver_action(maneuver: Maneuver) -> str:
  return {
    Maneuver.LEFT: "turn left",
    Maneuver.RIGHT: "turn right",
    Maneuver.STRAIGHT: "continue straight",
    Maneuver.U_TURN: "make a U-turn",
  }.get(maneuver, "continue")


def direction_prompts(maneuver: Maneuver, road_name: str, provider_instruction: str = "") -> tuple[str, str]:
  instruction = provider_instruction.strip().rstrip(".")
  if instruction:
    now_instruction = instruction[0].lower() + instruction[1:]
    separator = ", " if now_instruction.startswith("at ") else " "
    return f"{instruction}.", f"Now{separator}{now_instruction}."
  action = maneuver_action(maneuver)
  destination = f" onto {road_name.strip()}" if road_name.strip() else ""
  return f"{action.capitalize()} ahead{destination}.", f"Now {action}{destination}."


class VoiceAnnouncementScheduler:
  def __init__(self):
    self.step_id: str | None = None
    self.announced: set[str] = set()

  def candidate(self, step_id: str, maneuver: Maneuver, road_name: str,
                distance_m: float, speed_mps: float, provider_instruction: str = "") -> VoiceCue | None:
    if step_id != self.step_id:
      self.step_id = step_id
      self.announced.clear()

    early_text, now_text = direction_prompts(maneuver, road_name, provider_instruction)
    early_distance = min(450.0, max(120.0, speed_mps * 12.0))
    now_distance = min(80.0, max(25.0, speed_mps * 2.5))
    if distance_m <= now_distance and "now" not in self.announced:
      return VoiceCue("now", now_text, 25)
    if distance_m <= early_distance and "early" not in self.announced and "now" not in self.announced:
      return VoiceCue("early", early_text, 15)
    return None

  def mark_announced(self, cue: VoiceCue) -> None:
    self.announced.add(cue.phase)


def _generate_mp3(api_key: str, voice_id: str, text: str) -> bytes:
  url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}/stream?output_format={OUTPUT_FORMAT}"
  body = json.dumps({
    "text": text,
    "model_id": MODEL_ID,
    "voice_settings": {
      "stability": 0.72,
      "similarity_boost": 0.78,
      "style": 0.08,
      "use_speaker_boost": True,
    },
  }).encode()
  request = urllib.request.Request(url, data=body, method="POST", headers={
    "Accept": "audio/mpeg",
    "Content-Type": "application/json",
    "xi-api-key": api_key,
  })
  try:
    with urllib.request.urlopen(request, timeout=60) as response:
      return response.read()
  except urllib.error.HTTPError as error:
    detail = error.read().decode(errors="replace")[:300]
    raise RuntimeError(f"ElevenLabs generation failed ({error.code}): {detail}") from error
  except (urllib.error.URLError, TimeoutError) as error:
    raise RuntimeError(f"ElevenLabs generation failed: {error}") from error


class ElevenLabsSpeechCache:
  def __init__(self, api_key: str | None, voice_id: str | None):
    self.api_key = api_key or ""
    self.voice_id = voice_id or ""
    self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="koalanav-voice")
    self.futures: dict[str, Future[Path]] = {}
    self.reported_failures: set[str] = set()
    self.retry_after: dict[str, float] = {}

  @property
  def enabled(self) -> bool:
    return bool(self.api_key and self.voice_id)

  def cache_key(self, text: str) -> str:
    identity = json.dumps({"model": MODEL_ID, "voice": self.voice_id, "text": text}, sort_keys=True).encode()
    return hashlib.sha256(identity).hexdigest()

  def prepare(self, text: str) -> None:
    if not self.enabled:
      return
    cache_key = self.cache_key(text)
    if (navigation_voice_cache_path(cache_key).is_file() or cache_key in self.futures
        or time.monotonic() < self.retry_after.get(cache_key, 0.0)):
      return
    self.futures[cache_key] = self.executor.submit(self._generate, cache_key, text)

  def ready_cache_key(self, text: str) -> str | None:
    if not self.enabled:
      return None
    cache_key = self.cache_key(text)
    path = navigation_voice_cache_path(cache_key)
    if path.is_file():
      return cache_key
    future = self.futures.get(cache_key)
    if future is None or not future.done():
      return None
    try:
      future.result()
      return cache_key if path.is_file() else None
    except Exception as error:
      if cache_key not in self.reported_failures:
        cloudlog.warning(f"KoalaNav voice generation failed: {error}")
        self.reported_failures.add(cache_key)
      self.futures.pop(cache_key, None)
      self.retry_after[cache_key] = time.monotonic() + 30.0
      return None

  def _generate(self, cache_key: str, text: str) -> Path:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
      raise RuntimeError("ffmpeg is required for KoalaNav navigation speech")
    cache_dir = navigation_voice_cache_dir()
    cache_dir.mkdir(parents=True, exist_ok=True)
    destination = navigation_voice_cache_path(cache_key)
    if destination.is_file():
      return destination

    with tempfile.TemporaryDirectory(prefix="koalanav-voice-", dir=cache_dir) as temp_dir:
      source = Path(temp_dir) / "speech.mp3"
      converted = Path(temp_dir) / "speech.wav"
      source.write_bytes(_generate_mp3(self.api_key, self.voice_id, text))
      subprocess.run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-i", str(source),
        "-af", "loudnorm=I=-18:TP=-1.5:LRA=7", "-ar", "48000", "-ac", "1", "-c:a", "pcm_s16le", str(converted),
      ], check=True)
      os.replace(converted, destination)
    return destination
