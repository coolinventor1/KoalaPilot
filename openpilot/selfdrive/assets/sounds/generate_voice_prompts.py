#!/usr/bin/env python3
"""Generate KoalaPilot's offline voice prompts with ElevenLabs."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import urllib.error
import urllib.request


PROMPTS = {
  "take_over_immediately": "Take over immediately.",
  "take_control": "Take control.",
  "lane_change_left": "Lane change left.",
  "lane_change_right": "Lane change right.",
  "lane_change_complete": "Lane change complete.",
  "openpilot_engaged": "Open Pilot engaged.",
  "openpilot_disengaged": "Open Pilot disengaged.",
}

MODEL_ID = "eleven_multilingual_v2"
OUTPUT_FORMAT = "mp3_44100_128"
OUTPUT_DIR = Path(__file__).resolve().parent / "voice"


def generate_mp3(api_key: str, voice_id: str, text: str) -> bytes:
  url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}?output_format={OUTPUT_FORMAT}"
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
    detail = error.read().decode(errors="replace")[:500]
    raise RuntimeError(f"ElevenLabs generation failed ({error.code}): {detail}") from error


def convert_to_soundd_wav(ffmpeg: str, source: Path, destination: Path) -> None:
  subprocess.run([
    ffmpeg,
    "-hide_banner",
    "-loglevel", "error",
    "-y",
    "-i", str(source),
    "-af", "loudnorm=I=-18:TP=-1.5:LRA=7",
    "-ar", "48000",
    "-ac", "1",
    "-c:a", "pcm_s16le",
    str(destination),
  ], check=True)


def main() -> None:
  api_key = os.environ.get("ELEVENLABS_API_KEY")
  voice_id = os.environ.get("ELEVENLABS_VOICE_ID")
  if not api_key or not voice_id:
    raise RuntimeError("ELEVENLABS_API_KEY and ELEVENLABS_VOICE_ID must be set")

  ffmpeg = shutil.which("ffmpeg")
  if ffmpeg is None:
    raise RuntimeError("ffmpeg is required to create soundd-compatible WAV files")

  OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix="koalapilot-voice-") as temp_dir:
    temp_path = Path(temp_dir)
    for name, text in PROMPTS.items():
      print(f"Generating {name}: {text}")
      mp3_path = temp_path / f"{name}.mp3"
      mp3_path.write_bytes(generate_mp3(api_key, voice_id, text))
      convert_to_soundd_wav(ffmpeg, mp3_path, OUTPUT_DIR / f"{name}.wav")

  print(f"Generated {len(PROMPTS)} prompts in {OUTPUT_DIR}")


if __name__ == "__main__":
  main()
