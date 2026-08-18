# KoalaPilot voice prompts

These WAV files are pre-generated ElevenLabs speech assets. They are mono,
48 kHz, signed 16-bit PCM for direct playback by `soundd`; the vehicle does not
need an API key or network connection.

Regenerate them from the repository root with:

```sh
python openpilot/selfdrive/assets/sounds/generate_voice_prompts.py
```

The generator reads `ELEVENLABS_API_KEY` and `ELEVENLABS_VOICE_ID` from the
environment. Credentials and the selected voice ID are not stored in the
repository.
