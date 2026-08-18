# KoalaNav

KoalaNav is a separate, default-off navigation planner. It combines live GPS data with a Mapbox route and turn instructions, then publishes a `koalaNavPlan` at 10 Hz. This phase is shadow-only: it observes and reports upcoming maneuvers but is not connected to `controlsd`, `plannerd`, `carControl`, or Koala CAN transmission.

`koalanav_providerd` requests a full GeoJSON route from the Mapbox Directions API when a destination is set. It converts Mapbox maneuver locations, outgoing bearings, road names, and geometry into KoalaNav's dedicated cereal messages. It requests another route only when the destination changes or the GPS position remains more than 80 m off-route for eight seconds.

## Inputs

- `gpsLocationExternal`
- `koalaNavRoute`: a route ID, revision, and ordered latitude/longitude geometry
- `koalaNavInstruction`: the next maneuver, its target point, outgoing road bearing, and navigation-provider distance

The route and instruction carry the same content-derived route ID. GPS accuracy must be 25 m or better, and both the current position and maneuver target must be within 40 m of the supplied route.

## Output

`koalaNavPlan` reports the fused state (`monitoring`, `approach`, `ready`, or a waiting/aborted state), GPS-derived distance to the turn, signed turn angle, route match, and confidence. `controlAllowed` is unconditionally false.

`turn_desire.py` translates the navigation maneuver and incoming/outgoing road bearings into a checked high-level turn intent. It deliberately has no curvature, steering-torque, or actuator fields, keeping KoalaNav independent from normal openpilot driving.

## Navigation speech

The provider prepares two ElevenLabs prompts for each of the next two maneuvers:

- An advance prompt, such as `Turn left ahead onto Main Street.`
- A near-turn prompt, such as `Now turn left onto Main Street.`

Speech generation runs in background threads and is cached by voice, model, and exact text under the KoalaPilot download cache. `soundd` loads only validated mono, 48 kHz, signed 16-bit WAV files from that cache. Navigation prompts have lower interruption priority than gear, lane-change, takeover, and immediate-warning speech. A network interruption therefore cannot suppress a safety alert, and a previously generated direction remains usable offline.

ElevenLabs returns MP3 audio; `ffmpeg` must be installed so the provider can create the WAV format required by `soundd`.

On Ubuntu, install it once with `sudo apt-get install ffmpeg`.

## Configuration

Credentials are never committed or written to logs. The processes read `MAPBOX_ACCESS_TOKEN`, `ELEVENLABS_API_KEY`, and `ELEVENLABS_VOICE_ID` from the environment first. They can alternatively read the non-logged `MapboxAccessToken`, `ElevenLabsApiKey`, and `ElevenLabsVoiceId` parameters.

To import existing environment credentials into the persistent, non-logged parameters:

```bash
.venv/bin/python - <<'PY'
import os
from openpilot.common.params import Params

p = Params()
for environment_name, param_name in (
  ("MAPBOX_ACCESS_TOKEN", "MapboxAccessToken"),
  ("ELEVENLABS_API_KEY", "ElevenLabsApiKey"),
  ("ELEVENLABS_VOICE_ID", "ElevenLabsVoiceId"),
):
  value = os.environ.get(environment_name)
  if value:
    p.put(param_name, value, block=True)
PY
```

Set a destination with decimal latitude and longitude:

```bash
.venv/bin/python - <<'PY'
from openpilot.common.params import Params

Params().put("KoalaNavDestination", {
  "latitude": 41.8781,
  "longitude": -87.6298,
}, block=True)
PY
```

## Enabling shadow mode

KoalaNav is disabled by default. On a development installation:

```bash
.venv/bin/python -c 'from openpilot.common.params import Params; p = Params(); p.put("KoalaNavMode", "shadow", block=True); p.put_bool("KoalaNavEnabled", True, block=True)'
```

The navigation provider must publish `koalaNavRoute` after `koalanavd` starts, then publish `koalaNavInstruction` at 1 Hz. `navigation_input.py` contains builders defining that provider contract.

With Mapbox configured, `koalanav_providerd` performs those publications automatically. Without a token or destination it stays idle and `koalanavd` remains in a waiting state.

No actuator consumer should be added until replay and simulator acceptance criteria are defined and passed. The eventual actuation interface should consume a separately reviewed trajectory message, not this observational plan directly.
