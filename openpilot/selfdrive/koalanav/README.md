# KoalaNav foundation

KoalaNav is a separate, default-off navigation planner. It combines live GPS data with an externally supplied route and turn instruction, then publishes a `koalaNavPlan` at 10 Hz. This first phase is shadow-only: it observes and reports upcoming maneuvers but is not connected to `controlsd`, `plannerd`, `carControl`, or Koala CAN transmission.

## Inputs

- `gpsLocationExternal` (preferred) or `gpsLocation`
- `koalaNavRoute`: a route ID, revision, and ordered latitude/longitude geometry
- `koalaNavInstruction`: the next maneuver, its target point, outgoing road bearing, and navigation-provider distance

The route and instruction must carry the same non-empty route ID. GPS accuracy must be 25 m or better, and both the current position and maneuver target must be within 40 m of the supplied route.

## Output

`koalaNavPlan` reports the fused state (`monitoring`, `approach`, `ready`, or a waiting/aborted state), GPS-derived distance to the turn, signed turn angle, route match, and confidence. `controlAllowed` is unconditionally false.

`turn_desire.py` translates the navigation maneuver and incoming/outgoing road bearings into a checked high-level turn intent. It deliberately has no curvature, steering-torque, or actuator fields, keeping KoalaNav independent from normal openpilot driving.

## Enabling shadow mode

KoalaNav is disabled by default. On a development installation:

```bash
.venv/bin/python -c 'from openpilot.common.params import Params; p = Params(); p.put("KoalaNavMode", "shadow", block=True); p.put_bool("KoalaNavEnabled", True, block=True)'
```

The navigation provider must publish `koalaNavRoute` after `koalanavd` starts, then publish `koalaNavInstruction` at 1 Hz. `navigation_input.py` contains builders defining that provider contract.

No actuator consumer should be added until replay and simulator acceptance criteria are defined and passed. The eventual actuation interface should consume a separately reviewed trajectory message, not this observational plan directly.
