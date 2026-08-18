# KoalaPilot continuation handoff

Snapshot date: 2026-08-18

This file is the durable starting point for continuing Koala and KoalaPilot on
another computer or in a new Codex task. Read this file, `KOALAPILOT.md`, and
the sibling firmware documentation before changing code or hardware claims.

## Objective

Koala is a custom, Panda-like USB vehicle interface built around a castellated
Raspberry Pi Pico 2. KoalaPilot is an openpilot fork that recognizes Koala as
its own hardware instead of pretending it is a comma Panda.

The intended vehicle is a US-market 2021 Honda CR-V with a Honda Bosch A comma
harness. The current design is a development prototype. Physical CAN traffic,
relay contacts, heartbeat failsafes, and harness pin mapping still require
bench and stationary-vehicle verification before any road testing.

## Critical Git state

Repository:

- Windows path: `C:\Users\Owner\Documents\Codex\2026-08-11\codex-mcp-add-flux-url-https\KoalaPilot`
- Intended Ubuntu path: `~/KoalaPilot`
- Branch: `agent/koala-awareness`
- Current committed base: `d9c394d1cf48b85bae0548e63248bc8ddcfc680b`
- Commit title: `Recognize Koala hardware before Panda updates`
- `origin`: `https://github.com/coolinventor1/KoalaPilot.git`
- `upstream`: `https://github.com/commaai/openpilot.git`

Important: the latest Koala USB transport, UI, audio, and simulator work is
still present as uncommitted changes in the Windows working tree. A fresh clone
of GitHub alone will not contain all of it. Preserve or transfer the entire
working tree, including untracked files, before relying on Ubuntu.

Important untracked source/assets include:

- `openpilot/selfdrive/pandad/koala_device.cc`
- `openpilot/selfdrive/pandad/koala_device.h`
- `openpilot/selfdrive/pandad/usb.cc`
- `openpilot/selfdrive/pandad/tests/test_koala_device.cc`
- `openpilot/selfdrive/assets/fonts/After-Regular.ttf`
- `openpilot/selfdrive/assets/sounds/gear/*.wav`
- `openpilot/selfdrive/assets/sounds/voice/*.wav`

Do not copy generated caches or test executables as source. In particular,
`__pycache__` files and `openpilot/selfdrive/pandad/tests/test_koala_device`
can be regenerated.

## Workspace map

Several parts of the project are siblings of this repository and are not
included by cloning KoalaPilot:

- PCB record: `../KOALA-PCB.md`
- PCB renders and audit: `../outputs/`
- Koala firmware source: `../firmware/koala_panda/`
- Factory test firmware: `../firmware/koala_factory_test/`
- Koala information utility: `../apps/koala_info/`
- JLCPCB production archive:
  `C:\Users\Owner\Downloads\Koala_2_PCB1_20260817_105140.zip_Y31.zip`
- Production archive SHA-256:
  `8037931236AB31DD0592068CB5390EC8D678125FF7D00E81282C410FFC2922B3`

Copy those sibling folders and the production archive if the Ubuntu machine
needs the complete hardware history, rather than only KoalaPilot software.

## Koala Rev0.3 hardware snapshot

- MCU module: non-wireless Raspberry Pi Pico 2 / RP2350, hand-soldered through
  its castellated pads after JLCPCB assembly.
- Three CAN controllers: MCP2518FD.
- Three CAN transceivers: MCP2562FD.
- Shared transmit gate: automotive-qualified SN74LVC125A-Q1 at U2.
- CAN termination: R20, R21, and R22 are 120 ohm, fitted per bus. The selected
  JLCPCB replacement was `0603WAF1200T5E`, LCSC `C22787`.
- Relay: deenergized by default. Its normally-closed path is intended to retain
  the car's original CAN bridge when Koala has no USB power or firmware control.
- Relay control and CAN transmit permission are separate firmware gates.
- J2 is the physical transmit-arm shunt; J3 is the physical relay-enable shunt.
- CAN0, CAN1, and CAN2 each have a green activity LED. The red RELAY LED is
  electrically across the relay coil and therefore indicates real coil power.
- The Pico is intentionally powered over USB, not from the vehicle harness.
- The EasyEDA PCB passed its final reported DRC with zero errors.
- The boards and JLCPCB assembly were ordered for USD 116.66. The Pico 2 was
  ordered separately.

The 18-pin connector mechanically matches the existing Bosch A harness, but
mechanical fit and a zero-error PCB DRC do not prove the vehicle-side pinout.
Confirm continuity from the physical harness pins to Koala CANH/CANL pairs,
ground, and relay contacts during first-board validation.

## Production firmware

Firmware directory:

`../firmware/koala_panda/`

Primary production image:

`../firmware/koala_panda/artifacts/koala_panda-pico2-koala-rev03.uf2`

SHA-256:

`B91E893062A8FC9901B4EB3B6DB7E55564F14AA1C24A971A9CC6C978DA92FE63`

Standalone Pico 2 W diagnostic image:

`../firmware/koala_panda/artifacts/koala_panda-pico2w-standalone-test.uf2`

SHA-256:

`D27E19E44096DBE4293826218CC6CA3D593DD18ED8E4331FD374A3E9A18D5806`

The standalone Pico 2 W build was live-tested. Confirmed behavior included USB
enumeration as `3801:DDCC`, legacy hardware type `0x07`, health/version reads,
the versioned `KOAL` record, relay-request gating, heartbeat timeout, and a
working onboard activity LED. All MCP probes failed cleanly because that Pico
was not attached to the Koala carrier. The faint high-pitched sound heard while
its wireless onboard LED was illuminated was noted but did not block testing.

Do not flash `koala_panda-pico2-legacy-pinmap.uf2` onto Rev0.3.

### Rev0.3 GPIO map

| Function | GPIO |
| --- | ---: |
| SPI MISO | GP16 |
| MCP0 CS / interrupt | GP17 / GP20 |
| SPI clock / MOSI | GP18 / GP19 |
| MCP1 CS / interrupt | GP21 / GP22 |
| MCP2 CS / interrupt | GP26 / GP27 |
| Transceiver standby, CAN0/1/2 | GP2 / GP3 / GP4 |
| Shared TX arm | GP5 |
| CAN LEDs, CAN0/1/2 | GP6 / GP7 / GP8 |
| Relay enable | GP15 |

The full firmware behavior, hardware-record ABI, and fail-safe conditions are
documented in `../firmware/koala_panda/README.md`.

## KoalaPilot integration snapshot

The current working tree implements or contains work for:

- `KOALAPILOT_DEVICE=koala` selection.
- libusb-based Panda-compatible control and bulk endpoints.
- Validation of the 20-byte versioned `KOAL` hardware record before accepting
  a device as Koala.
- A first-class `KoalaDevice` and `PandaState.PandaType.koala`.
- Bypassing STM32H7 Panda auto-flashing only after Koala identification.
- Publishing Koala state to the UI and displaying `KOALA ONLINE` in the corner.
- KoalaPilot branding. The `koala` part of the prominent wordmark uses the
  bundled After font while `pilot` retains the normal UI font.
- Pre-generated ElevenLabs prompts for engagement, disengagement, lane changes,
  takeover requests, and immediate takeover. The API key is not needed in-car.
- HondaDash Park, Reverse, Neutral, and Drive WAV files. The first valid gear
  reading after startup is deliberately silent.
- Unit coverage for Koala identification/CAN protocol and the sound prompt
  selection/mixing logic.

Confirmed on standalone firmware does not yet equal confirmed through the
KoalaPilot `pandad` path. USB bulk CAN transfer has not been proven end to end
with a populated Koala Rev0.3 board.

## Simulator status

The MetaDrive simulator was run under WSL, but rendering was laggy and the car
did not initially remain on the road. The working tree contains simulator-only
changes for:

- a larger readable UI (`SCALE=2.0` by default);
- lower-resolution camera rendering with upscale before model input;
- WSL D3D12 selection when `/dev/dxg` is available;
- reduced false service-health alerts during simulation;
- lower simulated speed;
- an optional MetaDrive IDM lane-keeping helper enabled by
  `SIM_LANE_ASSIST=1`.

The IDM helper keeps the simulated vehicle on its lane for UI/integration work;
it is not evidence that openpilot's lateral-control output is steering the
simulated car correctly. Native Ubuntu should be used to retest performance,
model inference, audio output, and actual openpilot steering separately.

### Launch on native Ubuntu

After transferring the complete working tree:

```bash
cd ~/KoalaPilot
git submodule update --init --recursive
git lfs pull
./tools/op.sh setup
```

Then launch the simulator with the helper:

```bash
cd ~/KoalaPilot
./tools/op.sh sim
```

If running the two processes separately is easier for debugging:

```bash
# Terminal 1
cd ~/KoalaPilot
./openpilot/tools/sim/launch_openpilot.sh
```

```bash
# Terminal 2
cd ~/KoalaPilot
source .venv/bin/activate
./openpilot/tools/sim/run_bridge.py
```

Simulator controls: press `2` to engage, `1` to increase the set speed, `2` to
decrease it, `S` to brake/disengage, `R` to reset, and `Q` to quit.

For a diagnostic run without the MetaDrive lane helper:

```bash
SIM_LANE_ASSIST=0 ./openpilot/tools/sim/run_bridge.py
```

That mode is the meaningful check for whether KoalaPilot itself is producing
usable steering in the simulator.

## First assembled-board validation order

Keep the car disconnected for the first tests.

1. Inspect orientation, solder bridges, Pico castellated joints, connector
   seating, and the fitted U2/R20/R21/R22 parts.
2. With J2 and J3 open, power only over USB. Confirm the relay is deenergized,
   the red RELAY LED is off, and the three CAN LEDs perform their startup sweep.
3. Read the USB identity, version, health, live status flags, and `KOAL` hardware
   record. Confirm all three MCP initialized bits become set.
4. Confirm heartbeat loss and USB disconnect leave TX disabled and the relay
   deenergized.
5. Use an isolated CAN fixture to validate receive and transmit separately on
   CAN0, CAN1, and CAN2. Confirm each matching LED pulses and verify behavior
   with and without the fitted termination as appropriate for the fixture.
6. With a meter and an isolated harness fixture, verify the relay's normally
   closed continuity, energized routing, J3 gating, and drop-out on heartbeat
   loss.
7. Verify J2 is required before any CAN transmit path becomes active.
8. Only after those checks pass, connect to the stationary vehicle with
   transmit disabled and log all three buses. Compare observed traffic and
   physical pin continuity with the verified Bosch A pinout.
9. Enable controlled transmission only on a bench or stationary test setup
   after receive-only behavior, bus voltages, error counters, and relay routing
   are understood.

## Known unknowns and next work

- Physical Rev0.3 MCP SPI detection and three-bus CAN traffic.
- End-to-end Koala firmware to libusb `pandad` CAN transport.
- Exact physical Bosch A harness-to-Koala net continuity on the purchased cable.
- Relay contact routing and timing on the assembled PCB.
- Native-Ubuntu simulator performance and audio-device behavior.
- Actual lateral-control steering in MetaDrive with `SIM_LANE_ASSIST=0`.
- A clean, reviewed commit of all current modified and untracked source/assets.
- Automated regression tests for USB disconnects, heartbeat loss, malformed
  hardware records, CAN framing, and relay/TX gate independence.

## Prompt for a new Codex task

Use this after opening `~/KoalaPilot` in Codex:

> Read HANDOFF.md and KOALAPILOT.md completely. Also inspect the sibling
> firmware/koala_panda/README.md if that folder is available. Preserve all
> existing uncommitted work. First verify the Git branch and working-tree state,
> then set up native Ubuntu and launch the MetaDrive simulator. Report actual
> UI frame rate, audio-device errors, model/service health, and whether steering
> works with SIM_LANE_ASSIST=0. Do not treat the lane-assist helper as proof of
> openpilot lateral control. Do not make vehicle-readiness claims until the
> first-board validation checklist has real measurements.
