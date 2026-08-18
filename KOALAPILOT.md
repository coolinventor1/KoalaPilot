# KoalaPilot

KoalaPilot is the openpilot fork for the Koala Rev0.3 automotive interface. The
goal is to retain openpilot's driving stack while replacing the assumption that
the vehicle interface is a comma Panda connected through its current native
transport.

## Current state

- The project is forked from `commaai/openpilot` with upstream Git history intact.
- Koala identifies over USB using Panda-compatible VID/PID values.
- Koala USB control transfers, hardware identity, firmware version, and serial
  reads have been verified on the RP2350 firmware.
- KoalaPilot detects the versioned `KOAL` hardware record before evaluating a
  Panda firmware signature and passes the device family to the runtime through
  `KOALAPILOT_DEVICE=koala`.
- The existing `pandad` runtime selects `KoalaUsbHandle` in Koala mode,
  enumerates only devices with a valid `KOAL` record, and retains that record
  in a first-class `KoalaDevice`. Panda mode continues to use SPI.
- After positive identification, `pandad` publishes the dedicated `koala`
  hardware type. The on-device sidebar reports `KOALA ONLINE` while the
  firmware retains its legacy Red Panda response for third-party compatibility.
- Panda STM32H7 automatic flashing is bypassed for a positively identified
  Koala. A Koala firmware version without a valid hardware record also blocks
  the Panda update path.
- Koala implements three MCP2518FD controllers and three MCP2562FD transceivers.
- Native USB control and bulk endpoints are now wired into `pandad`; USB bulk
  CAN transfer has not yet been verified end-to-end on a physical Koala PCB.
- On-road alerts use bundled ElevenLabs voice prompts for engagement, lane
  changes, and takeover requests. Speech is generated ahead of time and mixed
  with the existing alert tones, so no network or API key is required in-car.
- Park, Reverse, Neutral, and Drive transitions use the supplied HondaDash WAV
  prompts. The first valid gear reading after startup is intentionally silent.
- Stock openpilot firmware-signature handling must not attempt to flash Koala
  with STM32H7 Panda firmware.
- The default-off `koalanavd` foundation consumes GPS, route geometry, and
  maneuver instructions on dedicated cereal services. It publishes a checked
  10 Hz shadow plan but has no connection to the normal openpilot control path.
  Its `controlAllowed` output is always false.

## Compatibility plan

1. Synchronize health and CAN packet ABIs with the selected openpilot revision.
2. Verify control transfers, heartbeats, safety-mode changes, and USB bulk CAN
   using a loopback fixture.
3. Verify all three CAN buses and fail-safe relay behavior on a populated Koala
   PCB before any vehicle integration.
4. Add automated Koala transport and protocol regression tests.
5. Feed recorded navigation routes through KoalaNav in replay and simulation;
   keep it shadow-only until its turn intent and route matching are measured.

## Repository structure

Internal module names remain `openpilot` so upstream changes can be merged with
small, reviewable conflicts. Koala-specific code and documentation use the
KoalaPilot name.

The remotes are intended to remain:

- `origin`: `https://github.com/coolinventor1/KoalaPilot.git`
- `upstream`: `https://github.com/commaai/openpilot.git`

## Project status

KoalaPilot is in hardware-integration development. It is not yet a drop-in
replacement for a comma Panda.
