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
- Koala implements three MCP2518FD controllers and three MCP2562FD transceivers.
- USB bulk CAN transfer has not yet been verified end-to-end on a physical
  Koala PCB.
- Stock openpilot firmware-signature handling must not attempt to flash Koala
  with STM32H7 Panda firmware.

## Compatibility plan

1. Add a Koala-aware USB transport to `pandad`.
2. Detect Koala explicitly and bypass Panda STM32H7 automatic flashing.
3. Synchronize health and CAN packet ABIs with the selected openpilot revision.
4. Verify control transfers, heartbeats, safety-mode changes, and USB bulk CAN
   using a loopback fixture.
5. Verify all three CAN buses and fail-safe relay behavior on a populated Koala
   PCB before any vehicle integration.
6. Add automated Koala transport and protocol regression tests.

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
