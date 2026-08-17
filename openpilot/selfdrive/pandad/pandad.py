#!/usr/bin/env python3
# simple pandad wrapper that updates the panda first
import os
import usb1
import time
import signal
import subprocess

from panda import Panda, PandaDFU, PandaProtocolMismatch, McuType, FW_PATH
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.hardware import HARDWARE
from openpilot.common.swaglog import cloudlog
from openpilot.common.koala_hardware import KoalaHardwareInfo, identify_koala_hardware, probe_koala_hardware


def get_expected_signature() -> bytes:
  fn = os.path.join(FW_PATH, McuType.H7.config.app_fn)
  return Panda.get_signature_from_firmware(fn)

def flash_panda(panda_serial: str) -> KoalaHardwareInfo | None:
  panda = Panda(panda_serial)
  try:
    panda_version = "bootstub" if panda.bootstub else panda.get_version()
    koala_info = identify_koala_hardware(panda, panda_version)
    if koala_info is not None:
      cloudlog.event(
        "koalapilot.koala_connected",
        serial=panda_serial,
        version=panda_version,
        **koala_info.log_fields(),
      )
      return koala_info

    fw_signature = get_expected_signature()
    internal_panda = panda.is_internal()
    panda_signature = b"" if panda.bootstub else panda.get_signature()
    cloudlog.warning(f"Panda {panda_serial} connected, version: {panda_version}, signature {panda_signature.hex()[:16]}, expected {fw_signature.hex()[:16]}")

    if panda.bootstub or panda_signature != fw_signature:
      cloudlog.info("Panda firmware out of date, update required")
      panda.flash()
      cloudlog.info("Done flashing")

    if panda.bootstub:
      bootstub_version = panda.get_version()
      cloudlog.info(f"Flashed firmware not booting, flashing development bootloader. {bootstub_version=}, {internal_panda=}")
      if internal_panda:
        HARDWARE.recover_internal_panda()
      panda.recover(reset=(not internal_panda))
      cloudlog.info("Done flashing bootstub")

    if panda.bootstub:
      cloudlog.info("Panda still not booting, exiting")
      raise AssertionError

    panda_signature = panda.get_signature()
    if panda_signature != fw_signature:
      cloudlog.info("Version mismatch after flashing, exiting")
      raise AssertionError

    return None
  finally:
    panda.close()


def main() -> None:
  # signal pandad to close the relay and exit
  def signal_handler(signum, frame):
    cloudlog.info(f"Caught signal {signum}, exiting")
    nonlocal do_exit
    do_exit = True
    if process is not None:
      process.send_signal(signal.SIGINT)

  process = None
  do_exit = False
  signal.signal(signal.SIGINT, signal_handler)

  # check health for lost heartbeat
  try:
    for s in Panda.list():
      with Panda(s) as p:
        koala_info = probe_koala_hardware(p)
        if koala_info is not None:
          cloudlog.event("koalapilot.koala_detected", serial=s, **koala_info.log_fields())
          continue
        health = p.health()
        if p.is_internal() and health["heartbeat_lost"]:
          Params().put_bool("PandaHeartbeatLost", True, block=True)
          cloudlog.event("heartbeat lost", deviceState=health)
  except Exception:
    cloudlog.exception("pandad.uncaught_exception")

  count = 0
  while not do_exit:
    try:
      cloudlog.event("pandad.flash_and_connect", count=count)
      if (count % 2) == 0:
        HARDWARE.reset_internal_panda()
      else:
        HARDWARE.recover_internal_panda()
      count += 1

      # Flash all Pandas in DFU mode
      for serial in PandaDFU.list():
        cloudlog.info(f"Panda in DFU mode found, flashing recovery {serial}")
        PandaDFU(serial).recover()
        time.sleep(1)

      panda_serials = Panda.list()
      if len(panda_serials):
        assert len(panda_serials) == 1
        cloudlog.info(f"{len(panda_serials)} panda found, connecting - {panda_serials}")
        koala_info = flash_panda(panda_serials[0])
        if koala_info is not None:
          os.environ["KOALAPILOT_DEVICE"] = "koala"
        else:
          os.environ.pop("KOALAPILOT_DEVICE", None)

        # run real pandad
        os.environ['MANAGER_DAEMON'] = 'pandad'
        process = subprocess.Popen(["./pandad"], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"))
        process.wait()
    # TODO: wrap all panda exceptions in a base panda exception
    except (usb1.USBErrorNoDevice, usb1.USBErrorPipe):
      # a panda was disconnected while setting everything up. let's try again
      cloudlog.exception("Panda USB exception while setting up")
    except PandaProtocolMismatch:
      cloudlog.exception("pandad.protocol_mismatch")
    except Exception:
      cloudlog.exception("pandad.uncaught_exception")


if __name__ == "__main__":
  main()
