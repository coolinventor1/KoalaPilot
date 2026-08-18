#!/usr/bin/env python3

import struct
import unittest

from openpilot.common.koala_hardware import (
  KOALA_HARDWARE_INFO_LENGTH,
  KOALA_HARDWARE_INFO_SELECTOR,
  KOALA_USB_REQUEST,
  KoalaIdentityError,
  KoalaHardwareInfo,
  identify_koala_hardware,
  is_koala_firmware_version,
  probe_koala_hardware,
)


VALID_RECORD = bytes.fromhex("4B4F414C01010003010101037F00000001020000")


class FakeHandle:
  def __init__(self, response: bytes):
    self.response = response
    self.request = None

  def controlRead(self, request_type, request, value, index, length):
    self.request = (request_type, request, value, index, length)
    return self.response


class FakePanda:
  REQUEST_IN = 0xC0

  def __init__(self, response: bytes, bootstub: bool = False):
    self.bootstub = bootstub
    self._handle = FakeHandle(response)


class TestKoalaHardwareInfo(unittest.TestCase):
  def test_parse_live_rev03_record(self):
    info = KoalaHardwareInfo.from_bytes(VALID_RECORD)

    self.assertEqual(info.schema_version, 1)
    self.assertEqual(info.product_id, 1)
    self.assertEqual(info.pcb_revision, "0.3")
    self.assertEqual(info.mcu_id, 1)
    self.assertEqual(info.can_controller_id, 1)
    self.assertEqual(info.can_transceiver_id, 1)
    self.assertEqual(info.can_channel_count, 3)
    self.assertEqual(info.capabilities, 0x7F)
    self.assertEqual(info.identity_source, 1)
    self.assertEqual(info.pico_variant, 2)

  def test_unknown_schema_still_identifies_koala(self):
    record = bytearray(VALID_RECORD)
    record[4] = 2

    self.assertEqual(KoalaHardwareInfo.from_bytes(record).schema_version, 2)

  def test_bad_magic_is_not_koala(self):
    record = bytearray(VALID_RECORD)
    record[:4] = b"PAND"

    with self.assertRaises(ValueError):
      KoalaHardwareInfo.from_bytes(record)

  def test_wrong_length_is_not_koala(self):
    with self.assertRaises(ValueError):
      KoalaHardwareInfo.from_bytes(VALID_RECORD[:-1])

  def test_probe_uses_koala_vendor_extension(self):
    panda = FakePanda(VALID_RECORD)
    info = probe_koala_hardware(panda)

    self.assertIsNotNone(info)
    self.assertEqual(
      panda._handle.request,
      (FakePanda.REQUEST_IN, KOALA_USB_REQUEST, KOALA_HARDWARE_INFO_SELECTOR, 0, KOALA_HARDWARE_INFO_LENGTH),
    )

  def test_probe_rejects_unrelated_response(self):
    panda = FakePanda(struct.pack("<5I", 0, 1, 2, 3, 4))
    self.assertIsNone(probe_koala_hardware(panda))

  def test_bootstub_is_not_probed(self):
    panda = FakePanda(VALID_RECORD, bootstub=True)
    self.assertIsNone(probe_koala_hardware(panda))
    self.assertIsNone(panda._handle.request)

  def test_firmware_version_failsafe(self):
    self.assertTrue(is_koala_firmware_version("KOALA-rev0.3-DEV"))
    self.assertFalse(is_koala_firmware_version("0.11.2-release"))

  def test_identified_koala_bypasses_panda_update_path(self):
    panda = FakePanda(VALID_RECORD)
    info = identify_koala_hardware(panda, "KOALA-rev0.3-DEV")
    self.assertIsInstance(info, KoalaHardwareInfo)

  def test_koala_version_with_broken_identity_blocks_panda_update_path(self):
    panda = FakePanda(b"")
    with self.assertRaises(KoalaIdentityError):
      identify_koala_hardware(panda, "KOALA-rev0.3-DEV")

  def test_standard_panda_uses_existing_update_path(self):
    panda = FakePanda(b"")
    self.assertIsNone(identify_koala_hardware(panda, "0.11.2-release"))


if __name__ == "__main__":
  unittest.main()
