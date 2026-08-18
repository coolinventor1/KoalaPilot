"""Koala hardware identity support for KoalaPilot host processes."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Any

import usb1


KOALA_USB_REQUEST = 0xFE
KOALA_HARDWARE_INFO_SELECTOR = 1
KOALA_HARDWARE_INFO_LENGTH = 20
KOALA_HARDWARE_MAGIC = b"KOAL"
KOALA_FIRMWARE_PREFIX = "KOALA-"

_HARDWARE_RECORD = struct.Struct("<4s8BIBB2s")


@dataclass(frozen=True)
class KoalaHardwareInfo:
  schema_version: int
  product_id: int
  pcb_revision_major: int
  pcb_revision_minor: int
  mcu_id: int
  can_controller_id: int
  can_transceiver_id: int
  can_channel_count: int
  capabilities: int
  identity_source: int
  pico_variant: int

  @classmethod
  def from_bytes(cls, data: bytes) -> KoalaHardwareInfo:
    if len(data) != _HARDWARE_RECORD.size:
      raise ValueError(f"invalid Koala hardware record length: {len(data)}")

    (
      magic,
      schema_version,
      product_id,
      pcb_revision_major,
      pcb_revision_minor,
      mcu_id,
      can_controller_id,
      can_transceiver_id,
      can_channel_count,
      capabilities,
      identity_source,
      pico_variant,
      _reserved,
    ) = _HARDWARE_RECORD.unpack(data)

    if magic != KOALA_HARDWARE_MAGIC:
      raise ValueError(f"invalid Koala hardware magic: {magic!r}")

    return cls(
      schema_version=schema_version,
      product_id=product_id,
      pcb_revision_major=pcb_revision_major,
      pcb_revision_minor=pcb_revision_minor,
      mcu_id=mcu_id,
      can_controller_id=can_controller_id,
      can_transceiver_id=can_transceiver_id,
      can_channel_count=can_channel_count,
      capabilities=capabilities,
      identity_source=identity_source,
      pico_variant=pico_variant,
    )

  @property
  def pcb_revision(self) -> str:
    return f"{self.pcb_revision_major}.{self.pcb_revision_minor}"

  def log_fields(self) -> dict[str, int | str]:
    return {
      "schema": self.schema_version,
      "product_id": self.product_id,
      "pcb_revision": self.pcb_revision,
      "mcu_id": self.mcu_id,
      "can_controller_id": self.can_controller_id,
      "can_transceiver_id": self.can_transceiver_id,
      "can_channels": self.can_channel_count,
      "capabilities": self.capabilities,
      "identity_source": self.identity_source,
      "pico_variant": self.pico_variant,
    }


class KoalaIdentityError(RuntimeError):
  pass


def probe_koala_hardware(panda: Any) -> KoalaHardwareInfo | None:
  """Return Koala identity when the device implements the Koala USB extension."""
  if panda.bootstub:
    return None

  try:
    # Panda currently exposes no generic vendor-read API.
    raw = panda._handle.controlRead(
      panda.REQUEST_IN,
      KOALA_USB_REQUEST,
      KOALA_HARDWARE_INFO_SELECTOR,
      0,
      KOALA_HARDWARE_INFO_LENGTH,
    )
  except usb1.USBErrorPipe:
    # A standard USB Panda stalls requests it does not implement.
    return None

  try:
    return KoalaHardwareInfo.from_bytes(bytes(raw))
  except ValueError:
    return None


def is_koala_firmware_version(version: str) -> bool:
  return version.startswith(KOALA_FIRMWARE_PREFIX)


def identify_koala_hardware(panda: Any, firmware_version: str) -> KoalaHardwareInfo | None:
  """Identify Koala before any Panda-specific firmware-update decision."""
  info = probe_koala_hardware(panda)
  if info is not None:
    return info

  if is_koala_firmware_version(firmware_version):
    raise KoalaIdentityError("Koala hardware identity is unavailable; Panda firmware update blocked")

  return None
