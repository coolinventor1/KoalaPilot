#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

class PandaCommsHandle;


struct __attribute__((packed)) KoalaHardwareRecord {
  char magic[4];
  uint8_t schema_version;
  uint8_t product_id;
  uint8_t pcb_revision_major;
  uint8_t pcb_revision_minor;
  uint8_t mcu_id;
  uint8_t can_controller_id;
  uint8_t can_transceiver_id;
  uint8_t can_channel_count;
  uint32_t capabilities;
  uint8_t identity_source;
  uint8_t pico_variant;
  uint8_t reserved[2];
};

static_assert(sizeof(KoalaHardwareRecord) == 20);


class KoalaDevice {
public:
  static constexpr uint8_t USB_REQUEST = 0xFE;
  static constexpr uint16_t HARDWARE_INFO_SELECTOR = 1;

  static bool requested();
  static bool valid_record(const uint8_t *data, size_t length);
  static std::optional<KoalaDevice> probe(PandaCommsHandle &handle);

  explicit KoalaDevice(const KoalaHardwareRecord &record) : hardware(record) {}

  std::string pcb_revision() const;
  const KoalaHardwareRecord &hardware_info() const { return hardware; }

private:
  KoalaHardwareRecord hardware;
};
