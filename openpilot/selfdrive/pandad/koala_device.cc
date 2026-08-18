#include "selfdrive/pandad/koala_device.h"

#include <cstdlib>
#include <cstring>

#include "selfdrive/pandad/panda_comms.h"


bool KoalaDevice::requested() {
  const char *device = std::getenv("KOALAPILOT_DEVICE");
  return device != nullptr && std::string(device) == "koala";
}

bool KoalaDevice::valid_record(const uint8_t *data, size_t length) {
  return data != nullptr &&
         length == sizeof(KoalaHardwareRecord) &&
         std::memcmp(data, "KOAL", 4) == 0;
}

std::optional<KoalaDevice> KoalaDevice::probe(PandaCommsHandle &handle) {
  KoalaHardwareRecord record = {};
  const int length = handle.control_read(USB_REQUEST, HARDWARE_INFO_SELECTOR, 0,
                                         reinterpret_cast<uint8_t *>(&record), sizeof(record), 1000);
  if (!valid_record(reinterpret_cast<const uint8_t *>(&record), length)) {
    return std::nullopt;
  }
  return KoalaDevice(record);
}

std::string KoalaDevice::pcb_revision() const {
  return std::to_string(hardware.pcb_revision_major) + "." + std::to_string(hardware.pcb_revision_minor);
}
