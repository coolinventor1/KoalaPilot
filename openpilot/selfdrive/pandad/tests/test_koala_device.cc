#include <algorithm>
#include <cstring>
#include <vector>

#include "common/tests/native_test.h"
#include "selfdrive/pandad/koala_device.h"
#include "selfdrive/pandad/panda_comms.h"


class FakeKoalaHandle final : public PandaCommsHandle {
public:
  explicit FakeKoalaHandle(std::vector<uint8_t> response) : response(std::move(response)) {}

  int control_write(uint8_t, uint16_t, uint16_t, unsigned int) override { return 0; }
  int bulk_write(unsigned char, unsigned char *, int, unsigned int) override { return 0; }
  int bulk_read(unsigned char, unsigned char *, int, unsigned int) override { return 0; }
  const char *transport_name() const override { return "fake"; }

  int control_read(uint8_t request, uint16_t param1, uint16_t param2,
                   unsigned char *data, uint16_t length, unsigned int timeout) override {
    CHECK(request == KoalaDevice::USB_REQUEST);
    CHECK(param1 == KoalaDevice::HARDWARE_INFO_SELECTOR);
    CHECK(param2 == 0);
    CHECK(length == sizeof(KoalaHardwareRecord));
    CHECK(timeout == 1000);
    std::memcpy(data, response.data(), std::min<size_t>(length, response.size()));
    return response.size();
  }

private:
  std::vector<uint8_t> response;
};


void test_valid_hardware_record() {
  const std::vector<uint8_t> record = {
    'K', 'O', 'A', 'L',
    1, 1, 0, 3, 1, 1, 1, 3,
    0x7f, 0x00, 0x00, 0x00,
    1, 2, 0, 0,
  };
  FakeKoalaHandle handle(record);

  const auto koala = KoalaDevice::probe(handle);
  REQUIRE(koala.has_value());
  CHECK(koala->pcb_revision() == "0.3");
  CHECK(koala->hardware_info().can_channel_count == 3);
  CHECK(koala->hardware_info().capabilities == 0x7f);
  CHECK(koala->hardware_info().pico_variant == 2);
}

void test_non_koala_record() {
  std::vector<uint8_t> record(sizeof(KoalaHardwareRecord), 0);
  FakeKoalaHandle handle(record);
  CHECK(!KoalaDevice::probe(handle).has_value());
}

void test_truncated_koala_record() {
  const std::vector<uint8_t> record = {'K', 'O', 'A', 'L'};
  FakeKoalaHandle handle(record);
  CHECK(!KoalaDevice::probe(handle).has_value());
}

void test_koala_device() {
  test_valid_hardware_record();
  test_non_koala_record();
  test_truncated_koala_record();
}

int main() {
  return run_native_test(test_koala_device);
}
