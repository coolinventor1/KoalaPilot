#include "selfdrive/pandad/panda_comms.h"

#include <libusb-1.0/libusb.h>

#include <array>
#include <stdexcept>

#include "common/swaglog.h"
#include "selfdrive/pandad/koala_device.h"


namespace {

constexpr uint16_t PANDA_USB_VENDOR_ID = 0x3801;
constexpr uint16_t PANDA_USB_PRODUCT_ID = 0xDDCC;
constexpr int PANDA_USB_INTERFACE = 0;
constexpr unsigned int KOALA_USB_BULK_READ_TIMEOUT_MS = 5;

libusb_context *init_usb_context() {
  libusb_context *context = nullptr;
  const int err = libusb_init(&context);
  if (err != LIBUSB_SUCCESS) {
    LOGE("Koala USB initialization failed: %s", libusb_error_name(err));
    return nullptr;
  }
  return context;
}

bool is_panda_usb_device(const libusb_device_descriptor &descriptor) {
  return descriptor.idVendor == PANDA_USB_VENDOR_ID && descriptor.idProduct == PANDA_USB_PRODUCT_ID;
}

std::string read_serial(libusb_device_handle *handle, uint8_t descriptor_index) {
  if (descriptor_index == 0) return {};

  std::array<unsigned char, 64> serial = {};
  const int length = libusb_get_string_descriptor_ascii(handle, descriptor_index, serial.data(), serial.size());
  return length > 0 ? std::string(reinterpret_cast<char *>(serial.data()), length) : std::string();
}

bool has_koala_identity(libusb_device_handle *handle) {
  std::array<uint8_t, sizeof(KoalaHardwareRecord)> record = {};
  constexpr uint8_t request_type = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
  const int length = libusb_control_transfer(handle, request_type, KoalaDevice::USB_REQUEST,
                                             KoalaDevice::HARDWARE_INFO_SELECTOR, 0,
                                             record.data(), record.size(), 1000);
  return KoalaDevice::valid_record(record.data(), length);
}

}  // namespace


struct KoalaUsbHandle::Impl {
  libusb_context *context = nullptr;
  libusb_device_handle *device = nullptr;
  bool interface_claimed = false;
};

KoalaUsbHandle::KoalaUsbHandle(const std::string &serial) : impl(std::make_unique<Impl>()) {
  libusb_device **devices = nullptr;

  try {
    impl->context = init_usb_context();
    if (impl->context == nullptr) {
      throw std::runtime_error("Error connecting to Koala: libusb initialization failed");
    }

    const ssize_t count = libusb_get_device_list(impl->context, &devices);
    if (count < 0) {
      throw std::runtime_error("Error connecting to Koala: USB enumeration failed");
    }

    for (ssize_t i = 0; i < count; ++i) {
      libusb_device_descriptor descriptor = {};
      if (libusb_get_device_descriptor(devices[i], &descriptor) != LIBUSB_SUCCESS || !is_panda_usb_device(descriptor)) {
        continue;
      }

      libusb_device_handle *candidate = nullptr;
      if (libusb_open(devices[i], &candidate) != LIBUSB_SUCCESS || candidate == nullptr) {
        continue;
      }

      const std::string candidate_serial = read_serial(candidate, descriptor.iSerialNumber);
      if ((serial.empty() || serial == candidate_serial) && has_koala_identity(candidate)) {
        impl->device = candidate;
        hw_serial = candidate_serial;
        break;
      }
      libusb_close(candidate);
    }
    libusb_free_device_list(devices, 1);
    devices = nullptr;

    if (impl->device == nullptr) {
      throw std::runtime_error("Error connecting to Koala: matching USB device not found");
    }

    if (libusb_kernel_driver_active(impl->device, PANDA_USB_INTERFACE) == 1) {
      const int err = libusb_detach_kernel_driver(impl->device, PANDA_USB_INTERFACE);
      if (err != LIBUSB_SUCCESS && err != LIBUSB_ERROR_NOT_SUPPORTED) {
        throw std::runtime_error("Error connecting to Koala: failed to detach kernel USB driver");
      }
    }

    int configuration = 0;
    int err = libusb_get_configuration(impl->device, &configuration);
    if (err != LIBUSB_SUCCESS) {
      throw std::runtime_error("Error connecting to Koala: failed to read USB configuration");
    }
    if (configuration != 1) {
      err = libusb_set_configuration(impl->device, 1);
      if (err != LIBUSB_SUCCESS) {
        throw std::runtime_error("Error connecting to Koala: failed to set USB configuration");
      }
    }

    err = libusb_claim_interface(impl->device, PANDA_USB_INTERFACE);
    if (err != LIBUSB_SUCCESS) {
      throw std::runtime_error("Error connecting to Koala: failed to claim USB interface");
    }
    impl->interface_claimed = true;
  } catch (...) {
    if (devices != nullptr) libusb_free_device_list(devices, 1);
    cleanup();
    throw;
  }
}

KoalaUsbHandle::~KoalaUsbHandle() {
  cleanup();
}

void KoalaUsbHandle::cleanup() {
  if (!impl) return;
  if (impl->device != nullptr) {
    if (impl->interface_claimed) libusb_release_interface(impl->device, PANDA_USB_INTERFACE);
    libusb_close(impl->device);
    impl->device = nullptr;
  }
  if (impl->context != nullptr) {
    libusb_exit(impl->context);
    impl->context = nullptr;
  }
  connected = false;
}

void KoalaUsbHandle::handle_usb_issue(int err, const char *operation) {
  LOGE_100("Koala USB error %d (%s) in %s", err, libusb_error_name(err), operation);
  if (err == LIBUSB_ERROR_NO_DEVICE) {
    connected = false;
  } else if (err != LIBUSB_ERROR_TIMEOUT) {
    comms_healthy = false;
  }
}

int KoalaUsbHandle::control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout) {
  if (!connected || impl->device == nullptr) return LIBUSB_ERROR_NO_DEVICE;

  constexpr uint8_t request_type = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
  const int result = libusb_control_transfer(impl->device, request_type, request, param1, param2, nullptr, 0, timeout);
  if (result < 0) handle_usb_issue(result, __func__);
  return result;
}

int KoalaUsbHandle::control_read(uint8_t request, uint16_t param1, uint16_t param2,
                                 unsigned char *data, uint16_t length, unsigned int timeout) {
  if (!connected || impl->device == nullptr) return LIBUSB_ERROR_NO_DEVICE;

  constexpr uint8_t request_type = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
  const int result = libusb_control_transfer(impl->device, request_type, request, param1, param2, data, length, timeout);
  if (result < 0) handle_usb_issue(result, __func__);
  return result;
}

int KoalaUsbHandle::bulk_write(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  if (!connected || impl->device == nullptr) return LIBUSB_ERROR_NO_DEVICE;

  int transferred = 0;
  const int result = libusb_bulk_transfer(impl->device, endpoint, data, length, &transferred, timeout);
  if (result == LIBUSB_ERROR_TIMEOUT) {
    LOGW("Koala USB transmit buffer full");
  } else if (result != LIBUSB_SUCCESS) {
    handle_usb_issue(result, __func__);
  } else if (transferred != length) {
    comms_healthy = false;
    LOGE("Koala USB short write: %d of %d bytes", transferred, length);
  }
  return transferred;
}

int KoalaUsbHandle::bulk_read(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  if (!connected || impl->device == nullptr) return LIBUSB_ERROR_NO_DEVICE;

  int transferred = 0;
  // libusb treats a zero timeout as infinite. pandad polls CAN before publishing
  // pandaStates, so an idle Koala must return regularly instead of blocking the
  // entire device loop when no CAN frames are available.
  const unsigned int read_timeout = timeout == 0 ? KOALA_USB_BULK_READ_TIMEOUT_MS : timeout;
  const int result = libusb_bulk_transfer(impl->device, endpoint, data, length, &transferred, read_timeout);
  if (result == LIBUSB_ERROR_OVERFLOW) {
    comms_healthy = false;
    LOGE_100("Koala USB receive overflow");
  } else if (result != LIBUSB_SUCCESS && result != LIBUSB_ERROR_TIMEOUT) {
    handle_usb_issue(result, __func__);
  }
  return transferred;
}

std::vector<std::string> KoalaUsbHandle::list() {
  std::vector<std::string> serials;
  libusb_context *context = init_usb_context();
  if (context == nullptr) return serials;

  libusb_device **devices = nullptr;
  const ssize_t count = libusb_get_device_list(context, &devices);
  if (count >= 0) {
    for (ssize_t i = 0; i < count; ++i) {
      libusb_device_descriptor descriptor = {};
      if (libusb_get_device_descriptor(devices[i], &descriptor) != LIBUSB_SUCCESS || !is_panda_usb_device(descriptor)) {
        continue;
      }

      libusb_device_handle *device = nullptr;
      if (libusb_open(devices[i], &device) == LIBUSB_SUCCESS && device != nullptr) {
        if (has_koala_identity(device)) {
          serials.push_back(read_serial(device, descriptor.iSerialNumber));
        }
        libusb_close(device);
      }
    }
  }

  if (devices != nullptr) libusb_free_device_list(devices, 1);
  libusb_exit(context);
  return serials;
}
