/**
 * @file src/common/json.cpp
 * @brief Definitions for JSON conversion functions.
 */
// header include
#include "display_device/json.h"

// special ordered include of details
#define DD_JSON_DETAIL
// clang-format off
#include "display_device/detail/json_serializer.h"
#include "display_device/detail/json_converter.h"
// clang-format on

namespace display_device {
  DD_JSON_DEFINE_CONVERTER(EdidData)
  DD_JSON_DEFINE_CONVERTER(EnumeratedDevice)
  DD_JSON_DEFINE_CONVERTER(EnumeratedDeviceList)
  DD_JSON_DEFINE_CONVERTER(SingleDisplayConfiguration)
  DD_JSON_DEFINE_CONVERTER(std::set<std::string>)
  DD_JSON_DEFINE_CONVERTER(std::string)
  DD_JSON_DEFINE_CONVERTER(bool)
}  // namespace display_device
