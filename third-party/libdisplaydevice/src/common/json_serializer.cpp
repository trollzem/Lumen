/**
 * @file src/common/json_serializer.cpp
 * @brief Definitions for private JSON serialization helpers.
 */
// special ordered include of details
#define DD_JSON_DETAIL
// clang-format off
#include "display_device/types.h"
#include "display_device/detail/json_serializer.h"
// clang-format on

namespace display_device {
  // Enums
  DD_JSON_DEFINE_SERIALIZE_ENUM_GCOVR_EXCL_BR_LINE(HdrState, {{HdrState::Disabled, "Disabled"}, {HdrState::Enabled, "Enabled"}})
  DD_JSON_DEFINE_SERIALIZE_ENUM_GCOVR_EXCL_BR_LINE(SingleDisplayConfiguration::DevicePreparation, {{SingleDisplayConfiguration::DevicePreparation::VerifyOnly, "VerifyOnly"}, {SingleDisplayConfiguration::DevicePreparation::EnsureActive, "EnsureActive"}, {SingleDisplayConfiguration::DevicePreparation::EnsurePrimary, "EnsurePrimary"}, {SingleDisplayConfiguration::DevicePreparation::EnsureOnlyDisplay, "EnsureOnlyDisplay"}})

  // Structs
  DD_JSON_DEFINE_SERIALIZE_STRUCT(Resolution, width, height)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(Rational, numerator, denominator)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(Point, x, y)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(EdidData, manufacturer_id, product_code, serial_number)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(EnumeratedDevice::Info, resolution, resolution_scale, refresh_rate, primary, origin_point, hdr_state)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(EnumeratedDevice, device_id, display_name, friendly_name, edid, info)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(SingleDisplayConfiguration, device_id, device_prep, resolution, refresh_rate, hdr_state)
}  // namespace display_device
