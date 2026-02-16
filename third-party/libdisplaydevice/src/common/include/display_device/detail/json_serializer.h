/**
 * @file src/common/include/display_device/detail/json_serializer.h
 * @brief Declarations for private JSON serialization helpers.
 */
#pragma once

// local includes
#include "json_serializer_details.h"

#ifdef DD_JSON_DETAIL
namespace display_device {
  // Enums
  DD_JSON_DECLARE_SERIALIZE_TYPE(HdrState)
  DD_JSON_DECLARE_SERIALIZE_TYPE(SingleDisplayConfiguration::DevicePreparation)

  // Structs
  DD_JSON_DECLARE_SERIALIZE_TYPE(Resolution)
  DD_JSON_DECLARE_SERIALIZE_TYPE(Rational)
  DD_JSON_DECLARE_SERIALIZE_TYPE(Point)
  DD_JSON_DECLARE_SERIALIZE_TYPE(EdidData)
  DD_JSON_DECLARE_SERIALIZE_TYPE(EnumeratedDevice::Info)
  DD_JSON_DECLARE_SERIALIZE_TYPE(EnumeratedDevice)
  DD_JSON_DECLARE_SERIALIZE_TYPE(SingleDisplayConfiguration)
}  // namespace display_device
#endif
