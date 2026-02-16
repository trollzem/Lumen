/**
 * @file src/windows/json.cpp
 * @brief Definitions for JSON conversion functions (Windows-only).
 */
// header include
#include "display_device/windows/json.h"

// special ordered include of details
#define DD_JSON_DETAIL
// clang-format off
#include "display_device/windows/detail/json_serializer.h"
#include "display_device/detail/json_converter.h"
// clang-format on

namespace display_device {
  const std::optional<unsigned int> JSON_COMPACT {std::nullopt};

  DD_JSON_DEFINE_CONVERTER(ActiveTopology)
  DD_JSON_DEFINE_CONVERTER(DeviceDisplayModeMap)
  DD_JSON_DEFINE_CONVERTER(HdrStateMap)
  DD_JSON_DEFINE_CONVERTER(SingleDisplayConfigState)
  DD_JSON_DEFINE_CONVERTER(WinWorkarounds)
}  // namespace display_device
