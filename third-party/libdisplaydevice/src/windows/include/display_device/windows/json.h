/**
 * @file src/windows/include/display_device/windows/json.h
 * @brief Declarations for JSON conversion functions (Windows-only).
 */
#pragma once

// local includes
#include "display_device/json.h"
#include "types.h"

// Windows' converters (add as needed)
namespace display_device {
  DD_JSON_DECLARE_CONVERTER(ActiveTopology)
  DD_JSON_DECLARE_CONVERTER(DeviceDisplayModeMap)
  DD_JSON_DECLARE_CONVERTER(HdrStateMap)
  DD_JSON_DECLARE_CONVERTER(SingleDisplayConfigState)
  DD_JSON_DECLARE_CONVERTER(WinWorkarounds)
}  // namespace display_device
