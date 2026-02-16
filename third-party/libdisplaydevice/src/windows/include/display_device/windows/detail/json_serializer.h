/**
 * @file src/windows/include/display_device/windows/detail/json_serializer.h
 * @brief Declarations for private JSON serialization helpers (Windows-only).
 */
#pragma once

// local includes
#include "display_device/detail/json_serializer.h"

#ifdef DD_JSON_DETAIL
namespace display_device {
  // Structs
  DD_JSON_DECLARE_SERIALIZE_TYPE(DisplayMode)
  DD_JSON_DECLARE_SERIALIZE_TYPE(SingleDisplayConfigState::Initial)
  DD_JSON_DECLARE_SERIALIZE_TYPE(SingleDisplayConfigState::Modified)
  DD_JSON_DECLARE_SERIALIZE_TYPE(SingleDisplayConfigState)
  DD_JSON_DECLARE_SERIALIZE_TYPE(WinWorkarounds)
}  // namespace display_device
#endif
