/**
 * @file src/windows/json_serializer.cpp
 * @brief Definitions for private JSON serialization helpers (Windows-only).
 */
// special ordered include of details
#define DD_JSON_DETAIL
// clang-format off
#include "display_device/windows/types.h"
#include "display_device/windows/detail/json_serializer.h"
// clang-format on

namespace display_device {
  // Structs
  DD_JSON_DEFINE_SERIALIZE_STRUCT(DisplayMode, resolution, refresh_rate)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(SingleDisplayConfigState::Initial, topology, primary_devices)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(SingleDisplayConfigState::Modified, topology, original_modes, original_hdr_states, original_primary_device)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(SingleDisplayConfigState, initial, modified)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(WinWorkarounds, hdr_blank_delay)
}  // namespace display_device
