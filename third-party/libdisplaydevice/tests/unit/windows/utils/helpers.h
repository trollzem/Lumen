#pragma once

// system includes
#include <cstdint>

// local includes
#include "display_device/windows/types.h"
#include "display_device/windows/win_api_layer.h"

// Generic helper functions
std::optional<std::vector<std::string>> getAvailableDevices(display_device::WinApiLayer &layer, bool only_valid_output = true);

std::optional<std::vector<std::uint8_t>> serializeState(const std::optional<display_device::SingleDisplayConfigState> &state);
