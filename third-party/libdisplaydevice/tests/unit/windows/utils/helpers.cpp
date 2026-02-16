// header include
#include "helpers.h"

// local includes
#include "display_device/windows/json.h"

std::optional<std::vector<std::string>> getAvailableDevices(display_device::WinApiLayer &layer, const bool only_valid_output) {
  const auto all_devices {layer.queryDisplayConfig(display_device::QueryType::All)};
  if (!all_devices) {
    return std::nullopt;
  }

  std::set<std::string> device_ids;
  for (const auto &path : all_devices->m_paths) {
    if (only_valid_output && path.targetInfo.outputTechnology == DISPLAYCONFIG_OUTPUT_TECHNOLOGY_OTHER) {
      continue;
    }

    const auto device_id {layer.getDeviceId(path)};
    if (!device_id.empty()) {
      device_ids.insert(device_id);
    }
  }

  return std::vector<std::string> {device_ids.begin(), device_ids.end()};
}

std::optional<std::vector<std::uint8_t>> serializeState(const std::optional<display_device::SingleDisplayConfigState> &state) {
  if (state) {
    if (state->m_initial.m_topology.empty() && state->m_initial.m_primary_devices.empty() && state->m_modified.m_topology.empty() && !state->m_modified.hasModifications()) {
      return std::vector<std::uint8_t> {};
    }

    bool is_ok {false};
    const auto data_string {toJson(*state, 2, &is_ok)};
    if (is_ok) {
      return std::vector<std::uint8_t> {std::begin(data_string), std::end(data_string)};
    }
  }

  return std::nullopt;
}
