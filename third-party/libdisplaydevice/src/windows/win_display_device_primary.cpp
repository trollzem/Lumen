/**
 * @file src/windows/win_display_device_primary.cpp
 * @brief Definitions for the "primary device" related methods in WinDisplayDevice.
 */
// class header include
#include "display_device/windows/win_display_device.h"

// local includes
#include "display_device/logging.h"
#include "display_device/windows/win_api_utils.h"

namespace display_device {
  bool WinDisplayDevice::isPrimary(const std::string &device_id) const {
    if (device_id.empty()) {
      DD_LOG(error) << "Device id is empty!";
      return false;
    }

    const auto display_data {m_w_api->queryDisplayConfig(QueryType::Active)};
    if (!display_data) {
      // Error already logged
      return false;
    }

    const auto path {win_utils::getActivePath(*m_w_api, device_id, display_data->m_paths)};
    if (!path) {
      DD_LOG(error) << "Failed to find active device for " << device_id << "!";
      return false;
    }

    const auto source_mode {win_utils::getSourceMode(win_utils::getSourceIndex(*path, display_data->m_modes), display_data->m_modes)};
    if (!source_mode) {
      DD_LOG(error) << "Active device does not have a source mode: " << device_id << "!";
      return false;
    }

    return win_utils::isPrimary(*source_mode);
  }

  bool WinDisplayDevice::setAsPrimary(const std::string &device_id) {
    if (device_id.empty()) {
      DD_LOG(error) << "Device id is empty!";
      return false;
    }

    auto display_data {m_w_api->queryDisplayConfig(QueryType::Active)};
    if (!display_data) {
      // Error already logged
      return false;
    }

    // Get the current origin point of the device (the one that we want to make primary)
    POINTL origin;
    {
      const auto path {win_utils::getActivePath(*m_w_api, device_id, display_data->m_paths)};
      if (!path) {
        DD_LOG(error) << "Failed to find device for " << device_id << "!";
        return false;
      }

      const auto source_mode {win_utils::getSourceMode(win_utils::getSourceIndex(*path, display_data->m_modes), display_data->m_modes)};
      if (!source_mode) {
        DD_LOG(error) << "Active device does not have a source mode: " << device_id << "!";
        return false;
      }

      if (win_utils::isPrimary(*source_mode)) {
        DD_LOG(debug) << "Device " << device_id << " is already a primary device.";
        return true;
      }

      origin = source_mode->position;
    }

    // Shift the source mode origin points accordingly, so that the provided
    // device moves to (0, 0) position and others to their new positions.
    std::set<UINT32> modified_modes;
    for (auto &path : display_data->m_paths) {
      const auto current_id {m_w_api->getDeviceId(path)};
      const auto source_index {win_utils::getSourceIndex(path, display_data->m_modes)};
      auto source_mode {win_utils::getSourceMode(source_index, display_data->m_modes)};

      if (!source_index || !source_mode) {
        DD_LOG(error) << "Active device does not have a source mode: " << current_id << "!";
        return false;
      }

      if (modified_modes.find(*source_index) != std::end(modified_modes)) {
        // Happens when VIRTUAL_MODE_AWARE is not specified when querying paths, probably will never happen in our (since it's always set), but just to be safe...
        DD_LOG(debug) << "Device " << current_id << " shares the same mode index as a previous device. Device is duplicated. Skipping.";
        continue;
      }

      source_mode->position.x -= origin.x;
      source_mode->position.y -= origin.y;

      modified_modes.insert(*source_index);
    }

    const UINT32 flags {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE};
    const LONG result {m_w_api->setDisplayConfig(display_data->m_paths, display_data->m_modes, flags)};
    if (result != ERROR_SUCCESS) {
      DD_LOG(error) << m_w_api->getErrorString(result) << " failed to set primary mode for " << device_id << "!";
      return false;
    }

    return true;
  }
}  // namespace display_device
