/**
 * @file src/windows/win_display_device_modes.cpp
 * @brief Definitions for the display mode related methods in WinDisplayDevice.
 */
// class header include
#include "display_device/windows/win_display_device.h"

// system includes
#include <ranges>

// local includes
#include "display_device/logging.h"
#include "display_device/windows/win_api_utils.h"

namespace display_device {
  namespace {

    /**
     * @brief Strategy to be used when changing display modes.
     */
    enum class Strategy {
      Relaxed,
      Strict
    };

    /**
     * @see set_display_modes for a description as this was split off to reduce cognitive complexity.
     */
    bool doSetModes(WinApiLayerInterface &w_api, const DeviceDisplayModeMap &modes, const Strategy strategy) {
      auto display_data {w_api.queryDisplayConfig(QueryType::Active)};
      if (!display_data) {
        // Error already logged
        return false;
      }

      bool changes_applied {false};
      for (const auto &[device_id, mode] : modes) {
        const auto path {win_utils::getActivePath(w_api, device_id, display_data->m_paths)};
        if (!path) {
          DD_LOG(error) << "Failed to find device for " << device_id << "!";
          return false;
        }

        const auto source_mode {win_utils::getSourceMode(win_utils::getSourceIndex(*path, display_data->m_modes), display_data->m_modes)};
        if (!source_mode) {
          DD_LOG(error) << "Active device does not have a source mode: " << device_id << "!";
          return false;
        }

        bool new_changes {false};
        const bool resolution_changed {source_mode->width != mode.m_resolution.m_width || source_mode->height != mode.m_resolution.m_height};

        bool refresh_rate_changed;
        if (strategy == Strategy::Relaxed) {
          refresh_rate_changed = !win_utils::fuzzyCompareRefreshRates(Rational {path->targetInfo.refreshRate.Numerator, path->targetInfo.refreshRate.Denominator}, mode.m_refresh_rate);
        } else {
          // Since we are in strict mode, do not fuzzy compare it
          refresh_rate_changed = path->targetInfo.refreshRate.Numerator != mode.m_refresh_rate.m_numerator ||
                                 path->targetInfo.refreshRate.Denominator != mode.m_refresh_rate.m_denominator;
        }

        if (resolution_changed) {
          source_mode->width = mode.m_resolution.m_width;
          source_mode->height = mode.m_resolution.m_height;
          new_changes = true;
        }

        if (refresh_rate_changed) {
          path->targetInfo.refreshRate = {mode.m_refresh_rate.m_numerator, mode.m_refresh_rate.m_denominator};
          new_changes = true;
        }

        if (new_changes) {
          // Clear the target index so that Windows has to select/modify the target to best match the requirements.
          win_utils::setTargetIndex(*path, std::nullopt);
          win_utils::setDesktopIndex(*path, std::nullopt);  // Part of struct containing target index and so it needs to be cleared
        }

        changes_applied = changes_applied || new_changes;
      }

      if (!changes_applied) {
        DD_LOG(debug) << "No changes were made to display modes as they are equal.";
        return true;
      }

      UINT32 flags {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE};
      if (strategy == Strategy::Relaxed) {
        // It's probably best for Windows to select the "best" display settings for us. However, in case we
        // have custom resolution set in nvidia control panel for example, this flag will prevent successfully applying
        // settings to it.
        flags |= SDC_ALLOW_CHANGES;
      }

      const LONG result {w_api.setDisplayConfig(display_data->m_paths, display_data->m_modes, flags)};
      if (result != ERROR_SUCCESS) {
        DD_LOG(error) << w_api.getErrorString(result) << " failed to set display mode!";
        return false;
      }

      return true;
    }
  }  // namespace

  DeviceDisplayModeMap WinDisplayDevice::getCurrentDisplayModes(const std::set<std::string> &device_ids) const {
    if (device_ids.empty()) {
      DD_LOG(error) << "Device id set is empty!";
      return {};
    }

    const auto display_data {m_w_api->queryDisplayConfig(QueryType::Active)};
    if (!display_data) {
      // Error already logged
      return {};
    }

    DeviceDisplayModeMap current_modes;
    for (const auto &device_id : device_ids) {
      if (device_id.empty()) {
        DD_LOG(error) << "Device id is empty!";
        return {};
      }

      const auto path {win_utils::getActivePath(*m_w_api, device_id, display_data->m_paths)};
      if (!path) {
        DD_LOG(error) << "Failed to find device for " << device_id << "!";
        return {};
      }

      const auto source_mode {win_utils::getSourceMode(win_utils::getSourceIndex(*path, display_data->m_modes), display_data->m_modes)};
      if (!source_mode) {
        DD_LOG(error) << "Active device does not have a source mode: " << device_id << "!";
        return {};
      }

      // For whatever reason they put refresh rate into path, but not the resolution.
      const auto target_refresh_rate {path->targetInfo.refreshRate};
      current_modes[device_id] = DisplayMode {
        {source_mode->width, source_mode->height},
        {target_refresh_rate.Numerator, target_refresh_rate.Denominator}
      };
    }

    return current_modes;
  }

  bool WinDisplayDevice::setDisplayModes(const DeviceDisplayModeMap &modes) {
    if (modes.empty()) {
      DD_LOG(error) << "Modes map is empty!";
      return false;
    }

    // Here it is important to check that we have all the necessary modes, otherwise
    // setting modes will fail with ambiguous message.
    //
    // Duplicated devices can have different target modes (monitor) with different refresh rate,
    // however this does not apply to the source mode (frame buffer?) and they must have same
    // resolution.
    //
    // Without SDC_VIRTUAL_MODE_AWARE, devices would share the same source mode entry, but now
    // they have separate entries that are more or less identical.
    //
    // To avoid surprising end-user with unexpected source mode change, we validate that all duplicate
    // devices were provided instead of guessing modes automatically. This also resolve the problem of
    // having to choose refresh rate for duplicate display - leave it to the end-user of this function...
    const auto keys_view {std::ranges::views::keys(modes)};
    const std::set<std::string> device_ids {std::begin(keys_view), std::end(keys_view)};
    const auto all_device_ids {win_utils::getAllDeviceIdsAndMatchingDuplicates(*m_w_api, device_ids)};
    if (all_device_ids.empty()) {
      DD_LOG(error) << "Failed to get all duplicated devices!";
      return false;
    }

    if (all_device_ids.size() != device_ids.size()) {
      DD_LOG(error) << "Not all modes for duplicate displays were provided!";
      return false;
    }

    const auto &original_data {m_w_api->queryDisplayConfig(QueryType::All)};
    if (!original_data) {
      // Error already logged
      return false;
    }

    if (!doSetModes(*m_w_api, modes, Strategy::Relaxed)) {
      // Error already logged
      return false;
    }

    const auto all_modes_match = [&modes](const DeviceDisplayModeMap &current_modes) {
      for (const auto &[device_id, requested_mode] : modes) {
        auto mode_it {current_modes.find(device_id)};
        if (mode_it == std::end(current_modes)) {
          // This is a sanity check as `getCurrentDisplayModes` implicitly verifies this already.
          return false;
        }

        if (!win_utils::fuzzyCompareModes(mode_it->second, requested_mode)) {
          return false;
        }
      }

      return true;
    };

    auto current_modes {getCurrentDisplayModes(device_ids)};
    if (!current_modes.empty()) {
      if (all_modes_match(current_modes)) {
        return true;
      }

      // We have a problem when using SetDisplayConfig with SDC_ALLOW_CHANGES
      // where it decides to use our new mode merely as a suggestion.
      //
      // This is good, since we don't have to be very precise with refresh rate,
      // but also bad since it can just ignore our specified mode.
      //
      // However, it is possible that the user has created a custom display mode
      // which is not exposed to the via Windows settings app. To allow this
      // resolution to be selected, we actually need to omit SDC_ALLOW_CHANGES
      // flag.
      DD_LOG(info) << "Failed to change display modes using Windows recommended modes, trying to set modes more strictly!";
      if (doSetModes(*m_w_api, modes, Strategy::Strict)) {
        current_modes = getCurrentDisplayModes(device_ids);
        if (!current_modes.empty() && all_modes_match(current_modes)) {
          return true;
        }
      }
    }

    const UINT32 flags {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE};
    static_cast<void>(m_w_api->setDisplayConfig(original_data->m_paths, original_data->m_modes, flags));  // Return value does not matter as we are trying out best to undo
    DD_LOG(error) << "Failed to set display mode(-s) completely!";
    return false;
  }
}  // namespace display_device
