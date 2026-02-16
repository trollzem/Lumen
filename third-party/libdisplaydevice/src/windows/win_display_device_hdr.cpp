/**
 * @file src/windows/win_display_device_hdr.cpp
 * @brief Definitions for the HDR related methods in WinDisplayDevice.
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
    /** @brief HDR state map without optional values. */
    using HdrStateMapNoOpt = std::map<std::string, HdrState>;

    /**
     * @see setHdrStates for a description as this was split off to reduce cognitive complexity.
     */
    bool doSetHdrStates(WinApiLayerInterface &w_api, const PathAndModeData &display_data, const HdrStateMapNoOpt &states, HdrStateMapNoOpt *changed_states) {
      const auto try_set_state {
        [&w_api, &display_data](const auto &device_id, const auto &state, auto &current_state) {
          const auto path {win_utils::getActivePath(w_api, device_id, display_data.m_paths)};
          if (!path) {
            DD_LOG(error) << "Failed to find device for " << device_id << "!";
            return false;
          }

          const auto current_state_int {w_api.getHdrState(*path)};
          if (!current_state_int) {
            DD_LOG(error) << "HDR state cannot be changed for " << device_id << "!";
            return false;
          }

          if (state != *current_state_int) {
            if (!w_api.setHdrState(*path, state)) {
              // Error already logged
              return false;
            }

            current_state = current_state_int;
          }

          return true;
        }
      };

      for (const auto &[device_id, state] : states) {
        std::optional<HdrState> current_state;
        if (try_set_state(device_id, state, current_state)) {
          if (current_state && changed_states != nullptr) {
            (*changed_states)[device_id] = *current_state;
          }
        }
        // If we are undoing changes we don't want to return early and continue regardless of what error we get.
        else if (changed_states != nullptr) {
          return false;
        }
      }

      return true;
    }

  }  // namespace

  HdrStateMap WinDisplayDevice::getCurrentHdrStates(const std::set<std::string> &device_ids) const {
    if (device_ids.empty()) {
      DD_LOG(error) << "Device id set is empty!";
      return {};
    }

    const auto display_data {m_w_api->queryDisplayConfig(QueryType::Active)};
    if (!display_data) {
      // Error already logged
      return {};
    }

    HdrStateMap states;
    for (const auto &device_id : device_ids) {
      const auto path {win_utils::getActivePath(*m_w_api, device_id, display_data->m_paths)};
      if (!path) {
        DD_LOG(error) << "Failed to find device for " << device_id << "!";
        return {};
      }

      states[device_id] = m_w_api->getHdrState(*path);
    }

    return states;
  }

  bool WinDisplayDevice::setHdrStates(const HdrStateMap &states) {
    if (states.empty()) {
      DD_LOG(error) << "States map is empty!";
      return false;
    }

    HdrStateMapNoOpt states_without_opt;
    std::ranges::copy(states | std::ranges::views::filter([](const auto &entry) {
                        return static_cast<bool>(entry.second);
                      }) |
                        std::views::transform([](const auto &entry) {
                          return std::make_pair(entry.first, *entry.second);
                        }),
                      std::inserter(states_without_opt, std::begin(states_without_opt)));

    if (states_without_opt.empty()) {
      // Return early as there is nothing to do...
      return true;
    }

    const auto display_data {m_w_api->queryDisplayConfig(QueryType::Active)};
    if (!display_data) {
      // Error already logged
      return {};
    }

    HdrStateMapNoOpt changed_states;
    if (!doSetHdrStates(*m_w_api, *display_data, states_without_opt, &changed_states)) {
      if (!changed_states.empty()) {
        doSetHdrStates(*m_w_api, *display_data, changed_states, nullptr);  // return value does not matter
      }
      return false;
    }

    return true;
  }
}  // namespace display_device
