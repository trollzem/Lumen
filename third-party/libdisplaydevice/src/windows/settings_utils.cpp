/**
 * @file src/windows/settings_utils.cpp
 * @brief Definitions for settings related utility functions.
 */
// header include
#include "display_device/windows/settings_utils.h"

// system includes
#include <algorithm>
#include <cmath>
#include <thread>

// local includes
#include "display_device/logging.h"
#include "display_device/windows/json.h"

namespace display_device::win_utils {
  namespace {
    /**
     * @brief predicate for getDeviceIds.
     */
    bool anyDevice(const EnumeratedDevice &) {
      return true;
    }

    /**
     * @brief predicate for getDeviceIds.
     */
    bool primaryOnlyDevices(const EnumeratedDevice &device) {
      return device.m_info && device.m_info->m_primary;
    }

    /**
     * @brief Get device ids from device list matching the predicate.
     * @param devices List of devices.
     * @param predicate Predicate to use.
     * @return Device ids from device list matching the predicate.
     * @examples
     * const EnumeratedDeviceList devices { ... };
     *
     * const auto all_ids { getDeviceIds(devices, anyDevice) };
     * const auto primary_only_ids { getDeviceIds(devices, primaryOnlyDevices) };
     * @examples_end
     */
    std::set<std::string> getDeviceIds(const EnumeratedDeviceList &devices, std::add_lvalue_reference_t<bool(const EnumeratedDevice &)> &predicate) {
      std::set<std::string> device_ids;
      for (const auto &device : devices) {
        if (predicate(device)) {
          device_ids.insert(device.m_device_id);
        }
      }

      return device_ids;
    }

    /**
     * @brief Remove the topology device ids and groups that no longer have valid devices.
     * @param topology Topology to be stripped.
     * @param devices List of devices.
     * @return Topology without missing device ids.
     */
    ActiveTopology stripTopology(const ActiveTopology &topology, const EnumeratedDeviceList &devices) {
      const std::set<std::string> available_device_ids {getDeviceIds(devices, anyDevice)};

      ActiveTopology stripped_topology;
      for (const auto &group : topology) {
        std::vector<std::string> stripped_group;
        for (const auto &device_id : group) {
          if (available_device_ids.contains(device_id)) {
            stripped_group.push_back(device_id);
          }
        }

        if (!stripped_group.empty()) {
          stripped_topology.push_back(stripped_group);
        }
      }

      return stripped_topology;
    }

    /**
     * @brief Remove device ids that are no longer available.
     * @param device_ids Id list to be stripped.
     * @param devices List of devices.
     * @return List without missing device ids.
     */
    std::set<std::string> stripDevices(const std::set<std::string> &device_ids, const EnumeratedDeviceList &devices) {
      std::set<std::string> available_device_ids {getDeviceIds(devices, anyDevice)};

      std::set<std::string> available_devices;
      std::ranges::set_intersection(device_ids, available_device_ids, std::inserter(available_devices, std::begin(available_devices)));
      return available_devices;
    }

    /**
     * @brief Find topology group with matching id and get other ids from the group.
     * @param topology Topology to be searched.
     * @param target_device_id Device id whose group to search for.
     * @return Other ids in the group without (excluding the provided one).
     */
    std::set<std::string> tryGetOtherDevicesInTheSameGroup(const ActiveTopology &topology, const std::string &target_device_id) {
      std::set<std::string> device_ids;

      for (const auto &group : topology) {
        for (const auto &group_device_id : group) {
          if (group_device_id == target_device_id) {
            std::ranges::copy_if(group, std::inserter(device_ids, std::begin(device_ids)), [&target_device_id](const auto &id) {
              return id != target_device_id;
            });
            break;
          }
        }
      }

      return device_ids;
    }

    /**
     * @brief Merge the configurable devices into a vector.
     */
    std::vector<std::string> joinConfigurableDevices(const std::string &device_to_configure, const std::set<std::string> &additional_devices_to_configure) {
      std::vector<std::string> devices {device_to_configure};
      devices.insert(std::end(devices), std::begin(additional_devices_to_configure), std::end(additional_devices_to_configure));
      return devices;
    }
  }  // namespace

  std::set<std::string> flattenTopology(const ActiveTopology &topology) {
    std::set<std::string> flattened_topology;
    for (const auto &group : topology) {
      for (const auto &device_id : group) {
        flattened_topology.insert(device_id);
      }
    }

    return flattened_topology;
  }

  ActiveTopology createFullExtendedTopology(WinDisplayDeviceInterface &win_dd) {
    const auto devices {win_dd.enumAvailableDevices()};
    if (devices.empty()) {
      DD_LOG(error) << "Failed to enumerate available devices for full extended topology!";
      return {};
    }

    ActiveTopology topology;
    for (const auto &device : devices) {
      topology.push_back({device.m_device_id});
    }

    return topology;
  }

  std::string getPrimaryDevice(WinDisplayDeviceInterface &win_dd, const ActiveTopology &topology) {
    const auto flat_topology {flattenTopology(topology)};
    for (const auto &device_id : flat_topology) {
      if (win_dd.isPrimary(device_id)) {
        return device_id;
      }
    }

    return {};
  }

  std::optional<SingleDisplayConfigState::Initial> computeInitialState(const std::optional<SingleDisplayConfigState::Initial> &prev_state, const ActiveTopology &topology_before_changes, const EnumeratedDeviceList &devices) {
    // We first need to determine the "initial" state that will be used when reverting
    // the changes as the "go-to" final state we need to achieve. It will also be used
    // as the base for creating new topology based on the provided config settings.
    //
    // Having a constant base allows us to re-apply settings with different configuration
    // parameters without actually reverting the topology back to the initial one where
    // the primary display could have changed in between the first call to this method
    // and a next one.
    //
    // If the user wants to use a "fresh" and "current" system settings, they have to revert
    // changes as otherwise we are using the cached state as a base.
    if (prev_state) {
      return *prev_state;
    }

    const auto primary_devices {getDeviceIds(devices, primaryOnlyDevices)};
    if (primary_devices.empty()) {
      DD_LOG(error) << "Enumerated device list does not contain primary devices!";
      return std::nullopt;
    }

    return SingleDisplayConfigState::Initial {
      topology_before_changes,
      primary_devices
    };
  }

  ActiveTopology computeNewTopology(const SingleDisplayConfiguration::DevicePreparation device_prep, const bool configuring_primary_devices, const std::string &device_to_configure, const std::set<std::string> &additional_devices_to_configure, const ActiveTopology &initial_topology) {
    using DevicePrep = SingleDisplayConfiguration::DevicePreparation;

    if (device_prep != DevicePrep::VerifyOnly) {
      if (device_prep == DevicePrep::EnsureOnlyDisplay) {
        // Device needs to be the only one that's active OR if it's a PRIMARY device,
        // only the whole PRIMARY group needs to be active (in case they are duplicated)
        if (configuring_primary_devices) {
          return ActiveTopology {joinConfigurableDevices(device_to_configure, additional_devices_to_configure)};
        }

        return ActiveTopology {{device_to_configure}};
      }

      //  The device needs to be active at least for `DevicePrep::EnsureActive || DevicePrep::EnsurePrimary`.
      if (!flattenTopology(initial_topology).contains(device_to_configure)) {
        // Create an extended topology as it's probably what makes sense the most...
        ActiveTopology new_topology {initial_topology};
        new_topology.push_back({device_to_configure});
        return new_topology;
      }
    }

    return initial_topology;
  }

  std::optional<SingleDisplayConfigState::Initial> stripInitialState(const SingleDisplayConfigState::Initial &initial_state, const EnumeratedDeviceList &devices) {
    const auto stripped_initial_topology {stripTopology(initial_state.m_topology, devices)};
    auto initial_primary_devices {stripDevices(initial_state.m_primary_devices, devices)};

    if (stripped_initial_topology.empty()) {
      DD_LOG(error) << "Enumerated device list does not contain ANY of the devices from the initial state!";
      return std::nullopt;
    }

    if (initial_primary_devices.empty()) {
      // The initial primay device is no longer available, so maybe it makes sense to use the current one. Maybe...
      initial_primary_devices = getDeviceIds(devices, primaryOnlyDevices);
      if (initial_primary_devices.empty()) {
        DD_LOG(error) << "Enumerated device list does not contain primary devices!";
        return std::nullopt;
      }
    }

    if (initial_state.m_topology != stripped_initial_topology || initial_state.m_primary_devices != initial_primary_devices) {
      DD_LOG(warning) << "Trying to apply configuration without reverting back to initial topology first, however not all devices from that "
                         "topology are available.\n"
                      << "Will try adapting the initial topology that is used as a base:\n"
                      << "  - topology: " << toJson(initial_state.m_topology, JSON_COMPACT) << " -> " << toJson(stripped_initial_topology, JSON_COMPACT) << "\n"
                      << "  - primary devices: " << toJson(initial_state.m_primary_devices, JSON_COMPACT) << " -> " << toJson(initial_primary_devices, JSON_COMPACT);
    }

    return SingleDisplayConfigState::Initial {
      stripped_initial_topology,
      initial_primary_devices
    };
  }

  std::tuple<ActiveTopology, std::string, std::set<std::string>> computeNewTopologyAndMetadata(const SingleDisplayConfiguration::DevicePreparation device_prep, const std::string &device_id, const SingleDisplayConfigState::Initial &initial_state) {
    const bool configuring_unspecified_devices {device_id.empty()};
    const auto device_to_configure {configuring_unspecified_devices ? *std::begin(initial_state.m_primary_devices) : device_id};
    auto additional_devices_to_configure {configuring_unspecified_devices ? std::set<std::string> {std::next(std::begin(initial_state.m_primary_devices)), std::end(initial_state.m_primary_devices)} : tryGetOtherDevicesInTheSameGroup(initial_state.m_topology, device_to_configure)};
    DD_LOG(info) << "Will compute new display device topology from the following input:\n"
                 << "  - initial topology: " << toJson(initial_state.m_topology, JSON_COMPACT) << "\n"
                 << "  - initial primary devices: " << toJson(initial_state.m_primary_devices, JSON_COMPACT) << "\n"
                 << "  - configuring unspecified device: " << toJson(configuring_unspecified_devices, JSON_COMPACT) << "\n"
                 << "  - device to configure: " << toJson(device_to_configure, JSON_COMPACT) << "\n"
                 << "  - additional devices to configure: " << toJson(additional_devices_to_configure, JSON_COMPACT);

    const auto new_topology {computeNewTopology(device_prep, configuring_unspecified_devices, device_to_configure, additional_devices_to_configure, initial_state.m_topology)};
    additional_devices_to_configure = tryGetOtherDevicesInTheSameGroup(new_topology, device_to_configure);
    return std::make_tuple(new_topology, device_to_configure, additional_devices_to_configure);
  }

  DeviceDisplayModeMap computeNewDisplayModes(const std::optional<Resolution> &resolution, const std::optional<FloatingPoint> &refresh_rate, const bool configuring_primary_devices, const std::string &device_to_configure, const std::set<std::string> &additional_devices_to_configure, const DeviceDisplayModeMap &original_modes) {
    DeviceDisplayModeMap new_modes {original_modes};

    if (resolution) {
      // For duplicate devices the resolution must match no matter what, otherwise
      // they cannot be duplicated, which breaks Windows' rules. Therefore
      // we change resolution for all devices.
      const auto devices {joinConfigurableDevices(device_to_configure, additional_devices_to_configure)};
      for (const auto &device_id : devices) {
        new_modes[device_id].m_resolution = *resolution;
      }
    }

    if (refresh_rate) {
      const auto from_floating_point {[](const FloatingPoint &value) {
        if (const auto *rational_value {std::get_if<Rational>(&value)}; rational_value) {
          return *rational_value;
        }

        // It's hard to deal with floating values, so we just multiply it
        // to keep 4 decimal places (if any) and let Windows deal with it!
        // Genius idea if I'm being honest.
        constexpr unsigned int multiplier {10000};
        const double transformed_value {std::round(std::get<double>(value) * multiplier)};
        return Rational {static_cast<unsigned int>(transformed_value), multiplier};
      }};

      if (configuring_primary_devices) {
        // No device has been specified, so if they're all are primary devices
        // we need to apply the refresh rate change to all duplicates.
        const auto devices {joinConfigurableDevices(device_to_configure, additional_devices_to_configure)};
        for (const auto &device_id : devices) {
          new_modes[device_id].m_refresh_rate = from_floating_point(*refresh_rate);
        }
      } else {
        // Even if we have duplicate devices, their refresh rate may differ
        // and since the device was specified, let's apply the refresh
        // rate only to the specified device.
        new_modes[device_to_configure].m_refresh_rate = from_floating_point(*refresh_rate);
      }
    }

    return new_modes;
  }

  HdrStateMap computeNewHdrStates(const std::optional<HdrState> &hdr_state, bool configuring_primary_devices, const std::string &device_to_configure, const std::set<std::string> &additional_devices_to_configure, const HdrStateMap &original_states) {
    HdrStateMap new_states {original_states};

    if (hdr_state) {
      const auto try_update_new_state = [&new_states, &hdr_state](const std::string &device_id) {
        const auto current_state {new_states[device_id]};
        if (!current_state) {
          return;
        }

        new_states[device_id] = *hdr_state;
      };

      if (configuring_primary_devices) {
        // No device has been specified, so if they're all are primary devices
        // we need to update state for all duplicates.
        const auto devices {joinConfigurableDevices(device_to_configure, additional_devices_to_configure)};
        for (const auto &device_id : devices) {
          try_update_new_state(device_id);
        }
      } else {
        // Even if we have duplicate devices, their HDR states may differ
        // and since the device was specified, let's apply the HDR state
        // only to the specified device.
        try_update_new_state(device_to_configure);
      }
    }

    return new_states;
  }

  void blankHdrStates(WinDisplayDeviceInterface &win_dd, const std::optional<std::chrono::milliseconds> &delay) {
    if (!delay) {
      return;
    }

    const auto topology {win_dd.getCurrentTopology()};
    if (!win_dd.isTopologyValid(topology)) {
      DD_LOG(error) << "Got an invalid topology while trying to blank HDR states!";
      return;
    }

    const auto current_states {win_dd.getCurrentHdrStates(flattenTopology(topology))};
    if (current_states.empty()) {
      DD_LOG(error) << "Failed to get current HDR states! Topology:\n"
                    << toJson(topology);
      return;
    }

    std::set<std::string> device_ids;
    HdrStateMap original_states;
    HdrStateMap inverse_states;
    for (const auto &[device_id, state] : current_states) {
      if (!state || *state != HdrState::Enabled) {
        continue;
      }

      device_ids.insert(device_id);
      original_states[device_id] = HdrState::Enabled;
      inverse_states[device_id] = HdrState::Disabled;
    }

    if (device_ids.empty()) {
      // Nothing to do
      return;
    }

    DD_LOG(info) << "Applying HDR state \"blank\" workaround (" << delay->count() << "ms) to devices: " << toJson(device_ids, JSON_COMPACT);
    if (!win_dd.setHdrStates(inverse_states)) {
      DD_LOG(error) << "Failed to apply inverse HDR states during \"blank\"!";
      return;
    }

    std::this_thread::sleep_for(*delay);
    if (!win_dd.setHdrStates(original_states)) {
      DD_LOG(error) << "Failed to apply original HDR states during \"blank\"!";
    }
  }

  DdGuardFn topologyGuardFn(WinDisplayDeviceInterface &win_dd, const ActiveTopology &topology) {
    DD_LOG(debug) << "Got topology in topologyGuardFn:\n"
                  << toJson(topology);
    return [&win_dd, topology]() {
      if (!win_dd.setTopology(topology)) {
        DD_LOG(error) << "failed to revert topology in topologyGuardFn! Used the following topology:\n"
                      << toJson(topology);
      }
    };
  }

  DdGuardFn modeGuardFn(WinDisplayDeviceInterface &win_dd, const ActiveTopology &topology) {
    return modeGuardFn(win_dd, win_dd.getCurrentDisplayModes(flattenTopology(topology)));
  }

  DdGuardFn modeGuardFn(WinDisplayDeviceInterface &win_dd, const DeviceDisplayModeMap &modes) {
    DD_LOG(debug) << "Got modes in modeGuardFn:\n"
                  << toJson(modes);
    return [&win_dd, modes]() {
      if (!win_dd.setDisplayModes(modes)) {
        DD_LOG(error) << "failed to revert display modes in modeGuardFn! Used the following modes:\n"
                      << toJson(modes);
      }
    };
  }

  DdGuardFn primaryGuardFn(WinDisplayDeviceInterface &win_dd, const ActiveTopology &topology) {
    return primaryGuardFn(win_dd, getPrimaryDevice(win_dd, topology));
  }

  DdGuardFn primaryGuardFn(WinDisplayDeviceInterface &win_dd, const std::string &primary_device) {
    DD_LOG(debug) << "Got primary device in primaryGuardFn:\n"
                  << toJson(primary_device);
    return [&win_dd, primary_device]() {
      if (!win_dd.setAsPrimary(primary_device)) {
        DD_LOG(error) << "failed to revert primary device in primaryGuardFn! Used the following device id:\n"
                      << toJson(primary_device);
      }
    };
  }

  DdGuardFn hdrStateGuardFn(WinDisplayDeviceInterface &win_dd, const ActiveTopology &topology) {
    return hdrStateGuardFn(win_dd, win_dd.getCurrentHdrStates(flattenTopology(topology)));
  }

  DdGuardFn hdrStateGuardFn(WinDisplayDeviceInterface &win_dd, const HdrStateMap &states) {
    DD_LOG(debug) << "Got states in hdrStateGuardFn:\n"
                  << toJson(states);
    return [&win_dd, states]() {
      if (!win_dd.setHdrStates(states)) {
        DD_LOG(error) << "failed to revert HDR states in hdrStateGuardFn! Used the following HDR states:\n"
                      << toJson(states);
      }
    };
  }
}  // namespace display_device::win_utils
