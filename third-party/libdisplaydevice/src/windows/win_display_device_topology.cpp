/**
 * @file src/windows/win_display_device_topology.cpp
 * @brief Definitions for the topology related methods in WinDisplayDevice.
 */
// class header include
#include "display_device/windows/win_display_device.h"

// system includes
#include <algorithm>
#include <unordered_set>

// local includes
#include "display_device/logging.h"
#include "display_device/windows/win_api_utils.h"

namespace display_device {
  namespace {
    /**
     * @see set_topology for a description as this was split off to reduce cognitive complexity.
     */
    bool doSetTopology(WinApiLayerInterface &w_api, const ActiveTopology &new_topology, const PathAndModeData &display_data) {
      const auto path_data {win_utils::collectSourceDataForMatchingPaths(w_api, display_data.m_paths)};
      if (path_data.empty()) {
        // Error already logged
        return false;
      }

      auto paths {win_utils::makePathsForNewTopology(new_topology, path_data, display_data.m_paths)};
      if (paths.empty()) {
        // Error already logged
        return false;
      }

      UINT32 flags {SDC_APPLY | SDC_TOPOLOGY_SUPPLIED | SDC_ALLOW_PATH_ORDER_CHANGES | SDC_VIRTUAL_MODE_AWARE};
      LONG result {w_api.setDisplayConfig(paths, {}, flags)};
      if (result == ERROR_GEN_FAILURE) {
        DD_LOG(warning) << w_api.getErrorString(result) << " failed to change topology using the topology from Windows DB! Asking Windows to create the topology.";

        flags = SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_ALLOW_CHANGES /* This flag is probably not needed, but who knows really... (not MSDOCS at least) */ | SDC_VIRTUAL_MODE_AWARE | SDC_SAVE_TO_DATABASE;
        result = w_api.setDisplayConfig(paths, {}, flags);
        if (result != ERROR_SUCCESS) {
          DD_LOG(error) << w_api.getErrorString(result) << " failed to create new topology configuration!";
          return false;
        }
      } else if (result != ERROR_SUCCESS) {
        DD_LOG(error) << w_api.getErrorString(result) << " failed to change topology configuration!";
        return false;
      }

      return true;
    }
  }  // namespace

  ActiveTopology WinDisplayDevice::getCurrentTopology() const {
    const auto display_data {m_w_api->queryDisplayConfig(QueryType::Active)};
    if (!display_data) {
      // Error already logged
      return {};
    }

    // Duplicate displays can be identified by having the same x/y position. Here we have a
    // "position to index" map for a simple and lazy lookup in case we have to add a device to the
    // topology group.
    std::unordered_map<std::string, std::size_t> position_to_topology_index;
    ActiveTopology topology;
    for (const auto &path : display_data->m_paths) {
      const auto device_info {win_utils::getDeviceInfoForValidPath(*m_w_api, path, display_device::ValidatedPathType::Active)};
      if (!device_info) {
        continue;
      }

      const auto source_mode {win_utils::getSourceMode(win_utils::getSourceIndex(path, display_data->m_modes), display_data->m_modes)};
      if (!source_mode) {
        DD_LOG(error) << "Active device does not have a source mode: " << device_info->m_device_id << "!";
        return {};
      }

      const std::string lazy_lookup {std::to_string(source_mode->position.x) + std::to_string(source_mode->position.y)};
      auto index_it {position_to_topology_index.find(lazy_lookup)};

      if (index_it == std::end(position_to_topology_index)) {
        position_to_topology_index[lazy_lookup] = topology.size();
        topology.push_back({device_info->m_device_id});
      } else {
        topology.at(index_it->second).push_back(device_info->m_device_id);
      }
    }

    return topology;
  }

  bool WinDisplayDevice::isTopologyValid(const ActiveTopology &topology) const {
    if (topology.empty()) {
      DD_LOG(warning) << "Topology input is empty!";
      return false;
    }

    std::unordered_set<std::string> device_ids;
    for (const auto &group : topology) {
      // Size 2 is a Windows' limitation.
      // You CAN set the group to be more than 2, but then
      // Windows' settings app breaks since it was not designed for this :/
      if (group.empty() || group.size() > 2) {
        DD_LOG(warning) << "Topology group is invalid!";
        return false;
      }

      for (const auto &device_id : group) {
        if (!device_ids.insert(device_id).second) {
          DD_LOG(warning) << "Duplicate device ids found in topology!";
          return false;
        }
      }
    }

    return true;
  }

  bool WinDisplayDevice::isTopologyTheSame(const ActiveTopology &lhs, const ActiveTopology &rhs) const {
    const auto sort_topology = [](ActiveTopology &topology) {
      for (auto &group : topology) {
        std::sort(std::begin(group), std::end(group));
      }

      std::sort(std::begin(topology), std::end(topology));
    };

    auto lhs_copy {lhs};
    auto rhs_copy {rhs};

    // On Windows order does not matter.
    sort_topology(lhs_copy);
    sort_topology(rhs_copy);

    return lhs_copy == rhs_copy;
  }

  bool WinDisplayDevice::setTopology(const ActiveTopology &new_topology) {
    if (!isTopologyValid(new_topology)) {
      DD_LOG(error) << "Topology input is invalid!";
      return false;
    }

    const auto current_topology {getCurrentTopology()};
    if (!isTopologyValid(current_topology)) {
      DD_LOG(error) << "Failed to get current topology!";
      return false;
    }

    if (isTopologyTheSame(current_topology, new_topology)) {
      DD_LOG(debug) << "Same topology provided.";
      return true;
    }

    const auto &original_data {m_w_api->queryDisplayConfig(QueryType::All)};
    if (!original_data) {
      // Error already logged
      return false;
    }

    if (doSetTopology(*m_w_api, new_topology, *original_data)) {
      const auto updated_topology {getCurrentTopology()};
      if (isTopologyValid(updated_topology)) {
        if (isTopologyTheSame(new_topology, updated_topology)) {
          return true;
        } else {
          // There is an interesting bug in Windows when you have nearly
          // identical devices, drivers or something. For example, imagine you have:
          //    AM   - Actual Monitor
          //    IDD1 - Virtual display 1
          //    IDD2 - Virtual display 2
          //
          // You can have the following topology:
          //    [[AM, IDD1]]
          // but not this:
          //    [[AM, IDD2]]
          //
          // Windows API will just default to:
          //    [[AM, IDD1]]
          // even if you provide the second variant. Windows API will think
          // it's OK and just return ERROR_SUCCESS in this case and there is
          // nothing you can do. Even the Windows' settings app will not
          // be able to set the desired topology.
          //
          // There seems to be a workaround - you need to make sure the IDD1
          // device is used somewhere else in the topology, like:
          //    [[AM, IDD2], [IDD1]]
          //
          // However, since we have this bug an additional sanity check is needed
          // regardless of what Windows report back to us.
          DD_LOG(error) << "Failed to change topology due to Windows bug or because the display is in deep sleep!";
        }
      } else {
        DD_LOG(error) << "Failed to get updated topology!";
      }

      // Revert back to the original topology
      const UINT32 flags {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE};
      static_cast<void>(m_w_api->setDisplayConfig(original_data->m_paths, original_data->m_modes, flags));  // Return value does not matter as we are trying out best to undo
    }

    return false;
  }
}  // namespace display_device
