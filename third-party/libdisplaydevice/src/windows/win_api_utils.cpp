/**
 * @file src/windows/win_api_utils.cpp
 * @brief Definitions for lower level Windows API utility functions.
 */
// header include
#include "display_device/windows/win_api_utils.h"

// system includes
#include <unordered_set>

// local includes
#include "display_device/logging.h"

namespace {
  /**
   * @brief Check if adapter ids are NOT equal.
   * @param lhs First id to check.
   * @param rhs Second id to check.
   * @return True if NOT equal, false otherwise.
   * @examples
   * const bool not_equal = (LUID{ 12, 34 }) != (LUID{ 12, 56 });
   * @examples_end
   */
  bool operator!=(const LUID &lhs, const LUID &rhs) {
    return lhs.HighPart != rhs.HighPart || lhs.LowPart != rhs.LowPart;
  }

  /**
   * @brief Stringify adapter id.
   * @param id Id to stringify.
   * @return String representation of the id.
   * @examples
   * const bool id_string = to_string({ 12, 34 });
   * @examples_end
   */
  std::string toString(const LUID &id) {
    return std::to_string(id.HighPart) + std::to_string(id.LowPart);
  }

  /**
   * @brief Check if the source modes are duplicated (cloned).
   * @param lhs First mode to check.
   * @param rhs Second mode to check.
   * @returns True if both mode have the same origin point, false otherwise.
   * @note Windows enforces the behaviour that only the duplicate devices can
   *       have the same origin point as otherwise the configuration is considered invalid by the OS.
   * @examples
   * DISPLAYCONFIG_SOURCE_MODE mode_a;
   * DISPLAYCONFIG_SOURCE_MODE mode_b;
   * const bool are_duplicated = are_modes_duplicated(mode_a, mode_b);
   * @examples_end
   */
  bool are_modes_duplicated(const DISPLAYCONFIG_SOURCE_MODE &lhs, const DISPLAYCONFIG_SOURCE_MODE &rhs) {
    return lhs.position.x == rhs.position.x && lhs.position.y == rhs.position.y;
  }
}  // namespace

namespace display_device::win_utils {
  bool isAvailable(const DISPLAYCONFIG_PATH_INFO &path) {
    return path.targetInfo.targetAvailable == TRUE;
  }

  bool isActive(const DISPLAYCONFIG_PATH_INFO &path) {
    return static_cast<bool>(path.flags & DISPLAYCONFIG_PATH_ACTIVE);
  }

  void setActive(DISPLAYCONFIG_PATH_INFO &path) {
    path.flags |= DISPLAYCONFIG_PATH_ACTIVE;
  }

  bool isPrimary(const DISPLAYCONFIG_SOURCE_MODE &mode) {
    return mode.position.x == 0 && mode.position.y == 0;
  }

  std::optional<UINT32> getSourceIndex(const DISPLAYCONFIG_PATH_INFO &path, const std::vector<DISPLAYCONFIG_MODE_INFO> &modes) {
    // The MS docs is not clear when to access the index union struct or not. It appears that union struct is available,
    // whenever QDC_VIRTUAL_MODE_AWARE is specified when querying (always in our case).
    //
    // The docs state, however, that it is only available when DISPLAYCONFIG_PATH_SUPPORT_VIRTUAL_MODE flag is set, but
    // that is just BS (maybe copy-pasta mistake), because some cases were found where the flag is not set and the union
    // is still being used.

    const UINT32 index {path.sourceInfo.sourceModeInfoIdx};
    if (index == DISPLAYCONFIG_PATH_SOURCE_MODE_IDX_INVALID) {
      return std::nullopt;
    }

    if (index >= modes.size()) {
      DD_LOG(error) << "Source index " << index << " is out of range " << modes.size();
      return std::nullopt;
    }

    return index;
  }

  void setSourceIndex(DISPLAYCONFIG_PATH_INFO &path, const std::optional<UINT32> &index) {
    // The MS docs is not clear when to access the index union struct or not. It appears that union struct is available,
    // whenever QDC_VIRTUAL_MODE_AWARE is specified when querying (always in our case).
    //
    // The docs state, however, that it is only available when DISPLAYCONFIG_PATH_SUPPORT_VIRTUAL_MODE flag is set, but
    // that is just BS (maybe copy-pasta mistake), because some cases were found where the flag is not set and the union
    // is still being used.

    if (index) {
      path.sourceInfo.sourceModeInfoIdx = *index;
    } else {
      path.sourceInfo.sourceModeInfoIdx = DISPLAYCONFIG_PATH_SOURCE_MODE_IDX_INVALID;
    }
  }

  void setTargetIndex(DISPLAYCONFIG_PATH_INFO &path, const std::optional<UINT32> &index) {
    // The MS docs is not clear when to access the index union struct or not. It appears that union struct is available,
    // whenever QDC_VIRTUAL_MODE_AWARE is specified when querying (always in our case).
    //
    // The docs state, however, that it is only available when DISPLAYCONFIG_PATH_SUPPORT_VIRTUAL_MODE flag is set, but
    // that is just BS (maybe copy-pasta mistake), because some cases were found where the flag is not set and the union
    // is still being used.

    if (index) {
      path.targetInfo.targetModeInfoIdx = *index;
    } else {
      path.targetInfo.targetModeInfoIdx = DISPLAYCONFIG_PATH_TARGET_MODE_IDX_INVALID;
    }
  }

  void setDesktopIndex(DISPLAYCONFIG_PATH_INFO &path, const std::optional<UINT32> &index) {
    // The MS docs is not clear when to access the index union struct or not. It appears that union struct is available,
    // whenever QDC_VIRTUAL_MODE_AWARE is specified when querying (always in our case).
    //
    // The docs state, however, that it is only available when DISPLAYCONFIG_PATH_SUPPORT_VIRTUAL_MODE flag is set, but
    // that is just BS (maybe copy-pasta mistake), because some cases were found where the flag is not set and the union
    // is still being used.

    if (index) {
      path.targetInfo.desktopModeInfoIdx = *index;
    } else {
      path.targetInfo.desktopModeInfoIdx = DISPLAYCONFIG_PATH_DESKTOP_IMAGE_IDX_INVALID;
    }
  }

  void setCloneGroupId(DISPLAYCONFIG_PATH_INFO &path, const std::optional<UINT32> &id) {
    // The MS docs is not clear when to access the index union struct or not. It appears that union struct is available,
    // whenever QDC_VIRTUAL_MODE_AWARE is specified when querying (always in our case).
    //
    // The docs state, however, that it is only available when DISPLAYCONFIG_PATH_SUPPORT_VIRTUAL_MODE flag is set, but
    // that is just BS (maybe copy-pasta mistake), because some cases were found where the flag is not set and the union
    // is still being used.

    if (id) {
      path.sourceInfo.cloneGroupId = *id;
    } else {
      path.sourceInfo.cloneGroupId = DISPLAYCONFIG_PATH_CLONE_GROUP_INVALID;
    }
  }

  const DISPLAYCONFIG_SOURCE_MODE *getSourceMode(const std::optional<UINT32> &index, const std::vector<DISPLAYCONFIG_MODE_INFO> &modes) {
    if (!index) {
      return nullptr;
    }

    if (*index >= modes.size()) {
      DD_LOG(error) << "Source index " << *index << " is out of range " << modes.size();
      return nullptr;
    }

    const auto &mode {modes[*index]};
    if (mode.infoType != DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE) {
      DD_LOG(error) << "Mode at index " << *index << " is not source mode!";
      return nullptr;
    }

    return &mode.sourceMode;
  }

  DISPLAYCONFIG_SOURCE_MODE *getSourceMode(const std::optional<UINT32> &index, std::vector<DISPLAYCONFIG_MODE_INFO> &modes) {
    return const_cast<DISPLAYCONFIG_SOURCE_MODE *>(getSourceMode(index, const_cast<const std::vector<DISPLAYCONFIG_MODE_INFO> &>(modes)));
  }

  std::optional<ValidatedDeviceInfo> getDeviceInfoForValidPath(const WinApiLayerInterface &w_api, const DISPLAYCONFIG_PATH_INFO &path, const ValidatedPathType type) {
    if (!isAvailable(path)) {
      // Could be transient issue according to MSDOCS (no longer available, but still "active")
      return std::nullopt;
    }

    if (type == ValidatedPathType::Active) {
      if (!isActive(path)) {
        return std::nullopt;
      }
    }

    const auto device_path {w_api.getMonitorDevicePath(path)};
    if (device_path.empty()) {
      return std::nullopt;
    }

    const auto device_id {w_api.getDeviceId(path)};
    if (device_id.empty()) {
      return std::nullopt;
    }

    const auto display_name {w_api.getDisplayName(path)};
    if (display_name.empty()) {
      return std::nullopt;
    }

    return ValidatedDeviceInfo {device_path, device_id};
  }

  const DISPLAYCONFIG_PATH_INFO *getActivePath(const WinApiLayerInterface &w_api, const std::string &device_id, const std::vector<DISPLAYCONFIG_PATH_INFO> &paths) {
    for (const auto &path : paths) {
      const auto device_info {getDeviceInfoForValidPath(w_api, path, ValidatedPathType::Active)};
      if (!device_info) {
        continue;
      }

      if (device_info->m_device_id == device_id) {
        return &path;
      }
    }

    return nullptr;
  }

  DISPLAYCONFIG_PATH_INFO *getActivePath(const WinApiLayerInterface &w_api, const std::string &device_id, std::vector<DISPLAYCONFIG_PATH_INFO> &paths) {
    return const_cast<DISPLAYCONFIG_PATH_INFO *>(getActivePath(w_api, device_id, const_cast<const std::vector<DISPLAYCONFIG_PATH_INFO> &>(paths)));
  }

  PathSourceIndexDataMap collectSourceDataForMatchingPaths(const WinApiLayerInterface &w_api, const std::vector<DISPLAYCONFIG_PATH_INFO> &paths) {
    PathSourceIndexDataMap path_data;

    std::unordered_map<std::string, std::string> paths_to_ids;
    for (std::size_t index = 0; index < paths.size(); ++index) {
      const auto &path {paths[index]};

      const auto device_info {getDeviceInfoForValidPath(w_api, path, ValidatedPathType::Any)};
      if (!device_info) {
        // Path is not valid
        continue;
      }

      const auto prev_device_id_for_path_it {paths_to_ids.find(device_info->m_device_path)};
      if (prev_device_id_for_path_it != std::end(paths_to_ids)) {
        if (prev_device_id_for_path_it->second != device_info->m_device_id) {
          DD_LOG(error) << "Duplicate display device id found: " << device_info->m_device_id << " (device path: " << device_info->m_device_path << ")";
          return {};
        }
      } else {
        for (const auto &[device_path, device_id] : paths_to_ids) {
          if (device_id == device_info->m_device_id) {
            DD_LOG(error) << "Device id " << device_info->m_device_id << " is shared between 2 different paths: " << device_path << " and " << device_info->m_device_path;
            return {};
          }
        }

        paths_to_ids[device_info->m_device_path] = device_info->m_device_id;
      }

      auto path_data_it {path_data.find(device_info->m_device_id)};
      if (path_data_it != std::end(path_data)) {
        if (path_data_it->second.m_adapter_id != path.sourceInfo.adapterId) {
          // Sanity check, should not be possible since adapter in embedded in the device path
          DD_LOG(error) << "Device path " << device_info->m_device_path << " has different adapters!";
          return {};
        } else if (isActive(path)) {
          // Sanity check, should not be possible as all active paths are in the front
          DD_LOG(error) << "Device path " << device_info->m_device_path << " is active, but not the first entry in the list!";
          return {};
        } else if (path_data_it->second.m_source_id_to_path_index.contains(path.sourceInfo.id)) {
          // Sanity check, should not be possible unless Windows goes bonkers
          DD_LOG(error) << "Device path " << device_info->m_device_path << " has duplicate source ids!";
          return {};
        }

        path_data_it->second.m_source_id_to_path_index[path.sourceInfo.id] = index;
      } else {
        path_data[device_info->m_device_id] = {
          {{path.sourceInfo.id, index}},
          path.sourceInfo.adapterId,
          // Since active paths are always in the front, this is the only time we set it
          isActive(path) ? std::make_optional(path.sourceInfo.id) : std::nullopt
        };
      }

      DD_LOG(verbose) << "Device " << device_info->m_device_id << " (active: " << isActive(path) << ") at index " << index << " added to the source data list.";
    }

    if (path_data.empty()) {
      DD_LOG(error) << "Failed to collect path source data or none was available!";
    }
    return path_data;
  }

  std::vector<DISPLAYCONFIG_PATH_INFO> makePathsForNewTopology(const ActiveTopology &new_topology, const PathSourceIndexDataMap &path_source_data, const std::vector<DISPLAYCONFIG_PATH_INFO> &paths) {
    std::vector<DISPLAYCONFIG_PATH_INFO> new_paths;

    UINT32 group_id {0};
    std::unordered_map<std::string, std::unordered_set<UINT32>> used_source_ids_per_adapter;
    const auto is_source_id_already_used = [&used_source_ids_per_adapter](const LUID &adapter_id, UINT32 source_id) {
      auto entry_it {used_source_ids_per_adapter.find(toString(adapter_id))};
      if (entry_it != std::end(used_source_ids_per_adapter)) {
        return entry_it->second.contains(source_id);
      }

      return false;
    };

    for (const auto &group : new_topology) {
      std::unordered_map<std::string, UINT32> used_source_ids_per_adapter_per_group;
      const auto get_already_used_source_id_in_group = [&used_source_ids_per_adapter_per_group](const LUID &adapter_id) -> std::optional<UINT32> {
        auto entry_it {used_source_ids_per_adapter_per_group.find(toString(adapter_id))};
        if (entry_it != std::end(used_source_ids_per_adapter_per_group)) {
          return entry_it->second;
        }

        return std::nullopt;
      };

      for (const std::string &device_id : group) {
        auto path_source_data_it {path_source_data.find(device_id)};
        if (path_source_data_it == std::end(path_source_data)) {
          DD_LOG(error) << "Device " << device_id << " does not exist in the available path source data!";
          return {};
        }

        std::size_t selected_path_index {};
        const auto &source_data {path_source_data_it->second};

        const auto already_used_source_id {get_already_used_source_id_in_group(source_data.m_adapter_id)};
        if (already_used_source_id) {
          // Some device in the group is already using the source id, and we belong to the same adapter.
          // This means we must also use the path with matching source id.
          auto path_index_it {source_data.m_source_id_to_path_index.find(*already_used_source_id)};
          if (path_index_it == std::end(source_data.m_source_id_to_path_index)) {
            DD_LOG(error) << "Device " << device_id << " does not have a path with a source id " << *already_used_source_id << "!";
            return {};
          }

          selected_path_index = path_index_it->second;
        } else {
          // Here we want to select a path index that has the lowest index (the "best" of paths), but only
          // if the source id is still free. Technically we should not need to find the lowest index, but that's
          // what will match the Windows' behaviour the closest if we need to create new topology in the end.
          std::optional<std::size_t> path_index_candidate;
          UINT32 used_source_id {};
          for (const auto [source_id, index] : source_data.m_source_id_to_path_index) {
            if (is_source_id_already_used(source_data.m_adapter_id, source_id)) {
              continue;
            }

            if (!path_index_candidate || index < *path_index_candidate) {
              path_index_candidate = index;
              used_source_id = source_id;
            }
          }

          if (!path_index_candidate) {
            // Apparently nvidia GPU can only render 4 different sources at a time (according to Google).
            // However, it seems to be true only for physical connections as we also have virtual displays.
            //
            // Virtual displays have different adapter ids than the physical connection ones, but GPU still
            // has to render them, so I don't know how this 4 source limitation makes sense then?
            //
            // In short, this arbitrary limitation should not affect virtual displays when the GPU is at its limit.
            DD_LOG(error) << "Device " << device_id << " cannot be enabled as the adapter has no more free source ids (GPU limitation)!";
            return {};
          }

          selected_path_index = *path_index_candidate;
          used_source_ids_per_adapter[toString(source_data.m_adapter_id)].insert(used_source_id);
          used_source_ids_per_adapter_per_group[toString(source_data.m_adapter_id)] = used_source_id;
        }

        if (selected_path_index >= paths.size()) {
          DD_LOG(error) << "Selected path index " << selected_path_index << " is out of range! List size: " << paths.size();
          return {};
        }

        auto selected_path {paths[selected_path_index]};

        // All the indexes must be cleared and only the group id specified
        win_utils::setSourceIndex(selected_path, std::nullopt);
        win_utils::setTargetIndex(selected_path, std::nullopt);
        win_utils::setDesktopIndex(selected_path, std::nullopt);
        win_utils::setCloneGroupId(selected_path, group_id);
        win_utils::setActive(selected_path);  // We also need to mark it as active...

        new_paths.push_back(selected_path);
      }

      group_id++;
    }

    if (new_paths.empty()) {
      DD_LOG(error) << "Failed to make paths for new topology!";
    }
    return new_paths;
  }

  std::set<std::string> getAllDeviceIdsAndMatchingDuplicates(const WinApiLayerInterface &w_api, const std::set<std::string> &device_ids) {
    const auto display_data {w_api.queryDisplayConfig(QueryType::Active)};
    if (!display_data) {
      // Error already logged
      return {};
    }

    std::set<std::string> all_device_ids;
    for (const auto &device_id : device_ids) {
      if (device_id.empty()) {
        DD_LOG(error) << "Device it is empty!";
        return {};
      }

      const auto provided_path {getActivePath(w_api, device_id, display_data->m_paths)};
      if (!provided_path) {
        DD_LOG(warning) << "Failed to find device for " << device_id << "!";
        return {};
      }

      const auto provided_path_source_mode {getSourceMode(getSourceIndex(*provided_path, display_data->m_modes), display_data->m_modes)};
      if (!provided_path_source_mode) {
        DD_LOG(error) << "Active device does not have a source mode: " << device_id << "!";
        return {};
      }

      // We will now iterate over all the active paths (provided path included) and check if
      // any of them are duplicated.
      for (const auto &path : display_data->m_paths) {
        const auto device_info {getDeviceInfoForValidPath(w_api, path, ValidatedPathType::Active)};
        if (!device_info) {
          continue;
        }

        if (all_device_ids.contains(device_info->m_device_id)) {
          // Already checked
          continue;
        }

        const auto source_mode {getSourceMode(getSourceIndex(path, display_data->m_modes), display_data->m_modes)};
        if (!source_mode) {
          DD_LOG(error) << "Active device does not have a source mode: " << device_info->m_device_id << "!";
          return {};
        }

        if (!are_modes_duplicated(*provided_path_source_mode, *source_mode)) {
          continue;
        }

        all_device_ids.insert(device_info->m_device_id);
      }
    }

    return all_device_ids;
  }

  bool fuzzyCompareRefreshRates(const Rational &lhs, const Rational &rhs) {
    if (lhs.m_denominator > 0 && rhs.m_denominator > 0) {
      const double lhs_f {static_cast<double>(lhs.m_numerator) / static_cast<double>(lhs.m_denominator)};
      const double rhs_f {static_cast<double>(rhs.m_numerator) / static_cast<double>(rhs.m_denominator)};
      return (std::abs(lhs_f - rhs_f) <= 0.9);
    }

    return false;
  }

  bool fuzzyCompareModes(const DisplayMode &lhs, const DisplayMode &rhs) {
    return lhs.m_resolution.m_width == rhs.m_resolution.m_width &&
           lhs.m_resolution.m_height == rhs.m_resolution.m_height &&
           fuzzyCompareRefreshRates(lhs.m_refresh_rate, rhs.m_refresh_rate);
  }
}  // namespace display_device::win_utils
