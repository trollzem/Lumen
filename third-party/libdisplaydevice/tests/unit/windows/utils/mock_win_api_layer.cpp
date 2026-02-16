// local includes
#include "mock_win_api_layer.h"

namespace {
  display_device::PathAndModeData make3ActiveDeviceGroups(const bool include_duplicate) {
    display_device::PathAndModeData data;

    // 1st group (1 device)
    {
      data.m_paths.push_back({});
      data.m_paths.back().flags = DISPLAYCONFIG_PATH_ACTIVE;
      data.m_paths.back().sourceInfo.sourceModeInfoIdx = data.m_modes.size();
      data.m_paths.back().sourceInfo.adapterId = {1, 1};
      data.m_paths.back().sourceInfo.id = 1;
      data.m_paths.back().targetInfo.targetAvailable = TRUE;
      data.m_paths.back().targetInfo.refreshRate = {120, 1};

      data.m_modes.push_back({});
      data.m_modes.back().infoType = DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE;
      data.m_modes.back().sourceMode = {};  // Set the union
      data.m_modes.back().sourceMode.position = {0, 0};
      data.m_modes.back().sourceMode.width = 1920;
      data.m_modes.back().sourceMode.height = 1080;
    }

    // 2nd group (1+ device)
    {
      data.m_paths.push_back({});
      data.m_paths.back().flags = DISPLAYCONFIG_PATH_ACTIVE;
      data.m_paths.back().sourceInfo.sourceModeInfoIdx = data.m_modes.size();
      data.m_paths.back().sourceInfo.adapterId = {2, 2};
      data.m_paths.back().sourceInfo.id = 2;
      data.m_paths.back().targetInfo.targetAvailable = TRUE;
      data.m_paths.back().targetInfo.refreshRate = {119995, 1000};

      data.m_modes.push_back({});
      data.m_modes.back().infoType = DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE;
      data.m_modes.back().sourceMode = {};  // Set the union
      data.m_modes.back().sourceMode.position = {1921, 0};
      data.m_modes.back().sourceMode.width = 1920;
      data.m_modes.back().sourceMode.height = 2160;

      if (include_duplicate) {
        data.m_paths.push_back({});
        data.m_paths.back().flags = DISPLAYCONFIG_PATH_ACTIVE;
        data.m_paths.back().sourceInfo.sourceModeInfoIdx = data.m_modes.size();
        data.m_paths.back().sourceInfo.adapterId = {3, 3};
        data.m_paths.back().sourceInfo.id = 3;
        data.m_paths.back().targetInfo.targetAvailable = TRUE;
        data.m_paths.back().targetInfo.refreshRate = {60, 1};

        data.m_modes.push_back({});
        data.m_modes.back().infoType = DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE;
        data.m_modes.back().sourceMode = {};  // Set the union
        data.m_modes.back().sourceMode.position = {1921, 0};
        data.m_modes.back().sourceMode.width = 1920;
        data.m_modes.back().sourceMode.height = 2160;
      }
    }

    // 3rd group (1 device)
    {
      data.m_paths.push_back({});
      data.m_paths.back().flags = DISPLAYCONFIG_PATH_ACTIVE;
      data.m_paths.back().sourceInfo.sourceModeInfoIdx = data.m_modes.size();
      data.m_paths.back().sourceInfo.adapterId = {4, 4};
      data.m_paths.back().sourceInfo.id = 4;
      data.m_paths.back().targetInfo.targetAvailable = TRUE;
      data.m_paths.back().targetInfo.refreshRate = {90, 1};

      data.m_modes.push_back({});
      data.m_modes.back().infoType = DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE;
      data.m_modes.back().sourceMode = {};  // Set the union
      data.m_modes.back().sourceMode.position = {0, 1081};
      data.m_modes.back().sourceMode.width = 3840;
      data.m_modes.back().sourceMode.height = 2160;
    }

    return data;
  }
}  // namespace

namespace ut_consts {
  const std::optional<display_device::PathAndModeData> PAM_NULL {std::nullopt};
  const std::optional<display_device::PathAndModeData> PAM_EMPTY {display_device::PathAndModeData {}};
  const std::optional<display_device::PathAndModeData> PAM_3_ACTIVE {make3ActiveDeviceGroups(false)};
  const std::optional<display_device::PathAndModeData> PAM_3_ACTIVE_WITH_INVALID_MODE_IDX {[]() {
    auto data {make3ActiveDeviceGroups(false)};

    // Scramble the indexes
    auto invalid_idx {DISPLAYCONFIG_PATH_SOURCE_MODE_IDX_INVALID};
    for (auto &item : data.m_paths) {
      item.sourceInfo.sourceModeInfoIdx = invalid_idx--;
    }

    return data;
  }()};
  const std::optional<display_device::PathAndModeData> PAM_4_ACTIVE_WITH_2_DUPLICATES {make3ActiveDeviceGroups(true)};
}  // namespace ut_consts
