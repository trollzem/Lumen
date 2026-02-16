// local includes
#include "display_device/windows/win_api_layer.h"
#include "fixtures/fixtures.h"

namespace {
  // Test fixture(s) for this file
  class WinApiLayer: public BaseTest {
  public:
    display_device::WinApiLayer m_layer;
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, WinApiLayer, __VA_ARGS__)

  // Additional convenience global const(s)
  DISPLAYCONFIG_PATH_INFO INVALID_PATH {
    []() {
      DISPLAYCONFIG_PATH_INFO path;

      // This is a gamble, but no one sane uses the maximum values as their IDs
      path.sourceInfo.adapterId = {std::numeric_limits<DWORD>::max(), std::numeric_limits<LONG>::max()};
      path.sourceInfo.id = std::numeric_limits<UINT32>::max();
      path.targetInfo.adapterId = {std::numeric_limits<DWORD>::max(), std::numeric_limits<LONG>::max()};
      path.targetInfo.id = std::numeric_limits<UINT32>::max();

      return path;
    }()
  };
}  // namespace

TEST_F_S(GetErrorString) {
  EXPECT_TRUE(testRegex(m_layer.getErrorString(ERROR_INVALID_PARAMETER), R"(\[code: ERROR_INVALID_PARAMETER, .+?\])"));
  EXPECT_TRUE(testRegex(m_layer.getErrorString(ERROR_NOT_SUPPORTED), R"(\[code: ERROR_NOT_SUPPORTED, .+?\])"));
  EXPECT_TRUE(testRegex(m_layer.getErrorString(ERROR_ACCESS_DENIED), R"(\[code: ERROR_ACCESS_DENIED, .+?\])"));
  EXPECT_TRUE(testRegex(m_layer.getErrorString(ERROR_INSUFFICIENT_BUFFER), R"(\[code: ERROR_INSUFFICIENT_BUFFER, .+?\])"));
  EXPECT_TRUE(testRegex(m_layer.getErrorString(ERROR_GEN_FAILURE), R"(\[code: ERROR_GEN_FAILURE, .+?\])"));
  EXPECT_TRUE(testRegex(m_layer.getErrorString(ERROR_SUCCESS), R"(\[code: ERROR_SUCCESS, .+?\])"));
  EXPECT_TRUE(testRegex(m_layer.getErrorString(ERROR_ACCOUNT_DISABLED), R"(\[code: )" + std::to_string(ERROR_ACCOUNT_DISABLED) + R"(, .+?\])"));
}

TEST_F_S(QueryDisplayConfig, PathAndModeCount) {
  const auto active_devices {m_layer.queryDisplayConfig(display_device::QueryType::Active)};
  const auto all_devices {m_layer.queryDisplayConfig(display_device::QueryType::All)};

  ASSERT_TRUE(active_devices);
  ASSERT_TRUE(all_devices);

  // This test (and some others) is pointless without any paths. We should always have at least 1 active display device!
  EXPECT_TRUE(!active_devices->m_paths.empty());
  EXPECT_TRUE(all_devices->m_paths.size() >= active_devices->m_paths.size());
  EXPECT_TRUE(all_devices->m_modes.size() == active_devices->m_modes.size());
}

TEST_F_S(QueryDisplayConfig, ActivePaths) {
  const auto active_devices {m_layer.queryDisplayConfig(display_device::QueryType::Active)};
  ASSERT_TRUE(active_devices);

  for (const auto &path : active_devices->m_paths) {
    EXPECT_TRUE(static_cast<bool>(path.flags & DISPLAYCONFIG_PATH_ACTIVE));
  }
}

TEST_F_S(QueryDisplayConfig, ModeIndexValidity) {
  // The MS docs is not clear when to access the index union struct or not. It appears that union struct is available,
  // whenever QDC_VIRTUAL_MODE_AWARE is specified when querying (always in our case).
  //
  // The docs state, however, that it is only available when DISPLAYCONFIG_PATH_SUPPORT_VIRTUAL_MODE flag is set, but
  // that is just BS (maybe copy-pasta mistake), because some cases were found where the flag is not set and the union
  // is still being used.
  const auto active_devices {m_layer.queryDisplayConfig(display_device::QueryType::Active)};
  const auto all_devices {m_layer.queryDisplayConfig(display_device::QueryType::All)};

  for (const auto &devices : {active_devices, all_devices}) {
    ASSERT_TRUE(devices);

    for (const auto &path : devices->m_paths) {
      const auto clone_group_id = path.sourceInfo.cloneGroupId;
      const auto source_mode_index = path.sourceInfo.sourceModeInfoIdx;
      const auto target_mode_index = path.targetInfo.targetModeInfoIdx;
      const auto desktop_mode_index = path.targetInfo.desktopModeInfoIdx;

      // It is always invalid unless we are preparing paths for new topology
      EXPECT_EQ(clone_group_id, DISPLAYCONFIG_PATH_CLONE_GROUP_INVALID);

      if (source_mode_index != DISPLAYCONFIG_PATH_SOURCE_MODE_IDX_INVALID) {
        ASSERT_TRUE(source_mode_index < devices->m_modes.size());
        EXPECT_EQ(devices->m_modes[source_mode_index].infoType, DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE);
      }

      if (target_mode_index != DISPLAYCONFIG_PATH_TARGET_MODE_IDX_INVALID) {
        ASSERT_TRUE(target_mode_index < devices->m_modes.size());
        EXPECT_EQ(devices->m_modes[target_mode_index].infoType, DISPLAYCONFIG_MODE_INFO_TYPE_TARGET);
      }

      if (desktop_mode_index != DISPLAYCONFIG_PATH_DESKTOP_IMAGE_IDX_INVALID) {
        ASSERT_TRUE(desktop_mode_index < devices->m_modes.size());
        EXPECT_EQ(devices->m_modes[desktop_mode_index].infoType, DISPLAYCONFIG_MODE_INFO_TYPE_DESKTOP_IMAGE);
      }
    }
  }
}

TEST_F_S(GetDeviceId) {
  const auto all_devices {m_layer.queryDisplayConfig(display_device::QueryType::All)};
  ASSERT_TRUE(all_devices);

  std::map<std::string, std::string> device_id_per_device_path;
  for (const auto &path : all_devices->m_paths) {
    const auto device_id {m_layer.getDeviceId(path)};
    const auto device_id_2 {m_layer.getDeviceId(path)};
    const auto device_path {m_layer.getMonitorDevicePath(path)};

    // Testing soft persistence - ids remain the same between calls
    EXPECT_EQ(device_id, device_id_2);

    if (device_id.empty()) {
      EXPECT_EQ(path.targetInfo.targetAvailable, FALSE);
    } else {
      auto path_it {device_id_per_device_path.find(device_path)};
      if (path_it != std::end(device_id_per_device_path)) {
        // Devices with the same paths must have the same device ids.
        EXPECT_EQ(path_it->second, device_id);
      } else {
        EXPECT_TRUE(device_id_per_device_path.insert({device_path, device_id}).second);
      }
    }
  }
}

TEST_F_S(GetDeviceId, InvalidPath) {
  EXPECT_EQ(m_layer.getDeviceId(INVALID_PATH), std::string {});
}

TEST_F_S(GetMonitorDevicePath) {
  const auto all_devices {m_layer.queryDisplayConfig(display_device::QueryType::All)};
  ASSERT_TRUE(all_devices);

  std::set<std::string> current_device_paths;
  for (const auto &path : all_devices->m_paths) {
    const auto device_path {m_layer.getMonitorDevicePath(path)};
    const auto device_path_2 {m_layer.getMonitorDevicePath(path)};

    // Testing soft persistence - paths remain the same between calls
    EXPECT_EQ(device_path, device_path_2);

    if (device_path.empty()) {
      EXPECT_EQ(path.targetInfo.targetAvailable, FALSE);
    } else if (current_device_paths.contains(device_path)) {
      // In case we have a duplicate device path, the path must be inactive, because
      // active paths are always in the front of the list and therefore will be added
      // first (only 1 active path is possible).
      EXPECT_FALSE(static_cast<bool>(path.flags & DISPLAYCONFIG_PATH_ACTIVE));
    } else {
      EXPECT_TRUE(current_device_paths.insert(device_path).second);
    }
  }
}

TEST_F_S(GetMonitorDevicePath, InvalidPath) {
  EXPECT_EQ(m_layer.getMonitorDevicePath(INVALID_PATH), std::string {});
}

TEST_F_S(GetFriendlyName) {
  const auto all_devices {m_layer.queryDisplayConfig(display_device::QueryType::All)};
  ASSERT_TRUE(all_devices);

  for (const auto &path : all_devices->m_paths) {
    const auto friendly_name {m_layer.getFriendlyName(path)};
    const auto friendly_name_2 {m_layer.getFriendlyName(path)};

    // Testing soft persistence - ids remain the same between calls
    EXPECT_EQ(friendly_name, friendly_name_2);

    // We don't really have anything else to compare the friendly name against without going deep into EDID data.
    // Friendly name is also not mandatory in the EDID...
  }
}

TEST_F_S(GetFriendlyName, InvalidPath) {
  EXPECT_EQ(m_layer.getFriendlyName(INVALID_PATH), std::string {});
}

TEST_F_S(GetDisplayName) {
  const auto all_devices {m_layer.queryDisplayConfig(display_device::QueryType::All)};
  ASSERT_TRUE(all_devices);

  for (const auto &path : all_devices->m_paths) {
    const auto display_name {m_layer.getDisplayName(path)};
    const auto display_name_2 {m_layer.getDisplayName(path)};

    // Testing soft persistence - ids remain the same between calls
    EXPECT_EQ(display_name, display_name_2);

    // Display name is attached to the "source" mode (not a physical device) therefore a non-empty
    // value is always expected.
    EXPECT_TRUE(testRegex(display_name, R"(^\\\\.\\DISPLAY\d+$)"));
  }
}

TEST_F_S(GetDisplayName, InvalidPath) {
  EXPECT_EQ(m_layer.getDisplayName(INVALID_PATH), std::string {});
}

TEST_F_S(GetHdrState, InvalidPath) {
  EXPECT_EQ(m_layer.getHdrState(INVALID_PATH), std::nullopt);
}

TEST_F_S(SetHdrState, InvalidPath) {
  EXPECT_FALSE(m_layer.setHdrState(INVALID_PATH, display_device::HdrState::Enabled));
}

TEST_F_S(GetDisplayScale) {
  const auto active_devices {m_layer.queryDisplayConfig(display_device::QueryType::Active)};
  ASSERT_TRUE(active_devices);

  for (const auto &path : active_devices->m_paths) {
    const auto source_mode_index = path.sourceInfo.sourceModeInfoIdx;

    if (source_mode_index != DISPLAYCONFIG_PATH_SOURCE_MODE_IDX_INVALID) {
      ASSERT_TRUE(source_mode_index < active_devices->m_modes.size());
      ASSERT_EQ(active_devices->m_modes[source_mode_index].infoType, DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE);

      // Valid case
      {
        const auto scale {m_layer.getDisplayScale(m_layer.getDisplayName(path), active_devices->m_modes[source_mode_index].sourceMode)};
        EXPECT_TRUE(scale.has_value());  // Can't really compare against anything...
      }

      // Invalid display name case
      {
        const auto scale {m_layer.getDisplayScale("FAKE", active_devices->m_modes[source_mode_index].sourceMode)};
        EXPECT_FALSE(scale.has_value());
      }

      // Zero width case
      {
        auto mode {active_devices->m_modes[source_mode_index].sourceMode};
        mode.width = 0;

        const auto scale {m_layer.getDisplayScale(m_layer.getDisplayName(path), mode)};
        EXPECT_FALSE(scale.has_value());
      }
    }
  }
}
