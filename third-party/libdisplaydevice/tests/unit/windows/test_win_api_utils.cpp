// local includes
#include "display_device/windows/win_api_utils.h"
#include "fixtures/fixtures.h"
#include "utils/comparison.h"
#include "utils/mock_win_api_layer.h"

namespace {
  // Convenience keywords for GMock
  using ::testing::_;
  using ::testing::InSequence;
  using ::testing::Return;
  using ::testing::StrictMock;

  // Test fixture(s) for this file
  class WinApiUtilsMocked: public BaseTest {
  public:
    void setupExpectCallForValidPaths(int number_of_calls, InSequence & /* To ensure that sequence is created outside this scope */) {
      for (int i = 1; i <= number_of_calls; ++i) {
        EXPECT_CALL(m_layer, getMonitorDevicePath(_))
          .Times(1)
          .WillOnce(Return("Path" + std::to_string(i)))
          .RetiresOnSaturation();
        EXPECT_CALL(m_layer, getDeviceId(_))
          .Times(1)
          .WillOnce(Return("DeviceId" + std::to_string(i)))
          .RetiresOnSaturation();
        EXPECT_CALL(m_layer, getDisplayName(_))
          .Times(1)
          .WillOnce(Return("DisplayName" + std::to_string(i)))
          .RetiresOnSaturation();
      }
    }

    StrictMock<display_device::MockWinApiLayer> m_layer;
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, WinApiUtilsMocked, __VA_ARGS__)

  // Additional convenience global const(s)
  const DISPLAYCONFIG_PATH_INFO AVAILABLE_AND_ACTIVE_PATH {
    []() {
      DISPLAYCONFIG_PATH_INFO path;

      path.targetInfo.targetAvailable = TRUE;
      path.flags = DISPLAYCONFIG_PATH_ACTIVE;

      // Some arbitrary "valid" values for comparison after they have been set/reset.
      display_device::win_utils::setSourceIndex(path, 123);
      display_device::win_utils::setTargetIndex(path, 456);
      display_device::win_utils::setDesktopIndex(path, 789);
      display_device::win_utils::setCloneGroupId(path, std::nullopt);

      return path;
    }()
  };
  const DISPLAYCONFIG_PATH_INFO AVAILABLE_AND_INACTIVE_PATH {
    []() {
      DISPLAYCONFIG_PATH_INFO path;

      path.targetInfo.targetAvailable = TRUE;
      path.flags = ~DISPLAYCONFIG_PATH_ACTIVE;

      // Some arbitrary "valid" values for comparison after they have been set/reset.
      display_device::win_utils::setSourceIndex(path, std::nullopt);
      display_device::win_utils::setTargetIndex(path, std::nullopt);
      display_device::win_utils::setDesktopIndex(path, std::nullopt);
      display_device::win_utils::setCloneGroupId(path, std::nullopt);

      return path;
    }()
  };
  const std::vector<DISPLAYCONFIG_MODE_INFO> TARGET_AND_SOURCE_MODES {
    []() {
      std::vector<DISPLAYCONFIG_MODE_INFO> modes;

      modes.push_back({});
      modes.back().infoType = DISPLAYCONFIG_MODE_INFO_TYPE_TARGET;
      modes.back().targetMode = {};  // Set the union value

      modes.push_back({});
      modes.back().infoType = DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE;
      modes.back().sourceMode = {};  // Set the union value

      return modes;
    }()
  };
  const std::vector<DISPLAYCONFIG_PATH_INFO> PATHS_WITH_SOURCE_IDS {
    []() {
      // Contains the following:
      //    - 4 paths for the same adapter (1 active and 3 inactive)
      //        Note: source ids are out of order which is OK
      //    - 1 active path for a different adapter
      //    - 2 inactive path for yet another different adapter
      std::vector<DISPLAYCONFIG_PATH_INFO> paths;

      paths.push_back(AVAILABLE_AND_ACTIVE_PATH);
      paths.back().sourceInfo.adapterId = {1, 1};
      paths.back().sourceInfo.id = 1;

      paths.push_back(AVAILABLE_AND_ACTIVE_PATH);
      paths.back().sourceInfo.adapterId = {2, 2};
      paths.back().sourceInfo.id = 0;

      paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
      paths.back().sourceInfo.adapterId = {1, 1};
      paths.back().sourceInfo.id = 0;

      paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
      paths.back().sourceInfo.adapterId = {3, 3};
      paths.back().sourceInfo.id = 4;

      paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
      paths.back().sourceInfo.adapterId = {2, 2};
      paths.back().sourceInfo.id = 1;

      paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
      paths.back().sourceInfo.adapterId = {1, 1};
      paths.back().sourceInfo.id = 0;

      paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
      paths.back().sourceInfo.adapterId = {1, 1};
      paths.back().sourceInfo.id = 1;

      return paths;
    }()
  };
  const display_device::PathSourceIndexDataMap EXPECTED_SOURCE_INDEX_DATA {
    // Contains the expected data if generated from PATHS_WITH_SOURCE_IDS and some
    // sensibly chosen device paths and device ids.
    {"DeviceId1", {{{0, 2}, {1, 0}}, {1, 1}, {1}}},
    {"DeviceId2", {{{0, 1}, {1, 4}}, {2, 2}, {0}}},
    {"DeviceId3", {{{4, 3}}, {3, 3}, std::nullopt}},
    {"DeviceId4", {{{0, 5}, {1, 6}}, {1, 1}, std::nullopt}}
  };

  // Helper functions
  void wipeIndexesAndActivatePaths(std::vector<DISPLAYCONFIG_PATH_INFO> &paths) {
    for (auto &path : paths) {
      display_device::win_utils::setSourceIndex(path, std::nullopt);
      display_device::win_utils::setTargetIndex(path, std::nullopt);
      display_device::win_utils::setDesktopIndex(path, std::nullopt);
      display_device::win_utils::setActive(path);
    }
  }

}  // namespace

TEST_F_S_MOCKED(IsAvailable) {
  DISPLAYCONFIG_PATH_INFO available_path;
  DISPLAYCONFIG_PATH_INFO unavailable_path;

  available_path.targetInfo.targetAvailable = TRUE;
  unavailable_path.targetInfo.targetAvailable = FALSE;

  EXPECT_EQ(display_device::win_utils::isAvailable(available_path), true);
  EXPECT_EQ(display_device::win_utils::isAvailable(unavailable_path), false);
}

TEST_F_S_MOCKED(IsActiveAndSetActive) {
  DISPLAYCONFIG_PATH_INFO inactive_path;
  DISPLAYCONFIG_PATH_INFO only_active_path;
  DISPLAYCONFIG_PATH_INFO contains_active_path;

  inactive_path.flags = (~DISPLAYCONFIG_PATH_ACTIVE);
  only_active_path.flags = DISPLAYCONFIG_PATH_ACTIVE;
  contains_active_path.flags = inactive_path.flags | only_active_path.flags;

  EXPECT_EQ(display_device::win_utils::isActive(inactive_path), false);
  EXPECT_EQ(display_device::win_utils::isActive(only_active_path), true);
  EXPECT_EQ(display_device::win_utils::isActive(contains_active_path), true);

  display_device::win_utils::setActive(inactive_path);
  display_device::win_utils::setActive(only_active_path);
  display_device::win_utils::setActive(contains_active_path);

  EXPECT_EQ(display_device::win_utils::isActive(inactive_path), true);
  EXPECT_EQ(display_device::win_utils::isActive(only_active_path), true);
  EXPECT_EQ(display_device::win_utils::isActive(contains_active_path), true);
}

TEST_F_S_MOCKED(IsPrimary) {
  DISPLAYCONFIG_SOURCE_MODE primary_mode;
  DISPLAYCONFIG_SOURCE_MODE non_primary_mode_1;
  DISPLAYCONFIG_SOURCE_MODE non_primary_mode_2;

  primary_mode.position = {0, 0};
  non_primary_mode_1.position = {1, 0};
  non_primary_mode_2.position = {0, 2};

  EXPECT_TRUE(display_device::win_utils::isPrimary(primary_mode));
  EXPECT_FALSE(display_device::win_utils::isPrimary(non_primary_mode_1));
  EXPECT_FALSE(display_device::win_utils::isPrimary(non_primary_mode_2));
}

TEST_F_S_MOCKED(GetSourceIndex) {
  DISPLAYCONFIG_PATH_INFO path;
  std::vector<DISPLAYCONFIG_MODE_INFO> modes;

  path.sourceInfo.sourceModeInfoIdx = 1;
  modes.push_back({});  // Non-empty list
  modes.push_back({});  // Non-empty list

  EXPECT_EQ(display_device::win_utils::getSourceIndex(path, modes), std::make_optional<UINT32>(1));
}

TEST_F_S_MOCKED(GetSourceIndex, InvalidIndex) {
  DISPLAYCONFIG_PATH_INFO path;
  std::vector<DISPLAYCONFIG_MODE_INFO> modes;

  path.sourceInfo.sourceModeInfoIdx = DISPLAYCONFIG_PATH_SOURCE_MODE_IDX_INVALID;
  modes.push_back({});  // Non-empty list

  EXPECT_EQ(display_device::win_utils::getSourceIndex(path, modes), std::nullopt);
}

TEST_F_S_MOCKED(GetSourceIndex, OutOfRangeIndex) {
  DISPLAYCONFIG_PATH_INFO path;
  std::vector<DISPLAYCONFIG_MODE_INFO> modes;

  path.sourceInfo.sourceModeInfoIdx = 1;
  modes.push_back({});  // Non-empty list

  EXPECT_EQ(display_device::win_utils::getSourceIndex(path, modes), std::nullopt);
}

TEST_F_S_MOCKED(GetSourceIndex, EmptyList) {
  DISPLAYCONFIG_PATH_INFO path;
  std::vector<DISPLAYCONFIG_MODE_INFO> modes;

  path.sourceInfo.sourceModeInfoIdx = 0;

  EXPECT_EQ(display_device::win_utils::getSourceIndex(path, modes), std::nullopt);
}

TEST_F_S_MOCKED(SetSourceIndex) {
  DISPLAYCONFIG_PATH_INFO path;
  path.sourceInfo.sourceModeInfoIdx = 123;

  display_device::win_utils::setSourceIndex(path, 1);
  EXPECT_EQ(path.sourceInfo.sourceModeInfoIdx, 1);

  display_device::win_utils::setSourceIndex(path, std::nullopt);
  EXPECT_EQ(path.sourceInfo.sourceModeInfoIdx, DISPLAYCONFIG_PATH_SOURCE_MODE_IDX_INVALID);
}

TEST_F_S_MOCKED(SetTargetIndex) {
  DISPLAYCONFIG_PATH_INFO path;
  path.targetInfo.targetModeInfoIdx = 123;

  display_device::win_utils::setTargetIndex(path, 1);
  EXPECT_EQ(path.targetInfo.targetModeInfoIdx, 1);

  display_device::win_utils::setTargetIndex(path, std::nullopt);
  EXPECT_EQ(path.targetInfo.targetModeInfoIdx, DISPLAYCONFIG_PATH_TARGET_MODE_IDX_INVALID);
}

TEST_F_S_MOCKED(SetDesktopIndex) {
  DISPLAYCONFIG_PATH_INFO path;
  path.targetInfo.desktopModeInfoIdx = 123;

  display_device::win_utils::setDesktopIndex(path, 1);
  EXPECT_EQ(path.targetInfo.desktopModeInfoIdx, 1);

  display_device::win_utils::setDesktopIndex(path, std::nullopt);
  EXPECT_EQ(path.targetInfo.desktopModeInfoIdx, DISPLAYCONFIG_PATH_DESKTOP_IMAGE_IDX_INVALID);
}

TEST_F_S_MOCKED(SetCloneGroupId) {
  DISPLAYCONFIG_PATH_INFO path;
  path.sourceInfo.cloneGroupId = 123;

  display_device::win_utils::setCloneGroupId(path, 1);
  EXPECT_EQ(path.sourceInfo.cloneGroupId, 1);

  display_device::win_utils::setCloneGroupId(path, std::nullopt);
  EXPECT_EQ(path.sourceInfo.cloneGroupId, DISPLAYCONFIG_PATH_CLONE_GROUP_INVALID);
}

TEST_F_S_MOCKED(GetSourceMode) {
  auto *mode {display_device::win_utils::getSourceMode(1, TARGET_AND_SOURCE_MODES)};
  auto *const_mode {display_device::win_utils::getSourceMode(1, const_cast<const std::vector<DISPLAYCONFIG_MODE_INFO> &>(TARGET_AND_SOURCE_MODES))};

  EXPECT_EQ(mode, const_mode);
  EXPECT_EQ(mode, &TARGET_AND_SOURCE_MODES.at(1).sourceMode);
}

TEST_F_S_MOCKED(GetSourceMode, InvalidIndex) {
  auto *mode {display_device::win_utils::getSourceMode(std::nullopt, TARGET_AND_SOURCE_MODES)};
  auto *const_mode {display_device::win_utils::getSourceMode(std::nullopt, const_cast<const std::vector<DISPLAYCONFIG_MODE_INFO> &>(TARGET_AND_SOURCE_MODES))};

  EXPECT_EQ(mode, const_mode);
  EXPECT_EQ(mode, nullptr);
}

TEST_F_S_MOCKED(GetSourceMode, OutOfRangeIndex) {
  auto *mode {display_device::win_utils::getSourceMode(2, TARGET_AND_SOURCE_MODES)};
  auto *const_mode {display_device::win_utils::getSourceMode(2, const_cast<const std::vector<DISPLAYCONFIG_MODE_INFO> &>(TARGET_AND_SOURCE_MODES))};

  EXPECT_EQ(mode, const_mode);
  EXPECT_EQ(mode, nullptr);
}

TEST_F_S_MOCKED(GetSourceMode, InvalidModeType) {
  auto *mode {display_device::win_utils::getSourceMode(0, TARGET_AND_SOURCE_MODES)};
  auto *const_mode {display_device::win_utils::getSourceMode(0, const_cast<const std::vector<DISPLAYCONFIG_MODE_INFO> &>(TARGET_AND_SOURCE_MODES))};

  EXPECT_EQ(mode, const_mode);
  EXPECT_EQ(mode, nullptr);
}

TEST_F_S_MOCKED(GetSourceMode, EmptyList) {
  std::vector<DISPLAYCONFIG_MODE_INFO> modes;

  auto *mode {display_device::win_utils::getSourceMode(2, modes)};
  auto *const_mode {display_device::win_utils::getSourceMode(2, const_cast<const std::vector<DISPLAYCONFIG_MODE_INFO> &>(modes))};

  EXPECT_EQ(mode, const_mode);
  EXPECT_EQ(mode, nullptr);
}

TEST_F_S_MOCKED(GetDeviceInfo, AvailablePath, ActivePath, MustBeActiveIsTrue) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("DevicePath"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName"));

  const auto result {display_device::win_utils::getDeviceInfoForValidPath(m_layer, AVAILABLE_AND_ACTIVE_PATH, display_device::ValidatedPathType::Active)};
  ASSERT_TRUE(result);

  EXPECT_EQ(result->m_device_path, "DevicePath");
  EXPECT_EQ(result->m_device_id, "DeviceId");
}

TEST_F_S_MOCKED(GetDeviceInfo, AvailablePath, ActivePath, MustBeActiveIsFalse) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("DevicePath"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName"));

  const auto result {display_device::win_utils::getDeviceInfoForValidPath(m_layer, AVAILABLE_AND_ACTIVE_PATH, display_device::ValidatedPathType::Any)};
  ASSERT_TRUE(result);

  EXPECT_EQ(result->m_device_path, "DevicePath");
  EXPECT_EQ(result->m_device_id, "DeviceId");
}

TEST_F_S_MOCKED(GetDeviceInfo, AvailablePath, InactivePath, MustBeActiveIsTrue) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("DevicePath"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return(""));

  EXPECT_EQ(display_device::win_utils::getDeviceInfoForValidPath(m_layer, AVAILABLE_AND_ACTIVE_PATH, display_device::ValidatedPathType::Active), std::nullopt);
}

TEST_F_S_MOCKED(GetDeviceInfo, AvailablePath, InactivePath, MustBeActiveIsFalse) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("DevicePath"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName"));

  const auto result {display_device::win_utils::getDeviceInfoForValidPath(m_layer, AVAILABLE_AND_INACTIVE_PATH, display_device::ValidatedPathType::Any)};
  ASSERT_TRUE(result);

  EXPECT_EQ(result->m_device_path, "DevicePath");
  EXPECT_EQ(result->m_device_id, "DeviceId");
}

TEST_F_S_MOCKED(GetDeviceInfo, AvailablePath, ActivePath, EmptyDeviceId) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("DevicePath"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return(""));

  EXPECT_EQ(display_device::win_utils::getDeviceInfoForValidPath(m_layer, AVAILABLE_AND_ACTIVE_PATH, display_device::ValidatedPathType::Active), std::nullopt);
}

TEST_F_S_MOCKED(GetDeviceInfo, AvailablePath, ActivePath, EmptyDevicePath) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return(""));

  EXPECT_EQ(display_device::win_utils::getDeviceInfoForValidPath(m_layer, AVAILABLE_AND_ACTIVE_PATH, display_device::ValidatedPathType::Active), std::nullopt);
}

TEST_F_S_MOCKED(GetDeviceInfo, AvailablePath, InactivePath) {
  EXPECT_EQ(display_device::win_utils::getDeviceInfoForValidPath(m_layer, AVAILABLE_AND_INACTIVE_PATH, display_device::ValidatedPathType::Active), std::nullopt);
}

TEST_F_S_MOCKED(GetDeviceInfo, UnavailablePath, ActivePath) {
  DISPLAYCONFIG_PATH_INFO path;

  path.targetInfo.targetAvailable = FALSE;
  path.flags = DISPLAYCONFIG_PATH_ACTIVE;

  EXPECT_EQ(display_device::win_utils::getDeviceInfoForValidPath(m_layer, path, display_device::ValidatedPathType::Any), std::nullopt);
}

TEST_F_S_MOCKED(GetActivePath, InstantMatch) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(2)
    .WillRepeatedly(Return("Path1"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(2)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(2)
    .WillRepeatedly(Return("DeviceId1"));

  auto *path {display_device::win_utils::getActivePath(m_layer, "DeviceId1", const_cast<std::vector<DISPLAYCONFIG_PATH_INFO> &>(PATHS_WITH_SOURCE_IDS))};
  auto *const_path {display_device::win_utils::getActivePath(m_layer, "DeviceId1", PATHS_WITH_SOURCE_IDS)};

  EXPECT_EQ(path, const_path);
  EXPECT_EQ(path, &PATHS_WITH_SOURCE_IDS.at(0));
}

TEST_F_S_MOCKED(GetActivePath, SecondMatch) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(4)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(4)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(4)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"));

  auto *path {display_device::win_utils::getActivePath(m_layer, "DeviceId2", const_cast<std::vector<DISPLAYCONFIG_PATH_INFO> &>(PATHS_WITH_SOURCE_IDS))};
  auto *const_path {display_device::win_utils::getActivePath(m_layer, "DeviceId2", PATHS_WITH_SOURCE_IDS)};

  EXPECT_EQ(path, const_path);
  EXPECT_EQ(path, &PATHS_WITH_SOURCE_IDS.at(1));
}

TEST_F_S_MOCKED(GetActivePath, NoMatch) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(4)
    .WillOnce(Return(""));

  auto *path {display_device::win_utils::getActivePath(m_layer, "DeviceId1", const_cast<std::vector<DISPLAYCONFIG_PATH_INFO> &>(PATHS_WITH_SOURCE_IDS))};
  auto *const_path {display_device::win_utils::getActivePath(m_layer, "DeviceId1", PATHS_WITH_SOURCE_IDS)};

  EXPECT_EQ(path, const_path);
  EXPECT_EQ(path, nullptr);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(7)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path3"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path4"))
    .WillOnce(Return("Path4"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(7)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(7)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId3"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId4"))
    .WillOnce(Return("DeviceId4"));

  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, PATHS_WITH_SOURCE_IDS), EXPECTED_SOURCE_INDEX_DATA);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, TransientPathIssues) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(7)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path1"))
    .WillOnce(Return(""))  // Path is not available for some reason
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path4"))
    .WillOnce(Return("Path4"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(6)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(6)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId4"))
    .WillOnce(Return("DeviceId4"));

  display_device::PathSourceIndexDataMap expected_data {EXPECTED_SOURCE_INDEX_DATA};
  expected_data.erase(expected_data.find("DeviceId3"));

  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, PATHS_WITH_SOURCE_IDS), expected_data);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, DuplicatePathsWithDifferentIds) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(2)
    .WillRepeatedly(Return("PathSame"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(2)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(2)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"));

  const display_device::PathSourceIndexDataMap expected_data {};
  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, PATHS_WITH_SOURCE_IDS), expected_data);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, DifferentPathsWithSameId) {
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(3)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path3"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(3)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(3)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"));

  const display_device::PathSourceIndexDataMap expected_data {};
  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, PATHS_WITH_SOURCE_IDS), expected_data);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, MismatchingAdapterIdsForPaths, HighPart) {
  std::vector<DISPLAYCONFIG_PATH_INFO> paths {PATHS_WITH_SOURCE_IDS};

  // Invalidate the adapter id
  paths.at(2).sourceInfo.adapterId.HighPart++;

  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(3)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path1"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(3)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(3)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"));

  const display_device::PathSourceIndexDataMap expected_data {};
  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, paths), expected_data);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, MismatchingAdapterIdsForPaths, LowPart) {
  std::vector<DISPLAYCONFIG_PATH_INFO> paths {PATHS_WITH_SOURCE_IDS};

  // Invalidate the adapter id
  paths.at(2).sourceInfo.adapterId.LowPart++;

  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(3)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path1"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(3)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(3)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"));

  const display_device::PathSourceIndexDataMap expected_data {};
  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, paths), expected_data);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, ActiveDeviceNotFirst) {
  std::vector<DISPLAYCONFIG_PATH_INFO> paths {PATHS_WITH_SOURCE_IDS};

  // Swapping around the active/inactive pair
  std::swap(paths.at(0), paths.at(2));

  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(3)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path1"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(3)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(3)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"));

  const display_device::PathSourceIndexDataMap expected_data {};
  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, paths), expected_data);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, DuplicateSourceIds) {
  std::vector<DISPLAYCONFIG_PATH_INFO> paths {PATHS_WITH_SOURCE_IDS};

  // Making sure source ids match (adapter ids don't matter)
  paths.at(0).sourceInfo.id = paths.at(2).sourceInfo.id;

  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(3)
    .WillOnce(Return("Path1"))
    .WillOnce(Return("Path2"))
    .WillOnce(Return("Path1"));
  EXPECT_CALL(m_layer, getDisplayName(_))
    .Times(3)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(m_layer, getDeviceId(_))
    .Times(3)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId1"));

  const display_device::PathSourceIndexDataMap expected_data {};
  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, paths), expected_data);
}

TEST_F_S_MOCKED(CollectSourceDataForMatchingPaths, EmptyList) {
  std::vector<DISPLAYCONFIG_PATH_INFO> paths;

  const display_device::PathSourceIndexDataMap expected_data {};
  EXPECT_EQ(display_device::win_utils::collectSourceDataForMatchingPaths(m_layer, paths), expected_data);
}

TEST_F_S_MOCKED(MakePathsForNewTopology) {
  const display_device::ActiveTopology new_topology {{"DeviceId1"}, {"DeviceId2"}, {"DeviceId3", "DeviceId4"}};
  const std::vector<DISPLAYCONFIG_PATH_INFO> paths {PATHS_WITH_SOURCE_IDS};

  std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {{paths.at(0), paths.at(1), paths.at(3), paths.at(5)}};
  wipeIndexesAndActivatePaths(expected_paths);

  display_device::win_utils::setCloneGroupId(expected_paths.at(0), 0);
  display_device::win_utils::setCloneGroupId(expected_paths.at(1), 1);
  display_device::win_utils::setCloneGroupId(expected_paths.at(2), 2);
  display_device::win_utils::setCloneGroupId(expected_paths.at(3), 2);

  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, EXPECTED_SOURCE_INDEX_DATA, paths), expected_paths);
}

TEST_F_S_MOCKED(MakePathsForNewTopology, DevicesFromSameAdapterInAGroup) {
  const display_device::ActiveTopology new_topology {{"DeviceId1", "DeviceId4"}};
  const std::vector<DISPLAYCONFIG_PATH_INFO> paths {PATHS_WITH_SOURCE_IDS};

  std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {paths.at(0), paths.at(6)};
  wipeIndexesAndActivatePaths(expected_paths);

  display_device::win_utils::setCloneGroupId(expected_paths.at(0), 0);
  display_device::win_utils::setCloneGroupId(expected_paths.at(1), 0);

  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, EXPECTED_SOURCE_INDEX_DATA, paths), expected_paths);
}

TEST_F_S_MOCKED(MakePathsForNewTopology, UnknownDeviceInNewTopology) {
  const display_device::ActiveTopology new_topology {{"DeviceIdX", "DeviceId4"}};

  const std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {};
  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, EXPECTED_SOURCE_INDEX_DATA, PATHS_WITH_SOURCE_IDS), expected_paths);
}

TEST_F_S_MOCKED(MakePathsForNewTopology, MissingPathsForDuplicatedDisplays) {
  // There must be N-1 (up to a GPU limit) amount of source ids (for each path/deviceId combination) available.
  // For the same adapter, only devices with matching ids can be grouped (duplicated).
  // In this case, have only 0 and 1 ids. You may also notice that 0 != 1, and thus we cannot group them.
  const display_device::ActiveTopology new_topology {{"DeviceId1", "DeviceId2"}};
  std::vector<DISPLAYCONFIG_PATH_INFO> paths {};

  paths.push_back(AVAILABLE_AND_ACTIVE_PATH);
  paths.back().sourceInfo.adapterId = {1, 1};
  paths.back().sourceInfo.id = 0;

  paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
  paths.back().sourceInfo.adapterId = {1, 1};
  paths.back().sourceInfo.id = 1;

  const display_device::PathSourceIndexDataMap path_source_data {
    {"DeviceId1", {{{0, 0}}, {1, 1}, {0}}},
    {"DeviceId2", {{{1, 1}}, {1, 1}, std::nullopt}}
  };

  const std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {};
  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, path_source_data, paths), expected_paths);
}

TEST_F_S_MOCKED(MakePathsForNewTopology, GpuLimit, DuplicatedDisplays) {
  // We can only render 1 source, however since they are duplicated, source is reused
  // and can be rendered to different devices.
  const display_device::ActiveTopology new_topology {{"DeviceId1", "DeviceId2"}};
  std::vector<DISPLAYCONFIG_PATH_INFO> paths {};

  paths.push_back(AVAILABLE_AND_ACTIVE_PATH);
  paths.back().sourceInfo.adapterId = {1, 1};
  paths.back().sourceInfo.id = 0;

  paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
  paths.back().sourceInfo.adapterId = {1, 1};
  paths.back().sourceInfo.id = 0;

  const display_device::PathSourceIndexDataMap path_source_data {
    {"DeviceId1", {{{0, 0}}, {1, 1}, {0}}},
    {"DeviceId2", {{{0, 1}}, {1, 1}, std::nullopt}}
  };

  std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {paths.at(0), paths.at(1)};
  wipeIndexesAndActivatePaths(expected_paths);

  display_device::win_utils::setCloneGroupId(expected_paths.at(0), 0);
  display_device::win_utils::setCloneGroupId(expected_paths.at(1), 0);

  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, path_source_data, paths), expected_paths);
}

TEST_F_S_MOCKED(MakePathsForNewTopology, GpuLimit, ExtendedDisplays) {
  // We can only render 1 source and since want extended displays, we must have 2 and that's impossible.
  const display_device::ActiveTopology new_topology {{"DeviceId1"}, {"DeviceId2"}};
  std::vector<DISPLAYCONFIG_PATH_INFO> paths {};

  paths.push_back(AVAILABLE_AND_ACTIVE_PATH);
  paths.back().sourceInfo.adapterId = {1, 1};
  paths.back().sourceInfo.id = 0;

  paths.push_back(AVAILABLE_AND_INACTIVE_PATH);
  paths.back().sourceInfo.adapterId = {1, 1};
  paths.back().sourceInfo.id = 0;

  const display_device::PathSourceIndexDataMap path_source_data {
    {"DeviceId1", {{{0, 0}}, {1, 1}, {0}}},
    {"DeviceId2", {{{0, 1}}, {1, 1}, std::nullopt}}
  };

  const std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {};
  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, path_source_data, paths), expected_paths);
}

TEST_F_S_MOCKED(MakePathsForNewTopology, IndexOutOfRange) {
  const display_device::ActiveTopology new_topology {{"DeviceId1", "DeviceId4"}};

  const std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {};
  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, EXPECTED_SOURCE_INDEX_DATA, {}), expected_paths);
}

TEST_F_S_MOCKED(MakePathsForNewTopology, EmptyList) {
  const display_device::ActiveTopology new_topology {};

  const std::vector<DISPLAYCONFIG_PATH_INFO> expected_paths {};
  EXPECT_EQ(display_device::win_utils::makePathsForNewTopology(new_topology, EXPECTED_SOURCE_INDEX_DATA, PATHS_WITH_SOURCE_IDS), expected_paths);
}

TEST_F_S_MOCKED(GetAllDeviceIdsAndMatchingDuplicates) {
  InSequence sequence;
  EXPECT_CALL(m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));

  // DeviceId1 iterations
  {
    // Outer loop
    setupExpectCallForValidPaths(1, sequence);

    // Inner loop
    setupExpectCallForValidPaths(4, sequence);
  }

  // DeviceId2 iterations
  {
    // Outer loop
    setupExpectCallForValidPaths(2, sequence);

    // Inner loop
    setupExpectCallForValidPaths(4, sequence);
  }

  EXPECT_EQ(display_device::win_utils::getAllDeviceIdsAndMatchingDuplicates(m_layer, {"DeviceId1", "DeviceId2"}), (std::set<std::string> {"DeviceId1", "DeviceId2", "DeviceId3"}));
}

TEST_F_S_MOCKED(GetAllDeviceIdsAndMatchingDuplicates, FailedToQueryDevices) {
  EXPECT_CALL(m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  EXPECT_EQ(display_device::win_utils::getAllDeviceIdsAndMatchingDuplicates(m_layer, {"DeviceId2"}), std::set<std::string> {});
}

TEST_F_S_MOCKED(GetAllDeviceIdsAndMatchingDuplicates, EmptyDeviceIdInProvidedList) {
  //  InSequence sequence;
  //  setupExpectCallFor4ActivePathsAndModes(display_device::QueryType::Active, sequence);
  EXPECT_CALL(m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));

  EXPECT_EQ(display_device::win_utils::getAllDeviceIdsAndMatchingDuplicates(m_layer, {""}), std::set<std::string> {});
}

TEST_F_S_MOCKED(GetAllDeviceIdsAndMatchingDuplicates, FailedToFindActivePath) {
  EXPECT_CALL(m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(4)
    .WillOnce(Return(""));

  EXPECT_EQ(display_device::win_utils::getAllDeviceIdsAndMatchingDuplicates(m_layer, {"DeviceId2"}), std::set<std::string> {});
}

TEST_F_S_MOCKED(GetAllDeviceIdsAndMatchingDuplicates, NoSourceModeFound) {
  auto pam_no_modes {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  pam_no_modes->m_modes.clear();

  InSequence sequence;
  EXPECT_CALL(m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_no_modes));
  setupExpectCallForValidPaths(2, sequence);

  EXPECT_EQ(display_device::win_utils::getAllDeviceIdsAndMatchingDuplicates(m_layer, {"DeviceId2"}), std::set<std::string> {});
}

TEST_F_S_MOCKED(GetAllDeviceIdsAndMatchingDuplicates, IncompleteListOfSources) {
  auto pam_no_modes {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  pam_no_modes->m_modes.resize(2);

  InSequence sequence;
  EXPECT_CALL(m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_no_modes));
  setupExpectCallForValidPaths(2, sequence);
  EXPECT_CALL(m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return(""))
    .RetiresOnSaturation();
  setupExpectCallForValidPaths(1, sequence);

  EXPECT_EQ(display_device::win_utils::getAllDeviceIdsAndMatchingDuplicates(m_layer, {"DeviceId1"}), std::set<std::string> {});
}

TEST_F_S_MOCKED(FuzzyCompareRefreshRates) {
  EXPECT_EQ(display_device::win_utils::fuzzyCompareRefreshRates(display_device::Rational {60, 1}, display_device::Rational {5985, 100}), true);
  EXPECT_EQ(display_device::win_utils::fuzzyCompareRefreshRates(display_device::Rational {60, 1}, display_device::Rational {5920, 100}), true);
  EXPECT_EQ(display_device::win_utils::fuzzyCompareRefreshRates(display_device::Rational {60, 1}, display_device::Rational {5900, 100}), false);
  EXPECT_EQ(display_device::win_utils::fuzzyCompareRefreshRates(display_device::Rational {60, 0}, display_device::Rational {5985, 100}), false);
  EXPECT_EQ(display_device::win_utils::fuzzyCompareRefreshRates(display_device::Rational {60, 1}, display_device::Rational {5985, 0}), false);
}

TEST_F_S_MOCKED(FuzzyCompareModes) {
  EXPECT_EQ(display_device::win_utils::fuzzyCompareModes(display_device::DisplayMode {1920, 1080, {60, 1}}, display_device::DisplayMode {1920, 1080, {60, 1}}), true);
  EXPECT_EQ(display_device::win_utils::fuzzyCompareModes(display_device::DisplayMode {123, 1080, {60, 1}}, display_device::DisplayMode {1920, 1080, {60, 1}}), false);
  EXPECT_EQ(display_device::win_utils::fuzzyCompareModes(display_device::DisplayMode {1920, 123, {60, 1}}, display_device::DisplayMode {1920, 1080, {60, 1}}), false);
  EXPECT_EQ(display_device::win_utils::fuzzyCompareModes(display_device::DisplayMode {1920, 1080, {60, 1}}, display_device::DisplayMode {1920, 1080, {50, 1}}), false);
}
