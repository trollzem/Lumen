// system includes
#include <ranges>

// local includes
#include "display_device/windows/settings_utils.h"
#include "display_device/windows/win_api_layer.h"
#include "display_device/windows/win_api_utils.h"
#include "display_device/windows/win_display_device.h"
#include "fixtures/fixtures.h"
#include "utils/comparison.h"
#include "utils/guards.h"
#include "utils/mock_win_api_layer.h"

namespace {
  // Convenience keywords for GMock
  using ::testing::_;
  using ::testing::InSequence;
  using ::testing::Return;
  using ::testing::StrictMock;

  // Test fixture(s) for this file
  class WinDisplayDeviceModes: public BaseTest {
  public:
    bool isSystemTest() const override {
      return true;
    }

    std::shared_ptr<display_device::WinApiLayer> m_layer {std::make_shared<display_device::WinApiLayer>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  class WinDisplayDeviceModesMocked: public BaseTest {
  public:
    void setupExpectedGetActivePathCall(int id_number, InSequence & /* To ensure that sequence is created outside this scope */) {
      for (int i = 1; i <= id_number; ++i) {
        EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
          .Times(1)
          .WillOnce(Return("PathX"))
          .RetiresOnSaturation();
        EXPECT_CALL(*m_layer, getDeviceId(_))
          .Times(1)
          .WillOnce(Return("DeviceId" + std::to_string(i)))
          .RetiresOnSaturation();
        EXPECT_CALL(*m_layer, getDisplayName(_))
          .Times(1)
          .WillOnce(Return("DisplayNameX"))
          .RetiresOnSaturation();
      }
    }

    void setupExpectedGetCurrentDisplayModesCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::optional<display_device::PathAndModeData> &pam = ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES) {
      EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
        .Times(1)
        .WillOnce(Return(pam))
        .RetiresOnSaturation();

      for (int i = 1; i <= 4; ++i) {
        setupExpectedGetActivePathCall(i, sequence);
      }
    }

    void setupExpectedGetAllDeviceIdsCall(InSequence & /* To ensure that sequence is created outside this scope */, const std::set<int> &entries = {1, 2, 3, 4}) {
      EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
        .Times(1)
        .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
        .RetiresOnSaturation();

      for (const auto entry : entries) {
        for (int i = 1; i <= entry; ++i) {
          EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
            .Times(1)
            .WillOnce(Return("Path" + std::to_string(i)))
            .RetiresOnSaturation();
          EXPECT_CALL(*m_layer, getDeviceId(_))
            .Times(1)
            .WillOnce(Return("DeviceId" + std::to_string(i)))
            .RetiresOnSaturation();
          EXPECT_CALL(*m_layer, getDisplayName(_))
            .Times(1)
            .WillOnce(Return("DisplayName" + std::to_string(i)))
            .RetiresOnSaturation();
        }

        for (int i = 1; i <= 4; ++i) {
          EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
            .Times(1)
            .WillOnce(Return("Path" + std::to_string(i)))
            .RetiresOnSaturation();
          EXPECT_CALL(*m_layer, getDeviceId(_))
            .Times(1)
            .WillOnce(Return("DeviceId" + std::to_string(i)))
            .RetiresOnSaturation();
          EXPECT_CALL(*m_layer, getDisplayName(_))
            .Times(1)
            .WillOnce(Return("DisplayName" + std::to_string(i)))
            .RetiresOnSaturation();
        }
      }
    }

    std::shared_ptr<StrictMock<display_device::MockWinApiLayer>> m_layer {std::make_shared<StrictMock<display_device::MockWinApiLayer>>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, WinDisplayDeviceModes, __VA_ARGS__)
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, WinDisplayDeviceModesMocked, __VA_ARGS__)

  // Additional convenience global const(s)
  const UINT32 RELAXED_FLAGS {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE | SDC_ALLOW_CHANGES};
  const UINT32 STRICT_FLAGS {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE};
  const UINT32 UNDO_FLAGS {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE};

  // Helper functions
  std::optional<display_device::PathAndModeData> applyExpectedModesOntoInput(std::optional<display_device::PathAndModeData> input, const display_device::DeviceDisplayModeMap &modes, const std::set<std::string> &excluded_ids = {}) {
    if (!input) {
      return std::nullopt;
    }

    for (const auto &[device_id, mode] : modes) {
      if (excluded_ids.contains(device_id)) {
        continue;
      }

      auto path_index {std::stoi(device_id.substr(device_id.size() - 1, 1)) - 1};
      input->m_paths.at(path_index).targetInfo.refreshRate = {mode.m_refresh_rate.m_numerator, mode.m_refresh_rate.m_denominator};
      input->m_modes.at(input->m_paths.at(path_index).sourceInfo.sourceModeInfoIdx).sourceMode.width = mode.m_resolution.m_width;
      input->m_modes.at(input->m_paths.at(path_index).sourceInfo.sourceModeInfoIdx).sourceMode.height = mode.m_resolution.m_height;

      display_device::win_utils::setTargetIndex(input->m_paths.at(path_index), std::nullopt);
      display_device::win_utils::setDesktopIndex(input->m_paths.at(path_index), std::nullopt);
    }

    return input;
  }

  display_device::DisplayMode getTestMode(const int number) {
    if (number == 1) {
      return {1024, 768, {60, 1}};
    }

    return {1920, 1080, {60, 1}};
  }
}  // namespace

TEST_F_S(GetCurrentDisplayModes) {
  const auto flattened_topology {display_device::win_utils::flattenTopology(m_win_dd.getCurrentTopology())};
  const auto current_modes {m_win_dd.getCurrentDisplayModes(flattened_topology)};

  // Can't really compare anything else without knowing system specs
  const auto mode_keys_view {std::ranges::views::keys(current_modes)};
  const std::set<std::string> mode_keys {std::begin(mode_keys_view), std::end(mode_keys_view)};
  EXPECT_EQ(flattened_topology, mode_keys);
}

TEST_F_S(SetCurrentDisplayModes, ExtendedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 2) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto topology_guard {makeTopologyGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setTopology({{available_devices->at(0)}, {available_devices->at(1)}}));

  const display_device::DeviceDisplayModeMap mixed_modes_1 {
    {available_devices->at(0), getTestMode(0)},
    {available_devices->at(1), getTestMode(1)}
  };
  const display_device::DeviceDisplayModeMap mixed_modes_2 {
    {available_devices->at(0), getTestMode(1)},
    {available_devices->at(1), getTestMode(0)}
  };

  const auto mode_guard {makeModeGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setDisplayModes(mixed_modes_1));
  ASSERT_TRUE(m_win_dd.setDisplayModes(mixed_modes_2));
}

TEST_F_S(SetCurrentDisplayModes, DuplicatedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 2) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto topology_guard {makeTopologyGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setTopology({{available_devices->at(0), available_devices->at(1)}}));

  const display_device::DeviceDisplayModeMap same_modes_1 {
    {available_devices->at(0), getTestMode(1)},
    {available_devices->at(1), getTestMode(1)}
  };
  const display_device::DeviceDisplayModeMap same_modes_2 {
    {available_devices->at(0), getTestMode(0)},
    {available_devices->at(1), getTestMode(0)}
  };
  const display_device::DeviceDisplayModeMap mixed_modes {
    {available_devices->at(0), getTestMode(1)},
    {available_devices->at(1), getTestMode(0)}
  };

  const auto mode_guard {makeModeGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setDisplayModes(same_modes_1));
  ASSERT_TRUE(m_win_dd.setDisplayModes(same_modes_2));
  ASSERT_FALSE(m_win_dd.setDisplayModes(mixed_modes));
}

TEST_F_S(SetCurrentDisplayModes, MixedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 3) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto topology_guard {makeTopologyGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setTopology({{available_devices->at(0)}, {available_devices->at(1), available_devices->at(2)}}));

  const display_device::DeviceDisplayModeMap mixed_modes_1 {
    {available_devices->at(0), getTestMode(0)},
    {available_devices->at(1), getTestMode(1)},
    {available_devices->at(2), getTestMode(1)}
  };
  const display_device::DeviceDisplayModeMap mixed_modes_2 {
    {available_devices->at(0), getTestMode(1)},
    {available_devices->at(1), getTestMode(0)},
    {available_devices->at(2), getTestMode(0)}
  };

  const auto mode_guard {makeModeGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setDisplayModes(mixed_modes_1));
  ASSERT_TRUE(m_win_dd.setDisplayModes(mixed_modes_2));
}

TEST_F_S_MOCKED(GetCurrentDisplayModes) {
  InSequence sequence;
  setupExpectedGetCurrentDisplayModesCall(sequence);

  const auto current_modes {m_win_dd.getCurrentDisplayModes({"DeviceId1", "DeviceId2", "DeviceId3", "DeviceId4"})};
  const display_device::DeviceDisplayModeMap expected_modes {
    {"DeviceId1", {1920, 1080, {120, 1}}},
    {"DeviceId2", {1920, 2160, {119995, 1000}}},
    {"DeviceId3", {1920, 2160, {60, 1}}},
    {"DeviceId4", {3840, 2160, {90, 1}}},
  };
  EXPECT_EQ(current_modes, expected_modes);
}

TEST_F_S_MOCKED(GetCurrentDisplayModes, EmptyIdList) {
  const auto current_modes {m_win_dd.getCurrentDisplayModes({})};
  EXPECT_TRUE(current_modes.empty());
}

TEST_F_S_MOCKED(GetCurrentDisplayModes, NoDisplayData) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  const auto current_modes {m_win_dd.getCurrentDisplayModes({"DeviceId1"})};
  EXPECT_TRUE(current_modes.empty());
}

TEST_F_S_MOCKED(GetCurrentDisplayModes, EmptyDeviceId) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));

  const auto current_modes {m_win_dd.getCurrentDisplayModes({"", "DeviceId2"})};
  EXPECT_TRUE(current_modes.empty());
}

TEST_F_S_MOCKED(GetCurrentDisplayModes, FailedToGetActivePath) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY));

  const auto current_modes {m_win_dd.getCurrentDisplayModes({"DeviceId1"})};
  EXPECT_TRUE(current_modes.empty());
}

TEST_F_S_MOCKED(GetCurrentDisplayModes, FailedToGetSourceMode) {
  auto pam_no_modes {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  pam_no_modes->m_modes.clear();

  EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillRepeatedly(Return("PathX"));
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(1)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId1"));
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_no_modes));

  const auto current_modes {m_win_dd.getCurrentDisplayModes({"DeviceId1"})};
  EXPECT_TRUE(current_modes.empty());
}

TEST_F_S_MOCKED(SetDisplayModes, Relaxed) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 1}}},
    {"DeviceId2", {1920, 1000, {144, 1}}},
    {"DeviceId3", {1000, 1000, {90, 1}}},
    {"DeviceId4", {1000, 2160, {90, 10}}},
  };

  const auto pam_initial {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  const auto pam_submitted {applyExpectedModesOntoInput(pam_initial, new_modes, {"DeviceId1"})};

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();
  setupExpectedGetActivePathCall(1, sequence);
  setupExpectedGetActivePathCall(2, sequence);
  setupExpectedGetActivePathCall(3, sequence);
  setupExpectedGetActivePathCall(4, sequence);

  EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, RELAXED_FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS))
    .RetiresOnSaturation();
  setupExpectedGetCurrentDisplayModesCall(sequence, pam_submitted);

  EXPECT_TRUE(m_win_dd.setDisplayModes(new_modes));
}

TEST_F_S_MOCKED(SetDisplayModes, Strict) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 10}}},
    {"DeviceId2", {1000, 2160, {119995, 100}}},
    {"DeviceId3", {1000, 1000, {90, 1}}},
    {"DeviceId4", {3840, 2160, {90, 1}}},
  };

  const auto pam_initial {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  const auto pam_submitted {applyExpectedModesOntoInput(pam_initial, new_modes, {"DeviceId4"})};

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();

  // Relaxed try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, RELAXED_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
    setupExpectedGetCurrentDisplayModesCall(sequence, pam_initial);
  }

  // Strict try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, STRICT_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
    setupExpectedGetCurrentDisplayModesCall(sequence, pam_submitted);
  }

  EXPECT_TRUE(m_win_dd.setDisplayModes(new_modes));
}

TEST_F_S_MOCKED(SetDisplayModes, NoChanges) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 1}}},
    {"DeviceId2", {1920, 2160, {119995, 1000}}},
    {"DeviceId3", {1920, 2160, {60, 1}}},
    {"DeviceId4", {3840, 2160, {90, 1}}},
  };

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
    .RetiresOnSaturation();
  setupExpectedGetActivePathCall(1, sequence);
  setupExpectedGetActivePathCall(2, sequence);
  setupExpectedGetActivePathCall(3, sequence);
  setupExpectedGetActivePathCall(4, sequence);
  setupExpectedGetCurrentDisplayModesCall(sequence, ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES);

  EXPECT_TRUE(m_win_dd.setDisplayModes(new_modes));
}

TEST_F_S_MOCKED(SetDisplayModes, EmptyModeMap) {
  EXPECT_FALSE(m_win_dd.setDisplayModes({}));
}

TEST_F_S_MOCKED(SetDisplayModes, FailedToGetDuplicateDevices) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setDisplayModes({{"DeviceId1", {}}}));
}

TEST_F_S_MOCKED(SetDisplayModes, MissingDuplicateDisplayModes) {
  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence, {2});

  EXPECT_FALSE(m_win_dd.setDisplayModes({{"DeviceId2", {}}}));
}

TEST_F_S_MOCKED(SetDisplayModes, FailedToGetOriginalData) {
  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence, {1});
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setDisplayModes({{"DeviceId1", {}}}));
}

TEST_F_S_MOCKED(SetDisplayModes, DoSetModes, FailedToGetDisplayConfig) {
  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence, {1});
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setDisplayModes({{"DeviceId1", {}}}));
}

TEST_F_S_MOCKED(SetDisplayModes, DoSetModes, EmptyListFromGetDisplayConfig) {
  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence, {1});
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setDisplayModes({{"DeviceId1", {}}}));
}

TEST_F_S_MOCKED(SetDisplayModes, DoSetModes, FailedToGetActivePath) {
  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence, {1});
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
    .Times(4)
    .WillOnce(Return(""))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setDisplayModes({{"DeviceId1", {}}}));
}

TEST_F_S_MOCKED(SetDisplayModes, DoSetModes, FailedToGetSourceMode) {
  auto pam_no_modes {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  pam_no_modes->m_modes.clear();

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence, {1});
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_no_modes))
    .RetiresOnSaturation();
  setupExpectedGetActivePathCall(1, sequence);

  EXPECT_FALSE(m_win_dd.setDisplayModes({{"DeviceId1", {}}}));
}

TEST_F_S_MOCKED(SetDisplayModes, Relaxed, FailedToSetDisplayConfig) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 1}}},
    {"DeviceId2", {1920, 1000, {144, 1}}},
    {"DeviceId3", {1000, 1000, {90, 1}}},
    {"DeviceId4", {1000, 2160, {90, 10}}},
  };

  const auto pam_initial {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  const auto pam_submitted {applyExpectedModesOntoInput(pam_initial, new_modes, {"DeviceId1"})};

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();
  setupExpectedGetActivePathCall(1, sequence);
  setupExpectedGetActivePathCall(2, sequence);
  setupExpectedGetActivePathCall(3, sequence);
  setupExpectedGetActivePathCall(4, sequence);

  EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, RELAXED_FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_ACCESS_DENIED))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getErrorString(ERROR_ACCESS_DENIED))
    .Times(1)
    .WillRepeatedly(Return("ErrorDesc"))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setDisplayModes(new_modes));
}

TEST_F_S_MOCKED(SetDisplayModes, Relaxed, FailedToGetCurrentDisplayModes) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 1}}},
    {"DeviceId2", {1920, 1000, {144, 1}}},
    {"DeviceId3", {1000, 1000, {90, 1}}},
    {"DeviceId4", {1000, 2160, {90, 10}}},
  };

  const auto pam_initial {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  const auto pam_submitted {applyExpectedModesOntoInput(pam_initial, new_modes, {"DeviceId1"})};

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();
  setupExpectedGetActivePathCall(1, sequence);
  setupExpectedGetActivePathCall(2, sequence);
  setupExpectedGetActivePathCall(3, sequence);
  setupExpectedGetActivePathCall(4, sequence);

  EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, RELAXED_FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, setDisplayConfig(pam_initial->m_paths, pam_initial->m_modes, UNDO_FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setDisplayModes(new_modes));
}

TEST_F_S_MOCKED(SetDisplayModes, Strict, FailedToSetDisplayConfig) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 10}}},
    {"DeviceId2", {1000, 2160, {119995, 100}}},
    {"DeviceId3", {1000, 1000, {90, 1}}},
    {"DeviceId4", {3840, 2160, {90, 1}}},
  };

  const auto pam_initial {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  const auto pam_submitted {applyExpectedModesOntoInput(pam_initial, new_modes, {"DeviceId4"})};

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();

  // Relaxed try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, RELAXED_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
    setupExpectedGetCurrentDisplayModesCall(sequence, pam_initial);
  }

  // Strict try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, STRICT_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_ACCESS_DENIED))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, getErrorString(ERROR_ACCESS_DENIED))
      .Times(1)
      .WillRepeatedly(Return("ErrorDesc"))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, setDisplayConfig(pam_initial->m_paths, pam_initial->m_modes, UNDO_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
  }

  EXPECT_FALSE(m_win_dd.setDisplayModes(new_modes));
}

TEST_F_S_MOCKED(SetDisplayModes, Strict, FailedToGetCurrentDisplayModes) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 10}}},
    {"DeviceId2", {1000, 2160, {119995, 100}}},
    {"DeviceId3", {1000, 1000, {90, 1}}},
    {"DeviceId4", {3840, 2160, {90, 1}}},
  };

  const auto pam_initial {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  const auto pam_submitted {applyExpectedModesOntoInput(pam_initial, new_modes, {"DeviceId4"})};

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();

  // Relaxed try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, RELAXED_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
    setupExpectedGetCurrentDisplayModesCall(sequence, pam_initial);
  }

  // Strict try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, STRICT_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(ut_consts::PAM_NULL))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, setDisplayConfig(pam_initial->m_paths, pam_initial->m_modes, UNDO_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
  }

  EXPECT_FALSE(m_win_dd.setDisplayModes(new_modes));
}

TEST_F_S_MOCKED(SetDisplayModes, Strict, ModesDidNotChange) {
  const display_device::DeviceDisplayModeMap new_modes {
    {"DeviceId1", {1920, 1080, {120, 10}}},
    {"DeviceId2", {1000, 2160, {119995, 100}}},
    {"DeviceId3", {1000, 1000, {90, 1}}},
    {"DeviceId4", {3840, 2160, {90, 1}}},
  };

  const auto pam_initial {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  const auto pam_submitted {applyExpectedModesOntoInput(pam_initial, new_modes, {"DeviceId4"})};

  InSequence sequence;
  setupExpectedGetAllDeviceIdsCall(sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_initial))
    .RetiresOnSaturation();

  // Relaxed try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, RELAXED_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
    setupExpectedGetCurrentDisplayModesCall(sequence, pam_initial);
  }

  // Strict try
  {
    EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
      .Times(1)
      .WillOnce(Return(pam_initial))
      .RetiresOnSaturation();
    setupExpectedGetActivePathCall(1, sequence);
    setupExpectedGetActivePathCall(2, sequence);
    setupExpectedGetActivePathCall(3, sequence);
    setupExpectedGetActivePathCall(4, sequence);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_submitted->m_paths, pam_submitted->m_modes, STRICT_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
    setupExpectedGetCurrentDisplayModesCall(sequence, pam_initial);

    EXPECT_CALL(*m_layer, setDisplayConfig(pam_initial->m_paths, pam_initial->m_modes, UNDO_FLAGS))
      .Times(1)
      .WillOnce(Return(ERROR_SUCCESS))
      .RetiresOnSaturation();
  }

  EXPECT_FALSE(m_win_dd.setDisplayModes(new_modes));
}
