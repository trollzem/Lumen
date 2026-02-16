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
  class WinDisplayDevicePrimary: public BaseTest {
  public:
    bool isSystemTest() const override {
      return true;
    }

    std::shared_ptr<display_device::WinApiLayer> m_layer {std::make_shared<display_device::WinApiLayer>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  class WinDisplayDevicePrimaryMocked: public BaseTest {
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

    std::shared_ptr<StrictMock<display_device::MockWinApiLayer>> m_layer {std::make_shared<StrictMock<display_device::MockWinApiLayer>>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, WinDisplayDevicePrimary, __VA_ARGS__)
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, WinDisplayDevicePrimaryMocked, __VA_ARGS__)

  // Additional convenience global const(s)
  const UINT32 FLAGS {SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE};

  // Helper functions
  void shiftModeBy(std::optional<display_device::PathAndModeData> &pam, int path_index, POINTL point) {
    auto &mode {pam->m_modes.at(pam->m_paths.at(path_index).sourceInfo.sourceModeInfoIdx)};
    mode.sourceMode.position.x -= point.x;
    mode.sourceMode.position.y -= point.y;
  }
}  // namespace

TEST_F_S(IsPrimary) {
  const auto flat_topology {display_device::win_utils::flattenTopology(m_win_dd.getCurrentTopology())};
  EXPECT_TRUE(std::ranges::any_of(flat_topology, [&](auto device_id) {
    return m_win_dd.isPrimary(device_id);
  }));
}

TEST_F_S(SetAsPrimary, ExtendedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 2) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto topology_guard {makeTopologyGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setTopology({{available_devices->at(0)}, {available_devices->at(1)}}));

  const auto primary_guard {makePrimaryGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setAsPrimary(available_devices->at(0)));
  ASSERT_TRUE(m_win_dd.setAsPrimary(available_devices->at(1)));
}

TEST_F_S(SetAsPrimary, DuplicatedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 2) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto topology_guard {makeTopologyGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setTopology({{available_devices->at(0), available_devices->at(1)}}));

  const auto primary_guard {makePrimaryGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setAsPrimary(available_devices->at(0)));
  ASSERT_TRUE(m_win_dd.setAsPrimary(available_devices->at(1)));
}

TEST_F_S(SetAsPrimary, MixedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 3) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto topology_guard {makeTopologyGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setTopology({{available_devices->at(0)}, {available_devices->at(1), available_devices->at(2)}}));

  const auto primary_guard {makePrimaryGuard(m_win_dd)};
  ASSERT_TRUE(m_win_dd.setAsPrimary(available_devices->at(0)));
  ASSERT_TRUE(m_win_dd.setAsPrimary(available_devices->at(1)));
  ASSERT_TRUE(m_win_dd.setAsPrimary(available_devices->at(2)));
}

TEST_F_S_MOCKED(IsPrimary, Valid, True) {
  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));
  setupExpectedGetActivePathCall(1, sequence);

  EXPECT_TRUE(m_win_dd.isPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(IsPrimary, Valid, False) {
  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));
  setupExpectedGetActivePathCall(1, sequence);
  setupExpectedGetActivePathCall(2, sequence);

  EXPECT_FALSE(m_win_dd.isPrimary("DeviceId2"));
}

TEST_F_S_MOCKED(IsPrimary, EmptyId) {
  EXPECT_FALSE(m_win_dd.isPrimary(""));
}

TEST_F_S_MOCKED(IsPrimary, FailedToQueryDevices) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  EXPECT_FALSE(m_win_dd.isPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(IsPrimary, FailedToGetActivePath) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY));

  EXPECT_FALSE(m_win_dd.isPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(IsPrimary, FailedToGetSourceMode) {
  auto pam_no_modes {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  pam_no_modes->m_modes.clear();

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_no_modes));
  setupExpectedGetActivePathCall(1, sequence);

  EXPECT_FALSE(m_win_dd.isPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(SetAsPrimary, AlreadyPrimary) {
  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));
  setupExpectedGetActivePathCall(1, sequence);

  EXPECT_TRUE(m_win_dd.setAsPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(SetAsPrimary, DuplicatePrimaryDevicesSet) {
  const auto initial_pam {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};

  auto origin_point {initial_pam->m_modes.at(initial_pam->m_paths.at(1).sourceInfo.sourceModeInfoIdx).sourceMode.position};
  auto expected_pam {initial_pam};
  shiftModeBy(expected_pam, 0, origin_point);
  shiftModeBy(expected_pam, 1, origin_point);
  shiftModeBy(expected_pam, 2, origin_point);
  shiftModeBy(expected_pam, 3, origin_point);

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(initial_pam));
  setupExpectedGetActivePathCall(2, sequence);
  for (int i = 1; i <= 4; ++i) {
    EXPECT_CALL(*m_layer, getDeviceId(_))
      .Times(1)
      .WillOnce(Return("DeviceId" + std::to_string(i)))
      .RetiresOnSaturation();
  }

  EXPECT_CALL(*m_layer, setDisplayConfig(expected_pam->m_paths, expected_pam->m_modes, FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS))
    .RetiresOnSaturation();

  EXPECT_TRUE(m_win_dd.setAsPrimary("DeviceId2"));
}

TEST_F_S_MOCKED(SetAsPrimary, NonDuplicatePrimaryDeviceSet) {
  const auto initial_pam {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};

  auto origin_point {initial_pam->m_modes.at(initial_pam->m_paths.at(3).sourceInfo.sourceModeInfoIdx).sourceMode.position};
  auto expected_pam {initial_pam};
  shiftModeBy(expected_pam, 0, origin_point);
  shiftModeBy(expected_pam, 1, origin_point);
  shiftModeBy(expected_pam, 2, origin_point);
  shiftModeBy(expected_pam, 3, origin_point);

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(initial_pam));
  setupExpectedGetActivePathCall(4, sequence);
  for (int i = 1; i <= 4; ++i) {
    EXPECT_CALL(*m_layer, getDeviceId(_))
      .Times(1)
      .WillOnce(Return("DeviceId" + std::to_string(i)))
      .RetiresOnSaturation();
  }

  EXPECT_CALL(*m_layer, setDisplayConfig(expected_pam->m_paths, expected_pam->m_modes, FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS))
    .RetiresOnSaturation();

  EXPECT_TRUE(m_win_dd.setAsPrimary("DeviceId4"));
}

TEST_F_S_MOCKED(SetAsPrimary, SharedModeShiftedOnce) {
  auto initial_pam {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  initial_pam->m_paths.at(2).sourceInfo.sourceModeInfoIdx = initial_pam->m_paths.at(1).sourceInfo.sourceModeInfoIdx;

  auto origin_point {initial_pam->m_modes.at(initial_pam->m_paths.at(1).sourceInfo.sourceModeInfoIdx).sourceMode.position};
  auto expected_pam {initial_pam};
  shiftModeBy(expected_pam, 0, origin_point);
  shiftModeBy(expected_pam, 1, origin_point);
  shiftModeBy(expected_pam, 3, origin_point);

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(initial_pam));
  setupExpectedGetActivePathCall(2, sequence);
  for (int i = 1; i <= 4; ++i) {
    EXPECT_CALL(*m_layer, getDeviceId(_))
      .Times(1)
      .WillOnce(Return("DeviceId" + std::to_string(i)))
      .RetiresOnSaturation();
  }

  EXPECT_CALL(*m_layer, setDisplayConfig(expected_pam->m_paths, expected_pam->m_modes, FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS))
    .RetiresOnSaturation();

  EXPECT_TRUE(m_win_dd.setAsPrimary("DeviceId2"));
}

TEST_F_S_MOCKED(SetAsPrimary, EmptyId) {
  EXPECT_FALSE(m_win_dd.setAsPrimary(""));
}

TEST_F_S_MOCKED(SetAsPrimary, FailedToQueryDevices) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  EXPECT_FALSE(m_win_dd.setAsPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(SetAsPrimary, FailedToGetActivePath) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY));

  EXPECT_FALSE(m_win_dd.setAsPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(SetAsPrimary, FailedToGetSourceMode) {
  auto pam_no_modes {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  pam_no_modes->m_modes.clear();

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_no_modes));
  setupExpectedGetActivePathCall(1, sequence);

  EXPECT_FALSE(m_win_dd.setAsPrimary("DeviceId1"));
}

TEST_F_S_MOCKED(SetAsPrimary, FailedToGetSourceIndex, DuringShift) {
  auto pam_invalid_mode_idx {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  display_device::win_utils::setSourceIndex(pam_invalid_mode_idx->m_paths.at(0), std::nullopt);

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_invalid_mode_idx));
  setupExpectedGetActivePathCall(2, sequence);
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId1"))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setAsPrimary("DeviceId2"));
}

TEST_F_S_MOCKED(SetAsPrimary, FailedToGetSourceMode, DuringShift) {
  auto pam_invalid_mode_type {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};
  pam_invalid_mode_type->m_modes.at(pam_invalid_mode_type->m_paths.at(0).sourceInfo.sourceModeInfoIdx).infoType = DISPLAYCONFIG_MODE_INFO_TYPE_TARGET;

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(pam_invalid_mode_type));
  setupExpectedGetActivePathCall(2, sequence);
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId1"))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setAsPrimary("DeviceId2"));
}

TEST_F_S_MOCKED(SetAsPrimary, FailedToSetDisplayConfig) {
  const auto initial_pam {ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES};

  auto origin_point {initial_pam->m_modes.at(initial_pam->m_paths.at(3).sourceInfo.sourceModeInfoIdx).sourceMode.position};
  auto expected_pam {initial_pam};
  shiftModeBy(expected_pam, 0, origin_point);
  shiftModeBy(expected_pam, 1, origin_point);
  shiftModeBy(expected_pam, 2, origin_point);
  shiftModeBy(expected_pam, 3, origin_point);

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(initial_pam));
  setupExpectedGetActivePathCall(4, sequence);
  for (int i = 1; i <= 4; ++i) {
    EXPECT_CALL(*m_layer, getDeviceId(_))
      .Times(1)
      .WillOnce(Return("DeviceId" + std::to_string(i)))
      .RetiresOnSaturation();
  }

  EXPECT_CALL(*m_layer, setDisplayConfig(expected_pam->m_paths, expected_pam->m_modes, FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_ACCESS_DENIED))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getErrorString(ERROR_ACCESS_DENIED))
    .Times(1)
    .WillRepeatedly(Return("ErrorDesc"))
    .RetiresOnSaturation();

  EXPECT_FALSE(m_win_dd.setAsPrimary("DeviceId4"));
}
