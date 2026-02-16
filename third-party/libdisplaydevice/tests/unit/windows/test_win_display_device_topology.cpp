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
  class WinDisplayDeviceTopology: public BaseTest {
  public:
    bool isSystemTest() const override {
      return true;
    }

    std::shared_ptr<display_device::WinApiLayer> m_layer {std::make_shared<display_device::WinApiLayer>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  class WinDisplayDeviceTopologyMocked: public BaseTest {
  public:
    void setupExpectCallFor3ActivePathsAndModes(const display_device::QueryType query_type, InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_layer, queryDisplayConfig(query_type))
        .Times(1)
        .WillOnce(Return(ut_consts::PAM_3_ACTIVE))
        .RetiresOnSaturation();

      for (int i = 1; i <= 3; ++i) {
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

    static std::vector<DISPLAYCONFIG_PATH_INFO> getExpectedPathToBeSet() {
      auto path {ut_consts::PAM_3_ACTIVE->m_paths.at(0)};

      display_device::win_utils::setCloneGroupId(path, 0);
      display_device::win_utils::setSourceIndex(path, std::nullopt);
      display_device::win_utils::setTargetIndex(path, std::nullopt);
      display_device::win_utils::setDesktopIndex(path, std::nullopt);
      display_device::win_utils::setActive(path);

      return std::vector<DISPLAYCONFIG_PATH_INFO> {path};
    };

    std::shared_ptr<StrictMock<display_device::MockWinApiLayer>> m_layer {std::make_shared<StrictMock<display_device::MockWinApiLayer>>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, WinDisplayDeviceTopology, __VA_ARGS__)
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, WinDisplayDeviceTopologyMocked, __VA_ARGS__)

}  // namespace

TEST_F_S(GetCurrentTopology) {
  const auto active_devices {m_layer->queryDisplayConfig(display_device::QueryType::Active)};
  ASSERT_TRUE(active_devices);

  if (active_devices->m_paths.empty()) {
    GTEST_SKIP_("No active devices are available in the system.");
  }

  std::set<std::string> expected_devices;
  for (const auto &path : active_devices->m_paths) {
    const auto device_id {m_layer->getDeviceId(path)};
    EXPECT_FALSE(device_id.empty());
    EXPECT_TRUE(expected_devices.insert(device_id).second);
  }

  // It is enough to check whether the topology contains expected ids - others test cases check the structure.
  EXPECT_EQ(display_device::win_utils::flattenTopology(m_win_dd.getCurrentTopology()), expected_devices);
}

TEST_F_S(SetCurrentTopology, ExtendedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer, false)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 2) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto cleanup_guard {makeTopologyGuard(m_win_dd)};

  // We are changing to a single device to ensure that we are not in the "final" state
  const display_device::ActiveTopology single_device_topology {{available_devices->at(0)}};
  ASSERT_TRUE(m_win_dd.setTopology(single_device_topology));

  // We are limiting ourselves to 3 devices only to avoid GPU limitation issues (even if very unlikely)
  display_device::ActiveTopology multiple_device_topology {{available_devices->at(0)}, {available_devices->at(1)}};
  if (available_devices->size() > 2) {
    multiple_device_topology.push_back({available_devices->at(2)});
  }
  ASSERT_TRUE(m_win_dd.setTopology(multiple_device_topology));
}

TEST_F_S(SetCurrentTopology, DuplicatedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer, false)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 2) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto cleanup_guard {makeTopologyGuard(m_win_dd)};

  // We are changing to a single device to ensure that we are not in the "final" state
  const display_device::ActiveTopology single_device_topology {{available_devices->at(0)}};
  ASSERT_TRUE(m_win_dd.setTopology(single_device_topology));

  display_device::ActiveTopology multiple_device_topology {{available_devices->at(0), available_devices->at(1)}};
  ASSERT_TRUE(m_win_dd.setTopology(multiple_device_topology));
}

TEST_F_S(SetCurrentTopology, MixedTopology) {
  const auto available_devices {getAvailableDevices(*m_layer, false)};
  ASSERT_TRUE(available_devices);

  if (available_devices->size() < 3) {
    GTEST_SKIP_("Not enough devices are available in the system.");
  }

  const auto cleanup_guard {makeTopologyGuard(m_win_dd)};

  // We are changing to a single device to ensure that we are not in the "final" state
  const display_device::ActiveTopology single_device_topology {{available_devices->at(0)}};
  ASSERT_TRUE(m_win_dd.setTopology(single_device_topology));

  display_device::ActiveTopology multiple_device_topology {{available_devices->at(0), available_devices->at(1)}, {available_devices->at(2)}};
  ASSERT_TRUE(m_win_dd.setTopology(multiple_device_topology));
}

TEST_F_S_MOCKED(GetCurrentTopology, ExtendedDisplaysOnly) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);

  const display_device::ActiveTopology expected_topology {{"DeviceId1"}, {"DeviceId2"}, {"DeviceId3"}};
  EXPECT_EQ(m_win_dd.getCurrentTopology(), expected_topology);
}

TEST_F_S_MOCKED(GetCurrentTopology, ExtendedAndDuplicatedDisplays) {
  EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
    .Times(4)
    .WillRepeatedly(Return("PathX"));
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(4)
    .WillRepeatedly(Return("DisplayNameX"));
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(4)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId3"))
    .WillOnce(Return("DeviceId4"));
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_4_ACTIVE_WITH_2_DUPLICATES));

  const display_device::ActiveTopology expected_topology {{"DeviceId1"}, {"DeviceId2", "DeviceId3"}, {"DeviceId4"}};
  EXPECT_EQ(m_win_dd.getCurrentTopology(), expected_topology);
}

TEST_F_S_MOCKED(GetCurrentTopology, PathsContainingInvalidIndexes) {
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
    .WillOnce(Return(ut_consts::PAM_3_ACTIVE_WITH_INVALID_MODE_IDX));

  const display_device::ActiveTopology expected_topology {};
  EXPECT_EQ(m_win_dd.getCurrentTopology(), expected_topology);
}

TEST_F_S_MOCKED(GetCurrentTopology, TransientDisplayIssues) {
  EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
    .Times(3)
    .WillRepeatedly(Return("PathX"));
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(3)
    .WillOnce(Return("DisplayName1"))
    .WillOnce(Return(""))  // The display name does not exist for some reason, but it was still reported as active
    .WillOnce(Return("DisplayName2"));
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(3)
    .WillOnce(Return("DeviceId1"))
    .WillOnce(Return("DeviceId2"))
    .WillOnce(Return("DeviceId3"));
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_3_ACTIVE));

  const display_device::ActiveTopology expected_topology {{"DeviceId1"}, {"DeviceId3"}};
  EXPECT_EQ(m_win_dd.getCurrentTopology(), expected_topology);
}

TEST_F_S_MOCKED(GetCurrentTopology, EmptyDeviceList) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(_))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY));

  const display_device::ActiveTopology expected_topology {};
  EXPECT_EQ(m_win_dd.getCurrentTopology(), expected_topology);
}

TEST_F_S_MOCKED(GetCurrentTopology, NullDeviceList) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(_))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  const display_device::ActiveTopology expected_topology {};
  EXPECT_EQ(m_win_dd.getCurrentTopology(), expected_topology);
}

TEST_F_S_MOCKED(IsTopologyValid) {
  EXPECT_EQ(m_win_dd.isTopologyValid({/* no groups */}), false);
  EXPECT_EQ(m_win_dd.isTopologyValid({{/* empty group */}}), false);
  EXPECT_EQ(m_win_dd.isTopologyValid({{"ID_1"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyValid({{"ID_1"}, {"ID_2"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyValid({{"ID_1", "ID_2"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyValid({{"ID_1", "ID_1"}}), false);
  EXPECT_EQ(m_win_dd.isTopologyValid({{"ID_1"}, {"ID_1"}}), false);
  EXPECT_EQ(m_win_dd.isTopologyValid({{"ID_1", "ID_2", "ID_3"}}), false);
  EXPECT_EQ(m_win_dd.isTopologyValid({{"ID_1"}, {"ID_2"}, {"ID_3"}, {"ID_4"}, {"ID_5"}}), true);
}

TEST_F_S_MOCKED(isTopologyTheSame) {
  EXPECT_EQ(m_win_dd.isTopologyTheSame({/* no groups */}, {/* no groups */}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{/* empty group */}}, {{/* empty group */}}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{/* empty group */}}, {{/* empty group */}, {/* empty group */}}), false);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1"}}, {{"ID_1"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1"}}, {{"ID_1"}, {"ID_2"}}), false);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1"}, {"ID_2"}}, {{"ID_1"}, {"ID_2"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1"}, {"ID_2"}}, {{"ID_2"}, {"ID_1"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1"}, {"ID_2"}}, {{"ID_1", "ID_2"}}), false);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1", "ID_2"}}, {{"ID_1", "ID_2"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1", "ID_2"}}, {{"ID_2", "ID_1"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1", "ID_2"}}, {{"ID_2", "ID_1"}, {"ID_3"}}), false);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_1", "ID_2"}, {"ID_3"}}, {{"ID_2", "ID_1"}, {"ID_3"}}), true);
  EXPECT_EQ(m_win_dd.isTopologyTheSame({{"ID_3"}, {"ID_1", "ID_2"}}, {{"ID_2", "ID_1"}, {"ID_3"}}), true);
}

TEST_F_S_MOCKED(SetCurrentTopology) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::All, sequence);

  auto expected_path {ut_consts::PAM_3_ACTIVE->m_paths.at(0)};
  display_device::win_utils::setCloneGroupId(expected_path, 0);
  display_device::win_utils::setSourceIndex(expected_path, std::nullopt);
  display_device::win_utils::setTargetIndex(expected_path, std::nullopt);
  display_device::win_utils::setDesktopIndex(expected_path, std::nullopt);
  display_device::win_utils::setActive(expected_path);

  UINT32 expected_flags {SDC_APPLY | SDC_TOPOLOGY_SUPPLIED | SDC_ALLOW_PATH_ORDER_CHANGES | SDC_VIRTUAL_MODE_AWARE};
  EXPECT_CALL(*m_layer, setDisplayConfig(std::vector<DISPLAYCONFIG_PATH_INFO> {expected_path}, std::vector<DISPLAYCONFIG_MODE_INFO> {}, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS));

  // Report only 1 active device
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(display_device::PathAndModeData {{ut_consts::PAM_3_ACTIVE->m_paths.at(0)}, {ut_consts::PAM_3_ACTIVE->m_modes.at(0)}}))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("Path1"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId1"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName1"))
    .RetiresOnSaturation();

  EXPECT_TRUE(m_win_dd.setTopology({{"DeviceId1"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, InvalidTopologyProvided) {
  EXPECT_FALSE(m_win_dd.setTopology({}));
}

TEST_F_S_MOCKED(SetCurrentTopology, FailedToGetCurrentTopology) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceId1"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, CurrentTopologyIsTheSame) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);

  const display_device::ActiveTopology current_topology {{"DeviceId1"}, {"DeviceId2"}, {"DeviceId3"}};
  EXPECT_TRUE(m_win_dd.setTopology(current_topology));
}

TEST_F_S_MOCKED(SetCurrentTopology, FailedToQueryForAllDevices) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceId1"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, DevicePathsAreNoLongerAvailable) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY));

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceId1"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, FailedToMakePathSourceData) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::All, sequence);

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceIdUnknown"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, WindowsDoesNotKnowAboutTheTopology, FailedToSetTopology) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::All, sequence);

  UINT32 expected_flags {SDC_APPLY | SDC_TOPOLOGY_SUPPLIED | SDC_ALLOW_PATH_ORDER_CHANGES | SDC_VIRTUAL_MODE_AWARE};
  EXPECT_CALL(*m_layer, setDisplayConfig(getExpectedPathToBeSet(), std::vector<DISPLAYCONFIG_MODE_INFO> {}, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_GEN_FAILURE));

  EXPECT_CALL(*m_layer, getErrorString(ERROR_GEN_FAILURE))
    .Times(1)
    .WillRepeatedly(Return("ErrorDesc"));

  expected_flags = SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_ALLOW_CHANGES | SDC_VIRTUAL_MODE_AWARE | SDC_SAVE_TO_DATABASE;
  EXPECT_CALL(*m_layer, setDisplayConfig(getExpectedPathToBeSet(), std::vector<DISPLAYCONFIG_MODE_INFO> {}, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_GEN_FAILURE));

  EXPECT_CALL(*m_layer, getErrorString(ERROR_GEN_FAILURE))
    .Times(1)
    .WillRepeatedly(Return("ErrorDesc"));

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceId1"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, FailedToSetTopology, NoRecovery) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::All, sequence);

  UINT32 expected_flags {SDC_APPLY | SDC_TOPOLOGY_SUPPLIED | SDC_ALLOW_PATH_ORDER_CHANGES | SDC_VIRTUAL_MODE_AWARE};
  EXPECT_CALL(*m_layer, setDisplayConfig(getExpectedPathToBeSet(), std::vector<DISPLAYCONFIG_MODE_INFO> {}, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_INVALID_PARAMETER));

  EXPECT_CALL(*m_layer, getErrorString(ERROR_INVALID_PARAMETER))
    .Times(1)
    .WillRepeatedly(Return("ErrorDesc"));

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceId1"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, TopologyWasSetAccordingToWinApi, CouldNotGetCurrentTopologyToVerify) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::All, sequence);

  UINT32 expected_flags {SDC_APPLY | SDC_TOPOLOGY_SUPPLIED | SDC_ALLOW_PATH_ORDER_CHANGES | SDC_VIRTUAL_MODE_AWARE};
  EXPECT_CALL(*m_layer, setDisplayConfig(getExpectedPathToBeSet(), std::vector<DISPLAYCONFIG_MODE_INFO> {}, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS));

  // Called when getting the topology
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  // Called when doing the undo
  expected_flags = SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE;
  EXPECT_CALL(*m_layer, setDisplayConfig(ut_consts::PAM_3_ACTIVE->m_paths, ut_consts::PAM_3_ACTIVE->m_modes, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS));

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceId1"}}));
}

TEST_F_S_MOCKED(SetCurrentTopology, TopologyWasSetAccordingToWinApi, WinApiLied) {
  InSequence sequence;
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::All, sequence);

  UINT32 expected_flags {SDC_APPLY | SDC_TOPOLOGY_SUPPLIED | SDC_ALLOW_PATH_ORDER_CHANGES | SDC_VIRTUAL_MODE_AWARE};
  EXPECT_CALL(*m_layer, setDisplayConfig(getExpectedPathToBeSet(), std::vector<DISPLAYCONFIG_MODE_INFO> {}, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS));

  // Called when getting the topology
  setupExpectCallFor3ActivePathsAndModes(display_device::QueryType::Active, sequence);

  // Called when doing the undo
  expected_flags = SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_SAVE_TO_DATABASE | SDC_VIRTUAL_MODE_AWARE;
  EXPECT_CALL(*m_layer, setDisplayConfig(ut_consts::PAM_3_ACTIVE->m_paths, ut_consts::PAM_3_ACTIVE->m_modes, expected_flags))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS));

  EXPECT_FALSE(m_win_dd.setTopology({{"DeviceId1"}}));
}
