// local includes
#include "display_device/windows/settings_manager.h"
#include "display_device/windows/settings_utils.h"
#include "fixtures/fixtures.h"
#include "fixtures/mock_audio_context.h"
#include "fixtures/mock_settings_persistence.h"
#include "utils/helpers.h"
#include "utils/mock_win_display_device.h"

namespace {
  // Convenience keywords for GMock
  using ::testing::_;
  using ::testing::HasSubstr;
  using ::testing::InSequence;
  using ::testing::Return;
  using ::testing::StrictMock;

  // Additional convenience global const(s)
  const display_device::ActiveTopology DEFAULT_CURRENT_TOPOLOGY {{"DeviceId1", "DeviceId2"}, {"DeviceId3"}};
  const display_device::EnumeratedDeviceList DEFAULT_DEVICES {
    {.m_device_id = "DeviceId1", .m_info = display_device::EnumeratedDevice::Info {.m_primary = true}},
    {.m_device_id = "DeviceId2", .m_info = display_device::EnumeratedDevice::Info {.m_primary = true}},
    {.m_device_id = "DeviceId3", .m_info = display_device::EnumeratedDevice::Info {.m_primary = false}},
    {.m_device_id = "DeviceId4"}
  };
  const display_device::EnumeratedDeviceList DEVICES_WITH_NO_PRIMARY {
    {.m_device_id = "DeviceId1", .m_info = display_device::EnumeratedDevice::Info {.m_primary = false}},
    {.m_device_id = "DeviceId2", .m_info = display_device::EnumeratedDevice::Info {.m_primary = false}},
    {.m_device_id = "DeviceId3", .m_info = display_device::EnumeratedDevice::Info {.m_primary = false}},
    {.m_device_id = "DeviceId4"}
  };
  const display_device::DeviceDisplayModeMap DEFAULT_CURRENT_MODES {
    {"DeviceId1", {{1080, 720}, {120, 1}}},
    {"DeviceId2", {{1920, 1080}, {60, 1}}},
    {"DeviceId3", {{2560, 1440}, {30, 1}}}
  };
  const display_device::HdrStateMap DEFAULT_CURRENT_HDR_STATES {
    {"DeviceId1", display_device::HdrState::Disabled},
    {"DeviceId2", display_device::HdrState::Disabled},
    {"DeviceId3", std::nullopt}
  };
  const display_device::SingleDisplayConfigState DEFAULT_PERSISTENCE_INPUT_BASE {{DEFAULT_CURRENT_TOPOLOGY, {"DeviceId1", "DeviceId2"}}};

  // Test fixture(s) for this file
  class SettingsManagerApplyMocked: public BaseTest {
  public:
    display_device::SettingsManager &getImpl() {
      if (!m_impl) {
        m_impl = std::make_unique<display_device::SettingsManager>(m_dd_api, m_audio_context_api, std::make_unique<display_device::PersistentState>(m_settings_persistence_api), display_device::WinWorkarounds {
                                                                                                                                                                                   .m_hdr_blank_delay = std::chrono::milliseconds {123}  // Value is irrelevant for the tests
                                                                                                                                                                                 });
      }

      return *m_impl;
    }

    void expectedDefaultCallsUntilTopologyPrep(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::ActiveTopology &topology = DEFAULT_CURRENT_TOPOLOGY, const std::optional<display_device::SingleDisplayConfigState> &state = ut_consts::SDCS_EMPTY) {
      EXPECT_CALL(*m_settings_persistence_api, load())
        .Times(1)
        .WillOnce(Return(serializeState(state)))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isApiAccessAvailable())
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, getCurrentTopology())
        .Times(1)
        .WillOnce(Return(topology))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isTopologyValid(topology))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedIsCapturedCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const bool is_captured) {
      EXPECT_CALL(*m_audio_context_api, isCaptured())
        .Times(1)
        .WillOnce(Return(is_captured))
        .RetiresOnSaturation();
    }

    void expectedCaptureCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const bool success = true) {
      EXPECT_CALL(*m_audio_context_api, capture())
        .Times(1)
        .WillOnce(Return(success))
        .RetiresOnSaturation();
    }

    void expectedReleaseCall(InSequence &sequence /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_audio_context_api, release())
        .Times(1)
        .RetiresOnSaturation();
    }

    void expectedTopologyGuardTopologyCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::ActiveTopology &topology = DEFAULT_CURRENT_TOPOLOGY, const bool success = true) {
      EXPECT_CALL(*m_dd_api, setTopology(topology))
        .Times(1)
        .WillOnce(Return(success))
        .RetiresOnSaturation();
    }

    void expectedDeviceEnumCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::EnumeratedDeviceList &devices = DEFAULT_DEVICES) {
      EXPECT_CALL(*m_dd_api, enumAvailableDevices())
        .Times(1)
        .WillOnce(Return(devices))
        .RetiresOnSaturation();
    }

    void expectedIsTopologyTheSameCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::ActiveTopology &lhs, const display_device::ActiveTopology &rhs) {
      EXPECT_CALL(*m_dd_api, isTopologyTheSame(lhs, rhs))
        .Times(1)
        .WillOnce(Return(lhs == rhs))
        .RetiresOnSaturation();
    }

    void expectedSetTopologyCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::ActiveTopology &topology) {
      EXPECT_CALL(*m_dd_api, setTopology(topology))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedPersistenceCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::optional<display_device::SingleDisplayConfigState> &state, const bool success = true) {
      EXPECT_CALL(*m_settings_persistence_api, store(*serializeState(state)))
        .Times(1)
        .WillOnce(Return(success))
        .RetiresOnSaturation();
    }

    void expectedTopologyGuardNewlyCapturedContextCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const bool is_captured) {
      EXPECT_CALL(*m_audio_context_api, isCaptured())
        .Times(1)
        .WillOnce(Return(is_captured))
        .RetiresOnSaturation();

      if (is_captured) {
        EXPECT_CALL(*m_audio_context_api, release())
          .Times(1)
          .RetiresOnSaturation();
      }
    }

    void expectedIsPrimaryCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::string &device_id, const bool success = true) {
      EXPECT_CALL(*m_dd_api, isPrimary(device_id))
        .Times(1)
        .WillOnce(Return(success))
        .RetiresOnSaturation();
    }

    void expectedSetAsPrimaryCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::string &device_id, const bool success = true) {
      EXPECT_CALL(*m_dd_api, setAsPrimary(device_id))
        .Times(1)
        .WillOnce(Return(success))
        .RetiresOnSaturation();
    }

    void expectedPrimaryGuardCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::string &device_id) {
      EXPECT_CALL(*m_dd_api, setAsPrimary(device_id))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedSetDisplayModesCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::DeviceDisplayModeMap &modes, const bool success = true) {
      EXPECT_CALL(*m_dd_api, setDisplayModes(modes))
        .Times(1)
        .WillOnce(Return(success))
        .RetiresOnSaturation();
    }

    void expectedGetCurrentDisplayModesCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::set<std::string> &devices, const display_device::DeviceDisplayModeMap &modes) {
      EXPECT_CALL(*m_dd_api, getCurrentDisplayModes(devices))
        .Times(1)
        .WillOnce(Return(modes))
        .RetiresOnSaturation();
    }

    void expectedSetDisplayModesGuardCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::DeviceDisplayModeMap &modes) {
      EXPECT_CALL(*m_dd_api, setDisplayModes(modes))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedSetHdrStatesCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::HdrStateMap &states, const bool success = true) {
      EXPECT_CALL(*m_dd_api, setHdrStates(states))
        .Times(1)
        .WillOnce(Return(success))
        .RetiresOnSaturation();
    }

    void expectedGetCurrentHdrStatesCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::set<std::string> &devices, const display_device::HdrStateMap &states) {
      EXPECT_CALL(*m_dd_api, getCurrentHdrStates(devices))
        .Times(1)
        .WillOnce(Return(states))
        .RetiresOnSaturation();
    }

    void expectedSetHdrStatesGuardCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const display_device::HdrStateMap &states) {
      EXPECT_CALL(*m_dd_api, setHdrStates(states))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedHdrWorkaroundCalls(InSequence &sequence /* To ensure that sequence is created outside this scope */) {
      // Using the "failure" path, to keep it simple
      EXPECT_CALL(*m_dd_api, getCurrentTopology())
        .Times(1)
        .WillOnce(Return(display_device::ActiveTopology {}))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isTopologyValid(display_device::ActiveTopology {}))
        .Times(1)
        .WillOnce(Return(false))
        .RetiresOnSaturation();
    }

    std::shared_ptr<StrictMock<display_device::MockWinDisplayDevice>> m_dd_api {std::make_shared<StrictMock<display_device::MockWinDisplayDevice>>()};
    std::shared_ptr<StrictMock<display_device::MockSettingsPersistence>> m_settings_persistence_api {std::make_shared<StrictMock<display_device::MockSettingsPersistence>>()};
    std::shared_ptr<StrictMock<display_device::MockAudioContext>> m_audio_context_api {std::make_shared<StrictMock<display_device::MockAudioContext>>()};

  private:
    std::unique_ptr<display_device::SettingsManager> m_impl;
  };

  // Specialized TEST macro(s) for this test
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, SettingsManagerApplyMocked, __VA_ARGS__)
}  // namespace

TEST_F_S_MOCKED(NoApiAccess) {
  InSequence sequence;
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_EMPTY)));
  EXPECT_CALL(*m_dd_api, isApiAccessAvailable())
    .Times(1)
    .WillOnce(Return(false));

  EXPECT_EQ(getImpl().applySettings({}), display_device::SettingsManager::ApplyResult::ApiTemporarilyUnavailable);
}

TEST_F_S_MOCKED(CurrentTopologyIsInvalid) {
  InSequence sequence;
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_EMPTY)));
  EXPECT_CALL(*m_dd_api, isApiAccessAvailable())
    .Times(1)
    .WillOnce(Return(true));
  EXPECT_CALL(*m_dd_api, getCurrentTopology())
    .Times(1)
    .WillOnce(Return(DEFAULT_CURRENT_TOPOLOGY));
  EXPECT_CALL(*m_dd_api, isTopologyValid(DEFAULT_CURRENT_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(false));

  EXPECT_EQ(getImpl().applySettings({}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, FailedToEnumerateDevices) {
  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  EXPECT_CALL(*m_dd_api, enumAvailableDevices())
    .Times(1)
    .WillOnce(Return(display_device::EnumeratedDeviceList {}))
    .RetiresOnSaturation();

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, DeviceNotAvailable) {
  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceIdX"}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, FailedToComputeInitialState) {
  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence, DEVICES_WITH_NO_PRIMARY);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, FailedToStripInitialState) {
  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence, {{.m_device_id = "DeviceId4", .m_info = display_device::EnumeratedDevice::Info {.m_primary = true}}});

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4"}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, DeviceNotFoundInNewTopology) {
  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4"}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, NoChangeIsNeeded) {
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified = {DEFAULT_CURRENT_TOPOLOGY};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedPersistenceCall(sequence, persistence_input);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareTopology, RevertingSettingsFailed) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, ut_consts::SDCS_FULL);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_modified.m_topology, {{"DeviceId1"}});
  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_modified.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, AudioContextCaptureFailed) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedCaptureCall(sequence, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, TopologyChangeFailed) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedCaptureCall(sequence, true);
  EXPECT_CALL(*m_dd_api, setTopology(display_device::ActiveTopology {{"DeviceId1"}}))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, true);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::DevicePrepFailed);
}

TEST_F_S_MOCKED(PrepareTopology, AudioContextCaptured) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified = {{{"DeviceId1"}}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedCaptureCall(sequence, true);
  expectedSetTopologyCall(sequence, {{"DeviceId1"}});
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareTopology, AudioContextCaptureSkipped, NotInitialTopologySwitch) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {*ut_consts::SDCS_NO_MODIFICATIONS};
  persistence_input.m_modified = {{{"DeviceId1"}}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, ut_consts::SDCS_NO_MODIFICATIONS);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_modified.m_topology, {{"DeviceId1"}});

  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_initial.m_topology, DEFAULT_CURRENT_TOPOLOGY);
  expectedSetTopologyCall(sequence, {{"DeviceId1"}});
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_initial.m_topology, {{"DeviceId1"}});
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareTopology, AudioContextCaptureSkipped, NoDevicesAreGone) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified = {{{"DeviceId1", "DeviceId2"}, {"DeviceId3"}, {"DeviceId4"}}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedSetTopologyCall(sequence, persistence_input.m_modified.m_topology);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4", .m_device_prep = DevicePrep::EnsureActive}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, FailedToGetPrimaryDevice) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  const display_device::ActiveTopology topology {{"DeviceId1", "DeviceId2"}, {"DeviceId3"}, {"DeviceId4"}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, topology);
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedSetTopologyCall(sequence, topology);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, topology);

  expectedIsPrimaryCall(sequence, "DeviceId1", false);
  expectedIsPrimaryCall(sequence, "DeviceId2", false);
  expectedIsPrimaryCall(sequence, "DeviceId3", false);
  expectedIsPrimaryCall(sequence, "DeviceId4", false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4", .m_device_prep = DevicePrep::EnsurePrimary}), display_device::SettingsManager::ApplyResult::PrimaryDevicePrepFailed);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, FailedToSetPrimaryDevice) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  const display_device::ActiveTopology topology {{"DeviceId1", "DeviceId2"}, {"DeviceId3"}, {"DeviceId4"}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, topology);
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedSetTopologyCall(sequence, topology);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, topology);

  expectedIsPrimaryCall(sequence, "DeviceId1");
  expectedSetAsPrimaryCall(sequence, "DeviceId4", false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4", .m_device_prep = DevicePrep::EnsurePrimary}), display_device::SettingsManager::ApplyResult::PrimaryDevicePrepFailed);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, PrimaryDeviceSet) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = {{"DeviceId1", "DeviceId2"}, {"DeviceId3"}, {"DeviceId4"}};
  persistence_input.m_modified.m_original_primary_device = "DeviceId1";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedSetTopologyCall(sequence, persistence_input.m_modified.m_topology);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1");
  expectedSetAsPrimaryCall(sequence, "DeviceId4");
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4", .m_device_prep = DevicePrep::EnsurePrimary}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, PrimaryDeviceSet, CachedDeviceReused) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = {{"DeviceId1", "DeviceId2"}, {"DeviceId3"}, {"DeviceId4"}};
  initial_state.m_modified.m_original_primary_device = "DeviceId1";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, initial_state.m_modified.m_topology, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, initial_state.m_modified.m_topology, initial_state.m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1", false);
  expectedIsPrimaryCall(sequence, "DeviceId2", false);
  expectedIsPrimaryCall(sequence, "DeviceId3");
  expectedSetAsPrimaryCall(sequence, "DeviceId4");
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4", .m_device_prep = DevicePrep::EnsurePrimary}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, PrimaryDeviceSet, GuardInvoked) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = {{"DeviceId1", "DeviceId2"}, {"DeviceId3"}, {"DeviceId4"}};
  persistence_input.m_modified.m_original_primary_device = "DeviceId1";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedSetTopologyCall(sequence, persistence_input.m_modified.m_topology);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1");
  expectedSetAsPrimaryCall(sequence, "DeviceId4");
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedPrimaryGuardCall(sequence, "DeviceId1");
  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4", .m_device_prep = DevicePrep::EnsurePrimary}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, PrimaryDeviceSetSkipped) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = {{"DeviceId1", "DeviceId2"}, {"DeviceId3"}, {"DeviceId4"}};
  persistence_input.m_modified.m_original_primary_device = "DeviceId4";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);
  expectedIsCapturedCall(sequence, false);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedSetTopologyCall(sequence, persistence_input.m_modified.m_topology);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, persistence_input.m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1", false);
  expectedIsPrimaryCall(sequence, "DeviceId2", false);
  expectedIsPrimaryCall(sequence, "DeviceId3", false);
  expectedIsPrimaryCall(sequence, "DeviceId4");
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId4", .m_device_prep = DevicePrep::EnsurePrimary}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, FailedToRestorePrimaryDevice) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto intial_state {ut_consts::SDCS_NO_MODIFICATIONS};
  intial_state->m_modified.m_original_primary_device = "DeviceId1";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, intial_state->m_modified.m_topology, intial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, intial_state->m_modified.m_topology, intial_state->m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1", false);
  expectedIsPrimaryCall(sequence, "DeviceId3");
  expectedSetAsPrimaryCall(sequence, "DeviceId1", false);

  expectedTopologyGuardTopologyCall(sequence, intial_state->m_modified.m_topology);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId3", .m_device_prep = DevicePrep::EnsureActive}), display_device::SettingsManager::ApplyResult::PrimaryDevicePrepFailed);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, PrimaryDeviceRestored) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto intial_state {ut_consts::SDCS_NO_MODIFICATIONS};
  intial_state->m_modified.m_original_primary_device = "DeviceId1";

  auto persistence_input {intial_state};
  persistence_input->m_modified.m_original_primary_device = "";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, intial_state->m_modified.m_topology, intial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, intial_state->m_modified.m_topology, intial_state->m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1", false);
  expectedIsPrimaryCall(sequence, "DeviceId3");
  expectedSetAsPrimaryCall(sequence, "DeviceId1", true);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId3", .m_device_prep = DevicePrep::EnsureActive}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, PrimaryDeviceRestored, PersistenceFailed) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto intial_state {ut_consts::SDCS_NO_MODIFICATIONS};
  intial_state->m_modified.m_original_primary_device = "DeviceId1";

  auto persistence_input {intial_state};
  persistence_input->m_modified.m_original_primary_device = "";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, intial_state->m_modified.m_topology, intial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, intial_state->m_modified.m_topology, intial_state->m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1", false);
  expectedIsPrimaryCall(sequence, "DeviceId3");
  expectedSetAsPrimaryCall(sequence, "DeviceId1", true);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedPrimaryGuardCall(sequence, "DeviceId3");
  expectedTopologyGuardTopologyCall(sequence, intial_state->m_modified.m_topology);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId3", .m_device_prep = DevicePrep::EnsureActive}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PreparePrimaryDevice, PrimaryDeviceRestoreSkipped, PersistenceFailed) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto intial_state {ut_consts::SDCS_NO_MODIFICATIONS};
  intial_state->m_modified.m_original_primary_device = "DeviceId1";

  auto persistence_input {intial_state};
  persistence_input->m_modified.m_original_primary_device = "";

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, intial_state->m_modified.m_topology, intial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, intial_state->m_modified.m_topology, intial_state->m_modified.m_topology);

  expectedIsPrimaryCall(sequence, "DeviceId1");
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence, intial_state->m_modified.m_topology);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId3", .m_device_prep = DevicePrep::EnsureActive}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareDisplayModes, FailedToGetDisplayModes) {
  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), {});

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_resolution = {{1920, 1080}}}), display_device::SettingsManager::ApplyResult::DisplayModePrepFailed);
}

TEST_F_S_MOCKED(PrepareDisplayModes, FailedToSetDisplayModes) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_resolution = {1920, 1080};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_resolution = {{1920, 1080}}}), display_device::SettingsManager::ApplyResult::DisplayModePrepFailed);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSet, ResolutionOnly) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_resolution = {1920, 1080};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), new_modes);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_resolution = {{1920, 1080}}}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSet, RefreshRateOnly) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_refresh_rate = {308500, 10000};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), new_modes);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_refresh_rate = {{30.85}}}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSet, ResolutionAndRefreshRate) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_resolution = {1920, 1080};
  new_modes["DeviceId1"].m_refresh_rate = {308500, 10000};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), new_modes);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_resolution = {{1920, 1080}}, .m_refresh_rate = {{30.85}}}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSet, ResolutionAndRefreshRate, PrimaryDeviceSpecified) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_resolution = {1920, 1080};
  new_modes["DeviceId1"].m_refresh_rate = {308500, 10000};
  new_modes["DeviceId2"].m_resolution = {1920, 1080};
  new_modes["DeviceId2"].m_refresh_rate = {308500, 10000};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), new_modes);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_resolution = {{1920, 1080}}, .m_refresh_rate = {{30.85}}}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSet, CachedModesReused) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_resolution = {1920, 1080};

  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), new_modes);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_resolution = {{1920, 1080}}}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSet, GuardInvoked) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_resolution = {1920, 1080};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), new_modes);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedSetDisplayModesGuardCall(sequence, DEFAULT_CURRENT_MODES);
  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_resolution = {{1920, 1080}}}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSet, GuardNotInvoked) {
  auto new_modes {DEFAULT_CURRENT_MODES};
  new_modes["DeviceId1"].m_resolution = {1920, 1080};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, new_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_resolution = {{1920, 1080}}}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesSetSkipped) {
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedPersistenceCall(sequence, persistence_input);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId3", .m_resolution = {{2560, 1440}}}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareDisplayModes, FailedToRestoreDisplayModes) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;
  initial_state.m_modified.m_original_modes["DeviceId1"].m_resolution = {1920, 1080};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, initial_state.m_modified.m_original_modes, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::DisplayModePrepFailed);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesRestored) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;
  initial_state.m_modified.m_original_modes["DeviceId1"].m_resolution = {1920, 1080};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, initial_state.m_modified.m_original_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), initial_state.m_modified.m_original_modes);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesRestored, PersistenceFailed) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;
  initial_state.m_modified.m_original_modes["DeviceId1"].m_resolution = {1920, 1080};

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedSetDisplayModesCall(sequence, initial_state.m_modified.m_original_modes);
  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), initial_state.m_modified.m_original_modes);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedSetDisplayModesGuardCall(sequence, DEFAULT_CURRENT_MODES);
  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareDisplayModes, DisplayModesRestoreSkipped, PersistenceFailed) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_modes = DEFAULT_CURRENT_MODES;

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentDisplayModesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_MODES);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareHdrStates, FailedToGetHdrStates) {
  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), {});

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_hdr_state = display_device::HdrState::Enabled}), display_device::SettingsManager::ApplyResult::HdrStatePrepFailed);
}

TEST_F_S_MOCKED(PrepareHdrStates, FailedToSetHdrStates) {
  auto new_states {DEFAULT_CURRENT_HDR_STATES};
  new_states["DeviceId1"] = display_device::HdrState::Enabled;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, new_states, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_hdr_state = display_device::HdrState::Enabled}), display_device::SettingsManager::ApplyResult::HdrStatePrepFailed);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesSet) {
  auto new_states {DEFAULT_CURRENT_HDR_STATES};
  new_states["DeviceId1"] = display_device::HdrState::Enabled;

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, new_states);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_hdr_state = display_device::HdrState::Enabled}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesSet, PrimaryDeviceSpecified) {
  auto new_states {DEFAULT_CURRENT_HDR_STATES};
  new_states["DeviceId1"] = display_device::HdrState::Enabled;
  new_states["DeviceId2"] = display_device::HdrState::Enabled;

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, new_states);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_hdr_state = display_device::HdrState::Enabled}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesSet, CachedModesReused) {
  auto new_states {DEFAULT_CURRENT_HDR_STATES};
  new_states["DeviceId1"] = display_device::HdrState::Enabled;

  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, new_states);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_hdr_state = display_device::HdrState::Enabled}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesSet, GuardInvoked) {
  auto new_states {DEFAULT_CURRENT_HDR_STATES};
  new_states["DeviceId1"] = display_device::HdrState::Enabled;

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, new_states);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedSetHdrStatesGuardCall(sequence, DEFAULT_CURRENT_HDR_STATES);
  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_hdr_state = display_device::HdrState::Enabled}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesSetSkipped) {
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  persistence_input.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedPersistenceCall(sequence, persistence_input);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId3", .m_hdr_state = display_device::HdrState::Enabled}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareHdrStates, FailedToRestoreHdrStates) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;
  initial_state.m_modified.m_original_hdr_states["DeviceId1"] = display_device::HdrState::Enabled;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, initial_state.m_modified.m_original_hdr_states, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::HdrStatePrepFailed);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesRestored) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;
  initial_state.m_modified.m_original_hdr_states["DeviceId1"] = display_device::HdrState::Enabled;

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, initial_state.m_modified.m_original_hdr_states);
  expectedPersistenceCall(sequence, persistence_input);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesRestored, PersistenceFailed) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;
  initial_state.m_modified.m_original_hdr_states["DeviceId1"] = display_device::HdrState::Enabled;

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedSetHdrStatesCall(sequence, initial_state.m_modified.m_original_hdr_states);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedSetHdrStatesGuardCall(sequence, DEFAULT_CURRENT_HDR_STATES);
  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareHdrStates, HdrStatesRestoreSkipped, PersistenceFailed) {
  auto initial_state {DEFAULT_PERSISTENCE_INPUT_BASE};
  initial_state.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;
  initial_state.m_modified.m_original_hdr_states = DEFAULT_CURRENT_HDR_STATES;

  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified.m_topology = DEFAULT_CURRENT_TOPOLOGY;

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, initial_state);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);

  expectedGetCurrentHdrStatesCall(sequence, display_device::win_utils::flattenTopology(DEFAULT_CURRENT_TOPOLOGY), DEFAULT_CURRENT_HDR_STATES);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(AudioContextDelayedRelease) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {*ut_consts::SDCS_NO_MODIFICATIONS};
  persistence_input.m_modified = {{{"DeviceId1"}}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, ut_consts::SDCS_NO_MODIFICATIONS);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_modified.m_topology, {{"DeviceId1"}});

  expectedIsCapturedCall(sequence, true);
  expectedSetTopologyCall(sequence, persistence_input.m_initial.m_topology);
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_initial.m_topology, {{"DeviceId1"}});
  expectedPersistenceCall(sequence, persistence_input);
  expectedReleaseCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::Ok);
}

TEST_F_S_MOCKED(FailedToSaveNewState) {
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified = {DEFAULT_CURRENT_TOPOLOGY};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(AudioContextDelayedRelease, ViaGuard) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {*ut_consts::SDCS_NO_MODIFICATIONS};
  persistence_input.m_modified = {{{"DeviceId1"}}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, ut_consts::SDCS_NO_MODIFICATIONS);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_modified.m_topology, {{"DeviceId1"}});

  expectedIsCapturedCall(sequence, true);
  expectedSetTopologyCall(sequence, persistence_input.m_initial.m_topology);
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_initial.m_topology, {{"DeviceId1"}});

  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence, DEFAULT_CURRENT_TOPOLOGY, false);
  expectedReleaseCall(sequence);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(AudioContextDelayedRelease, SkippedDueToFailure) {
  using DevicePrep = display_device::SingleDisplayConfiguration::DevicePreparation;
  auto persistence_input {*ut_consts::SDCS_NO_MODIFICATIONS};
  persistence_input.m_modified = {{{"DeviceId1"}}};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence, DEFAULT_CURRENT_TOPOLOGY, ut_consts::SDCS_NO_MODIFICATIONS);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, {{"DeviceId1"}});
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_modified.m_topology, {{"DeviceId1"}});

  expectedIsCapturedCall(sequence, true);
  expectedSetTopologyCall(sequence, persistence_input.m_initial.m_topology);
  expectedIsTopologyTheSameCall(sequence, ut_consts::SDCS_FULL->m_initial.m_topology, {{"DeviceId1"}});

  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence, DEFAULT_CURRENT_TOPOLOGY, true);
  expectedTopologyGuardNewlyCapturedContextCall(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1", .m_device_prep = DevicePrep::EnsureOnlyDisplay}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(TopologyGuardFailedButNoContextIsReleased) {
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified = {DEFAULT_CURRENT_TOPOLOGY};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, true);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedPersistenceCall(sequence, persistence_input, false);

  expectedTopologyGuardTopologyCall(sequence, DEFAULT_CURRENT_TOPOLOGY, false);

  EXPECT_EQ(getImpl().applySettings({.m_device_id = "DeviceId1"}), display_device::SettingsManager::ApplyResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(PrepareTopology, PrimaryDeviceIsUsed) {
  auto persistence_input {DEFAULT_PERSISTENCE_INPUT_BASE};
  persistence_input.m_modified = {DEFAULT_CURRENT_TOPOLOGY};

  InSequence sequence;
  expectedDefaultCallsUntilTopologyPrep(sequence);
  expectedIsCapturedCall(sequence, false);
  expectedDeviceEnumCall(sequence);
  expectedIsTopologyTheSameCall(sequence, DEFAULT_CURRENT_TOPOLOGY, DEFAULT_CURRENT_TOPOLOGY);
  expectedPersistenceCall(sequence, persistence_input);

  EXPECT_EQ(getImpl().applySettings({}), display_device::SettingsManager::ApplyResult::Ok);
}
