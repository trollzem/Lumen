// local includes
#include "display_device/windows/settings_manager.h"
#include "display_device/windows/settings_utils.h"
#include "fixtures/fixtures.h"
#include "fixtures/mock_audio_context.h"
#include "fixtures/mock_settings_persistence.h"
#include "utils/comparison.h"
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
  const display_device::ActiveTopology CURRENT_TOPOLOGY {{"DeviceId4"}};
  const display_device::HdrStateMap CURRENT_MODIFIED_HDR_STATES {
    {"DeviceId1", {display_device::HdrState::Enabled}},
    {"DeviceId3", std::nullopt}
  };
  const display_device::DeviceDisplayModeMap CURRENT_MODIFIED_DISPLAY_MODES {
    {"DeviceId1", {{123, 456}, {120, 1}}},
    {"DeviceId3", {{456, 123}, {60, 1}}}
  };
  const std::string CURRENT_MODIFIED_PRIMARY_DEVICE {"DeviceId3"};
  const display_device::EnumeratedDeviceList CURRENT_DEVICES {
    {.m_device_id = "DeviceId1"},
    {.m_device_id = "DeviceId3"},
    {.m_device_id = "DeviceId4"}
  };
  const display_device::ActiveTopology FULL_EXTENDED_TOPOLOGY {
    {"DeviceId1"},
    {"DeviceId3"},
    {"DeviceId4"}
  };

  // Test fixture(s) for this file
  class SettingsManagerRevertMocked: public BaseTest {
  public:
    display_device::SettingsManager &getImpl() {
      if (!m_impl) {
        m_impl = std::make_unique<display_device::SettingsManager>(m_dd_api, m_audio_context_api, std::make_unique<display_device::PersistentState>(m_settings_persistence_api), display_device::WinWorkarounds {
                                                                                                                                                                                   .m_hdr_blank_delay = std::chrono::milliseconds {123}  // Value is irrelevant for the tests
                                                                                                                                                                                 });
      }

      return *m_impl;
    }

    void expectedDefaultCallsUntilModifiedSettings(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::optional<display_device::SingleDisplayConfigState> &state = ut_consts::SDCS_FULL) {
      EXPECT_CALL(*m_settings_persistence_api, load())
        .Times(1)
        .WillOnce(Return(serializeState(state)))
        .RetiresOnSaturation();
      expectedDefaultCallsUntilModifiedSettingsNoPersistence(sequence);
    }

    void expectedDefaultCallsUntilModifiedSettingsNoPersistence(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, isApiAccessAvailable())
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, getCurrentTopology())
        .Times(1)
        .WillOnce(Return(CURRENT_TOPOLOGY))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isTopologyValid(CURRENT_TOPOLOGY))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultMofifiedTopologyCalls(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_modified.m_topology))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, ut_consts::SDCS_FULL->m_modified.m_topology))
        .Times(1)
        .WillOnce(Return(false))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, setTopology(ut_consts::SDCS_FULL->m_modified.m_topology))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultHdrStateGuardInitCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, getCurrentHdrStates(display_device::win_utils::flattenTopology(ut_consts::SDCS_FULL->m_modified.m_topology)))
        .Times(1)
        .WillOnce(Return(CURRENT_MODIFIED_HDR_STATES))
        .RetiresOnSaturation();
    }

    void expectedDefaultHdrStateGuardCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, setHdrStates(CURRENT_MODIFIED_HDR_STATES))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultDisplayModeGuardInitCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, getCurrentDisplayModes(display_device::win_utils::flattenTopology(ut_consts::SDCS_FULL->m_modified.m_topology)))
        .Times(1)
        .WillOnce(Return(CURRENT_MODIFIED_DISPLAY_MODES))
        .RetiresOnSaturation();
    }

    void expectedDefaultDisplayModeGuardCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, setDisplayModes(CURRENT_MODIFIED_DISPLAY_MODES))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultPrimaryDeviceGuardInitCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, isPrimary("DeviceId1"))
        .Times(1)
        .WillOnce(Return(false))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isPrimary(CURRENT_MODIFIED_PRIMARY_DEVICE))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultPrimaryDeviceGuardCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, setAsPrimary(CURRENT_MODIFIED_PRIMARY_DEVICE))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultHdrStateCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, setHdrStates(ut_consts::SDCS_FULL->m_modified.m_original_hdr_states))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultDisplayModeCall(InSequence & /* To ensure that sequence is created outside this scope */, bool has_changed = true) {
      EXPECT_CALL(*m_dd_api, setDisplayModes(ut_consts::SDCS_FULL->m_modified.m_original_modes))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, getCurrentDisplayModes(display_device::win_utils::flattenTopology(ut_consts::SDCS_FULL->m_modified.m_topology)))
        .Times(1)
        .WillOnce(Return(has_changed ? ut_consts::SDCS_FULL->m_modified.m_original_modes : CURRENT_MODIFIED_DISPLAY_MODES))
        .RetiresOnSaturation();
    }

    void expectedDefaultPrimaryDeviceCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, setAsPrimary(ut_consts::SDCS_FULL->m_modified.m_original_primary_device))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultRevertModifiedSettingsCall(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::optional<display_device::SingleDisplayConfigState> &state = ut_consts::SDCS_FULL) {
      auto expected_persistent_input {state};
      expected_persistent_input->m_modified = {expected_persistent_input->m_modified.m_topology};

      expectedDefaultMofifiedTopologyCalls(sequence);
      expectedDefaultHdrStateGuardInitCall(sequence);
      expectedDefaultHdrStateCall(sequence);
      expectedDefaultDisplayModeGuardInitCall(sequence);
      expectedDefaultDisplayModeCall(sequence);
      expectedDefaultPrimaryDeviceGuardInitCall(sequence);
      expectedDefaultPrimaryDeviceCall(sequence);
      EXPECT_CALL(*m_settings_persistence_api, store(*serializeState(expected_persistent_input)))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultInitialTopologyCalls(InSequence &sequence /* To ensure that sequence is created outside this scope */, const std::optional<display_device::SingleDisplayConfigState> &state = ut_consts::SDCS_FULL) {
      EXPECT_CALL(*m_dd_api, isTopologyValid(state->m_initial.m_topology))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, state->m_initial.m_topology))
        .Times(1)
        .WillOnce(Return(CURRENT_TOPOLOGY == state->m_initial.m_topology))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, setTopology(state->m_initial.m_topology))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultFinalPersistenceCalls(InSequence &sequence /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_settings_persistence_api, clear())
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
    }

    void expectedDefaultAudioContextCalls(InSequence &sequence /* To ensure that sequence is created outside this scope */, const bool audio_captured) {
      EXPECT_CALL(*m_audio_context_api, isCaptured())
        .Times(1)
        .WillOnce(Return(audio_captured))
        .RetiresOnSaturation();

      if (audio_captured) {
        EXPECT_CALL(*m_audio_context_api, release())
          .Times(1)
          .RetiresOnSaturation();
      }
    }

    void expectedDefaultTopologyGuardCall(InSequence & /* To ensure that sequence is created outside this scope */) {
      EXPECT_CALL(*m_dd_api, enumAvailableDevices())
        .Times(1)
        .WillOnce(Return(CURRENT_DEVICES))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isTopologyValid(FULL_EXTENDED_TOPOLOGY))
        .Times(1)
        .WillOnce(Return(true))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, FULL_EXTENDED_TOPOLOGY))
        .Times(1)
        .WillOnce(Return(false))
        .RetiresOnSaturation();
      EXPECT_CALL(*m_dd_api, setTopology(FULL_EXTENDED_TOPOLOGY))
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
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, SettingsManagerRevertMocked, __VA_ARGS__)
}  // namespace

TEST_F_S_MOCKED(NoSettingsAvailable) {
  InSequence sequence;
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_EMPTY)));

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);
}

TEST_F_S_MOCKED(NoApiAccess) {
  InSequence sequence;
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));
  EXPECT_CALL(*m_dd_api, isApiAccessAvailable())
    .Times(1)
    .WillOnce(Return(false));

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::ApiTemporarilyUnavailable);
}

TEST_F_S_MOCKED(InvalidCurrentTopology) {
  InSequence sequence;
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));
  EXPECT_CALL(*m_dd_api, isApiAccessAvailable())
    .Times(1)
    .WillOnce(Return(true));
  EXPECT_CALL(*m_dd_api, getCurrentTopology())
    .Times(1)
    .WillOnce(Return(CURRENT_TOPOLOGY));
  EXPECT_CALL(*m_dd_api, isTopologyValid(CURRENT_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(false));

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::TopologyIsInvalid);
}

TEST_F_S_MOCKED(RevertModifiedSettings, InvalidModifiedTopology) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);

  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_modified.m_topology))
    .Times(1)
    .WillOnce(Return(false));
  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::TopologyIsInvalid);
}

TEST_F_S_MOCKED(RevertModifiedSettings, FailedToSetModifiedTopology) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);

  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_modified.m_topology))
    .Times(1)
    .WillOnce(Return(true));
  EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, ut_consts::SDCS_FULL->m_modified.m_topology))
    .Times(1)
    .WillOnce(Return(false));
  EXPECT_CALL(*m_dd_api, setTopology(ut_consts::SDCS_FULL->m_modified.m_topology))
    .Times(1)
    .WillOnce(Return(false));
  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::SwitchingTopologyFailed);
}

TEST_F_S_MOCKED(RevertModifiedSettings, FailedToRevertHdrStates) {
  auto sdcs_stripped {ut_consts::SDCS_FULL};
  sdcs_stripped->m_modified.m_original_modes.clear();
  sdcs_stripped->m_modified.m_original_primary_device.clear();

  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence, sdcs_stripped);
  expectedDefaultMofifiedTopologyCalls(sequence);
  expectedDefaultHdrStateGuardInitCall(sequence);
  EXPECT_CALL(*m_dd_api, setHdrStates(ut_consts::SDCS_FULL->m_modified.m_original_hdr_states))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::RevertingHdrStatesFailed);
}

TEST_F_S_MOCKED(RevertModifiedSettings, FailedToRevertDisplayModes) {
  auto sdcs_stripped {ut_consts::SDCS_FULL};
  sdcs_stripped->m_modified.m_original_hdr_states.clear();
  sdcs_stripped->m_modified.m_original_primary_device.clear();

  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence, sdcs_stripped);
  expectedDefaultMofifiedTopologyCalls(sequence);
  expectedDefaultDisplayModeGuardInitCall(sequence);
  EXPECT_CALL(*m_dd_api, setDisplayModes(ut_consts::SDCS_FULL->m_modified.m_original_modes))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::RevertingDisplayModesFailed);
}

TEST_F_S_MOCKED(RevertModifiedSettings, RevertedDisplayModes, PersistenceFailed, GuardNotInvoked) {
  auto sdcs_stripped {ut_consts::SDCS_FULL};
  sdcs_stripped->m_modified.m_original_hdr_states.clear();
  sdcs_stripped->m_modified.m_original_primary_device.clear();

  auto expected_persistent_input {ut_consts::SDCS_FULL};
  expected_persistent_input->m_modified = {expected_persistent_input->m_modified.m_topology};

  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence, sdcs_stripped);
  expectedDefaultMofifiedTopologyCalls(sequence);
  expectedDefaultDisplayModeGuardInitCall(sequence);
  expectedDefaultDisplayModeCall(sequence, false);
  EXPECT_CALL(*m_settings_persistence_api, store(*serializeState(expected_persistent_input)))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(RevertModifiedSettings, FailedToRevertPrimaryDevice) {
  auto sdcs_stripped {ut_consts::SDCS_FULL};
  sdcs_stripped->m_modified.m_original_hdr_states.clear();
  sdcs_stripped->m_modified.m_original_modes.clear();

  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence, sdcs_stripped);
  expectedDefaultMofifiedTopologyCalls(sequence);
  expectedDefaultPrimaryDeviceGuardInitCall(sequence);
  EXPECT_CALL(*m_dd_api, setAsPrimary(ut_consts::SDCS_FULL->m_modified.m_original_primary_device))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::RevertingPrimaryDeviceFailed);
}

TEST_F_S_MOCKED(RevertModifiedSettings, FailedToSetPersistence) {
  auto expected_persistent_input {ut_consts::SDCS_FULL};
  expected_persistent_input->m_modified = {expected_persistent_input->m_modified.m_topology};

  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultMofifiedTopologyCalls(sequence);
  expectedDefaultHdrStateGuardInitCall(sequence);
  expectedDefaultHdrStateCall(sequence);
  expectedDefaultDisplayModeGuardInitCall(sequence);
  expectedDefaultDisplayModeCall(sequence);
  expectedDefaultPrimaryDeviceGuardInitCall(sequence);
  expectedDefaultPrimaryDeviceCall(sequence);
  EXPECT_CALL(*m_settings_persistence_api, store(*serializeState(expected_persistent_input)))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultPrimaryDeviceGuardCall(sequence);
  expectedDefaultDisplayModeGuardCall(sequence);
  expectedDefaultHdrStateGuardCall(sequence);
  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(TopologyGuard, CurrentTopologyUsedAsFallback) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultRevertModifiedSettingsCall(sequence);
  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_initial.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, enumAvailableDevices())
    .Times(1)
    .WillOnce(Return(CURRENT_DEVICES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyValid(FULL_EXTENDED_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, CURRENT_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(true))
    .RetiresOnSaturation();
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::TopologyIsInvalid);
}

TEST_F_S_MOCKED(TopologyGuard, SystemSettingsUntouched) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_modified.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, enumAvailableDevices())
    .Times(1)
    .WillOnce(Return(CURRENT_DEVICES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyValid(FULL_EXTENDED_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, CURRENT_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(true))
    .RetiresOnSaturation();

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::TopologyIsInvalid);
}

TEST_F_S_MOCKED(TopologyGuard, FailedToSetTopology) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_modified.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, enumAvailableDevices())
    .Times(1)
    .WillOnce(Return(CURRENT_DEVICES))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyValid(FULL_EXTENDED_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(true))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, FULL_EXTENDED_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, setTopology(FULL_EXTENDED_TOPOLOGY))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::TopologyIsInvalid);
}

TEST_F_S_MOCKED(InvalidInitialTopology) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultRevertModifiedSettingsCall(sequence);
  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_initial.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::TopologyIsInvalid);
}

TEST_F_S_MOCKED(FailedToSetInitialTopology) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultRevertModifiedSettingsCall(sequence);
  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_initial.m_topology))
    .Times(1)
    .WillOnce(Return(true))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, ut_consts::SDCS_FULL->m_initial.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, setTopology(ut_consts::SDCS_FULL->m_initial.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::SwitchingTopologyFailed);
}

TEST_F_S_MOCKED(FailedToClearPersistence) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultRevertModifiedSettingsCall(sequence);
  expectedDefaultInitialTopologyCalls(sequence);
  EXPECT_CALL(*m_settings_persistence_api, clear())
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::PersistenceSaveFailed);
}

TEST_F_S_MOCKED(SuccesfullyReverted, WithAudioCapture) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultRevertModifiedSettingsCall(sequence);
  expectedDefaultInitialTopologyCalls(sequence);
  expectedDefaultFinalPersistenceCalls(sequence);
  expectedDefaultAudioContextCalls(sequence, true);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);
  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);  // Second call after success is NOOP
}

TEST_F_S_MOCKED(SuccesfullyReverted, NoAudioCapture) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultRevertModifiedSettingsCall(sequence);
  expectedDefaultInitialTopologyCalls(sequence);
  expectedDefaultFinalPersistenceCalls(sequence);
  expectedDefaultAudioContextCalls(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);
  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);  // Second call after success is NOOP
}

TEST_F_S_MOCKED(SuccesfullyReverted, TopologySetToBackToInitialSinceItWasChangedToModified) {
  auto initial_state {ut_consts::SDCS_FULL};
  initial_state->m_initial.m_topology = CURRENT_TOPOLOGY;

  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence, initial_state);
  expectedDefaultRevertModifiedSettingsCall(sequence, initial_state);
  expectedDefaultInitialTopologyCalls(sequence, initial_state);
  expectedDefaultFinalPersistenceCalls(sequence);
  expectedDefaultAudioContextCalls(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);
}

TEST_F_S_MOCKED(RevertModifiedSettings, CachedSettingsAreUpdated) {
  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence);
  expectedDefaultRevertModifiedSettingsCall(sequence);
  EXPECT_CALL(*m_dd_api, isTopologyValid(ut_consts::SDCS_FULL->m_initial.m_topology))
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  expectedDefaultTopologyGuardCall(sequence);
  expectedHdrWorkaroundCalls(sequence);
  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::TopologyIsInvalid);

  expectedDefaultCallsUntilModifiedSettingsNoPersistence(sequence);
  // No need for expectedDefaultRevertModifiedSettingsCall anymore.
  expectedDefaultInitialTopologyCalls(sequence);
  expectedDefaultFinalPersistenceCalls(sequence);
  expectedDefaultAudioContextCalls(sequence, false);
  expectedHdrWorkaroundCalls(sequence);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);
}

TEST_F_S_MOCKED(CurrentSettingsMatchPersistentOnes) {
  display_device::SingleDisplayConfigState state {
    {CURRENT_TOPOLOGY,
     {"DeviceId4"}},
    {CURRENT_TOPOLOGY,
     CURRENT_MODIFIED_DISPLAY_MODES,
     CURRENT_MODIFIED_HDR_STATES,
     "DeviceId4"}
  };

  InSequence sequence;
  expectedDefaultCallsUntilModifiedSettings(sequence, state);
  EXPECT_CALL(*m_dd_api, isTopologyValid(state.m_modified.m_topology)).Times(1).WillOnce(Return(true)).RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, state.m_modified.m_topology)).Times(1).WillOnce(Return(true)).RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, getCurrentHdrStates(display_device::win_utils::flattenTopology(state.m_modified.m_topology))).Times(1).WillOnce(Return(CURRENT_MODIFIED_HDR_STATES)).RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, getCurrentDisplayModes(display_device::win_utils::flattenTopology(state.m_modified.m_topology))).Times(1).WillOnce(Return(CURRENT_MODIFIED_DISPLAY_MODES)).RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isPrimary(state.m_modified.m_original_primary_device)).Times(1).WillOnce(Return(true)).RetiresOnSaturation();

  auto cleared_modifications {state};
  cleared_modifications.m_modified = {cleared_modifications.m_modified.m_topology};
  EXPECT_CALL(*m_settings_persistence_api, store(*serializeState(cleared_modifications))).Times(1).WillOnce(Return(true)).RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyValid(state.m_initial.m_topology)).Times(1).WillOnce(Return(true)).RetiresOnSaturation();
  EXPECT_CALL(*m_dd_api, isTopologyTheSame(CURRENT_TOPOLOGY, state.m_initial.m_topology)).Times(1).WillOnce(Return(true)).RetiresOnSaturation();
  expectedDefaultFinalPersistenceCalls(sequence);
  expectedDefaultAudioContextCalls(sequence, false);

  EXPECT_EQ(getImpl().revertSettings(), display_device::SettingsManager::RevertResult::Ok);
}
