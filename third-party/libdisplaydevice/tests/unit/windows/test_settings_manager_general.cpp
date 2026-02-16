// local includes
#include "display_device/noop_audio_context.h"
#include "display_device/noop_settings_persistence.h"
#include "display_device/windows/settings_manager.h"
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

  // Test fixture(s) for this file
  class SettingsManagerGeneralMocked: public BaseTest {
  public:
    display_device::SettingsManager &getImpl() {
      if (!m_impl) {
        m_impl = std::make_unique<display_device::SettingsManager>(m_dd_api, m_audio_context_api, std::make_unique<display_device::PersistentState>(m_settings_persistence_api), display_device::WinWorkarounds {});
      }

      return *m_impl;
    }

    std::shared_ptr<StrictMock<display_device::MockWinDisplayDevice>> m_dd_api {std::make_shared<StrictMock<display_device::MockWinDisplayDevice>>()};
    std::shared_ptr<StrictMock<display_device::MockSettingsPersistence>> m_settings_persistence_api {std::make_shared<StrictMock<display_device::MockSettingsPersistence>>()};
    std::shared_ptr<StrictMock<display_device::MockAudioContext>> m_audio_context_api {std::make_shared<StrictMock<display_device::MockAudioContext>>()};

  private:
    std::unique_ptr<display_device::SettingsManager> m_impl;
  };

  // Specialized TEST macro(s) for this test
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, SettingsManagerGeneralMocked, __VA_ARGS__)
}  // namespace

TEST_F_S_MOCKED(NullptrDisplayDeviceApiProvided) {
  EXPECT_THAT([]() {
    const display_device::SettingsManager settings_manager(nullptr, nullptr, nullptr, {});
  },
              ThrowsMessage<std::logic_error>(HasSubstr("Nullptr provided for WinDisplayDeviceInterface in SettingsManager!")));
}

TEST_F_S_MOCKED(NoopAudioContext) {
  class NakedSettingsManager: public display_device::SettingsManager {
  public:
    using SettingsManager::m_audio_context_api;
    using SettingsManager::SettingsManager;
  };

  const NakedSettingsManager settings_manager {m_dd_api, nullptr, std::make_unique<display_device::PersistentState>(nullptr), {}};
  EXPECT_TRUE(std::dynamic_pointer_cast<display_device::NoopAudioContext>(settings_manager.m_audio_context_api) != nullptr);
}

TEST_F_S_MOCKED(NullptrPersistentStateProvided) {
  EXPECT_THAT([this]() {
    const display_device::SettingsManager settings_manager(m_dd_api, nullptr, nullptr, {});
  },
              ThrowsMessage<std::logic_error>(HasSubstr("Nullptr provided for PersistentState in SettingsManager!")));
}

TEST_F_S_MOCKED(EnumAvailableDevices) {
  const display_device::EnumeratedDeviceList test_list {
    {"DeviceId1",
     "",
     "FriendlyName1",
     std::nullopt}
  };

  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_EMPTY)));
  EXPECT_CALL(*m_dd_api, enumAvailableDevices())
    .Times(1)
    .WillOnce(Return(test_list));

  EXPECT_EQ(getImpl().enumAvailableDevices(), test_list);
}

TEST_F_S_MOCKED(GetDisplayName) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_EMPTY)));
  EXPECT_CALL(*m_dd_api, getDisplayName("DeviceId1"))
    .Times(1)
    .WillOnce(Return("DeviceName1"));

  EXPECT_EQ(getImpl().getDisplayName("DeviceId1"), "DeviceName1");
}

TEST_F_S_MOCKED(ResetPersistence, NoPersistence) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_EMPTY)));

  EXPECT_TRUE(getImpl().resetPersistence());
}

TEST_F_S_MOCKED(ResetPersistence, FailedToReset) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));
  EXPECT_CALL(*m_settings_persistence_api, clear())
    .Times(1)
    .WillOnce(Return(false));

  EXPECT_FALSE(getImpl().resetPersistence());
}

TEST_F_S_MOCKED(ResetPersistence, PersistenceReset, NoCapturedDevice) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));
  EXPECT_CALL(*m_settings_persistence_api, clear())
    .Times(1)
    .WillOnce(Return(true));
  EXPECT_CALL(*m_audio_context_api, isCaptured())
    .Times(1)
    .WillOnce(Return(false))
    .RetiresOnSaturation();

  EXPECT_TRUE(getImpl().resetPersistence());
}

TEST_F_S_MOCKED(ResetPersistence, PersistenceReset, WithCapturedDevice) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));
  EXPECT_CALL(*m_settings_persistence_api, clear())
    .Times(1)
    .WillOnce(Return(true));
  EXPECT_CALL(*m_audio_context_api, isCaptured())
    .Times(1)
    .WillOnce(Return(true));
  EXPECT_CALL(*m_audio_context_api, release())
    .Times(1);

  EXPECT_TRUE(getImpl().resetPersistence());
}
