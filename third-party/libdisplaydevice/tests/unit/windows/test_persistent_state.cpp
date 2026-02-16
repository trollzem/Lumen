// local includes
#include "display_device/noop_settings_persistence.h"
#include "display_device/windows/settings_manager.h"
#include "fixtures/fixtures.h"
#include "fixtures/mock_settings_persistence.h"
#include "utils/comparison.h"
#include "utils/helpers.h"
#include "utils/mock_win_display_device.h"

namespace {
  // Convenience keywords for GMock
  using ::testing::HasSubstr;
  using ::testing::Return;
  using ::testing::StrictMock;

  // Test fixture(s) for this file
  class PersistentStateMocked: public BaseTest {
  public:
    display_device::PersistentState &getImpl(bool throw_on_load_error = false) {
      if (!m_impl) {
        m_impl = std::make_unique<display_device::PersistentState>(m_settings_persistence_api, throw_on_load_error);
      }

      return *m_impl;
    }

    std::shared_ptr<StrictMock<display_device::MockSettingsPersistence>> m_settings_persistence_api {std::make_shared<StrictMock<display_device::MockSettingsPersistence>>()};

  private:
    std::unique_ptr<display_device::PersistentState> m_impl;
  };

  // Specialized TEST macro(s) for this test
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, PersistentStateMocked, __VA_ARGS__)
}  // namespace

TEST_F_S_MOCKED(NoopSettingsPersistence) {
  class NakedPersistentState: public display_device::PersistentState {
  public:
    using PersistentState::m_settings_persistence_api;
    using PersistentState::PersistentState;
  };

  const NakedPersistentState persistent_state {nullptr};
  EXPECT_TRUE(std::dynamic_pointer_cast<display_device::NoopSettingsPersistence>(persistent_state.m_settings_persistence_api) != nullptr);
}

TEST_F_S_MOCKED(FailedToLoadPersitence) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_NULL)));

  EXPECT_THAT([this]() {
    getImpl(true);
  },
              ThrowsMessage<std::runtime_error>(HasSubstr("Failed to load persistent settings!")));
}

TEST_F_S_MOCKED(FailedToLoadPersitence, ThrowIsSuppressed) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_NULL)));

  EXPECT_EQ(getImpl(false).getState(), std::nullopt);
}

TEST_F_S_MOCKED(InvalidPersitenceData) {
  const std::string data_string {"SOMETHING"};
  const std::vector<std::uint8_t> data {std::begin(data_string), std::end(data_string)};

  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(data));

  EXPECT_THAT([this]() {
    getImpl(true);
  },
              ThrowsMessage<std::runtime_error>(HasSubstr("Failed to parse persistent settings! Error:\n"
                                                          "[json.exception.parse_error.101] parse error at line 1, column 1: syntax error while parsing value - invalid literal; last read: 'S'")));
}

TEST_F_S_MOCKED(InvalidPersitenceData, ThrowIsSuppressed) {
  const std::string data_string {"SOMETHING"};
  const std::vector<std::uint8_t> data {std::begin(data_string), std::end(data_string)};

  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(data));

  EXPECT_EQ(getImpl(false).getState(), std::nullopt);
}

TEST_F_S_MOCKED(NothingIsThrownOnSuccess) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));

  EXPECT_EQ(getImpl(true).getState(), ut_consts::SDCS_FULL);
}

TEST_F_S_MOCKED(FailedToPersistState, ClearFailed) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));
  EXPECT_CALL(*m_settings_persistence_api, clear())
    .Times(1)
    .WillOnce(Return(false));

  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_FULL);
  EXPECT_FALSE(getImpl().persistState(ut_consts::SDCS_NULL));
  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_FULL);
}

TEST_F_S_MOCKED(FailedToPersistState, BadJsonEncoding) {
  display_device::SingleDisplayConfigState invalid_state;
  invalid_state.m_modified.m_original_primary_device = "InvalidDeviceName\xC2";

  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_NO_MODIFICATIONS)));

  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_NO_MODIFICATIONS);
  EXPECT_FALSE(getImpl().persistState(invalid_state));
  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_NO_MODIFICATIONS);
}

TEST_F_S_MOCKED(FailedToPersistState, StoreFailed) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_NO_MODIFICATIONS)));
  EXPECT_CALL(*m_settings_persistence_api, store(*serializeState(ut_consts::SDCS_FULL)))
    .Times(1)
    .WillOnce(Return(false));

  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_NO_MODIFICATIONS);
  EXPECT_FALSE(getImpl().persistState(ut_consts::SDCS_FULL));
  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_NO_MODIFICATIONS);
}

TEST_F_S_MOCKED(ClearState) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));
  EXPECT_CALL(*m_settings_persistence_api, clear())
    .Times(1)
    .WillOnce(Return(true));

  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_FULL);
  EXPECT_TRUE(getImpl().persistState(ut_consts::SDCS_NULL));
  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_NULL);
}

TEST_F_S_MOCKED(StoreState) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_NO_MODIFICATIONS)));
  EXPECT_CALL(*m_settings_persistence_api, store(*serializeState(ut_consts::SDCS_FULL)))
    .Times(1)
    .WillOnce(Return(true));

  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_NO_MODIFICATIONS);
  EXPECT_TRUE(getImpl().persistState(ut_consts::SDCS_FULL));
  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_FULL);
}

TEST_F_S_MOCKED(PersistStateSkippedDueToEqValues) {
  EXPECT_CALL(*m_settings_persistence_api, load())
    .Times(1)
    .WillOnce(Return(serializeState(ut_consts::SDCS_FULL)));

  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_FULL);
  EXPECT_TRUE(getImpl().persistState(ut_consts::SDCS_FULL));
  EXPECT_EQ(getImpl().getState(), ut_consts::SDCS_FULL);
}
