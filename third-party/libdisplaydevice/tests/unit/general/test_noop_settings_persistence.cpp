// local includes
#include "display_device/noop_settings_persistence.h"
#include "fixtures/fixtures.h"

namespace {
  // Test fixture(s) for this file
  class NoopSettingsPersistenceTest: public BaseTest {
  public:
    display_device::NoopSettingsPersistence m_impl;
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, NoopSettingsPersistenceTest, __VA_ARGS__)
}  // namespace

TEST_F_S(Store) {
  EXPECT_TRUE(m_impl.store({}));
  EXPECT_TRUE(m_impl.store({0x01, 0x02, 0x03}));
}

TEST_F_S(Load) {
  EXPECT_EQ(m_impl.load(), std::vector<std::uint8_t> {});
}

TEST_F_S(Clear) {
  EXPECT_TRUE(m_impl.clear());
}
