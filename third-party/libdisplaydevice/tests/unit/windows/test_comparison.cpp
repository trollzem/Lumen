// local includes
#include "display_device/windows/types.h"
#include "fixtures/fixtures.h"

namespace {
  // Specialized TEST macro(s) for this test file
#define TEST_S(...) DD_MAKE_TEST(TEST, TypeComparison, __VA_ARGS__)
}  // namespace

TEST_S(DisplayMode) {
  EXPECT_EQ(display_device::DisplayMode({1, 1}, {1, 1}), display_device::DisplayMode({1, 1}, {1, 1}));
  EXPECT_NE(display_device::DisplayMode({1, 1}, {1, 1}), display_device::DisplayMode({1, 0}, {1, 1}));
  EXPECT_NE(display_device::DisplayMode({1, 1}, {1, 1}), display_device::DisplayMode({1, 1}, {1, 0}));
}

TEST_S(SingleDisplayConfigState, Initial) {
  using Initial = display_device::SingleDisplayConfigState::Initial;
  EXPECT_EQ(Initial({{{"1"}}}, {"1"}), Initial({{{"1"}}}, {"1"}));
  EXPECT_NE(Initial({{{"1"}}}, {"1"}), Initial({{{"0"}}}, {"1"}));
  EXPECT_NE(Initial({{{"1"}}}, {"1"}), Initial({{{"1"}}}, {"0"}));
}

TEST_S(SingleDisplayConfigState, Modified) {
  using Modified = display_device::SingleDisplayConfigState::Modified;
  EXPECT_EQ(Modified({{{"1"}}}, {{"1", {}}}, {{"1", {}}}, "1"), Modified({{{"1"}}}, {{"1", {}}}, {{"1", {}}}, "1"));
  EXPECT_NE(Modified({{{"1"}}}, {{"1", {}}}, {{"1", {}}}, "1"), Modified({{{"0"}}}, {{"1", {}}}, {{"1", {}}}, "1"));
  EXPECT_NE(Modified({{{"1"}}}, {{"1", {}}}, {{"1", {}}}, "1"), Modified({{{"1"}}}, {{"0", {}}}, {{"1", {}}}, "1"));
  EXPECT_NE(Modified({{{"1"}}}, {{"1", {}}}, {{"1", {}}}, "1"), Modified({{{"1"}}}, {{"1", {}}}, {{"0", {}}}, "1"));
  EXPECT_NE(Modified({{{"1"}}}, {{"1", {}}}, {{"1", {}}}, "1"), Modified({{{"1"}}}, {{"1", {}}}, {{"1", {}}}, "0"));
}

TEST_S(SingleDisplayConfigState) {
  using SDSC = display_device::SingleDisplayConfigState;
  EXPECT_EQ(SDSC({{{"1"}}}, {{{"1"}}}), SDSC({{{"1"}}}, {{{"1"}}}));
  EXPECT_NE(SDSC({{{"1"}}}, {{{"1"}}}), SDSC({{{"0"}}}, {{{"1"}}}));
  EXPECT_NE(SDSC({{{"1"}}}, {{{"1"}}}), SDSC({{{"1"}}}, {{{"0"}}}));
}
