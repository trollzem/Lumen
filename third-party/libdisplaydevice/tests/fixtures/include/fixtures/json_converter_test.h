#pragma once

// local includes
#include "display_device/json.h"
#include "fixtures.h"

class JsonConverterTest: public BaseTest {
public:
  template<class T>
  void executeTestCase(const T &input, const std::string &expected_string) {
    bool success {false};
    const auto json_string {display_device::toJson(input, std::nullopt, &success)};
    EXPECT_TRUE(success);
    EXPECT_EQ(json_string, expected_string);

    std::string error_message {};
    T defaulted_input {};
    if (!display_device::fromJson(json_string, defaulted_input, &error_message)) {
      GTEST_FAIL() << error_message;
    }
    EXPECT_EQ(input, defaulted_input);
  }
};
