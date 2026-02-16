// special ordered include of details
#define DD_JSON_DETAIL
// clang-format off
#include "display_device/json.h"
#include "display_device/detail/json_serializer.h"
#include "display_device/detail/json_converter.h"
// clang-format on

// local includes
#include "fixtures/fixtures.h"

namespace display_device {
  enum class TestEnum {
    Value1,
    Value2,
    Value3
  };

  struct TestStruct {
    struct Nested {
      int m_c {};
    };

    std::string m_a {};
    Nested m_b {};
  };

  using TestVariant = std::variant<double, Rational>;

  bool operator==(const TestStruct::Nested &lhs, const TestStruct::Nested &rhs) {
    return lhs.m_c == rhs.m_c;
  }

  bool operator==(const TestStruct &lhs, const TestStruct &rhs) {
    return lhs.m_a == rhs.m_a && lhs.m_b == rhs.m_b;
  }

  DD_JSON_DEFINE_SERIALIZE_ENUM_GCOVR_EXCL_BR_LINE(TestEnum, {{TestEnum::Value1, "Value1"}, {TestEnum::Value2, "ValueMaybe2"}})
  DD_JSON_DEFINE_SERIALIZE_STRUCT(TestStruct::Nested, c)
  DD_JSON_DEFINE_SERIALIZE_STRUCT(TestStruct, a, b)

  DD_JSON_DEFINE_CONVERTER(TestEnum)
  DD_JSON_DEFINE_CONVERTER(TestStruct)
  DD_JSON_DEFINE_CONVERTER(TestVariant)
  DD_JSON_DEFINE_CONVERTER(std::chrono::nanoseconds)
  DD_JSON_DEFINE_CONVERTER(std::chrono::microseconds)
  DD_JSON_DEFINE_CONVERTER(std::chrono::milliseconds)
  DD_JSON_DEFINE_CONVERTER(std::chrono::seconds)
  DD_JSON_DEFINE_CONVERTER(std::chrono::minutes)
  DD_JSON_DEFINE_CONVERTER(std::chrono::hours)
  DD_JSON_DEFINE_CONVERTER(std::chrono::days)
  DD_JSON_DEFINE_CONVERTER(std::chrono::weeks)
  DD_JSON_DEFINE_CONVERTER(std::chrono::months)
  DD_JSON_DEFINE_CONVERTER(std::chrono::years)
}  // namespace display_device

namespace {
  // Specialized TEST macro(s) for this test file
#define TEST_S(...) DD_MAKE_TEST(TEST, JsonTest, __VA_ARGS__)

  // Additional convenience global const(s)
  constexpr auto MIN_NANO_VAL {std::numeric_limits<decltype(std::chrono::nanoseconds {}.count())>::min()};
  constexpr auto MAX_NANO_VAL {std::numeric_limits<decltype(std::chrono::nanoseconds {}.count())>::max()};
}  // namespace

TEST_S(ToJson, NoError, WithSuccessParam) {
  bool success {false};
  const auto json_string {display_device::toJson(display_device::TestStruct {}, std::nullopt, &success)};

  EXPECT_TRUE(success);
  EXPECT_EQ(json_string, R"({"a":"","b":{"c":0}})");
}

TEST_S(ToJson, NoError, WithoutSuccessParam) {
  const auto json_string {display_device::toJson(display_device::TestStruct {}, std::nullopt, nullptr)};
  EXPECT_EQ(json_string, R"({"a":"","b":{"c":0}})");
}

TEST_S(ToJson, Error, WithSuccessParam) {
  bool success {true};
  const auto json_string {display_device::toJson(display_device::TestStruct {"123\xC2"}, std::nullopt, &success)};

  EXPECT_FALSE(success);
  EXPECT_EQ(json_string, "[json.exception.type_error.316] incomplete UTF-8 string; last byte: 0xC2");
}

TEST_S(ToJson, Error, WithoutSuccessParam) {
  const auto json_string {display_device::toJson(display_device::TestStruct {"123\xC2"}, std::nullopt, nullptr)};
  EXPECT_EQ(json_string, "[json.exception.type_error.316] incomplete UTF-8 string; last byte: 0xC2");
}

TEST_S(ToJson, Compact) {
  const auto json_string {display_device::toJson(display_device::TestStruct {}, std::nullopt, nullptr)};
  EXPECT_EQ(json_string, "{\"a\":\"\",\"b\":{\"c\":0}}");
}

TEST_S(ToJson, NoIndent) {
  const auto json_string {display_device::toJson(display_device::TestStruct {}, 0, nullptr)};
  EXPECT_EQ(json_string, "{\n\"a\": \"\",\n\"b\": {\n\"c\": 0\n}\n}");
}

TEST_S(ToJson, WithIndent) {
  const auto json_string {display_device::toJson(display_device::TestStruct {}, 3, nullptr)};
  EXPECT_EQ(json_string, "{\n   \"a\": \"\",\n   \"b\": {\n      \"c\": 0\n   }\n}");
}

TEST_S(FromJson, NoError, WithErrorMessageParam) {
  display_device::TestStruct original {"A", {1}};
  display_device::TestStruct expected {"B", {2}};
  display_device::TestStruct copy {original};
  std::string error_message {"some_string"};

  EXPECT_EQ(original, copy);
  EXPECT_NE(copy, expected);
  EXPECT_FALSE(error_message.empty());

  EXPECT_TRUE(display_device::fromJson(R"({"a":"B","b":{"c":2}})", copy, &error_message));
  EXPECT_EQ(copy, expected);
  EXPECT_TRUE(error_message.empty());
}

TEST_S(FromJson, NoError, WithoutErrorMessageParam) {
  display_device::TestStruct original {"A", {1}};
  display_device::TestStruct expected {"B", {2}};
  display_device::TestStruct copy {original};

  EXPECT_EQ(original, copy);
  EXPECT_NE(copy, expected);

  EXPECT_TRUE(display_device::fromJson(R"({"a":"B","b":{"c":2}})", copy, nullptr));
  EXPECT_EQ(copy, expected);
}

TEST_S(FromJson, Error, WithErrorMessageParam) {
  display_device::TestStruct original {"A", {1}};
  display_device::TestStruct copy {original};
  std::string error_message {};

  EXPECT_EQ(original, copy);
  EXPECT_TRUE(error_message.empty());

  EXPECT_FALSE(display_device::fromJson(R"({"a":"B"})", copy, &error_message));
  EXPECT_EQ(original, copy);
  EXPECT_EQ(error_message, "[json.exception.out_of_range.403] key 'b' not found");
}

TEST_S(FromJson, Error, WithoutErrorMessageParam) {
  display_device::TestStruct original {"A", {1}};
  display_device::TestStruct copy {original};

  EXPECT_EQ(original, copy);

  EXPECT_FALSE(display_device::fromJson(R"({"a":"B"})", copy, nullptr));
  EXPECT_EQ(original, copy);
}

TEST_S(ToJson, Enum) {
  EXPECT_EQ(display_device::toJson(display_device::TestEnum::Value1, std::nullopt, nullptr), R"("Value1")");
  EXPECT_EQ(display_device::toJson(display_device::TestEnum::Value2, std::nullopt, nullptr), R"("ValueMaybe2")");
}

TEST_S(FromJson, Enum) {
  display_device::TestEnum value {};

  EXPECT_TRUE(display_device::fromJson(R"("ValueMaybe2")", value, nullptr));
  EXPECT_EQ(value, display_device::TestEnum::Value2);

  EXPECT_TRUE(display_device::fromJson(R"("Value1")", value, nullptr));
  EXPECT_EQ(value, display_device::TestEnum::Value1);
}

TEST_S(ToJson, Enum, MissingMappingValue) {
  const auto json_string {display_device::toJson(display_device::TestEnum::Value3, std::nullopt, nullptr)};
  EXPECT_EQ(json_string, "TestEnum is missing enum mapping!");
}

TEST_S(FromJson, Enum, MissingMappingValue) {
  display_device::TestEnum value {};
  std::string error_message {};

  EXPECT_FALSE(display_device::fromJson(R"("OtherValue")", value, &error_message));
  EXPECT_EQ(error_message, "TestEnum is missing enum mapping!");
}

TEST_S(ToJson, TestVariant) {
  EXPECT_EQ(toJson(display_device::TestVariant {123.}, std::nullopt, nullptr), R"({"type":"double","value":123.0})");
  EXPECT_EQ(toJson(display_device::TestVariant {display_device::Rational {1, 2}}, std::nullopt, nullptr), R"({"type":"rational","value":{"denominator":2,"numerator":1}})");
}

TEST_S(FromJson, TestVariant) {
  display_device::TestVariant variant {};

  EXPECT_TRUE(display_device::fromJson(R"({"type":"double","value":123.0})", variant, nullptr));
  EXPECT_EQ(std::get<double>(variant), 123.0);  // Relying on GTest to properly compare floats

  EXPECT_TRUE(display_device::fromJson(R"({"type":"rational","value":{"denominator":2,"numerator":1}})", variant, nullptr));
  EXPECT_EQ(std::get<display_device::Rational>(variant), display_device::Rational({1, 2}));
}

TEST_S(FromJson, TestVariant, UnknownVariantType) {
  display_device::TestVariant variant {};
  std::string error_message {};

  EXPECT_FALSE(display_device::fromJson(R"({"type":"SomeUnknownType","value":123.0})", variant, &error_message));
  EXPECT_EQ(error_message, "Could not parse variant from type SomeUnknownType!");
}

TEST_S(ToJson, ChronoDuration) {
  using namespace std::chrono;

  EXPECT_EQ(display_device::toJson(nanoseconds {2000000000}, std::nullopt, nullptr), R"(2000000000)");
  EXPECT_EQ(display_device::toJson(microseconds {2000000}, std::nullopt, nullptr), R"(2000000)");
  EXPECT_EQ(display_device::toJson(milliseconds {2000}, std::nullopt, nullptr), R"(2000)");
  EXPECT_EQ(display_device::toJson(seconds {2}, std::nullopt, nullptr), R"(2)");
  EXPECT_EQ(display_device::toJson(minutes {20}, std::nullopt, nullptr), R"(20)");
  EXPECT_EQ(display_device::toJson(hours {20}, std::nullopt, nullptr), R"(20)");
  EXPECT_EQ(display_device::toJson(days {20}, std::nullopt, nullptr), R"(20)");
  EXPECT_EQ(display_device::toJson(weeks {20}, std::nullopt, nullptr), R"(20)");
  EXPECT_EQ(display_device::toJson(months {20}, std::nullopt, nullptr), R"(20)");
  EXPECT_EQ(display_device::toJson(years {20}, std::nullopt, nullptr), R"(20)");
}

TEST_S(FromJson, ChronoDuration) {
  const auto doTest {[]<class T>(const std::string &string_input, T expected_value) {
    T value {};

    EXPECT_TRUE(display_device::fromJson(string_input, value, nullptr));
    EXPECT_EQ(value, expected_value);
  }};

  using namespace std::chrono;

  doTest(R"(2000000000)", nanoseconds {2000000000});
  doTest(R"(2000000)", microseconds {2000000});
  doTest(R"(2000)", milliseconds {2000});
  doTest(R"(2)", seconds {2});
  doTest(R"(20)", minutes {20});
  doTest(R"(20)", hours {20});
  doTest(R"(20)", days {20});
  doTest(R"(20)", weeks {20});
  doTest(R"(20)", months {20});
  doTest(R"(20)", years {20});
}

TEST_S(ToJson, ChronoDuration, Ranges) {
  EXPECT_EQ(display_device::toJson(std::chrono::nanoseconds {MIN_NANO_VAL}, std::nullopt, nullptr), std::to_string(MIN_NANO_VAL));
  EXPECT_EQ(display_device::toJson(std::chrono::nanoseconds {MAX_NANO_VAL}, std::nullopt, nullptr), std::to_string(MAX_NANO_VAL));
}

TEST_S(FromJson, ChronoDuration, Ranges) {
  std::chrono::nanoseconds value {};

  EXPECT_TRUE(display_device::fromJson(std::to_string(MIN_NANO_VAL), value, nullptr));
  EXPECT_EQ(value, std::chrono::nanoseconds {MIN_NANO_VAL});

  EXPECT_TRUE(display_device::fromJson(std::to_string(MAX_NANO_VAL), value, nullptr));
  EXPECT_EQ(value, std::chrono::nanoseconds {MAX_NANO_VAL});
}
