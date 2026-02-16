// local includes
#include "fixtures/json_converter_test.h"

namespace {
  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, JsonConverterTest, __VA_ARGS__)
}  // namespace

TEST_F_S(EdidData) {
  display_device::EdidData item {
    .m_manufacturer_id = "LOL",
    .m_product_code = "ABCD",
    .m_serial_number = 777777
  };

  executeTestCase(display_device::EdidData {}, R"({"manufacturer_id":"","product_code":"","serial_number":0})");
  executeTestCase(item, R"({"manufacturer_id":"LOL","product_code":"ABCD","serial_number":777777})");
}

TEST_F_S(EnumeratedDevice) {
  display_device::EnumeratedDevice item_1 {
    "ID_1",
    "NAME_2",
    "FU_NAME_3",
    std::nullopt,
    display_device::EnumeratedDevice::Info {
      {1920, 1080},
      display_device::Rational {175, 100},
      119.9554,
      false,
      {1, 2},
      display_device::HdrState::Enabled
    }
  };
  display_device::EnumeratedDevice item_2 {
    "ID_2",
    "NAME_2",
    "FU_NAME_2",
    display_device::EdidData {},
    display_device::EnumeratedDevice::Info {
      {1920, 1080},
      1.75,
      display_device::Rational {1199554, 10000},
      true,
      {0, 0},
      display_device::HdrState::Disabled
    }
  };

  executeTestCase(display_device::EnumeratedDevice {}, R"({"device_id":"","display_name":"","edid":null,"friendly_name":"","info":null})");
  executeTestCase(item_1, R"({"device_id":"ID_1","display_name":"NAME_2","edid":null,"friendly_name":"FU_NAME_3","info":{"hdr_state":"Enabled","origin_point":{"x":1,"y":2},"primary":false,"refresh_rate":{"type":"double","value":119.9554},"resolution":{"height":1080,"width":1920},"resolution_scale":{"type":"rational","value":{"denominator":100,"numerator":175}}}})");
  executeTestCase(item_2, R"({"device_id":"ID_2","display_name":"NAME_2","edid":{"manufacturer_id":"","product_code":"","serial_number":0},"friendly_name":"FU_NAME_2","info":{"hdr_state":"Disabled","origin_point":{"x":0,"y":0},"primary":true,"refresh_rate":{"type":"rational","value":{"denominator":10000,"numerator":1199554}},"resolution":{"height":1080,"width":1920},"resolution_scale":{"type":"double","value":1.75}}})");
}

TEST_F_S(EnumeratedDeviceList) {
  display_device::EnumeratedDevice item_1 {
    "ID_1",
    "NAME_2",
    "FU_NAME_3",
    std::nullopt,
    display_device::EnumeratedDevice::Info {
      {1920, 1080},
      display_device::Rational {175, 100},
      119.9554,
      false,
      {1, 2},
      display_device::HdrState::Enabled
    }
  };
  display_device::EnumeratedDevice item_2 {
    "ID_2",
    "NAME_2",
    "FU_NAME_2",
    display_device::EdidData {},
    display_device::EnumeratedDevice::Info {
      {1920, 1080},
      1.75,
      display_device::Rational {1199554, 10000},
      true,
      {0, 0},
      display_device::HdrState::Disabled
    }
  };
  display_device::EnumeratedDevice item_3 {};

  executeTestCase(display_device::EnumeratedDeviceList {}, R"([])");
  executeTestCase(display_device::EnumeratedDeviceList {item_1, item_2, item_3}, R"([{"device_id":"ID_1","display_name":"NAME_2","edid":null,"friendly_name":"FU_NAME_3","info":{"hdr_state":"Enabled","origin_point":{"x":1,"y":2},"primary":false,"refresh_rate":{"type":"double","value":119.9554},"resolution":{"height":1080,"width":1920},"resolution_scale":{"type":"rational","value":{"denominator":100,"numerator":175}}}},)"
                                                                                 R"({"device_id":"ID_2","display_name":"NAME_2","edid":{"manufacturer_id":"","product_code":"","serial_number":0},"friendly_name":"FU_NAME_2","info":{"hdr_state":"Disabled","origin_point":{"x":0,"y":0},"primary":true,"refresh_rate":{"type":"rational","value":{"denominator":10000,"numerator":1199554}},"resolution":{"height":1080,"width":1920},"resolution_scale":{"type":"double","value":1.75}}},)"
                                                                                 R"({"device_id":"","display_name":"","edid":null,"friendly_name":"","info":null}])");
}

TEST_F_S(SingleDisplayConfiguration) {
  display_device::SingleDisplayConfiguration config_1 {"ID_1", display_device::SingleDisplayConfiguration::DevicePreparation::VerifyOnly, {{156, 123}}, 85., display_device::HdrState::Enabled};
  display_device::SingleDisplayConfiguration config_2 {"ID_2", display_device::SingleDisplayConfiguration::DevicePreparation::EnsureActive, std::nullopt, display_device::Rational {85, 1}, display_device::HdrState::Disabled};
  display_device::SingleDisplayConfiguration config_3 {"ID_3", display_device::SingleDisplayConfiguration::DevicePreparation::EnsureOnlyDisplay, {{156, 123}}, std::nullopt, std::nullopt};
  display_device::SingleDisplayConfiguration config_4 {"ID_4", display_device::SingleDisplayConfiguration::DevicePreparation::EnsurePrimary, std::nullopt, std::nullopt, std::nullopt};

  executeTestCase(display_device::SingleDisplayConfiguration {}, R"({"device_id":"","device_prep":"VerifyOnly","hdr_state":null,"refresh_rate":null,"resolution":null})");
  executeTestCase(config_1, R"({"device_id":"ID_1","device_prep":"VerifyOnly","hdr_state":"Enabled","refresh_rate":{"type":"double","value":85.0},"resolution":{"height":123,"width":156}})");
  executeTestCase(config_2, R"({"device_id":"ID_2","device_prep":"EnsureActive","hdr_state":"Disabled","refresh_rate":{"type":"rational","value":{"denominator":1,"numerator":85}},"resolution":null})");
  executeTestCase(config_3, R"({"device_id":"ID_3","device_prep":"EnsureOnlyDisplay","hdr_state":null,"refresh_rate":null,"resolution":{"height":123,"width":156}})");
  executeTestCase(config_4, R"({"device_id":"ID_4","device_prep":"EnsurePrimary","hdr_state":null,"refresh_rate":null,"resolution":null})");
}

TEST_F_S(StringSet) {
  executeTestCase(std::set<std::string> {}, R"([])");
  executeTestCase(std::set<std::string> {"ABC", "DEF"}, R"(["ABC","DEF"])");
  executeTestCase(std::set<std::string> {"DEF", "ABC"}, R"(["ABC","DEF"])");
}

TEST_F_S(String) {
  executeTestCase(std::string {}, R"("")");
  executeTestCase(std::string {"ABC"}, R"("ABC")");
}

TEST_F_S(Bool) {
  executeTestCase(true, R"(true)");
  executeTestCase(false, R"(false)");
}
