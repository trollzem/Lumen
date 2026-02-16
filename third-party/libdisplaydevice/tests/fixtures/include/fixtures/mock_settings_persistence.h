#pragma once

// system includes
#include <gmock/gmock.h>

// local includes
#include "display_device/settings_persistence_interface.h"

namespace display_device {
  class MockSettingsPersistence: public SettingsPersistenceInterface {
  public:
    MOCK_METHOD(bool, store, (const std::vector<std::uint8_t> &), (override));
    MOCK_METHOD(std::optional<std::vector<std::uint8_t>>, load, (), (const, override));
    MOCK_METHOD(bool, clear, (), (override));
  };
}  // namespace display_device
