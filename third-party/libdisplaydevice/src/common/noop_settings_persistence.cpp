/**
 * @file src/common/noop_settings_persistence.cpp
 * @brief Definitions for NoopSettingsPersistence.
 */
// local includes
#include "display_device/noop_settings_persistence.h"

namespace display_device {
  bool NoopSettingsPersistence::store(const std::vector<std::uint8_t> &) {
    return true;
  }

  std::optional<std::vector<std::uint8_t>> NoopSettingsPersistence::load() const {
    return std::vector<std::uint8_t> {};
  }

  bool NoopSettingsPersistence::clear() {
    return true;
  }
}  // namespace display_device
