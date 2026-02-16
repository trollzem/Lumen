/**
 * @file src/windows/persistent_state.cpp
 * @brief Definitions for the PersistentState.
 */
// class header include
#include "display_device/windows/persistent_state.h"

// local includes
#include "display_device/logging.h"
#include "display_device/noop_settings_persistence.h"
#include "display_device/windows/json.h"

namespace display_device {
  PersistentState::PersistentState(std::shared_ptr<SettingsPersistenceInterface> settings_persistence_api, const bool throw_on_load_error):
      m_settings_persistence_api {std::move(settings_persistence_api)} {
    if (!m_settings_persistence_api) {
      m_settings_persistence_api = std::make_shared<NoopSettingsPersistence>();
    }

    std::string error_message;
    if (const auto persistent_settings {m_settings_persistence_api->load()}) {
      if (!persistent_settings->empty()) {
        m_cached_state = SingleDisplayConfigState {};
        if (!fromJson({std::begin(*persistent_settings), std::end(*persistent_settings)}, *m_cached_state, &error_message)) {
          error_message = "Failed to parse persistent settings! Error:\n" + error_message;
        }
      }
    } else {
      error_message = "Failed to load persistent settings!";
    }

    if (!error_message.empty()) {
      if (throw_on_load_error) {
        throw std::runtime_error {error_message};
      }

      DD_LOG(error) << error_message;
      m_cached_state = std::nullopt;
    }
  }

  bool PersistentState::persistState(const std::optional<SingleDisplayConfigState> &state) {
    if (m_cached_state == state) {
      return true;
    }

    if (!state) {
      if (!m_settings_persistence_api->clear()) {
        return false;
      }

      m_cached_state = std::nullopt;
      return true;
    }

    bool success {false};
    const auto json_string {toJson(*state, 2, &success)};
    if (!success) {
      DD_LOG(error) << "Failed to serialize new persistent state! Error:\n"
                    << json_string;
      return false;
    }

    if (!m_settings_persistence_api->store({std::begin(json_string), std::end(json_string)})) {
      return false;
    }

    m_cached_state = *state;
    return true;
  }

  const std::optional<SingleDisplayConfigState> &PersistentState::getState() const {
    return m_cached_state;
  }
}  // namespace display_device
