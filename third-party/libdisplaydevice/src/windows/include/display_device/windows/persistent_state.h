/**
 * @file src/windows/include/display_device/windows/persistent_state.h
 * @brief Declarations for the PersistentState.
 */
#pragma once

// system includes
#include <memory>

// local includes
#include "display_device/settings_persistence_interface.h"
#include "display_device/windows/win_display_device_interface.h"

namespace display_device {
  /**
   * @brief A simple wrapper around the SettingsPersistenceInterface and cached local state to keep them in sync.
   */
  class PersistentState {
  public:
    /**
     * Default constructor for the class.
     * @param settings_persistence_api [Optional] A pointer to the Settings Persistence interface.
     * @param throw_on_load_error Specify whether to throw exception in constructor in case settings fail to load.
     */
    explicit PersistentState(std::shared_ptr<SettingsPersistenceInterface> settings_persistence_api, bool throw_on_load_error = false);

    /**
     * @brief Store the new state via the interface and cache it.
     * @param state New state to be set.
     * @return True if the state was succesfully updated, false otherwise.
     */
    [[nodiscard]] bool persistState(const std::optional<SingleDisplayConfigState> &state);

    /**
     * @brief Get cached state.
     * @return Cached state
     */
    [[nodiscard]] const std::optional<SingleDisplayConfigState> &getState() const;

  protected:
    std::shared_ptr<SettingsPersistenceInterface> m_settings_persistence_api;

  private:
    std::optional<SingleDisplayConfigState> m_cached_state;
  };
}  // namespace display_device
