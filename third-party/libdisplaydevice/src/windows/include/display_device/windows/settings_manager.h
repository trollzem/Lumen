/**
 * @file src/windows/include/display_device/windows/settings_manager.h
 * @brief Declarations for the SettingsManager.
 */
#pragma once

// system includes
#include <memory>

// local includes
#include "display_device/audio_context_interface.h"
#include "display_device/settings_manager_interface.h"
#include "display_device/windows/win_display_device_interface.h"
#include "persistent_state.h"

namespace display_device {
  /**
   * @brief Default implementation for the SettingsManagerInterface.
   */
  class SettingsManager: public SettingsManagerInterface {
  public:
    /**
     * Default constructor for the class.
     * @param dd_api A pointer to the Windows Display Device interface. Will throw on nullptr!
     * @param audio_context_api [Optional] A pointer to the Audio Context interface.
     * @param persistent_state A pointer to a class for managing persistence.
     * @param workarounds Workaround settings for the APIs.
     */
    explicit SettingsManager(
      std::shared_ptr<WinDisplayDeviceInterface> dd_api,
      std::shared_ptr<AudioContextInterface> audio_context_api,
      std::unique_ptr<PersistentState> persistent_state,
      WinWorkarounds workarounds
    );

    /** For details @see SettingsManagerInterface::enumAvailableDevices */
    [[nodiscard]] EnumeratedDeviceList enumAvailableDevices() const override;

    /** For details @see SettingsManagerInterface::getDisplayName */
    [[nodiscard]] std::string getDisplayName(const std::string &device_id) const override;

    /** For details @see SettingsManagerInterface::applySettings */
    [[nodiscard]] ApplyResult applySettings(const SingleDisplayConfiguration &config) override;

    /** For details @see SettingsManagerInterface::revertSettings */
    [[nodiscard]] RevertResult revertSettings() override;

    /** For details @see SettingsManagerInterface::resetPersistence */
    [[nodiscard]] bool resetPersistence() override;

  protected:
    /**
     * @brief Preps the topology so that the further settings could be applied.
     * @param config Configuration to be used for preparing topology.
     * @param topology_before_changes The current topology before any changes.
     * @param release_context Specifies whether the audio context should be released at the very end IF everything else has succeeded.
     * @param system_settings_touched Inticates whether a "write" operation could have been performed on the OS.
     * @return A tuple of (new_state that is to be updated/persisted, device_to_configure, additional_devices_to_configure).
     */
    [[nodiscard]] std::optional<std::tuple<SingleDisplayConfigState, std::string, std::set<std::string>>> prepareTopology(const SingleDisplayConfiguration &config, const ActiveTopology &topology_before_changes, bool &release_context, bool &system_settings_touched);

    /**
     * @brief Changes or restores the primary device based on the cached state, new state and configuration.
     * @param config Configuration to be used for preparing primary device.
     * @param device_to_configure The main device to be used for preparation.
     * @param guard_fn Reference to the guard function which will be set to restore original state (if needed) in case something else fails down the line.
     * @param new_state Reference to the new state which is to be updated accordingly.
     * @param system_settings_touched Inticates whether a "write" operation could have been performed on the OS.
     * @return True if no errors have occured, false otherwise.
     */
    [[nodiscard]] bool preparePrimaryDevice(const SingleDisplayConfiguration &config, const std::string &device_to_configure, DdGuardFn &guard_fn, SingleDisplayConfigState &new_state, bool &system_settings_touched);

    /**
     * @brief Changes or restores the display modes based on the cached state, new state and configuration.
     * @param config Configuration to be used for preparing display modes.
     * @param device_to_configure The main device to be used for preparation.
     * @param additional_devices_to_configure Additional devices that should be configured.
     * @param guard_fn Reference to the guard function which will be set to restore original state (if needed) in case something else fails down the line.
     * @param new_state Reference to the new state which is to be updated accordingly.
     * @param system_settings_touched Inticates whether a "write" operation could have been performed on the OS.
     * @return True if no errors have occured, false otherwise.
     */
    [[nodiscard]] bool prepareDisplayModes(const SingleDisplayConfiguration &config, const std::string &device_to_configure, const std::set<std::string> &additional_devices_to_configure, DdGuardFn &guard_fn, SingleDisplayConfigState &new_state, bool &system_settings_touched);

    /**
     * @brief Changes or restores the HDR states based on the cached state, new state and configuration.
     * @param config Configuration to be used for preparing HDR states.
     * @param device_to_configure The main device to be used for preparation.
     * @param additional_devices_to_configure Additional devices that should be configured.
     * @param guard_fn Reference to the guard function which will be set to restore original state (if needed) in case something else fails down the line.
     * @param new_state Reference to the new state which is to be updated accordingly.
     * @param system_settings_touched Inticates whether a "write" operation could have been performed on the OS.
     * @return True if no errors have occured, false otherwise.
     */
    [[nodiscard]] bool prepareHdrStates(const SingleDisplayConfiguration &config, const std::string &device_to_configure, const std::set<std::string> &additional_devices_to_configure, DdGuardFn &guard_fn, SingleDisplayConfigState &new_state, bool &system_settings_touched);

    /**
     * @brief Try to revert the modified settings.
     * @param current_topology Topology before this method is called.
     * @param system_settings_touched Indicates whether a "write" operation could have been performed on the OS.
     * @param switched_topology [Optional] Indicates whether the current topology was switched to revert settings.
     * @returns Result enum indicating success or failure.
     * @warning The method assumes that the caller will ensure restoring the topology
     *          in case of a failure!
     */
    [[nodiscard]] RevertResult revertModifiedSettings(const ActiveTopology &current_topology, bool &system_settings_touched, bool *switched_topology = nullptr);

    std::shared_ptr<WinDisplayDeviceInterface> m_dd_api;
    std::shared_ptr<AudioContextInterface> m_audio_context_api;
    std::unique_ptr<PersistentState> m_persistence_state;
    WinWorkarounds m_workarounds;
  };
}  // namespace display_device
