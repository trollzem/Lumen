/**
 * @file src/windows/settings_manager_revert.cpp
 * @brief Definitions for the methods for reverting settings in SettingsManager.
 */
// class header include
#include "display_device/windows/settings_manager.h"

// system includes
#include <boost/scope/scope_exit.hpp>

// local includes
#include "display_device/logging.h"
#include "display_device/windows/json.h"
#include "display_device/windows/settings_utils.h"

namespace display_device {
  namespace {
    /**
     * @brief Function that does nothing.
     */
    void noopFn() {
    }
  }  // namespace

  SettingsManager::RevertResult SettingsManager::revertSettings() {
    const auto &cached_state {m_persistence_state->getState()};
    if (!cached_state) {
      return RevertResult::Ok;
    }

    const auto api_access {m_dd_api->isApiAccessAvailable()};
    DD_LOG(info) << "Trying to revert applied display device settings. API is available: " << toJson(api_access);

    if (!api_access) {
      return RevertResult::ApiTemporarilyUnavailable;
    }

    const auto current_topology {m_dd_api->getCurrentTopology()};
    if (!m_dd_api->isTopologyValid(current_topology)) {
      DD_LOG(error) << "Retrieved current topology is invalid:\n"
                    << toJson(current_topology);
      return RevertResult::TopologyIsInvalid;
    }

    bool system_settings_touched {false};
    boost::scope::scope_exit hdr_blank_always_executed_guard {[this, &system_settings_touched]() {
      if (system_settings_touched) {
        win_utils::blankHdrStates(*m_dd_api, m_workarounds.m_hdr_blank_delay);
      }
    }};
    boost::scope::scope_exit topology_prep_guard {[this, &current_topology, &system_settings_touched]() {
      auto topology_to_restore {win_utils::createFullExtendedTopology(*m_dd_api)};
      if (!m_dd_api->isTopologyValid(topology_to_restore)) {
        topology_to_restore = current_topology;
      }

      const bool is_topology_the_same {m_dd_api->isTopologyTheSame(current_topology, topology_to_restore)};
      system_settings_touched = system_settings_touched || !is_topology_the_same;
      if (!is_topology_the_same && !m_dd_api->setTopology(topology_to_restore)) {
        DD_LOG(error) << "failed to revert topology in revertSettings topology guard! Used the following topology:\n"
                      << toJson(topology_to_restore);
      }
    }};

    // We can revert the modified setting independently before playing around with initial topology.
    bool switched_to_modified_topology {false};
    if (const auto result = revertModifiedSettings(current_topology, system_settings_touched, &switched_to_modified_topology); result != RevertResult::Ok) {
      // Error already logged
      return result;
    }

    if (!m_dd_api->isTopologyValid(cached_state->m_initial.m_topology)) {
      DD_LOG(error) << "Trying to revert to an invalid initial topology:\n"
                    << toJson(cached_state->m_initial.m_topology);
      return RevertResult::TopologyIsInvalid;
    }

    const bool is_topology_the_same {m_dd_api->isTopologyTheSame(current_topology, cached_state->m_initial.m_topology)};
    const bool need_to_switch_topology {!is_topology_the_same || switched_to_modified_topology};
    system_settings_touched = system_settings_touched || !is_topology_the_same;
    if (need_to_switch_topology && !m_dd_api->setTopology(cached_state->m_initial.m_topology)) {
      DD_LOG(error) << "Failed to change topology to:\n"
                    << toJson(cached_state->m_initial.m_topology);
      return RevertResult::SwitchingTopologyFailed;
    }

    if (!m_persistence_state->persistState(std::nullopt)) {
      DD_LOG(error) << "Failed to save reverted settings! Undoing initial topology changes...";
      return RevertResult::PersistenceSaveFailed;
    }

    if (m_audio_context_api->isCaptured()) {
      m_audio_context_api->release();
    }

    // Disable guards
    topology_prep_guard.set_active(false);
    return RevertResult::Ok;
  }

  SettingsManager::RevertResult SettingsManager::revertModifiedSettings(const ActiveTopology &current_topology, bool &system_settings_touched, bool *switched_topology) {
    const auto &cached_state {m_persistence_state->getState()};
    if (!cached_state || !cached_state->m_modified.hasModifications()) {
      return RevertResult::Ok;
    }

    if (!m_dd_api->isTopologyValid(cached_state->m_modified.m_topology)) {
      DD_LOG(error) << "Trying to revert modified settings using invalid topology:\n"
                    << toJson(cached_state->m_modified.m_topology);
      return RevertResult::TopologyIsInvalid;
    }

    const bool is_topology_the_same {m_dd_api->isTopologyTheSame(current_topology, cached_state->m_modified.m_topology)};
    system_settings_touched = !is_topology_the_same;
    if (!is_topology_the_same && !m_dd_api->setTopology(cached_state->m_modified.m_topology)) {
      DD_LOG(error) << "Failed to change topology to:\n"
                    << toJson(cached_state->m_modified.m_topology);
      return RevertResult::SwitchingTopologyFailed;
    }
    if (switched_topology) {
      *switched_topology = !is_topology_the_same;
    }

    DdGuardFn hdr_guard_fn {noopFn};
    boost::scope::scope_exit<DdGuardFn &> hdr_guard {hdr_guard_fn};
    if (!cached_state->m_modified.m_original_hdr_states.empty()) {
      const auto current_states {m_dd_api->getCurrentHdrStates(win_utils::flattenTopology(cached_state->m_modified.m_topology))};
      if (current_states != cached_state->m_modified.m_original_hdr_states) {
        system_settings_touched = true;

        DD_LOG(info) << "Trying to change back the HDR states to:\n"
                     << toJson(cached_state->m_modified.m_original_hdr_states);
        if (!m_dd_api->setHdrStates(cached_state->m_modified.m_original_hdr_states)) {
          // Error already logged
          return RevertResult::RevertingHdrStatesFailed;
        }

        hdr_guard_fn = win_utils::hdrStateGuardFn(*m_dd_api, current_states);
      }
    }

    DdGuardFn mode_guard_fn {noopFn};
    boost::scope::scope_exit<DdGuardFn &> mode_guard {mode_guard_fn};
    if (!cached_state->m_modified.m_original_modes.empty()) {
      const auto current_modes {m_dd_api->getCurrentDisplayModes(win_utils::flattenTopology(cached_state->m_modified.m_topology))};
      if (current_modes != cached_state->m_modified.m_original_modes) {
        DD_LOG(info) << "Trying to change back the display modes to:\n"
                     << toJson(cached_state->m_modified.m_original_modes);
        if (!m_dd_api->setDisplayModes(cached_state->m_modified.m_original_modes)) {
          system_settings_touched = true;
          // Error already logged
          return RevertResult::RevertingDisplayModesFailed;
        }

        // It is possible that the display modes will not actually change even though the "current != new" condition is true.
        // This is because of some additional internal checks that determine whether the change is actually needed.
        // Therefore, we should check the current display modes after the fact!
        if (current_modes != m_dd_api->getCurrentDisplayModes(win_utils::flattenTopology(cached_state->m_modified.m_topology))) {
          system_settings_touched = true;
          mode_guard_fn = win_utils::modeGuardFn(*m_dd_api, current_modes);
        }
      }
    }

    DdGuardFn primary_guard_fn {noopFn};
    boost::scope::scope_exit<DdGuardFn &> primary_guard {primary_guard_fn};
    if (!cached_state->m_modified.m_original_primary_device.empty()) {
      const auto current_primary_device {win_utils::getPrimaryDevice(*m_dd_api, cached_state->m_modified.m_topology)};
      if (current_primary_device != cached_state->m_modified.m_original_primary_device) {
        system_settings_touched = true;

        DD_LOG(info) << "Trying to change back the original primary device to: " << toJson(cached_state->m_modified.m_original_primary_device);
        if (!m_dd_api->setAsPrimary(cached_state->m_modified.m_original_primary_device)) {
          // Error already logged
          return RevertResult::RevertingPrimaryDeviceFailed;
        }

        primary_guard_fn = win_utils::primaryGuardFn(*m_dd_api, current_primary_device);
      }
    }

    auto cleared_data {*cached_state};
    cleared_data.m_modified = {cleared_data.m_modified.m_topology};
    if (!m_persistence_state->persistState(cleared_data)) {
      DD_LOG(error) << "Failed to save reverted settings! Undoing changes to modified topology...";
      return RevertResult::PersistenceSaveFailed;
    }

    // Disable guards
    hdr_guard.set_active(false);
    mode_guard.set_active(false);
    primary_guard.set_active(false);
    return RevertResult::Ok;
  }
}  // namespace display_device
