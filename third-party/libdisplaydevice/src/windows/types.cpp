/**
 * @file src/windows/types.cpp
 * @brief Definitions for Windows specific types.
 */
// header include
#include "display_device/windows/types.h"

namespace display_device {
  bool operator==(const DisplayMode &lhs, const DisplayMode &rhs) {
    return lhs.m_refresh_rate == rhs.m_refresh_rate && lhs.m_resolution == rhs.m_resolution;
  }

  bool SingleDisplayConfigState::Modified::hasModifications() const {
    return !m_original_modes.empty() || !m_original_hdr_states.empty() || !m_original_primary_device.empty();
  }

  bool operator==(const SingleDisplayConfigState::Initial &lhs, const SingleDisplayConfigState::Initial &rhs) {
    return lhs.m_topology == rhs.m_topology && lhs.m_primary_devices == rhs.m_primary_devices;
  }

  bool operator==(const SingleDisplayConfigState::Modified &lhs, const SingleDisplayConfigState::Modified &rhs) {
    return lhs.m_topology == rhs.m_topology && lhs.m_original_modes == rhs.m_original_modes && lhs.m_original_hdr_states == rhs.m_original_hdr_states && lhs.m_original_primary_device == rhs.m_original_primary_device;
  }

  bool operator==(const SingleDisplayConfigState &lhs, const SingleDisplayConfigState &rhs) {
    return lhs.m_initial == rhs.m_initial && lhs.m_modified == rhs.m_modified;
  }

  bool operator==(const WinWorkarounds &lhs, const WinWorkarounds &rhs) {
    return lhs.m_hdr_blank_delay == rhs.m_hdr_blank_delay;
  }
}  // namespace display_device
