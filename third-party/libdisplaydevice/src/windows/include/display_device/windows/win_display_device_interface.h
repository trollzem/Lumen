/**
 * @file src/windows/include/display_device/windows/win_display_device_interface.h
 * @brief Declarations for the WinDisplayDeviceInterface.
 */
#pragma once

// system includes
#include <set>

// local includes
#include "display_device/windows/types.h"

namespace display_device {
  /**
   * @brief Higher level abstracted API for interacting Windows' display device(s).
   */
  class WinDisplayDeviceInterface {
  public:
    /**
     * @brief Default virtual destructor.
     */
    virtual ~WinDisplayDeviceInterface() = default;

    /**
     * @brief Check if the API for changing display settings is accessible.
     * @returns True if display settings can be changed, false otherwise.
     * @examples
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * const bool have_access { iface->isApiAccessAvailable() };
     * @examples_end
     */
    [[nodiscard]] virtual bool isApiAccessAvailable() const = 0;

    /**
     * @brief Enumerate the available (active and inactive) devices.
     * @returns A list of available devices.
     *          Empty list can also be returned if an error has occurred.
     * @examples
     * const auto devices { enumAvailableDevices() };
     * @examples_end
     */
    [[nodiscard]] virtual EnumeratedDeviceList enumAvailableDevices() const = 0;

    /**
     * @brief Get display name associated with the device.
     * @param device_id A device to get display name for.
     * @returns A display name for the device, or an empty string if the device is inactive or not found.
     *          Empty string can also be returned if an error has occurred.
     * @examples
     * const std::string device_id { "MY_DEVICE_ID" };
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * const std::string display_name = iface->getDisplayName(device_id);
     * @examples_end
     */
    [[nodiscard]] virtual std::string getDisplayName(const std::string &device_id) const = 0;

    /**
     * @brief Get the active (current) topology.
     * @returns A list representing the current topology.
     *          Empty list can also be returned if an error has occurred.
     * @examples
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * const auto current_topology { iface->getCurrentTopology() };
     * @examples_end
     */
    [[nodiscard]] virtual ActiveTopology getCurrentTopology() const = 0;

    /**
     * @brief Verify if the active topology is valid.
     *
     * This is mostly meant as a sanity check or to verify that it is still valid
     * after a manual modification to an existing topology.
     *
     * @param topology Topology to validate.
     * @returns True if it is valid, false otherwise.
     * @examples
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * auto current_topology { iface->getCurrentTopology() };
     * // Modify the current_topology
     * const bool is_valid = iface->isTopologyValid(current_topology);
     * @examples_end
     */
    [[nodiscard]] virtual bool isTopologyValid(const ActiveTopology &topology) const = 0;

    /**
     * @brief Check if the topologies are close enough to be considered the same by the OS.
     * @param lhs First topology to compare.
     * @param rhs Second topology to compare.
     * @returns True if topologies are close enough, false otherwise.
     * @examples
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * auto current_topology { iface->getCurrentTopology() };
     * auto new_topology { current_topology };
     * // Modify the new_topology
     * const bool is_the_same = iface->isTopologyTheSame(current_topology, new_topology);
     * @examples_end
     */
    [[nodiscard]] virtual bool isTopologyTheSame(const ActiveTopology &lhs, const ActiveTopology &rhs) const = 0;

    /**
     * @brief Set a new active topology for the OS.
     * @param new_topology New device topology to set.
     * @returns True if the new topology has been set, false otherwise.
     * @examples
     * WinDisplayDeviceInterface* iface = getIface(...);
     * auto current_topology { iface->getCurrentTopology() };
     * // Modify the current_topology
     * const bool success = iface->setTopology(current_topology);
     * @examples_end
     */
    [[nodiscard]] virtual bool setTopology(const ActiveTopology &new_topology) = 0;

    /**
     * @brief Get current display modes for the devices.
     * @param device_ids A list of devices to get the modes for.
     * @returns A map of device modes per a device or an empty map if a mode could not be found (e.g. device is inactive).
     *          Empty map can also be returned if an error has occurred.
     * @examples
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * const std::set<std::string> device_ids { "DEVICE_ID_1", "DEVICE_ID_2" };
     * const auto current_modes = iface->getCurrentDisplayModes(device_ids);
     * @examples_end
     */
    [[nodiscard]] virtual DeviceDisplayModeMap getCurrentDisplayModes(const std::set<std::string> &device_ids) const = 0;

    /**
     * @brief Set new display modes for the devices.
     * @param modes A map of modes to set.
     * @returns True if modes were set, false otherwise.
     * @warning if any of the specified devices are duplicated, modes be provided
     *          for duplicates too!
     * @examples
     * WinDisplayDeviceInterface* iface = getIface(...);
     * const std::string display_a { "MY_ID_1" };
     * const std::string display_b { "MY_ID_2" };
     * const auto success = iface->setDisplayModes({ { display_a, { { 1920, 1080 }, { 60, 1 } } },
     *                                               { display_b, { { 1920, 1080 }, { 120, 1 } } } });
     * @examples_end
     */
    [[nodiscard]] virtual bool setDisplayModes(const DeviceDisplayModeMap &modes) = 0;

    /**
     * @brief Check whether the specified device is primary.
     * @param device_id A device to perform the check for.
     * @returns True if the device is primary, false otherwise.
     * @examples
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * const std::string device_id { "MY_DEVICE_ID" };
     * const bool is_primary = iface->isPrimary(device_id);
     * @examples_end
     */
    [[nodiscard]] virtual bool isPrimary(const std::string &device_id) const = 0;

    /**
     * @brief Set the device as a primary display.
     * @param device_id A device to set as primary.
     * @returns True if the device is or was set as primary, false otherwise.
     * @note On Windows if the device is duplicated, the other duplicated device(-s) will also become a primary device.
     * @examples
     * WinDisplayDeviceInterface* iface = getIface(...);
     * const std::string device_id { "MY_DEVICE_ID" };
     * const bool success = iface->set_as_primary_device(device_id);
     * @examples_end
     */
    [[nodiscard]] virtual bool setAsPrimary(const std::string &device_id) = 0;

    /**
     * @brief Get HDR state for the devices.
     * @param device_ids A list of devices to get the HDR states for.
     * @returns A map of HDR states per a device or an empty map if an error has occurred.
     * @note On Windows the state cannot be retrieved until the device is active even if it supports it.
     * @examples
     * const WinDisplayDeviceInterface* iface = getIface(...);
     * const std::unordered_set<std::string> device_ids { "DEVICE_ID_1", "DEVICE_ID_2" };
     * const auto current_hdr_states = iface->getCurrentHdrStates(device_ids);
     * @examples_end
     */
    [[nodiscard]] virtual HdrStateMap getCurrentHdrStates(const std::set<std::string> &device_ids) const = 0;

    /**
     * @brief Set HDR states for the devices.
     * @param states A map of HDR states to set.
     * @returns True if HDR states were set, false otherwise.
     * @note If `unknown` states are provided, they will be silently ignored
     *       and current state will not be changed.
     * @examples
     * WinDisplayDeviceInterface* iface = getIface(...);
     * const std::string display_a { "MY_ID_1" };
     * const std::string display_b { "MY_ID_2" };
     * const auto success = iface->setHdrStates({ { display_a, HdrState::Enabled },
     *                                            { display_b, HdrState::Disabled } });
     * @examples_end
     */
    [[nodiscard]] virtual bool setHdrStates(const HdrStateMap &states) = 0;
  };
}  // namespace display_device
