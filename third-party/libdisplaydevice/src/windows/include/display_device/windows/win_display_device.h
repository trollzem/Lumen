/**
 * @file src/windows/include/display_device/windows/win_display_device.h
 * @brief Declarations for the WinDisplayDevice.
 */
#pragma once

// system includes
#include <memory>

// local includes
#include "win_api_layer_interface.h"
#include "win_display_device_interface.h"

namespace display_device {
  /**
   * @brief Default implementation for the WinDisplayDeviceInterface.
   */
  class WinDisplayDevice: public WinDisplayDeviceInterface {
  public:
    /**
     * Default constructor for the class.
     * @param w_api A pointer to the Windows API layer. Will throw on nullptr!
     */
    explicit WinDisplayDevice(std::shared_ptr<WinApiLayerInterface> w_api);

    /** For details @see WinDisplayDeviceInterface::isApiAccessAvailable */
    [[nodiscard]] bool isApiAccessAvailable() const override;

    /** For details @see WinDisplayDeviceInterface::enumAvailableDevices */
    [[nodiscard]] EnumeratedDeviceList enumAvailableDevices() const override;

    /** For details @see WinDisplayDeviceInterface::getDisplayName */
    [[nodiscard]] std::string getDisplayName(const std::string &device_id) const override;

    /** For details @see WinDisplayDeviceInterface::getCurrentTopology */
    [[nodiscard]] ActiveTopology getCurrentTopology() const override;

    /** For details @see WinDisplayDeviceInterface::isTopologyValid */
    [[nodiscard]] bool isTopologyValid(const ActiveTopology &topology) const override;

    /** For details @see WinDisplayDeviceInterface::getCurrentTopology */
    [[nodiscard]] bool isTopologyTheSame(const ActiveTopology &lhs, const ActiveTopology &rhs) const override;

    /** For details @see WinDisplayDeviceInterface::setTopology */
    [[nodiscard]] bool setTopology(const ActiveTopology &new_topology) override;

    /** For details @see WinDisplayDeviceInterface::getCurrentDisplayModes */
    [[nodiscard]] DeviceDisplayModeMap getCurrentDisplayModes(const std::set<std::string> &device_ids) const override;

    /** For details @see WinDisplayDeviceInterface::setDisplayModes */
    [[nodiscard]] bool setDisplayModes(const DeviceDisplayModeMap &modes) override;

    /** For details @see WinDisplayDeviceInterface::isPrimary */
    [[nodiscard]] bool isPrimary(const std::string &device_id) const override;

    /** For details @see WinDisplayDeviceInterface::setAsPrimary */
    [[nodiscard]] bool setAsPrimary(const std::string &device_id) override;

    /** For details @see WinDisplayDeviceInterface::getCurrentHdrStates */
    [[nodiscard]] HdrStateMap getCurrentHdrStates(const std::set<std::string> &device_ids) const override;

    /** For details @see WinDisplayDeviceInterface::setHdrStates */
    [[nodiscard]] bool setHdrStates(const HdrStateMap &states) override;

  private:
    std::shared_ptr<WinApiLayerInterface> m_w_api;
  };
}  // namespace display_device
