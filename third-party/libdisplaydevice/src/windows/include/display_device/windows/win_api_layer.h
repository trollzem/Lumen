/**
 * @file src/windows/include/display_device/windows/win_api_layer.h
 * @brief Declarations for the WinApiLayer.
 */
#pragma once

// local includes
#include "win_api_layer_interface.h"

namespace display_device {
  /**
   * @brief Default implementation for the WinApiLayerInterface.
   */
  class WinApiLayer: public WinApiLayerInterface {
  public:
    /** For details @see WinApiLayerInterface::getErrorString */
    [[nodiscard]] std::string getErrorString(LONG error_code) const override;

    /** For details @see WinApiLayerInterface::queryDisplayConfig */
    [[nodiscard]] std::optional<PathAndModeData> queryDisplayConfig(QueryType type) const override;

    /** For details @see WinApiLayerInterface::getDeviceId */
    [[nodiscard]] std::string getDeviceId(const DISPLAYCONFIG_PATH_INFO &path) const override;

    /** For details @see WinApiLayerInterface::getEdid */
    [[nodiscard]] std::vector<std::byte> getEdid(const DISPLAYCONFIG_PATH_INFO &path) const override;

    /** For details @see WinApiLayerInterface::getMonitorDevicePath */
    [[nodiscard]] std::string getMonitorDevicePath(const DISPLAYCONFIG_PATH_INFO &path) const override;

    /** For details @see WinApiLayerInterface::getFriendlyName */
    [[nodiscard]] std::string getFriendlyName(const DISPLAYCONFIG_PATH_INFO &path) const override;

    /** For details @see WinApiLayerInterface::getDisplayName */
    [[nodiscard]] std::string getDisplayName(const DISPLAYCONFIG_PATH_INFO &path) const override;

    /** For details @see WinApiLayerInterface::setDisplayConfig */
    [[nodiscard]] LONG setDisplayConfig(std::vector<DISPLAYCONFIG_PATH_INFO> paths, std::vector<DISPLAYCONFIG_MODE_INFO> modes, UINT32 flags) override;

    /** For details @see WinApiLayerInterface::getHdrState */
    [[nodiscard]] std::optional<HdrState> getHdrState(const DISPLAYCONFIG_PATH_INFO &path) const override;

    /** For details @see WinApiLayerInterface::setHdrState */
    [[nodiscard]] bool setHdrState(const DISPLAYCONFIG_PATH_INFO &path, HdrState state) override;

    /** For details @see WinApiLayerInterface::getDisplayScale */
    [[nodiscard]] std::optional<Rational> getDisplayScale(const std::string &display_name, const DISPLAYCONFIG_SOURCE_MODE &source_mode) const override;
  };
}  // namespace display_device
