#pragma once

// system includes
#include <gmock/gmock.h>

// local includes
#include "display_device/windows/win_api_layer_interface.h"

namespace display_device {
  class MockWinApiLayer: public WinApiLayerInterface {
  public:
    MOCK_METHOD(std::string, getErrorString, (LONG), (const, override));
    MOCK_METHOD(std::optional<PathAndModeData>, queryDisplayConfig, (QueryType), (const, override));
    MOCK_METHOD(std::string, getDeviceId, (const DISPLAYCONFIG_PATH_INFO &), (const, override));
    MOCK_METHOD(std::vector<std::byte>, getEdid, (const DISPLAYCONFIG_PATH_INFO &), (const, override));
    MOCK_METHOD(std::string, getMonitorDevicePath, (const DISPLAYCONFIG_PATH_INFO &), (const, override));
    MOCK_METHOD(std::string, getFriendlyName, (const DISPLAYCONFIG_PATH_INFO &), (const, override));
    MOCK_METHOD(std::string, getDisplayName, (const DISPLAYCONFIG_PATH_INFO &), (const, override));
    MOCK_METHOD(LONG, setDisplayConfig, (std::vector<DISPLAYCONFIG_PATH_INFO>, std::vector<DISPLAYCONFIG_MODE_INFO>, UINT32), (override));
    MOCK_METHOD(std::optional<HdrState>, getHdrState, (const DISPLAYCONFIG_PATH_INFO &), (const, override));
    MOCK_METHOD(bool, setHdrState, (const DISPLAYCONFIG_PATH_INFO &, HdrState), (override));
    MOCK_METHOD(std::optional<Rational>, getDisplayScale, (const std::string &, const DISPLAYCONFIG_SOURCE_MODE &), (const, override));
  };
}  // namespace display_device

/**
 * @brief Contains some useful predefined structures for UTs.
 * @note Data is to be extended with relevant information as needed.
 */
namespace ut_consts {
  extern const std::optional<display_device::PathAndModeData> PAM_NULL;
  extern const std::optional<display_device::PathAndModeData> PAM_EMPTY;
  extern const std::optional<display_device::PathAndModeData> PAM_3_ACTIVE;
  extern const std::optional<display_device::PathAndModeData> PAM_3_ACTIVE_WITH_INVALID_MODE_IDX;
  extern const std::optional<display_device::PathAndModeData> PAM_4_ACTIVE_WITH_2_DUPLICATES;
}  // namespace ut_consts
