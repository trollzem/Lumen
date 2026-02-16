#pragma once

// system includes
#include <gmock/gmock.h>

// local includes
#include "display_device/windows/win_display_device.h"

namespace display_device {
  class MockWinDisplayDevice: public WinDisplayDeviceInterface {
  public:
    MOCK_METHOD(bool, isApiAccessAvailable, (), (const, override));
    MOCK_METHOD(EnumeratedDeviceList, enumAvailableDevices, (), (const, override));
    MOCK_METHOD(std::string, getDisplayName, (const std::string &), (const, override));
    MOCK_METHOD(ActiveTopology, getCurrentTopology, (), (const, override));
    MOCK_METHOD(bool, isTopologyValid, (const ActiveTopology &), (const, override));
    MOCK_METHOD(bool, isTopologyTheSame, (const ActiveTopology &, const ActiveTopology &), (const, override));
    MOCK_METHOD(bool, setTopology, (const ActiveTopology &), (override));
    MOCK_METHOD(DeviceDisplayModeMap, getCurrentDisplayModes, (const std::set<std::string> &), (const, override));
    MOCK_METHOD(bool, setDisplayModes, (const DeviceDisplayModeMap &), (override));
    MOCK_METHOD(bool, isPrimary, (const std::string &), (const, override));
    MOCK_METHOD(bool, setAsPrimary, (const std::string &), (override));
    MOCK_METHOD(HdrStateMap, getCurrentHdrStates, (const std::set<std::string> &), (const, override));
    MOCK_METHOD(bool, setHdrStates, (const HdrStateMap &), (override));
  };
}  // namespace display_device

/**
 * @brief Contains some useful predefined structures for UTs.
 * @note Data is to be extended with relevant information as needed.
 */
namespace ut_consts {
  extern const std::optional<display_device::SingleDisplayConfigState> SDCS_NULL;
  extern const std::optional<display_device::SingleDisplayConfigState> SDCS_EMPTY;
  extern const std::optional<display_device::SingleDisplayConfigState> SDCS_FULL;
  extern const std::optional<display_device::SingleDisplayConfigState> SDCS_NO_MODIFICATIONS;
}  // namespace ut_consts
