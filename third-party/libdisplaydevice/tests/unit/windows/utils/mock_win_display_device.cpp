// header include
#include "mock_win_display_device.h"

// local includes
#include "helpers.h"

namespace ut_consts {
  const std::optional<display_device::SingleDisplayConfigState> SDCS_NULL {std::nullopt};
  const std::optional<display_device::SingleDisplayConfigState> SDCS_EMPTY {display_device::SingleDisplayConfigState {}};
  const std::optional<display_device::SingleDisplayConfigState> SDCS_FULL {[]() {
    const display_device::SingleDisplayConfigState state {
      {{{"DeviceId1"}},
       {"DeviceId1"}},
      {display_device::SingleDisplayConfigState::Modified {
        {{"DeviceId1"}, {"DeviceId3"}},
        {{"DeviceId1", {{1920, 1080}, {120, 1}}},
         {"DeviceId3", {{1920, 1080}, {60, 1}}}},
        {{"DeviceId1", {display_device::HdrState::Disabled}},
         {"DeviceId3", display_device::HdrState::Enabled}},
        {"DeviceId1"},
      }}
    };

    return state;
  }()};
  const std::optional<display_device::SingleDisplayConfigState> SDCS_NO_MODIFICATIONS {[]() {
    const display_device::SingleDisplayConfigState state {
      {{{"DeviceId1"}},
       {"DeviceId1"}},
      {display_device::SingleDisplayConfigState::Modified {
        {{"DeviceId1"}, {"DeviceId3"}}
      }}
    };

    return state;
  }()};
}  // namespace ut_consts
