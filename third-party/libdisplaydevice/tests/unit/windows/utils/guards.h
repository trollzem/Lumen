#pragma once

// system includes
#include <boost/scope/scope_exit.hpp>

// local includes
#include "display_device/windows/win_display_device_interface.h"
#include "helpers.h"

// Helper functions to make guards for restoring previous state
boost::scope::scope_exit<display_device::DdGuardFn> makeTopologyGuard(display_device::WinDisplayDeviceInterface &win_dd);

boost::scope::scope_exit<display_device::DdGuardFn> makeModeGuard(display_device::WinDisplayDeviceInterface &win_dd);

boost::scope::scope_exit<display_device::DdGuardFn> makePrimaryGuard(display_device::WinDisplayDeviceInterface &win_dd);

boost::scope::scope_exit<display_device::DdGuardFn> makeHdrStateGuard(display_device::WinDisplayDeviceInterface &win_dd);
