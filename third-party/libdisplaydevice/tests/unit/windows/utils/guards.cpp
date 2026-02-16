// header include
#include "guards.h"

// local includes
#include "display_device/windows/settings_utils.h"

boost::scope::scope_exit<display_device::DdGuardFn> makeTopologyGuard(display_device::WinDisplayDeviceInterface &win_dd) {
  return boost::scope::scope_exit<display_device::DdGuardFn>(display_device::win_utils::topologyGuardFn(win_dd, win_dd.getCurrentTopology()));
}

boost::scope::scope_exit<display_device::DdGuardFn> makeModeGuard(display_device::WinDisplayDeviceInterface &win_dd) {
  return boost::scope::scope_exit<display_device::DdGuardFn>(display_device::win_utils::modeGuardFn(win_dd, win_dd.getCurrentTopology()));
}

boost::scope::scope_exit<display_device::DdGuardFn> makePrimaryGuard(display_device::WinDisplayDeviceInterface &win_dd) {
  return boost::scope::scope_exit<display_device::DdGuardFn>(display_device::win_utils::primaryGuardFn(win_dd, win_dd.getCurrentTopology()));
}

boost::scope::scope_exit<display_device::DdGuardFn> makeHdrStateGuard(display_device::WinDisplayDeviceInterface &win_dd) {
  return boost::scope::scope_exit<display_device::DdGuardFn>(display_device::win_utils::hdrStateGuardFn(win_dd, win_dd.getCurrentTopology()));
}
