// local includes
#include "comparison.h"

bool operator==(const LUID &lhs, const LUID &rhs) {
  return lhs.HighPart == rhs.HighPart && lhs.LowPart == rhs.LowPart;
}

bool operator==(const POINTL &lhs, const POINTL &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator==(const RECTL &lhs, const RECTL &rhs) {
  return lhs.bottom == rhs.bottom && lhs.left == rhs.left && lhs.right == rhs.right && lhs.top == rhs.top;
}

bool operator==(const DISPLAYCONFIG_RATIONAL &lhs, const DISPLAYCONFIG_RATIONAL &rhs) {
  return lhs.Denominator == rhs.Denominator && lhs.Numerator == rhs.Numerator;
}

bool operator==(const DISPLAYCONFIG_2DREGION &lhs, const DISPLAYCONFIG_2DREGION &rhs) {
  return lhs.cx == rhs.cx && lhs.cy == rhs.cy;
}

bool operator==(const DISPLAYCONFIG_PATH_SOURCE_INFO &lhs, const DISPLAYCONFIG_PATH_SOURCE_INFO &rhs) {
  // clang-format off
  return lhs.adapterId == rhs.adapterId &&
         lhs.id == rhs.id &&
         lhs.cloneGroupId == rhs.cloneGroupId &&
         lhs.sourceModeInfoIdx == rhs.sourceModeInfoIdx &&
         lhs.statusFlags == rhs.statusFlags;
  // clang-format on
}

bool operator==(const DISPLAYCONFIG_PATH_TARGET_INFO &lhs, const DISPLAYCONFIG_PATH_TARGET_INFO &rhs) {
  // clang-format off
  return lhs.adapterId == rhs.adapterId &&
         lhs.id == rhs.id &&
         lhs.desktopModeInfoIdx == rhs.desktopModeInfoIdx &&
         lhs.targetModeInfoIdx == rhs.targetModeInfoIdx &&
         lhs.outputTechnology == rhs.outputTechnology &&
         lhs.rotation == rhs.rotation &&
         lhs.scaling == rhs.scaling &&
         lhs.refreshRate == rhs.refreshRate &&
         lhs.scanLineOrdering == rhs.scanLineOrdering &&
         lhs.targetAvailable == rhs.targetAvailable &&
         lhs.statusFlags == rhs.statusFlags;
  // clang-format on
}

bool operator==(const DISPLAYCONFIG_PATH_INFO &lhs, const DISPLAYCONFIG_PATH_INFO &rhs) {
  return lhs.sourceInfo == rhs.sourceInfo && lhs.targetInfo == rhs.targetInfo && lhs.flags == rhs.flags;
}

bool operator==(const DISPLAYCONFIG_SOURCE_MODE &lhs, const DISPLAYCONFIG_SOURCE_MODE &rhs) {
  return lhs.width == rhs.width && lhs.height == rhs.height && lhs.pixelFormat == rhs.pixelFormat && lhs.position == rhs.position;
}

bool operator==(const DISPLAYCONFIG_VIDEO_SIGNAL_INFO &lhs, const DISPLAYCONFIG_VIDEO_SIGNAL_INFO &rhs) {
  // clang-format on
  return lhs.pixelRate == rhs.pixelRate &&
         lhs.hSyncFreq == rhs.hSyncFreq &&
         lhs.vSyncFreq == rhs.vSyncFreq &&
         lhs.activeSize == rhs.activeSize &&
         lhs.totalSize == rhs.totalSize &&
         lhs.videoStandard == rhs.videoStandard &&
         lhs.scanLineOrdering == rhs.scanLineOrdering;
  // clang-format oon
}

bool operator==(const DISPLAYCONFIG_TARGET_MODE &lhs, const DISPLAYCONFIG_TARGET_MODE &rhs) {
  return lhs.targetVideoSignalInfo == rhs.targetVideoSignalInfo;
}

bool operator==(const DISPLAYCONFIG_DESKTOP_IMAGE_INFO &lhs, const DISPLAYCONFIG_DESKTOP_IMAGE_INFO &rhs) {
  return lhs.PathSourceSize == rhs.PathSourceSize && lhs.DesktopImageRegion == rhs.DesktopImageRegion && lhs.DesktopImageClip == rhs.DesktopImageClip;
}

bool operator==(const DISPLAYCONFIG_MODE_INFO &lhs, const DISPLAYCONFIG_MODE_INFO &rhs) {
  if (lhs.infoType == rhs.infoType && lhs.id == rhs.id && lhs.adapterId == rhs.adapterId) {
    if (lhs.infoType == DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE) {
      return lhs.sourceMode == rhs.sourceMode;
    } else if (lhs.infoType == DISPLAYCONFIG_MODE_INFO_TYPE_TARGET) {
      return lhs.targetMode == rhs.targetMode;
    } else if (lhs.infoType == DISPLAYCONFIG_MODE_INFO_TYPE_DESKTOP_IMAGE) {
      // TODO: fix once implemented
      return false;
    } else {
      return true;
    }
  }
  return false;
}

namespace display_device {
  bool operator==(const PathSourceIndexData &lhs, const PathSourceIndexData &rhs) {
    return lhs.m_source_id_to_path_index == rhs.m_source_id_to_path_index && lhs.m_adapter_id == rhs.m_adapter_id && lhs.m_active_source == rhs.m_active_source;
  }
}  // namespace display_device
