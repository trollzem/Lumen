/**
 * @file src/windows/win_api_layer.cpp
 * @brief Definitions for the WinApiLayer.
 */
// class header include
#include "display_device/windows/win_api_layer.h"

// system includes
#include <boost/algorithm/string.hpp>
#include <boost/scope/scope_exit.hpp>
#include <boost/uuid/name_generator_sha1.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cmath>
#include <cstdint>
#include <iomanip>

// local includes
#include "display_device/logging.h"

// Windows includes after "windows.h"
#include <SetupApi.h>

namespace display_device {
  namespace {
    /** @brief Dumps the result of @see queryDisplayConfig into a string */
    std::string dumpPath(const DISPLAYCONFIG_PATH_INFO &info) {
      std::ostringstream output;
      std::ios state(nullptr);
      state.copyfmt(output);

      // clang-format off
      output << "sourceInfo:" << std::endl;
      output << "    adapterId: [" << info.sourceInfo.adapterId.HighPart << ", " << info.sourceInfo.adapterId.LowPart << "]" << std::endl;
      output << "    id: " << info.sourceInfo.id << std::endl;
      output << "        cloneGroupId: " << info.sourceInfo.cloneGroupId << std::endl;
      output << "        sourceModeInfoIdx: " << info.sourceInfo.sourceModeInfoIdx << std::endl;
      output << "        modeInfoIdx: " << info.sourceInfo.modeInfoIdx << std::endl;
      output << "    statusFlags: 0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.sourceInfo.statusFlags << std::endl;
      output.copyfmt(state);
      output << "targetInfo:" << std::endl;
      output << "    adapterId: [" << info.targetInfo.adapterId.HighPart << ", " << info.targetInfo.adapterId.LowPart << "]" << std::endl;
      output << "    id: " << info.targetInfo.id << std::endl;
      output << "        desktopModeInfoIdx: " << info.targetInfo.desktopModeInfoIdx << std::endl;
      output << "        targetModeInfoIdx: " << info.targetInfo.targetModeInfoIdx << std::endl;
      output << "        modeInfoIdx: " << info.targetInfo.modeInfoIdx << std::endl;
      output << "    outputTechnology:  0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.targetInfo.outputTechnology << std::endl;
      output << "    rotation: 0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.targetInfo.rotation << std::endl;
      output << "    scaling: 0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.targetInfo.scaling << std::endl;
      output.copyfmt(state);
      output << "    refreshRate: " << info.targetInfo.refreshRate.Numerator << "/" << info.targetInfo.refreshRate.Denominator << std::endl;
      output << "    scanLineOrdering: 0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.targetInfo.scanLineOrdering << std::endl;
      output << "    targetAvailable: 0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.targetInfo.targetAvailable << std::endl;
      output << "    statusFlags: 0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.targetInfo.statusFlags << std::endl;
      output << "flags: 0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << info.flags;
      // clang-format on

      return output.str();
    }

    /** @brief Dumps the result of @see queryDisplayConfig into a string */
    std::string dumpMode(const DISPLAYCONFIG_MODE_INFO &info) {
      std::stringstream output;
      std::ios state(nullptr);
      state.copyfmt(output);

      if (info.infoType == DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE) {
        // clang-format off
        output << "width: " << info.sourceMode.width << std::endl;
        output << "height: " << info.sourceMode.height << std::endl;
        output << "pixelFormat: " << info.sourceMode.pixelFormat << std::endl;
        output << "position: [" << info.sourceMode.position.x << ", " << info.sourceMode.position.y << "]";
        // clang-format on
      } else if (info.infoType == DISPLAYCONFIG_MODE_INFO_TYPE_TARGET) {
        // clang-format off
        output << "pixelRate: " << info.targetMode.targetVideoSignalInfo.pixelRate << std::endl;
        output << "hSyncFreq: " << info.targetMode.targetVideoSignalInfo.hSyncFreq.Numerator << "/" << info.targetMode.targetVideoSignalInfo.hSyncFreq.Denominator << std::endl;
        output << "vSyncFreq: " << info.targetMode.targetVideoSignalInfo.vSyncFreq.Numerator << "/" << info.targetMode.targetVideoSignalInfo.vSyncFreq.Denominator << std::endl;
        output << "activeSize: [" << info.targetMode.targetVideoSignalInfo.activeSize.cx << ", " << info.targetMode.targetVideoSignalInfo.activeSize.cy << "]" << std::endl;
        output << "totalSize: [" << info.targetMode.targetVideoSignalInfo.totalSize.cx << ", " << info.targetMode.targetVideoSignalInfo.totalSize.cy << "]" << std::endl;
        output << "videoStandard: " << info.targetMode.targetVideoSignalInfo.videoStandard << std::endl;
        output << "scanLineOrdering: " << info.targetMode.targetVideoSignalInfo.scanLineOrdering;
        // clang-format on
      } else if (info.infoType == DISPLAYCONFIG_MODE_INFO_TYPE_DESKTOP_IMAGE) {
        // TODO: One day MinGW will add updated struct definition and the following code can be enabled
        // clang-format off
        // output << "PathSourceSize: [" << info.desktopImageInfo.PathSourceSize.x << ", " << info.desktopImageInfo.PathSourceSize.y << "]" << std::endl;
        // output << "DesktopImageRegion: [" << info.desktopImageInfo.DesktopImageRegion.bottom << ", " << info.desktopImageInfo.DesktopImageRegion.left << ", " << info.desktopImageInfo.DesktopImageRegion.right << ", " << info.desktopImageInfo.DesktopImageRegion.top << "]" << std::endl;
        // output << "DesktopImageClip: [" << info.desktopImageInfo.DesktopImageClip.bottom << ", " << info.desktopImageInfo.DesktopImageClip.left << ", " << info.desktopImageInfo.DesktopImageClip.right << ", " << info.desktopImageInfo.DesktopImageClip.top << "]";
        // clang-format on
        output << "NOT SUPPORTED BY COMPILER YET...";
      } else {
        output << "NOT IMPLEMENTED YET...";
      }

      return output.str();
    }

    /** @brief Dumps the result of @see queryDisplayConfig into a string */
    std::string dumpPathsAndModes(const std::vector<DISPLAYCONFIG_PATH_INFO> &paths, const std::vector<DISPLAYCONFIG_MODE_INFO> &modes) {
      std::ostringstream output;

      output << std::endl
             << "Got " << paths.size() << " path(s):";
      bool path_dumped {false};
      for (auto i {0u}; i < paths.size(); ++i) {
        output << std::endl
               << "----------------------------------------[index: " << i << "]" << std::endl;

        output << dumpPath(paths[i]);
        path_dumped = true;
      }

      if (path_dumped) {
        output << std::endl
               << std::endl;
      }

      output << "Got " << modes.size() << " mode(s):";
      for (auto i {0u}; i < modes.size(); ++i) {
        output << std::endl
               << "----------------------------------------[index: " << i << "]" << std::endl;

        output << dumpMode(modes[i]);
      }

      return output.str();
    }

    /**
     * @see getMonitorDevicePath description for more information as this
     *      function is identical except that it returns wide-string instead
     *      of a normal one.
     */
    std::wstring getMonitorDevicePathWstr(const WinApiLayerInterface &w_api, const DISPLAYCONFIG_PATH_INFO &path) {
      DISPLAYCONFIG_TARGET_DEVICE_NAME target_name = {};
      target_name.header.adapterId = path.targetInfo.adapterId;
      target_name.header.id = path.targetInfo.id;
      target_name.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_TARGET_NAME;
      target_name.header.size = sizeof(target_name);

      LONG result {DisplayConfigGetDeviceInfo(&target_name.header)};
      if (result != ERROR_SUCCESS) {
        DD_LOG(error) << w_api.getErrorString(result) << " failed to get target device name!";
        return {};
      }

      return std::wstring {target_name.monitorDevicePath};
    }

    /**
     * @brief Helper method for dealing with SetupAPI.
     * @returns True if device interface path was retrieved and is non-empty, false otherwise.
     * @see getDeviceId implementation for more context regarding this madness.
     */
    bool getDeviceInterfaceDetail(const WinApiLayerInterface &w_api, HDEVINFO dev_info_handle, SP_DEVICE_INTERFACE_DATA &dev_interface_data, std::wstring &dev_interface_path, SP_DEVINFO_DATA &dev_info_data) {
      DWORD required_size_in_bytes {0};
      if (SetupDiGetDeviceInterfaceDetailW(dev_info_handle, &dev_interface_data, nullptr, 0, &required_size_in_bytes, nullptr)) {
        DD_LOG(error) << "\"SetupDiGetDeviceInterfaceDetailW\" did not fail, what?!";
        return false;
      } else if (required_size_in_bytes <= 0) {
        DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " \"SetupDiGetDeviceInterfaceDetailW\" failed while getting size.";
        return false;
      }

      std::vector<std::uint8_t> buffer;
      buffer.resize(required_size_in_bytes);

      // This part is just EVIL!
      auto detail_data {reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(buffer.data())};
      detail_data->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

      if (!SetupDiGetDeviceInterfaceDetailW(dev_info_handle, &dev_interface_data, detail_data, required_size_in_bytes, nullptr, &dev_info_data)) {
        DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " \"SetupDiGetDeviceInterfaceDetailW\" failed.";
        return false;
      }

      dev_interface_path = std::wstring {detail_data->DevicePath};
      return !dev_interface_path.empty();
    }

    /**
     * @brief Helper method for dealing with SetupAPI.
     * @returns True if instance id was retrieved and is non-empty, false otherwise.
     * @see getDeviceId implementation for more context regarding this madness.
     */
    bool getDeviceInstanceId(const WinApiLayerInterface &w_api, HDEVINFO dev_info_handle, SP_DEVINFO_DATA &dev_info_data, std::wstring &instance_id) {
      DWORD required_size_in_characters {0};
      if (SetupDiGetDeviceInstanceIdW(dev_info_handle, &dev_info_data, nullptr, 0, &required_size_in_characters)) {
        DD_LOG(error) << "\"SetupDiGetDeviceInstanceIdW\" did not fail, what?!";
        return false;
      } else if (required_size_in_characters <= 0) {
        DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " \"SetupDiGetDeviceInstanceIdW\" failed while getting size.";
        return false;
      }

      instance_id.resize(required_size_in_characters);
      if (!SetupDiGetDeviceInstanceIdW(dev_info_handle, &dev_info_data, instance_id.data(), instance_id.size(), nullptr)) {
        DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " \"SetupDiGetDeviceInstanceIdW\" failed.";
        return false;
      }

      return !instance_id.empty();
    }

    /**
     * @brief Helper method for dealing with SetupAPI.
     * @returns True if EDID was retrieved and is non-empty, false otherwise.
     * @see getDeviceId implementation for more context regarding this madness.
     */
    bool getDeviceEdid(const WinApiLayerInterface &w_api, HDEVINFO dev_info_handle, SP_DEVINFO_DATA &dev_info_data, std::vector<std::byte> &edid) {
      // We could just directly open the registry key as the path is known, but we can also use the this
      HKEY reg_key {SetupDiOpenDevRegKey(dev_info_handle, &dev_info_data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ)};
      if (reg_key == INVALID_HANDLE_VALUE) {
        DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " \"SetupDiOpenDevRegKey\" failed.";
        return false;
      }

      const auto reg_key_cleanup {
        boost::scope::scope_exit([&w_api, &reg_key]() {
          const auto status {RegCloseKey(reg_key)};
          if (status != ERROR_SUCCESS) {
            DD_LOG(error) << w_api.getErrorString(status) << " \"RegCloseKey\" failed.";
          }
        })
      };

      DWORD required_size_in_bytes {0};
      auto status {RegQueryValueExW(reg_key, L"EDID", nullptr, nullptr, nullptr, &required_size_in_bytes)};
      if (status != ERROR_SUCCESS) {
        DD_LOG(error) << w_api.getErrorString(status) << " \"RegQueryValueExW\" failed when getting size.";
        return false;
      }

      edid.resize(required_size_in_bytes);

      status = RegQueryValueExW(reg_key, L"EDID", nullptr, nullptr, reinterpret_cast<LPBYTE>(edid.data()), &required_size_in_bytes);
      if (status != ERROR_SUCCESS) {
        DD_LOG(error) << w_api.getErrorString(status) << " \"RegQueryValueExW\" failed when getting data.";
        return false;
      }

      return !edid.empty();
    }

    /**
     * @brief Get instance ID and EDID via SetupAPI.
     * @param w_api Reference to the WinApiLayer.
     * @param device_path Device path to find device for.
     * @return A tuple of instance ID and EDID, or empty optional if not device was found or error has occurred.
     */
    std::optional<std::tuple<std::wstring, std::vector<std::byte>>> getInstanceIdAndEdid(const WinApiLayerInterface &w_api, const std::wstring &device_path) {
      static const GUID monitor_guid {0xe6f07b5f, 0xee97, 0x4a90, {0xb0, 0x76, 0x33, 0xf5, 0x7b, 0xf4, 0xea, 0xa7}};

      HDEVINFO dev_info_handle {SetupDiGetClassDevsW(&monitor_guid, nullptr, nullptr, DIGCF_DEVICEINTERFACE)};
      if (dev_info_handle) {
        const auto dev_info_handle_cleanup {
          boost::scope::scope_exit([&dev_info_handle, &w_api]() {
            if (!SetupDiDestroyDeviceInfoList(dev_info_handle)) {
              DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " \"SetupDiDestroyDeviceInfoList\" failed.";
            }
          })
        };

        SP_DEVICE_INTERFACE_DATA dev_interface_data {};
        dev_interface_data.cbSize = sizeof(dev_interface_data);
        for (DWORD monitor_index = 0;; ++monitor_index) {
          if (!SetupDiEnumDeviceInterfaces(dev_info_handle, nullptr, &monitor_guid, monitor_index, &dev_interface_data)) {
            const DWORD error_code {GetLastError()};
            if (error_code == ERROR_NO_MORE_ITEMS) {
              break;
            }

            DD_LOG(warning) << w_api.getErrorString(static_cast<LONG>(error_code)) << " \"SetupDiEnumDeviceInterfaces\" failed.";
            continue;
          }

          std::wstring dev_interface_path;
          SP_DEVINFO_DATA dev_info_data {};
          dev_info_data.cbSize = sizeof(dev_info_data);
          if (!getDeviceInterfaceDetail(w_api, dev_info_handle, dev_interface_data, dev_interface_path, dev_info_data)) {
            // Error already logged
            continue;
          }

          if (!boost::iequals(dev_interface_path, device_path)) {
            continue;
          }

          std::wstring instance_id;
          if (!getDeviceInstanceId(w_api, dev_info_handle, dev_info_data, instance_id)) {
            // Error already logged
            break;
          }

          std::vector<std::byte> edid;
          if (!getDeviceEdid(w_api, dev_info_handle, dev_info_data, edid)) {
            // Error already logged
            break;
          }

          return std::make_tuple(std::move(instance_id), std::move(edid));
        }
      }

      return std::nullopt;
    }

    /**
     * @brief Converts a UTF-16 wide string into a UTF-8 string.
     * @param w_api Reference to the WinApiLayer.
     * @param value The UTF-16 wide string.
     * @return The converted UTF-8 string.
     */
    std::string toUtf8(const WinApiLayerInterface &w_api, const std::wstring &value) {
      // No conversion needed if the string is empty
      if (value.empty()) {
        return {};
      }

      // Get the output size required to store the string
      auto output_size = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, value.data(), static_cast<int>(value.size()), nullptr, 0, nullptr, nullptr);
      if (output_size == 0) {
        DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " failed to get UTF-8 buffer size.";
        return {};
      }

      // Perform the conversion
      std::string output(output_size, '\0');
      output_size = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, value.data(), static_cast<int>(value.size()), output.data(), static_cast<int>(output.size()), nullptr, nullptr);
      if (output_size == 0) {
        DD_LOG(error) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " failed to convert string to UTF-8.";
        return {};
      }

      return output;
    }

    /**
     * @brief Check if the Windows 11 version is equal to 24H2 update or later.
     * @param w_api Reference to the WinApiLayer.
     * @return True if version >= W11 24H2, false otherwise.
     */
    bool is_W11_24H2_OrAbove(const WinApiLayerInterface &w_api) {
      OSVERSIONINFOEXA os_version_info;
      os_version_info.dwOSVersionInfoSize = sizeof(os_version_info);
      os_version_info.dwMajorVersion = HIBYTE(_WIN32_WINNT_WIN10);
      os_version_info.dwMinorVersion = LOBYTE(_WIN32_WINNT_WIN10);
      os_version_info.dwBuildNumber = 26100;  // The earliest pre-release version is 25947, whereas the stable is 26100

      ULONGLONG condition_mask {0};
      condition_mask = VerSetConditionMask(condition_mask, VER_MAJORVERSION, VER_GREATER_EQUAL);  // Major version condition
      condition_mask = VerSetConditionMask(condition_mask, VER_MINORVERSION, VER_GREATER_EQUAL);  // Minor version condition
      condition_mask = VerSetConditionMask(condition_mask, VER_BUILDNUMBER, VER_GREATER_EQUAL);  // Build number condition

      BOOL result {VerifyVersionInfoA(&os_version_info, VER_MAJORVERSION | VER_MINORVERSION | VER_BUILDNUMBER, condition_mask)};
      if (result == FALSE) {
        DD_LOG(verbose) << w_api.getErrorString(static_cast<LONG>(GetLastError())) << " \"is_W11_24H2_OrAbove\" returned false.";
        return false;
      }

      DD_LOG(verbose) << "\"is_W11_24H2_OrAbove\" returned true.";
      return true;
    }
  }  // namespace

  std::string WinApiLayer::getErrorString(LONG error_code) const {
    std::ostringstream error;
    error << "[code: ";
    switch (error_code) {
      case ERROR_INVALID_PARAMETER:
        error << "ERROR_INVALID_PARAMETER";
        break;
      case ERROR_NOT_SUPPORTED:
        error << "ERROR_NOT_SUPPORTED";
        break;
      case ERROR_ACCESS_DENIED:
        error << "ERROR_ACCESS_DENIED";
        break;
      case ERROR_INSUFFICIENT_BUFFER:
        error << "ERROR_INSUFFICIENT_BUFFER";
        break;
      case ERROR_GEN_FAILURE:
        error << "ERROR_GEN_FAILURE";
        break;
      case ERROR_SUCCESS:
        error << "ERROR_SUCCESS";
        break;
      default:
        error << error_code;
        break;
    }
    error << ", message: " << std::system_category().message(static_cast<int>(error_code)) << "]";
    return error.str();
  }

  std::optional<PathAndModeData> WinApiLayer::queryDisplayConfig(QueryType type) const {
    std::vector<DISPLAYCONFIG_PATH_INFO> paths;
    std::vector<DISPLAYCONFIG_MODE_INFO> modes;
    LONG result = ERROR_SUCCESS;

    // When we want to enable/disable displays, we need to get all paths as they will not be active.
    // This will require some additional filtering of duplicate and otherwise useless paths.
    UINT32 flags = type == QueryType::Active ? QDC_ONLY_ACTIVE_PATHS : QDC_ALL_PATHS;
    flags |= QDC_VIRTUAL_MODE_AWARE;  // supported from W10 onwards

    do {
      UINT32 path_count {0};
      UINT32 mode_count {0};

      result = GetDisplayConfigBufferSizes(flags, &path_count, &mode_count);
      if (result != ERROR_SUCCESS) {
        DD_LOG(error) << getErrorString(result) << " failed to get display paths and modes!";
        return std::nullopt;
      }

      paths.resize(path_count);
      modes.resize(mode_count);
      result = QueryDisplayConfig(flags, &path_count, paths.data(), &mode_count, modes.data(), nullptr);

      // The function may have returned fewer paths/modes than estimated
      paths.resize(path_count);
      modes.resize(mode_count);

      // It's possible that between the call to GetDisplayConfigBufferSizes and QueryDisplayConfig
      // that the display state changed, so loop on the case of ERROR_INSUFFICIENT_BUFFER.
    } while (result == ERROR_INSUFFICIENT_BUFFER);

    if (result != ERROR_SUCCESS) {
      DD_LOG(error) << getErrorString(result) << " failed to query display paths and modes!";
      return std::nullopt;
    }

    DD_LOG(verbose) << "Result of " << (type == QueryType::Active ? "ACTIVE" : "ALL") << " display config query:\n"
                    << dumpPathsAndModes(paths, modes) << "\n";
    return PathAndModeData {paths, modes};
  }

  std::string WinApiLayer::getDeviceId(const DISPLAYCONFIG_PATH_INFO &path) const {
    const auto device_path {getMonitorDevicePathWstr(*this, path)};
    if (device_path.empty()) {
      // Error already logged
      return {};
    }

    std::vector<std::byte> device_id_data;
    auto instance_id_and_edid {getInstanceIdAndEdid(*this, device_path)};
    if (instance_id_and_edid) {
      // Instance ID is unique in the system and persists restarts, but not driver re-installs.
      // It looks like this:
      //     DISPLAY\ACI27EC\5&4FD2DE4&5&UID4352 (also used in the device path it seems)
      //                a    b    c    d    e
      //
      //  a) Hardware ID - stable
      //  b) Either a bus number or has something to do with device capabilities - stable
      //  c) Another ID, somehow tied to adapter (not an adapter ID from path object) - stable
      //  d) Some sort of rotating counter thing, changes after driver reinstall - unstable
      //  e) Seems to be the same as a target ID from path, it changes based on GPU port - semi-stable
      //
      // The instance ID also seems to be a part of the registry key (in case some other info is needed in the future):
      //     HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\DISPLAY\ACI27EC\5&4fd2de4&5&UID4352
      [this, &device_id_data, &instance_id_and_edid]() {
        auto [instance_id, edid] = *instance_id_and_edid;

        // We are going to discard the unstable parts of the instance ID and merge the stable parts with the edid buffer (if available)
        auto unstable_part_index = instance_id.find_first_of(L'&', 0);
        if (unstable_part_index != std::wstring::npos) {
          unstable_part_index = instance_id.find_first_of(L'&', unstable_part_index + 1);
        }

        if (unstable_part_index == std::wstring::npos) {
          DD_LOG(error) << "Failed to split off the stable part from instance id string " << toUtf8(*this, instance_id);
          return;
        }

        auto semi_stable_part_index = instance_id.find_first_of(L'&', unstable_part_index + 1);
        if (semi_stable_part_index == std::wstring::npos) {
          DD_LOG(error) << "Failed to split off the semi-stable part from instance id string " << toUtf8(*this, instance_id);
          return;
        }

        device_id_data.swap(edid);
        device_id_data.insert(std::end(device_id_data), reinterpret_cast<const std::byte *>(instance_id.data()), reinterpret_cast<const std::byte *>(instance_id.data() + unstable_part_index));
        device_id_data.insert(std::end(device_id_data), reinterpret_cast<const std::byte *>(instance_id.data() + semi_stable_part_index), reinterpret_cast<const std::byte *>(instance_id.data() + instance_id.size()));

        static const auto dump_device_id_data {[](const auto &data) -> std::string {
          if (data.empty()) {
            return {};
          }

          std::ostringstream output;
          output << "[";
          for (std::size_t i = 0; i < data.size(); ++i) {
            output << "0x" << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << static_cast<int>(data[i]);
            if (i + 1 < data.size()) {
              output << " ";
            }
          }
          output << "]";

          return output.str();
        }};
        DD_LOG(verbose) << "Creating device id from EDID + instance ID: " << dump_device_id_data(device_id_data);
      }();
    }

    if (device_id_data.empty()) {
      // Using the device path as a fallback, which is always unique, but not as stable as the preferred one
      DD_LOG(verbose) << "Creating device id from path " << toUtf8(*this, device_path);
      device_id_data.insert(std::end(device_id_data), reinterpret_cast<const std::byte *>(device_path.data()), reinterpret_cast<const std::byte *>(device_path.data() + device_path.size()));
    }

    static constexpr boost::uuids::uuid ns_id {};  // null namespace = no salt
    const auto boost_uuid {boost::uuids::name_generator_sha1 {ns_id}(device_id_data.data(), device_id_data.size())};
    const std::string device_id {"{" + boost::uuids::to_string(boost_uuid) + "}"};

    DD_LOG(verbose) << "Created device id: " << toUtf8(*this, device_path) << " -> " << device_id;
    return device_id;
  }

  std::vector<std::byte> WinApiLayer::getEdid(const DISPLAYCONFIG_PATH_INFO &path) const {
    const auto device_path {getMonitorDevicePathWstr(*this, path)};
    if (device_path.empty()) {
      // Error already logged
      return {};
    }

    auto instance_id_and_edid {getInstanceIdAndEdid(*this, device_path)};
    return instance_id_and_edid ? std::get<1>(*instance_id_and_edid) : std::vector<std::byte> {};
  }

  std::string WinApiLayer::getMonitorDevicePath(const DISPLAYCONFIG_PATH_INFO &path) const {
    return toUtf8(*this, getMonitorDevicePathWstr(*this, path));
  }

  std::string WinApiLayer::getFriendlyName(const DISPLAYCONFIG_PATH_INFO &path) const {
    DISPLAYCONFIG_TARGET_DEVICE_NAME target_name = {};
    target_name.header.adapterId = path.targetInfo.adapterId;
    target_name.header.id = path.targetInfo.id;
    target_name.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_TARGET_NAME;
    target_name.header.size = sizeof(target_name);

    LONG result {DisplayConfigGetDeviceInfo(&target_name.header)};
    if (result != ERROR_SUCCESS) {
      DD_LOG(error) << getErrorString(result) << " failed to get target device name!";
      return {};
    }

    return target_name.flags.friendlyNameFromEdid ? toUtf8(*this, target_name.monitorFriendlyDeviceName) : std::string {};
  }

  std::string WinApiLayer::getDisplayName(const DISPLAYCONFIG_PATH_INFO &path) const {
    DISPLAYCONFIG_SOURCE_DEVICE_NAME source_name = {};
    source_name.header.id = path.sourceInfo.id;
    source_name.header.adapterId = path.sourceInfo.adapterId;
    source_name.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME;
    source_name.header.size = sizeof(source_name);

    LONG result {DisplayConfigGetDeviceInfo(&source_name.header)};
    if (result != ERROR_SUCCESS) {
      DD_LOG(error) << getErrorString(result) << " failed to get display name!";
      return {};
    }

    return toUtf8(*this, source_name.viewGdiDeviceName);
  }

  LONG WinApiLayer::setDisplayConfig(std::vector<DISPLAYCONFIG_PATH_INFO> paths, std::vector<DISPLAYCONFIG_MODE_INFO> modes, UINT32 flags) {
    // std::vector::data() "may or may not return a null pointer, if size() is 0", therefore we want to enforce nullptr...
    return ::SetDisplayConfig(
      paths.size(),
      paths.empty() ? nullptr : paths.data(),
      modes.size(),
      modes.empty() ? nullptr : modes.data(),
      flags
    );
  }

  std::optional<HdrState> WinApiLayer::getHdrState(const DISPLAYCONFIG_PATH_INFO &path) const {
    if (is_W11_24H2_OrAbove(*this)) {
      DISPLAYCONFIG_GET_ADVANCED_COLOR_INFO_2 color_info = {};
      color_info.header.adapterId = path.targetInfo.adapterId;
      color_info.header.id = path.targetInfo.id;
      color_info.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_ADVANCED_COLOR_INFO_2;
      color_info.header.size = sizeof(color_info);

      LONG result {DisplayConfigGetDeviceInfo(&color_info.header)};
      if (result != ERROR_SUCCESS) {
        DD_LOG(error) << getErrorString(result) << " failed to get advanced color info 2!";
        return std::nullopt;
      }

      return color_info.highDynamicRangeSupported ? std::make_optional(color_info.highDynamicRangeUserEnabled ? HdrState::Enabled : HdrState::Disabled) : std::nullopt;
    }

    DISPLAYCONFIG_GET_ADVANCED_COLOR_INFO color_info = {};
    color_info.header.adapterId = path.targetInfo.adapterId;
    color_info.header.id = path.targetInfo.id;
    color_info.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_ADVANCED_COLOR_INFO;
    color_info.header.size = sizeof(color_info);

    LONG result {DisplayConfigGetDeviceInfo(&color_info.header)};
    if (result != ERROR_SUCCESS) {
      DD_LOG(error) << getErrorString(result) << " failed to get advanced color info!";
      return std::nullopt;
    }

    return color_info.advancedColorSupported ? std::make_optional(color_info.advancedColorEnabled ? HdrState::Enabled : HdrState::Disabled) : std::nullopt;
  }

  bool WinApiLayer::setHdrState(const DISPLAYCONFIG_PATH_INFO &path, HdrState state) {
    if (is_W11_24H2_OrAbove(*this)) {
      DISPLAYCONFIG_SET_HDR_STATE hdr_state = {};
      hdr_state.header.adapterId = path.targetInfo.adapterId;
      hdr_state.header.id = path.targetInfo.id;
      hdr_state.header.type = DISPLAYCONFIG_DEVICE_INFO_TYPE(16);
      hdr_state.header.size = sizeof(hdr_state);
      hdr_state.enableHdr = state == HdrState::Enabled ? 1 : 0;

      LONG result {DisplayConfigSetDeviceInfo(&hdr_state.header)};
      if (result != ERROR_SUCCESS) {
        DD_LOG(error) << getErrorString(result) << " failed to set HDR state!";
        return false;
      }

      return true;
    }

    DISPLAYCONFIG_SET_ADVANCED_COLOR_STATE color_state = {};
    color_state.header.adapterId = path.targetInfo.adapterId;
    color_state.header.id = path.targetInfo.id;
    color_state.header.type = DISPLAYCONFIG_DEVICE_INFO_SET_ADVANCED_COLOR_STATE;
    color_state.header.size = sizeof(color_state);
    color_state.enableAdvancedColor = state == HdrState::Enabled ? 1 : 0;

    LONG result {DisplayConfigSetDeviceInfo(&color_state.header)};
    if (result != ERROR_SUCCESS) {
      DD_LOG(error) << getErrorString(result) << " failed to set advanced color info!";
      return false;
    }

    return true;
  }

  std::optional<Rational> WinApiLayer::getDisplayScale(const std::string &display_name, const DISPLAYCONFIG_SOURCE_MODE &source_mode) const {
    // Note: implementation based on https://stackoverflow.com/a/74046173
    struct EnumData {
      std::string m_display_name;
      std::optional<int> m_width;
    };

    EnumData enum_data {display_name, std::nullopt};
    EnumDisplayMonitors(
      nullptr,
      nullptr,
      [](HMONITOR monitor, HDC, LPRECT, LPARAM user_data) -> BOOL {
        auto *data = reinterpret_cast<EnumData *>(user_data);
        if (data == nullptr) {
          // Sanity check
          DD_LOG(error) << "EnumData is a nullptr!";
          return FALSE;
        }

        MONITORINFOEXA monitor_info {sizeof(MONITORINFOEXA)};
        if (GetMonitorInfoA(monitor, &monitor_info)) {
          if (data->m_display_name == monitor_info.szDevice) {
            data->m_width = monitor_info.rcMonitor.right - monitor_info.rcMonitor.left;
            return FALSE;
          }
        }

        return TRUE;
      },
      reinterpret_cast<LPARAM>(&enum_data)
    );

    if (!enum_data.m_width) {
      DD_LOG(debug) << "Failed to get monitor info for " << display_name << "!";
      return std::nullopt;
    }

    if (*enum_data.m_width * source_mode.width == 0) {
      DD_LOG(debug) << "Cannot get display scale for " << display_name << " from a width of 0!";
      return std::nullopt;
    }

    const auto width {static_cast<double>(*enum_data.m_width) / static_cast<double>(source_mode.width)};
    return Rational {static_cast<unsigned int>(std::round((static_cast<double>(GetDpiForSystem()) / 96. / width) * 100)), 100};
  }
}  // namespace display_device
