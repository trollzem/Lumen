/**
 * @file src/common/include/display_device/types.h
 * @brief Declarations for common display device types.
 */
#pragma once

// system includes
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace display_device {
  /**
   * @brief The device's HDR state in the operating system.
   */
  enum class HdrState {
    Disabled,  ///< HDR is disabled
    Enabled  ///< HDR is enabled
  };

  /**
   * @brief Display's resolution.
   */
  struct Resolution {
    unsigned int m_width {};
    unsigned int m_height {};

    /**
     * @brief Comparator for strict equality.
     */
    friend bool operator==(const Resolution &lhs, const Resolution &rhs);
  };

  /**
   * @brief An arbitrary point object.
   */
  struct Point {
    int m_x {};
    int m_y {};

    /**
     * @brief Comparator for strict equality.
     */
    friend bool operator==(const Point &lhs, const Point &rhs);
  };

  /**
   * @brief Floating point stored in a "numerator/denominator" form.
   */
  struct Rational {
    unsigned int m_numerator {};
    unsigned int m_denominator {};

    /**
     * @brief Comparator for strict equality.
     */
    friend bool operator==(const Rational &lhs, const Rational &rhs);
  };

  /**
   * @brief Floating point type.
   */
  using FloatingPoint = std::variant<double, Rational>;

  /**
   * @brief Parsed EDID data.
   */
  struct EdidData {
    std::string m_manufacturer_id {};
    std::string m_product_code {};
    std::uint32_t m_serial_number {};

    /**
     * @brief Parse EDID data.
     * @param data Data to parse.
     * @return Parsed data or empty optional if failed to parse it.
     */
    static std::optional<EdidData> parse(const std::vector<std::byte> &data);

    /**
     * @brief Comparator for strict equality.
     */
    friend bool operator==(const EdidData &lhs, const EdidData &rhs);
  };

  /**
   * @brief Enumerated display device information.
   */
  struct EnumeratedDevice {
    /**
     * @brief Available information for the active display only.
     */
    struct Info {
      Resolution m_resolution {}; /**< Resolution of an active device. */
      FloatingPoint m_resolution_scale {}; /**< Resolution scaling of an active device. */
      FloatingPoint m_refresh_rate {}; /**< Refresh rate of an active device. */
      bool m_primary {}; /**< Indicates whether the device is a primary display. */
      Point m_origin_point {}; /**< A starting point of the display. */
      std::optional<HdrState> m_hdr_state {}; /**< HDR of an active device. */

      /**
       * @brief Comparator for strict equality.
       */
      friend bool operator==(const Info &lhs, const Info &rhs);
    };

    std::string m_device_id {}; /**< A unique device ID used by this API to identify the device. */
    std::string m_display_name {}; /**< A logical name representing given by the OS for a display. */
    std::string m_friendly_name {}; /**< A human-readable name for the device. */
    std::optional<EdidData> m_edid {}; /**< Some basic parsed EDID data. */
    std::optional<Info> m_info {}; /**< Additional information about an active display device. */

    /**
     * @brief Comparator for strict equality.
     */
    friend bool operator==(const EnumeratedDevice &lhs, const EnumeratedDevice &rhs);
  };

  /**
   * @brief A list of EnumeratedDevice objects.
   */
  using EnumeratedDeviceList = std::vector<EnumeratedDevice>;

  /**
   * @brief Configuration centered around a single display.
   *
   * Allows to easily configure the display without providing a complete configuration
   * for all of the system display devices.
   */
  struct SingleDisplayConfiguration {
    /**
     * @brief Enum detailing how to prepare the display device.
     */
    enum class DevicePreparation {
      VerifyOnly, /**< User has to make sure the display device is active, we will only verify. */
      EnsureActive, /**< Activate the device if needed. */
      EnsurePrimary, /**< Activate the device if needed and make it a primary display. */
      EnsureOnlyDisplay /**< Deactivate other displays and turn on the specified one only. */
    };

    std::string m_device_id {}; /**< Device to perform configuration for (can be empty if primary device should be used). */
    DevicePreparation m_device_prep {}; /**< Instruction on how to prepare device. */
    std::optional<Resolution> m_resolution {}; /**< Resolution to configure. */
    std::optional<FloatingPoint> m_refresh_rate {}; /**< Refresh rate to configure. */
    std::optional<HdrState> m_hdr_state {}; /**< HDR state to configure (if supported by the display). */

    /**
     * @brief Comparator for strict equality.
     */
    friend bool operator==(const SingleDisplayConfiguration &lhs, const SingleDisplayConfiguration &rhs);
  };
}  // namespace display_device
