/**
 * @file src/common/include/display_device/json.h
 * @brief Declarations for JSON conversion functions.
 */
#pragma once

// system includes
#include <set>

// local includes
#include "types.h"

/**
 * @brief Helper MACRO to declare the toJson and fromJson converters for a type.
 * @examples
 * EnumeratedDeviceList devices;
 * DD_LOG(info) << "Got devices:\n" << toJson(devices);
 * @examples_end
 */
#define DD_JSON_DECLARE_CONVERTER(Type) \
  [[nodiscard]] std::string toJson(const Type &obj, const std::optional<unsigned int> &indent = 2u, bool *success = nullptr); \
  [[nodiscard]] bool fromJson(const std::string &string, Type &obj, std::string *error_message = nullptr);  // NOLINT(*-macro-parentheses)

// Shared converters (add as needed)
namespace display_device {
  extern const std::optional<unsigned int> JSON_COMPACT;

  DD_JSON_DECLARE_CONVERTER(EdidData)
  DD_JSON_DECLARE_CONVERTER(EnumeratedDevice)
  DD_JSON_DECLARE_CONVERTER(EnumeratedDeviceList)
  DD_JSON_DECLARE_CONVERTER(SingleDisplayConfiguration)
  DD_JSON_DECLARE_CONVERTER(std::set<std::string>)
  DD_JSON_DECLARE_CONVERTER(std::string)
  DD_JSON_DECLARE_CONVERTER(bool)
}  // namespace display_device
