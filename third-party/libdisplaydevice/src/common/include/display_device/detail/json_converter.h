/**
 * @file src/common/include/display_device/detail/json_converter.h
 * @brief Declarations for private JSON conversion helpers.
 */
#pragma once

#ifdef DD_JSON_DETAIL
  // system includes
  #include <nlohmann/json.hpp>

namespace display_device {
  // A shared "toJson" implementation. Extracted here for UTs + coverage.
  template<typename Type>
  std::string toJsonHelper(const Type &obj, const std::optional<unsigned int> &indent, bool *success) {
    try {
      if (success) {
        *success = true;
      }

      nlohmann::json json_obj = obj;
      return json_obj.dump(static_cast<int>(indent.value_or(-1)));
    } catch (const std::exception &err) {  // GCOVR_EXCL_BR_LINE for fallthrough branch
      if (success) {
        *success = false;
      }

      return err.what();
    }
  }

  // A shared "fromJson" implementation. Extracted here for UTs + coverage.
  template<typename Type>
  bool fromJsonHelper(const std::string &string, Type &obj, std::string *error_message = nullptr) {
    try {
      if (error_message) {
        error_message->clear();
      }

      Type parsed_obj = nlohmann::json::parse(string);
      obj = std::move(parsed_obj);
      return true;
    } catch (const std::exception &err) {
      if (error_message) {
        *error_message = err.what();
      }

      return false;
    }
  }

  #define DD_JSON_DEFINE_CONVERTER(Type) \
    std::string toJson(const Type &obj, const std::optional<unsigned int> &indent, bool *success) { \
      return toJsonHelper(obj, indent, success); \
    } \
    bool fromJson(const std::string &string, Type &obj, std::string *error_message) { \
      return fromJsonHelper<Type>(string, obj, error_message); \
    }
}  // namespace display_device
#endif
