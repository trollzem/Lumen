/**
 * @file src/common/include/display_device/detail/json_serializer_details.h
 * @brief Declarations for private JSON serialization details.
 */
#pragma once

#ifdef DD_JSON_DETAIL
  // system includes
  #include <nlohmann/json.hpp>

  // Special versions of the NLOHMANN definitions to remove the "m_" prefix in string form ('cause I like it that way ;P)
  #define DD_JSON_TO(v1) nlohmann_json_j[#v1] = nlohmann_json_t.m_##v1;
  #define DD_JSON_FROM(v1) nlohmann_json_j.at(#v1).get_to(nlohmann_json_t.m_##v1);

  // Coverage has trouble with inlined functions when they are included in different units,
  // therefore the usual macro was split into declaration and definition
  #define DD_JSON_DECLARE_SERIALIZE_TYPE(Type) \
    void to_json(nlohmann::json &nlohmann_json_j, const Type &nlohmann_json_t); \
    void from_json(const nlohmann::json &nlohmann_json_j, Type &nlohmann_json_t);

  #define DD_JSON_DEFINE_SERIALIZE_STRUCT(Type, ...) \
    void to_json(nlohmann::json &nlohmann_json_j, const Type &nlohmann_json_t) { \
      NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(DD_JSON_TO, __VA_ARGS__)) \
    } \
\
    void from_json(const nlohmann::json &nlohmann_json_j, Type &nlohmann_json_t) { \
      NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(DD_JSON_FROM, __VA_ARGS__)) \
    }

  // Coverage has trouble with getEnumMap() function since it has a lot of "fallthrough"
  // branches when creating a map, therefore the macro has baked in pattern to disable branch coverage
  // in GCOVR
  #define DD_JSON_DEFINE_SERIALIZE_ENUM_GCOVR_EXCL_BR_LINE(Type, ...) \
    const std::map<Type, nlohmann::json> & \
      getEnumMap(const Type &) { \
      static_assert(std::is_enum<Type>::value, #Type " must be an enum!"); \
      static const std::map<Type, nlohmann::json> map = __VA_ARGS__; \
      return map; \
    } \
\
    void to_json(nlohmann::json &nlohmann_json_j, const Type &nlohmann_json_t) { \
      nlohmann_json_j = findInEnumMap<Type>(#Type " is missing enum mapping!", [nlohmann_json_t](const auto &pair) { \
                          return pair.first == nlohmann_json_t; \
                        })->second; \
    } \
\
    void from_json(const nlohmann::json &nlohmann_json_j, Type &nlohmann_json_t) { \
      nlohmann_json_t = findInEnumMap<Type>(#Type " is missing enum mapping!", [&nlohmann_json_j](const auto &pair) { \
                          return pair.second == nlohmann_json_j; \
                        })->first; \
    }

namespace display_device {
  /**
   * @brief Holds information for serializing variants.
   */
  namespace detail {
    template<class T>
    struct JsonTypeName;

    template<>
    struct JsonTypeName<double> {
      static constexpr std::string_view m_name {"double"};
    };

    template<>
    struct JsonTypeName<Rational> {
      static constexpr std::string_view m_name {"rational"};
    };

    template<class T, class... Ts>
    bool variantFromJson(const nlohmann::json &nlohmann_json_j, std::variant<Ts...> &value) {
      if (nlohmann_json_j.at("type").get<std::string_view>() != JsonTypeName<T>::m_name) {
        return false;
      }

      value = nlohmann_json_j.at("value").get<T>();
      return true;
    }
  }  // namespace detail

  // A shared function for enums to find values in the map. Extracted here for UTs + coverage
  template<class T, class Predicate>
  typename std::map<T, nlohmann::json>::const_iterator findInEnumMap(const char *error_msg, Predicate predicate) {
    const auto &map {getEnumMap(T {})};
    auto it {std::find_if(std::begin(map), std::end(map), predicate)};
    if (it == std::end(map)) {  // GCOVR_EXCL_BR_LINE for fallthrough branch
      throw std::runtime_error(error_msg);  // GCOVR_EXCL_BR_LINE for fallthrough branch
    }
    return it;
  }
}  // namespace display_device

namespace nlohmann {
  // Specialization for optional types until they actually implement it.
  template<class T>
  struct adl_serializer<std::optional<T>> {
    static void to_json(json &nlohmann_json_j, const std::optional<T> &nlohmann_json_t) {
      if (nlohmann_json_t == std::nullopt) {
        nlohmann_json_j = nullptr;
      } else {
        nlohmann_json_j = *nlohmann_json_t;
      }
    }

    static void from_json(const json &nlohmann_json_j, std::optional<T> &nlohmann_json_t) {
      if (nlohmann_json_j.is_null()) {
        nlohmann_json_t = std::nullopt;
      } else {
        nlohmann_json_t = nlohmann_json_j.get<T>();
      }
    }
  };

  // Specialization for variant type.
  // See https://github.com/nlohmann/json/issues/1261#issuecomment-2048770747
  template<typename... Ts>
  struct adl_serializer<std::variant<Ts...>> {
    static void to_json(json &nlohmann_json_j, const std::variant<Ts...> &nlohmann_json_t) {
      std::visit(
        [&nlohmann_json_j]<class T>(const T &value) {
          nlohmann_json_j["type"] = display_device::detail::JsonTypeName<std::decay_t<T>>::m_name;
          nlohmann_json_j["value"] = value;
        },
        nlohmann_json_t
      );
    }

    static void from_json(const json &nlohmann_json_j, std::variant<Ts...> &nlohmann_json_t) {
      // Call variant_from_json for all types, only one will succeed
      const bool found {(display_device::detail::variantFromJson<Ts>(nlohmann_json_j, nlohmann_json_t) || ...)};
      if (!found) {
        const std::string error {"Could not parse variant from type " + nlohmann_json_j.at("type").get<std::string>() + "!"};
        throw std::runtime_error(error);
      }
    }
  };

  // Specialization for chrono duration.
  template<class Rep, class Period>
  struct adl_serializer<std::chrono::duration<Rep, Period>> {
    using NanoRep = decltype(std::chrono::nanoseconds {}.count());
    static_assert(std::numeric_limits<Rep>::max() <= std::numeric_limits<NanoRep>::max(), "Duration support above nanoseconds have not been tested/verified yet!");

    static void to_json(json &nlohmann_json_j, const std::chrono::duration<Rep, Period> &nlohmann_json_t) {
      nlohmann_json_j = nlohmann_json_t.count();
    }

    static void from_json(const json &nlohmann_json_j, std::chrono::duration<Rep, Period> &nlohmann_json_t) {
      nlohmann_json_t = std::chrono::duration<Rep, Period> {nlohmann_json_j.get<Rep>()};
    }
  };
}  // namespace nlohmann
#endif
