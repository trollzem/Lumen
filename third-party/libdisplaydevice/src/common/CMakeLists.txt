# A global identifier for the library
set(MODULE libdisplaydevice_common)
set(MODULE_ALIAS libdisplaydevice::common)

# Globing headers (so that they appear in some IDEs) and sources
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "include/display_device/*.h")
file(GLOB HEADER_DETAIL_LIST CONFIGURE_DEPENDS "include/display_device/detail/*.h")
file(GLOB SOURCE_LIST CONFIGURE_DEPENDS "*.cpp")

# Automatic library - will be static or dynamic based on user setting
add_library(${MODULE} ${HEADER_LIST} ${HEADER_DETAIL_LIST} ${SOURCE_LIST})
add_library(${MODULE_ALIAS} ALIAS ${MODULE})

# Provide the includes together with this library
target_include_directories(${MODULE} PUBLIC include)

# Additional external libraries
include(Json_DD)

# Link the additional libraries
target_link_libraries(${MODULE} PRIVATE nlohmann_json::nlohmann_json)
