# A global identifier for the library
set(MODULE libfixtures)

# Globing headers (so that they appear in some IDEs) and sources
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "include/fixtures/*.h")
file(GLOB SOURCE_LIST CONFIGURE_DEPENDS "*.cpp")

# Automatic library - will be static or dynamic based on user setting
add_library(${MODULE} ${HEADER_LIST} ${SOURCE_LIST})

# Provide the includes together with this library
target_include_directories(${MODULE} PUBLIC include)

# Additional external libraries
include(Boost_DD)

# Link the additional libraries
target_link_libraries(${MODULE}
        PUBLIC
        Boost::preprocessor

        PRIVATE
        gtest
        libdisplaydevice::common
)
