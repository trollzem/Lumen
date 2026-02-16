#
# Project configuration
#
cmake_minimum_required(VERSION 3.24)
project(libdisplaydevice VERSION 0.0.0
        DESCRIPTION "Library to modify display devices."
        HOMEPAGE_URL "https://app.lizardbyte.dev"
        LANGUAGES CXX)

set(PROJECT_LICENSE "GPL-3.0")
set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# Add our custom CMake modules to the global path
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

#
# Project optional configuration
#
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    option(BUILD_DOCS "Build documentation" ON)
    option(BUILD_TESTS "Build tests" ON)
endif()

#
# Testing and documentation are only available if this is the main project
#
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    if(BUILD_DOCS)
        add_subdirectory(third-party/doxyconfig docs)
    endif()

    if(BUILD_TESTS)
        #
        # Additional setup for coverage
        # https://gcovr.com/en/stable/guide/compiling.html#compiler-options
        #
        if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
            set(CMAKE_CXX_FLAGS "-fprofile-arcs -ftest-coverage -ggdb -O0")
            set(CMAKE_C_FLAGS "-fprofile-arcs -ftest-coverage -ggdb -O0")
        endif()

        enable_testing()
        add_subdirectory(tests)
    endif()
endif()

#
# Library code is located here
# When building tests this must be after the coverage flags are set
#
add_subdirectory(src)
