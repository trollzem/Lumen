#
# Project configuration
#
cmake_minimum_required(VERSION 3.13 FATAL_ERROR) # target_link_directories
project(tray VERSION 0.0.0
        DESCRIPTION "A cross-platform system tray library"
        HOMEPAGE_URL "https://app.lizardbyte.dev"
        LANGUAGES C)

set(PROJECT_LICENSE "MIT")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# Add our custom CMake modules to the global path
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#
# Project optional configuration
#
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    option(BUILD_DOCS "Build documentation" ON)
    option(BUILD_TESTS "Build tests" ON)
endif()

# Generate 'compile_commands.json' for clang_complete
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package (PkgConfig REQUIRED)

file(GLOB TRAY_SOURCES
        "${CMAKE_SOURCE_DIR}/src/*.h"
        "${CMAKE_SOURCE_DIR}/icons/*.ico"
        "${CMAKE_SOURCE_DIR}/icons/*.png")

if(WIN32)
    list(APPEND TRAY_SOURCES "${CMAKE_SOURCE_DIR}/src/tray_windows.c")
else()
    if(UNIX)
        if(APPLE)
            find_library(COCOA Cocoa REQUIRED)
            list(APPEND TRAY_SOURCES "${CMAKE_SOURCE_DIR}/src/tray_darwin.m")
        else()
            find_package(APPINDICATOR REQUIRED)
            find_package(LibNotify REQUIRED)
            list(APPEND TRAY_SOURCES "${CMAKE_SOURCE_DIR}/src/tray_linux.c")
        endif()
    endif()
endif()

add_library(${PROJECT_NAME} STATIC ${TRAY_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 99)

if(WIN32)
    list(APPEND TRAY_DEFINITIONS TRAY_WINAPI=1 WIN32_LEAN_AND_MEAN NOMINMAX)
    if(MSVC)
        list(APPEND TRAY_COMPILE_OPTIONS "/MT$<$<CONFIG:Debug>:d>")
    endif()
else()
    if(UNIX)
        if(APPLE)
            list(APPEND TRAY_DEFINITIONS TRAY_APPKIT=1)
            list(APPEND TRAY_EXTERNAL_LIBRARIES ${COCOA})
        else()
            list(APPEND TRAY_COMPILE_OPTIONS ${APPINDICATOR_CFLAGS})
            list(APPEND TRAY_EXTERNAL_DIRECTORIES ${APPINDICATOR_LIBRARY_DIRS})
            list(APPEND TRAY_DEFINITIONS TRAY_APPINDICATOR=1)
            if(APPINDICATOR_AYATANA)
                list(APPEND TRAY_DEFINITIONS TRAY_AYATANA_APPINDICATOR=1)
            endif()
            if(APPINDICATOR_LEGACY)
                list(APPEND TRAY_DEFINITIONS TRAY_LEGACY_APPINDICATOR=1)
            endif()
            list(APPEND TRAY_LIBNOTIFY=1)
            list(APPEND TRAY_EXTERNAL_LIBRARIES ${APPINDICATOR_LIBRARIES} ${LIBNOTIFY_LIBRARIES})

            include_directories(SYSTEM ${APPINDICATOR_INCLUDE_DIRS} ${LIBNOTIFY_INCLUDE_DIRS})
            link_directories(${APPINDICATOR_LIBRARY_DIRS} ${LIBNOTIFY_LIBRARY_DIRS})
        endif()
    endif()
endif()

add_library(tray::tray ALIAS ${PROJECT_NAME})

add_executable(tray_example "${CMAKE_SOURCE_DIR}/src/example.c")
target_link_libraries(tray_example tray::tray)

configure_file("${CMAKE_SOURCE_DIR}/icons/icon.ico" "${CMAKE_BINARY_DIR}/icon.ico" COPYONLY)
configure_file("${CMAKE_SOURCE_DIR}/icons/icon.png" "${CMAKE_BINARY_DIR}/icon.png" COPYONLY)

INSTALL(TARGETS tray tray DESTINATION lib)

IF(NOT WIN32)
    INSTALL(FILES tray.h DESTINATION include)
ENDIF()

target_compile_definitions(${PROJECT_NAME} PRIVATE ${TRAY_DEFINITIONS})
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAY_COMPILE_OPTIONS})
target_link_directories(${PROJECT_NAME} PRIVATE ${TRAY_EXTERNAL_DIRECTORIES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${TRAY_EXTERNAL_LIBRARIES})

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
