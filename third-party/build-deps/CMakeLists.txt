cmake_minimum_required(VERSION 3.25)

project(build-deps
        DESCRIPTION "Pre-built dependencies for LizardByte projects."
        VERSION 0.0.0
)

# Main options
option(BUILD_ALL "Build all dependencies" ON)
option(BUILD_ALL_LIBDISPLAYDEVICE "Build all libdisplaydevice dependencies" OFF)
option(BUILD_ALL_SUNSHINE "Build all Sunshine dependencies" OFF)

# Boost options
option(BUILD_BOOST "Build Boost" OFF)
option(BUILD_BOOST_LIBDISPLAYDEVICE "Build boost for libdisplaydevice" ON)
option(BUILD_BOOST_SUNSHINE "Build boost for Sunshine" ON)

# FFmpeg options
option(BUILD_FFMPEG "Build FFmpeg" OFF)
option(BUILD_FFMPEG_ALL_PATCHES "Apply FFmpeg patches" ON)
option(BUILD_FFMPEG_AMF "Build FFmpeg AMF" ON)
option(BUILD_FFMPEG_AMF_PATCHES "Apply FFmpeg AMF patches" ON)
option(BUILD_FFMPEG_CBS "Build FFmpeg CBS" ON)
option(BUILD_FFMPEG_CBS_PATCHES "Apply FFmpeg CBS patches" ON)
option(BUILD_FFMPEG_MF "Build FFmpeg Media Foundation" ON)
option(BUILD_FFMPEG_MF_PATCHES "Apply FFmpeg Media Foundation patches" ON)
option(BUILD_FFMPEG_NV_CODEC_HEADERS "Build FFmpeg NV Codec Headers" ON)
option(BUILD_FFMPEG_NV_CODEC_HEADERS_PATCHES "Apply FFmpeg NV Codec Headers patches" ON)
option(BUILD_FFMPEG_SVT_AV1 "Build FFmpeg SVT-AV1" ON)
option(BUILD_FFMPEG_SVT_AV1_PATCHES "Apply FFmpeg SVT-AV1 patches" ON)
option(BUILD_FFMPEG_LIBVA "Build FFmpeg with libva support" ON)
option(BUILD_FFMPEG_LIBVA_PATCHES "Apply FFmpeg libva patches" ON)
option(BUILD_FFMPEG_X264 "Build FFmpeg x264" ON)
option(BUILD_FFMPEG_X264_PATCHES "Apply FFmpeg x264 patches" ON)
option(BUILD_FFMPEG_X265 "Build FFmpeg x265" ON)
option(BUILD_FFMPEG_X265_PATCHES "Apply FFmpeg x265 patches" ON)

# common includes
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_git_patch.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/unix_path.cmake)

# CPM
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cpm/CPM.cmake)
CPMUsePackageLock(package-lock.cmake)

if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
    set(FREEBSD ON)
endif()

if(UNIX AND NOT APPLE)
    include(GNUInstallDirs)
endif()

set(EXTRA_COMPILE_FLAGS "-w" CACHE STRING "Extra compiler flags to use for all targets")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_COMPILE_FLAGS}")

# Set compiler the various targets that use make directly
set(MAKE_COMPILER_FLAGS "CFLAGS='${CMAKE_C_FLAGS}' CXXFLAGS='${CMAKE_CXX_FLAGS}'")
if(CMAKE_C_COMPILER)
    set(MAKE_COMPILER_FLAGS "${MAKE_COMPILER_FLAGS} CC=${CMAKE_C_COMPILER}")
endif()
if(CMAKE_CXX_COMPILER)
    set(MAKE_COMPILER_FLAGS "${MAKE_COMPILER_FLAGS} CXX=${CMAKE_CXX_COMPILER}")
endif()
message(STATUS "make compiler flags: ${MAKE_COMPILER_FLAGS}")

# if PARALLEL_BUILDS is not defined, set it to the number of processors
if(NOT DEFINED PARALLEL_BUILDS)
    include(ProcessorCount)
    ProcessorCount(N_PROC)
else()
    set(N_PROC ${PARALLEL_BUILDS})
endif()
if(N_PROC EQUAL 0)
    set(N_PROC 1)
elseif(NOT N_PROC MATCHES "^[0-9]+$")
    # if not a number, set it to 1
    set(N_PROC 1)
endif()
message(STATUS "parallel builds: ${N_PROC}")

set(MSYS2_ROOT "C:/msys64")
if(WIN32 AND DEFINED ENV{MSYS2_ROOT})
    set(MSYS2_ROOT "$ENV{MSYS2_ROOT}")
    message(STATUS "Detected MSYS2_ROOT, get msys2 path from environment: ${MSYS2_ROOT}")
endif()

if(NOT DEFINED BASH_EXECUTABLE)
    find_program(BASH_EXECUTABLE
            NAMES zsh bash
            HINTS "${MSYS2_ROOT}/usr/bin" /bin /usr/bin /usr/local/bin
            DOC "Bash shell executable"
            REQUIRED)
    message(STATUS "Found bash: ${BASH_EXECUTABLE}")
endif()

if(FREEBSD)
    set(MAKE_FIND_NAMES "gmake")  # We get invalid line types in nv-codec-headers Makefile if we use BSD make
else()
    set(MAKE_FIND_NAMES "mingw32-make;make;gmake")
endif()

find_program(MAKE_EXECUTABLE
        NAMES ${MAKE_FIND_NAMES}
        NAMES_PER_DIR
        HINTS "${MSYS2_ROOT}/usr/bin" /bin /usr/bin /usr/local/bin
        DOC "GNU Make"
        REQUIRED)
message(STATUS "Found make: ${MAKE_EXECUTABLE}")

if(WIN32)
    set(MAKE_EXECUTABLE "make")
    message(STATUS "Detected Windows, falling back to using make: ${MAKE_EXECUTABLE}")

    # fatal error when cross compiling
    if(CMAKE_CROSSCOMPILING)
        message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
        message(STATUS "Host system processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
        message(FATAL_ERROR "Cross-compiling is not supported on Windows")
    endif()

    set(MSYSTEM "UCRT64")
    set(MSYS2_ARG "-ucrt64")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "ARM64")
        set(MSYSTEM "CLANGARM64")
        set(MSYS2_ARG "-clangarm64")
    endif()

    find_file(MSYS2_EXECUTABLE NAMES msys2_shell.cmd REQUIRED HINTS ${MSYS2_ROOT})
    message(STATUS "Found MSYS2: ${MSYS2_EXECUTABLE}")

    if(NOT DEFINED MSYS2_OPTION OR MSYS2_OPTION STREQUAL "1" OR MSYS2_OPTION STREQUAL "")
        set(SHELL_CMD ${MSYS2_EXECUTABLE} ${MSYS2_ARG} -defterm -here -no-start -shell bash -c)
    elseif(MSYS2_OPTION STREQUAL "2")
        # Theoretically, this is equivalent to the above
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msys2.cmd.in ${CMAKE_CURRENT_BINARY_DIR}/msys2.cmd @ONLY)
        set(SHELL_CMD ${CMAKE_CURRENT_BINARY_DIR}/msys2.cmd)
    elseif(MSYS2_OPTION STREQUAL "3")
        # This one works okayish in CI, locally the PATH does not seem to be defined correctly
        set(SHELL_CMD ${CMAKE_COMMAND} -E env MSYSTEM=${MSYSTEM} ${BASH_EXECUTABLE} -c)
    endif()
else()
    set(SHELL_CMD ${BASH_EXECUTABLE} -c)
endif()

message(STATUS "Using shell command: ${SHELL_CMD}")

if(CMAKE_GENERATOR STREQUAL "MSYS Makefiles")
    # Ninja can handle the conversion automatically, but MSYS Makefiles cannot
    UNIX_PATH(MAKE_EXECUTABLE ${MAKE_EXECUTABLE})
endif()

UNIX_PATH(CMAKE_CURRENT_BINARY_DIR_UNIX ${CMAKE_CURRENT_BINARY_DIR})
UNIX_PATH(CMAKE_INSTALL_PREFIX_UNIX ${CMAKE_INSTALL_PREFIX})

# Set up separate install prefixes for BOOST and FFMPEG
# This allows us to package them separately
if(NOT DEFINED BOOST_INSTALL_PREFIX)
    set(BOOST_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/boost")
endif()
if(NOT DEFINED FFMPEG_INSTALL_PREFIX)
    set(FFMPEG_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/ffmpeg")
endif()

UNIX_PATH(BOOST_INSTALL_PREFIX_UNIX ${BOOST_INSTALL_PREFIX})
UNIX_PATH(FFMPEG_INSTALL_PREFIX_UNIX ${FFMPEG_INSTALL_PREFIX})

message(STATUS "Boost install prefix: ${BOOST_INSTALL_PREFIX}")
message(STATUS "FFmpeg install prefix: ${FFMPEG_INSTALL_PREFIX}")

# setup pkg-config path
if(CMAKE_CROSSCOMPILING AND UNIX AND NOT APPLE)
    set(PKG_CONFIG_PATH /usr/lib/${CMAKE_C_COMPILER_TARGET}/pkgconfig)
else()
    set(PKG_CONFIG_PATH $ENV{PKG_CONFIG_PATH})
    message(STATUS "env PKG_CONFIG_PATH: ${PKG_CONFIG_PATH}")

    # The ENV variable is not populated when running from CLion
    if(NOT PKG_CONFIG_PATH)
        # call a command to get the path
        execute_process(COMMAND ${SHELL_CMD} "echo \$PKG_CONFIG_PATH"
                OUTPUT_VARIABLE PKG_CONFIG_PATH
                OUTPUT_STRIP_TRAILING_WHITESPACE)
        message(STATUS "PKG_CONFIG_PATH from CLI command: ${PKG_CONFIG_PATH}")
    endif()

    if(WIN32)
        # normalize the path

        # TODO: try this, instead of the below code
        # cmake_path(CONVERT "${PKG_CONFIG_PATH}" TO_CMAKE_PATH_LIST PKG_CONFIG_PATH)
        # message(STATUS "Converted PKG_CONFIG_PATH: ${PKG_CONFIG_PATH}")

        file(TO_CMAKE_PATH "${PKG_CONFIG_PATH}" PKG_CONFIG_PATH)
        foreach(path ${PKG_CONFIG_PATH})
            UNIX_PATH(path ${path})
            list(APPEND PKG_CONFIG_PATH_UNIX ${path})
        endforeach()
        string(REPLACE ";" ":" PKG_CONFIG_PATH "${PKG_CONFIG_PATH_UNIX}")
    endif()
endif()
set(PKG_CONFIG_PATH "${FFMPEG_INSTALL_PREFIX_UNIX}/lib/pkgconfig:${PKG_CONFIG_PATH}")
message(STATUS "Initial PKG_CONFIG_PATH: ${PKG_CONFIG_PATH}")

find_program(PKG_CONFIG_EXECUTABLE
        NAMES pkg-config
        REQUIRED
        HINTS "${MSYS2_ROOT}/usr/bin" /bin /usr/bin /usr/local/bin)
UNIX_PATH(PKG_CONFIG_EXECUTABLE ${PKG_CONFIG_EXECUTABLE})
message(STATUS "Found pkg-config: ${PKG_CONFIG_EXECUTABLE}")

# set target os for ffmpeg build
string(TOLOWER ${CMAKE_SYSTEM_NAME} TARGET_OS)

# Custom target to use to ensure all dependencies are built
add_custom_target(${CMAKE_PROJECT_NAME}
        COMMENT "Completed build-deps"
)

# set architecture
string(TOLOWER ${CMAKE_SYSTEM_PROCESSOR} arch)

message(STATUS "Detected system name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Detected architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CI Matrix name should be: '${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}'")

# boost
# TODO: boost build fails on Windows due to NASM errors during configure?
# https://github.com/Orphis/boost-cmake/issues/42
# https://github.com/libsdl-org/SDL/issues/10250
if(BUILD_ALL OR BUILD_ALL_LIBDISPLAYDEVICE OR BUILD_ALL_SUNSHINE OR BUILD_BOOST)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/boost/_main.cmake)
endif()

# FFmpeg
if(BUILD_ALL OR BUILD_ALL_SUNSHINE OR BUILD_FFMPEG)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/_main.cmake)
endif()
