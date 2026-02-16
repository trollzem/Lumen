# CMake toolchain file for cross compiling to linux powerpc64le

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR ppc64le)

if(DEFINED ENV{LINUX_GCC_VERSION})  # cmake-lint: disable=W0106
    set(LINUX_GCC_VERSION "-$ENV{LINUX_GCC_VERSION}")
else()
    set(LINUX_GCC_VERSION "")  # default to no version suffix
endif()

set(CMAKE_C_COMPILER_TARGET powerpc64le-linux-gnu)
set(CMAKE_CXX_COMPILER_TARGET ${CMAKE_C_COMPILER_TARGET})
set(CMAKE_C_COMPILER ${CMAKE_C_COMPILER_TARGET}-gcc${LINUX_GCC_VERSION})
set(CMAKE_CXX_COMPILER ${CMAKE_C_COMPILER_TARGET}-g++${LINUX_GCC_VERSION})

set(CMAKE_FIND_ROOT_PATH  /usr/${CMAKE_C_COMPILER_TARGET})
