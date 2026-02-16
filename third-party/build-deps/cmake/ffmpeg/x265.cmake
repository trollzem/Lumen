set(X265_GENERATED_SRC_PATH ${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/x265_git)

file(GLOB X265_GIT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/x265_git/*.patch)
foreach(patch_file ${X265_GIT_FILES})
    APPLY_GIT_PATCH(${X265_GENERATED_SRC_PATH} ${patch_file})
endforeach()

if(BUILD_FFMPEG_ALL_PATCHES OR BUILD_FFMPEG_X265_PATCHES)
    file(GLOB FFMPEG_X265_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/FFmpeg/x265/*.patch)

    foreach(patch_file ${FFMPEG_X265_FILES})
        APPLY_GIT_PATCH(${FFMPEG_GENERATED_SRC_PATH} ${patch_file})
    endforeach()
endif()

# options for x265
set(ENABLE_CLI OFF CACHE BOOL "Enable CLI")
set(ENABLE_SHARED OFF CACHE BOOL "Enable shared libraries")
set(STATIC_LINK_CRT ON CACHE BOOL "Static link CRT")

# not currently supported for aarch64
if(${arch} STREQUAL "amd64" OR ${arch} STREQUAL "x86_64")
    set(ENABLE_HDR10_PLUS ON CACHE BOOL "Enable HDR10+ support")
endif()

# ensure x265 installs into the ffmpeg prefix
set(_original_cmake_install_prefix ${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_PREFIX ${FFMPEG_INSTALL_PREFIX})

# build x265
add_subdirectory(${X265_GENERATED_SRC_PATH}/source x265 SYSTEM)
add_dependencies(${CMAKE_PROJECT_NAME} x265-static)

# install x265 as a build target, this must be installed before building FFmpeg
add_custom_target(x265
        COMMAND ${CMAKE_COMMAND} -P cmake_install.cmake
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/x265
        COMMENT "Installing x265"
)
add_dependencies(x265 x265-static)
add_dependencies(${CMAKE_PROJECT_NAME} x265)

if(ENABLE_HDR10_PLUS)
    add_dependencies(${CMAKE_PROJECT_NAME} hdr10plus-static)
    add_dependencies(x265 hdr10plus-static)
endif()

# PKG_CONFIG_PATH already set since this is installed directly to the prefix

set(CMAKE_INSTALL_PREFIX ${_original_cmake_install_prefix})
