set(SVT_AV1_GENERATED_SRC_PATH ${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/SVT-AV1)

if(BUILD_FFMPEG_ALL_PATCHES OR BUILD_FFMPEG_SVT_AV1_PATCHES)
    file(GLOB FFMPEG_SVT_AV1_PATCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/FFmpeg/SVT-AV1/*.patch)

    foreach(patch_file ${FFMPEG_SVT_AV1_PATCH_FILES})
        APPLY_GIT_PATCH(${FFMPEG_GENERATED_SRC_PATH} ${patch_file})
    endforeach()
endif()

# options for SVT-AV1
set(BUILD_APPS OFF CACHE BOOL "Build applications")
set(BUILD_DEC OFF CACHE BOOL "Build decoders")
set(ENABLE_AVX512 ON CACHE BOOL "Enable AVX512")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries")

# disable LTO because it forces projects linking against our pre-build to use the same compiler version
set(SVT_AV1_LTO OFF CACHE BOOL "Enable Link Time Optimization (LTO)")

# ensure SVT-AV1 installs into the ffmpeg prefix
set(_original_cmake_install_prefix ${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_PREFIX ${FFMPEG_INSTALL_PREFIX})

# build SVT-AV1
add_subdirectory(${SVT_AV1_GENERATED_SRC_PATH} SVT-AV1 SYSTEM)
add_dependencies(${CMAKE_PROJECT_NAME} SvtAv1Enc)

# install SVT-AV1 as a build target, this must be installed before building FFmpeg
add_custom_target(SvtAv1
        COMMAND ${CMAKE_COMMAND} -P cmake_install.cmake
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/SVT-AV1
        COMMENT "Installing SVT-AV1"
)
add_dependencies(SvtAv1 SvtAv1Enc)
add_dependencies(${CMAKE_PROJECT_NAME} SvtAv1)

# PKG_CONFIG_PATH already set since this is installed directly to the prefix

set(CMAKE_INSTALL_PREFIX ${_original_cmake_install_prefix})
