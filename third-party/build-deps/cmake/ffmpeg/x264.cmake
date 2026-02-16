set(X264_GENERATED_SRC_PATH ${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/x264)

if(BUILD_FFMPEG_ALL_PATCHES OR BUILD_FFMPEG_X264_PATCHES)
    file(GLOB FFMPEG_X264_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/FFmpeg/x264/*.patch)

    foreach(patch_file ${FFMPEG_X264_FILES})
        APPLY_GIT_PATCH(${FFMPEG_GENERATED_SRC_PATH} ${patch_file})
    endforeach()
endif()

if(${arch} STREQUAL "aarch64" OR ${arch} STREQUAL "arm64")
    set(X264_ARCH aarch64)
elseif (${arch} STREQUAL "ppc64le")
    set(X264_ARCH powerpc64le)
elseif (${arch} STREQUAL "amd64" OR ${arch} STREQUAL "x86_64")
    set(X264_ARCH x86_64)
elseif (${arch} STREQUAL "mips")
    set(X264_ARCH mips)  # TODO: unknown if this is the correct value
else()
    message(FATAL_ERROR "Unsupported system processor:" ${CMAKE_SYSTEM_PROCESSOR})
endif()

if(WIN32)
    set(X264_HOST ${X264_ARCH}-mingw64)
elseif(APPLE)
    set(X264_HOST ${X264_ARCH}-darwin)
elseif(UNIX)
    set(X264_HOST ${X264_ARCH}-linux)
else()
    message(FATAL_ERROR "Unsupported system name:" ${CMAKE_SYSTEM_NAME})
endif()

if(CMAKE_CROSSCOMPILING)
    if(UNIX AND NOT APPLE)
        set(FFMPEG_X264_EXTRA_CONFIGURE
                --cross-prefix=/usr/bin/${CMAKE_C_COMPILER_TARGET}-
                --host=${X264_HOST}
        )
    endif()
endif()

# convert list to string
# configure command will only take the first argument if not converted to string
string(REPLACE ";" " " FFMPEG_X264_EXTRA_CONFIGURE "${FFMPEG_X264_EXTRA_CONFIGURE}")

# On Windows, the x264 submodule needs to have line endings converted to LF, see the README.md

set(WORKING_DIR "${X264_GENERATED_SRC_PATH}")
UNIX_PATH(WORKING_DIR_UNIX ${WORKING_DIR})
add_custom_target(x264 ALL
        COMMAND ${SHELL_CMD} "${MAKE_COMPILER_FLAGS} ./configure \
--prefix=${CMAKE_CURRENT_BINARY_DIR_UNIX}/x264 \
--disable-cli \
--enable-static \
${FFMPEG_X264_EXTRA_CONFIGURE}"
        COMMAND ${SHELL_CMD} "${MAKE_COMPILER_FLAGS} ${MAKE_EXECUTABLE} --jobs=${N_PROC}"
        COMMAND ${SHELL_CMD} "${MAKE_COMPILER_FLAGS} ${MAKE_EXECUTABLE} install"
        WORKING_DIRECTORY ${WORKING_DIR}
        COMMENT "Target: x264"
        COMMAND_EXPAND_LISTS
        USES_TERMINAL
        VERBATIM
        BYPRODUCTS "x264/lib/libx264.a" "x264/include/x264.h" "x264/include/x264_config.h"
)
add_dependencies(${CMAKE_PROJECT_NAME} x264)
set(X264_HEADER_FILES
        ${CMAKE_CURRENT_BINARY_DIR}/x264/include/x264.h
        ${CMAKE_CURRENT_BINARY_DIR}/x264/include/x264_config.h
)
install(FILES ${X264_HEADER_FILES}
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/x264/lib/libx264.a
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/lib)
set(PKG_CONFIG_PATH "${CMAKE_CURRENT_BINARY_DIR_UNIX}/x264/lib/pkgconfig:${PKG_CONFIG_PATH}")
