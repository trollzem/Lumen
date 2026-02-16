CPMGetPackage(libva)

set(LIBVA_GENERATED_SRC_PATH ${libva_SOURCE_DIR})

if(BUILD_FFMPEG_ALL_PATCHES OR BUILD_FFMPEG_LIBVA_PATCHES)
    file(GLOB FFMPEG_LIBVA_PATCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/libva/*.patch)

    foreach(patch_file ${FFMPEG_LIBVA_PATCH_FILES})
        APPLY_GIT_PATCH(${LIBVA_GENERATED_SRC_PATH} ${patch_file})
    endforeach()
endif()

# libva uses autotools build system
set(WORKING_DIR "${LIBVA_GENERATED_SRC_PATH}")
UNIX_PATH(WORKING_DIR_UNIX ${WORKING_DIR})

# Configure options for libva
list(APPEND LIBVA_EXTRA_CONFIGURE
        --prefix=${CMAKE_CURRENT_BINARY_DIR_UNIX}/libva
        --enable-static
        --disable-shared
        --enable-drm
        --enable-x11
        --enable-glx
        --enable-wayland
        --without-legacy
)

# On FreeBSD, disable dependency tracking to avoid gmake/make compatibility issues
if(FREEBSD)
    list(APPEND LIBVA_EXTRA_CONFIGURE --disable-dependency-tracking)
endif()

if(CMAKE_CROSSCOMPILING)
    set(LIBVA_EXTRA_CONFIGURE
            ${LIBVA_EXTRA_CONFIGURE}
            --host=${CMAKE_C_COMPILER_TARGET}
    )
endif()

# Convert list to string
# configure command will only take the first argument if not converted to string
string(REPLACE ";" " " LIBVA_EXTRA_CONFIGURE "${LIBVA_EXTRA_CONFIGURE}")

add_custom_target(libva ALL
        COMMAND ${SHELL_CMD} "${MAKE_COMPILER_FLAGS} ./autogen.sh ${LIBVA_EXTRA_CONFIGURE}"
        COMMAND ${SHELL_CMD} "${MAKE_COMPILER_FLAGS} ${MAKE_EXECUTABLE} --jobs=${N_PROC}"
        COMMAND ${SHELL_CMD} "${MAKE_COMPILER_FLAGS} ${MAKE_EXECUTABLE} install"
        WORKING_DIRECTORY ${WORKING_DIR}
        COMMENT "Target: libva"
        COMMAND_EXPAND_LISTS
        USES_TERMINAL
        VERBATIM
        BYPRODUCTS
            "${CMAKE_CURRENT_BINARY_DIR}/libva/lib/libva.a"
            "${CMAKE_CURRENT_BINARY_DIR}/libva/lib/libva-drm.a"
            "${CMAKE_CURRENT_BINARY_DIR}/libva/lib/libva-x11.a"
            "${CMAKE_CURRENT_BINARY_DIR}/libva/lib/libva-glx.a"
            "${CMAKE_CURRENT_BINARY_DIR}/libva/lib/libva-wayland.a"
)
add_dependencies(${CMAKE_PROJECT_NAME} libva)

# Add to PKG_CONFIG_PATH for FFmpeg to find
set(PKG_CONFIG_PATH "${CMAKE_CURRENT_BINARY_DIR_UNIX}/libva/lib/pkgconfig:${PKG_CONFIG_PATH}")
