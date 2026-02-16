if(BUILD_FFMPEG_ALL_PATCHES OR BUILD_FFMPEG_CBS_PATCHES)
    file(GLOB FFMPEG_CBS_PATCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/FFmpeg/cbs/*.patch)

    foreach(patch_file ${FFMPEG_CBS_PATCH_FILES})
        APPLY_GIT_PATCH(${FFMPEG_GENERATED_SRC_PATH} ${patch_file})
    endforeach()
endif()

if(${arch} STREQUAL "aarch64" OR ${arch} STREQUAL "arm64")
    set(CBS_ARCH_PATH arm)
elseif (${arch} STREQUAL "ppc64le")
    set(CBS_ARCH_PATH ppc)
elseif (${arch} STREQUAL "amd64" OR ${arch} STREQUAL "x86_64")
    set(CBS_ARCH_PATH x86)
elseif (${arch} STREQUAL "mips")
    set(CBS_ARCH_PATH mips)
else()
    message(FATAL_ERROR "Unsupported system processor:" ${CMAKE_SYSTEM_PROCESSOR})
endif()

list(APPEND FFMPEG_EXTRA_CONFIGURE
        --prefix=${CMAKE_CURRENT_BINARY_DIR_UNIX}/FFmpeg
        --pkg-config=${PKG_CONFIG_EXECUTABLE}
        --extra-cflags='${CMAKE_C_FLAGS}'
        --extra-cxxflags='${CMAKE_CXX_FLAGS}'
        --cc=${CMAKE_C_COMPILER}
        --cxx=${CMAKE_CXX_COMPILER}
        --ar=${CMAKE_AR}
        --ranlib=${CMAKE_RANLIB}
        --pkg-config-flags='--static'
        --extra-cflags='-I${CMAKE_CURRENT_BINARY_DIR_UNIX}/usr/local/include'
        --extra-cflags='-I${CMAKE_CURRENT_BINARY_DIR_UNIX}/x264/include'
        --extra-cflags='-I${CMAKE_CURRENT_BINARY_DIR_UNIX}/libva/include'
        --extra-ldflags='-L${CMAKE_CURRENT_BINARY_DIR_UNIX}/usr/local/lib'
        --extra-ldflags='-L${CMAKE_CURRENT_BINARY_DIR_UNIX}/x264/lib'
        --extra-ldflags='-L${CMAKE_CURRENT_BINARY_DIR_UNIX}/libva/lib'
        --extra-libs='-lpthread -lm'
        --disable-all
        --disable-autodetect
        --disable-iconv
        --enable-gpl
        --enable-static
        --enable-avcodec
        --enable-avutil
        --enable-bsfs  # ensure config.h will have CONFIG_CBS_ flags
        --enable-swscale
)

if(BUILD_FFMPEG_AMF)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-amf
            --enable-encoder=h264_amf,hevc_amf,av1_amf
    )
endif()
if(BUILD_FFMPEG_MF)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-encoder=h264_mf,hevc_mf,av1_mf
            --enable-mediafoundation
    )
endif()
if(BUILD_FFMPEG_NV_CODEC_HEADERS)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-cuda
            --enable-encoder=h264_nvenc,hevc_nvenc,av1_nvenc
            --enable-ffnvcodec
            --enable-nvenc
    )
    if(UNIX AND NOT APPLE AND NOT FREEBSD)
        list(APPEND FFMPEG_EXTRA_CONFIGURE
                --enable-cuda_llvm
        )
    endif()
endif()
if(BUILD_FFMPEG_SVT_AV1)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-libsvtav1
            --enable-encoder=libsvtav1
    )
endif()
if(BUILD_FFMPEG_LIBVA)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-vaapi
            --enable-encoder=h264_vaapi,hevc_vaapi,av1_vaapi
    )
endif()
if(BUILD_FFMPEG_X264)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-libx264
            --enable-encoder=libx264
    )
endif()
if(BUILD_FFMPEG_X265)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-libx265
            --enable-encoder=libx265
    )
endif()

# OS specific options not defined by the above BUILD_FFMPEG_* options
if(WIN32)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-d3d11va
            --enable-encoder=h264_qsv,hevc_qsv,av1_qsv
            --enable-libvpl
    )
elseif(APPLE)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-encoder=h264_videotoolbox,hevc_videotoolbox
            --enable-videotoolbox
    )
elseif(FREEBSD)
    # FFmpeg does not build with this encoder on aarch64 FreeBSD
    if(${arch} STREQUAL "amd64")
        list(APPEND FFMPEG_EXTRA_CONFIGURE
                --enable-encoder=h264_v4l2m2m
                --enable-v4l2_m2m
        )
    endif()
elseif(UNIX)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --enable-encoder=h264_v4l2m2m
            --enable-v4l2_m2m
    )
endif()

if(CMAKE_CROSSCOMPILING)
    list(APPEND FFMPEG_EXTRA_CONFIGURE
            --arch=${arch}
            --enable-cross-compile
            --target-os=${TARGET_OS}
    )
    if(UNIX AND NOT APPLE)
        list(APPEND FFMPEG_EXTRA_CONFIGURE
                --cross-prefix=/usr/bin/${CMAKE_C_COMPILER_TARGET}-
        )
    endif()
endif()

# convert list to string
# configure command will only take the first argument if not converted to string
string(REPLACE ";" " " FFMPEG_EXTRA_CONFIGURE "${FFMPEG_EXTRA_CONFIGURE}")
message(STATUS "FFmpeg configure options: ${FFMPEG_EXTRA_CONFIGURE}")

set(WORKING_DIR "${FFMPEG_GENERATED_SRC_PATH}")
UNIX_PATH(WORKING_DIR_UNIX ${WORKING_DIR})
add_custom_target(ffmpeg ALL
        COMMAND ${SHELL_CMD} "PKG_CONFIG_PATH='${PKG_CONFIG_PATH}' \
./configure \
${FFMPEG_EXTRA_CONFIGURE}"
        COMMAND ${SHELL_CMD} "${MAKE_EXECUTABLE} --jobs=${N_PROC}"
        COMMAND ${SHELL_CMD} "${MAKE_EXECUTABLE} install"
        WORKING_DIRECTORY ${WORKING_DIR}
        COMMENT "Target: FFmpeg"
        COMMAND_EXPAND_LISTS
        USES_TERMINAL
        VERBATIM
)
if(BUILD_FFMPEG_AMF)
    add_dependencies(ffmpeg amf)
endif()
if(BUILD_FFMPEG_NV_CODEC_HEADERS)
    add_dependencies(ffmpeg nv-codec-headers)
endif()
if(BUILD_FFMPEG_SVT_AV1)
    add_dependencies(ffmpeg SvtAv1)
endif()
if(BUILD_FFMPEG_LIBVA)
    add_dependencies(ffmpeg libva)
endif()
if(BUILD_FFMPEG_X264)
    add_dependencies(ffmpeg x264)
endif()
if(BUILD_FFMPEG_X265)
    add_dependencies(ffmpeg x265)
endif()
add_dependencies(${CMAKE_PROJECT_NAME} ffmpeg)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/include/"
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/include)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/lib/"
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/lib)


#
# cbs
#
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/libcbs.pc.in
        ${CMAKE_CURRENT_BINARY_DIR}/libcbs.pc @ONLY)

set(AVCODEC_GENERATED_SRC_PATH ${FFMPEG_GENERATED_SRC_PATH}/libavcodec)
set(AVUTIL_GENERATED_SRC_PATH ${FFMPEG_GENERATED_SRC_PATH}/libavutil)

set(EXTRA_FFMPEG_INCLUDE_FILES
        ${FFMPEG_GENERATED_SRC_PATH}/config.h
)
set(EXTRA_AVCODEC_INCLUDE_FILES
        ${AVCODEC_GENERATED_SRC_PATH}/av1.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_av1.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_bsf.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_h2645.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_h264.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_h265.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_jpeg.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_mpeg2.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_sei.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_vp8.h
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_vp9.h
        ${AVCODEC_GENERATED_SRC_PATH}/codec_desc.h
        ${AVCODEC_GENERATED_SRC_PATH}/codec_id.h
        ${AVCODEC_GENERATED_SRC_PATH}/codec_par.h
        ${AVCODEC_GENERATED_SRC_PATH}/defs.h
        ${AVCODEC_GENERATED_SRC_PATH}/get_bits.h
        ${AVCODEC_GENERATED_SRC_PATH}/h264_levels.h
        ${AVCODEC_GENERATED_SRC_PATH}/h2645_parse.h
        ${AVCODEC_GENERATED_SRC_PATH}/h264.h
        ${AVCODEC_GENERATED_SRC_PATH}/mathops.h
        ${AVCODEC_GENERATED_SRC_PATH}/packet.h
        ${AVCODEC_GENERATED_SRC_PATH}/sei.h
        ${AVCODEC_GENERATED_SRC_PATH}/version_major.h
        ${AVCODEC_GENERATED_SRC_PATH}/vlc.h
)
set(EXTRA_AVCODEC_HEVC_INCLUDE_FILES
        ${AVCODEC_GENERATED_SRC_PATH}/hevc/hevc.h
)
set(EXTRA_AVUTIL_INCLUDE_FILES
        ${AVUTIL_GENERATED_SRC_PATH}/attributes.h
        ${AVUTIL_GENERATED_SRC_PATH}/attributes_internal.h
        ${AVUTIL_GENERATED_SRC_PATH}/intmath.h
)

set(CBS_SOURCE_FILES
        ${AVCODEC_GENERATED_SRC_PATH}/cbs.c
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_h2645.c
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_av1.c
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_vp8.c
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_vp9.c
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_mpeg2.c
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_jpeg.c
        ${AVCODEC_GENERATED_SRC_PATH}/cbs_sei.c
        ${AVCODEC_GENERATED_SRC_PATH}/h264_levels.c
        ${AVCODEC_GENERATED_SRC_PATH}/h2645_parse.c
        ${AVCODEC_GENERATED_SRC_PATH}/vp8data.c
        ${AVUTIL_GENERATED_SRC_PATH}/intmath.c
)

add_library(cbs STATIC ${CBS_SOURCE_FILES})
target_include_directories(cbs PRIVATE
        ${FFMPEG_GENERATED_SRC_PATH}
)
target_compile_options(cbs PRIVATE -Wall -Wno-incompatible-pointer-types -Wno-format -Wno-format-extra-args)
add_dependencies(cbs ffmpeg)
add_dependencies(${CMAKE_PROJECT_NAME} cbs)

# install cbs target headers
install(FILES ${EXTRA_FFMPEG_INCLUDE_FILES}
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/include)
install(FILES ${EXTRA_AVCODEC_INCLUDE_FILES}
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/include/libavcodec)
install(FILES ${EXTRA_AVCODEC_HEVC_INCLUDE_FILES}
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/include/libavcodec/hevc)
install(FILES ${EXTRA_AVUTIL_INCLUDE_FILES}
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/include/libavutil)
install(TARGETS cbs
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/lib)

# conditional headers based on architecture
if (EXISTS ${AVCODEC_GENERATED_SRC_PATH}/${CBS_ARCH_PATH}/mathops.h)
    install(FILES ${AVCODEC_GENERATED_SRC_PATH}/${CBS_ARCH_PATH}/mathops.h
            DESTINATION ${FFMPEG_INSTALL_PREFIX}/include/libavcodec/${CBS_ARCH_PATH})
endif()
if (EXISTS ${AVUTIL_GENERATED_SRC_PATH}/${CBS_ARCH_PATH}/asm.h)
    install(FILES ${AVUTIL_GENERATED_SRC_PATH}/${CBS_ARCH_PATH}/asm.h
            DESTINATION ${FFMPEG_INSTALL_PREFIX}/include/libavutil/${CBS_ARCH_PATH})
endif()
if (EXISTS ${AVUTIL_GENERATED_SRC_PATH}/${CBS_ARCH_PATH}/intmath.h)
    install(FILES ${AVUTIL_GENERATED_SRC_PATH}/${CBS_ARCH_PATH}/intmath.h
            DESTINATION ${FFMPEG_INSTALL_PREFIX}/include/libavutil/${CBS_ARCH_PATH})
endif()

# install pkg-config file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libcbs.pc
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/lib/pkgconfig)
