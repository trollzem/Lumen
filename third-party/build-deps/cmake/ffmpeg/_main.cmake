# the destination directory needs to be same level down as the original source directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/third-party/FFmpeg DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(FFMPEG_GENERATED_SRC_PATH ${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/FFmpeg)
set(AVCODEC_GENERATED_SRC_PATH ${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/libavcodec)

if(WIN32)
    set(BUILD_FFMPEG_LIBVA OFF)

    # We must disable CUDA and NVENC on ARM64 until following issues is resolved
    # https://github.com/FFmpeg/FFmpeg/blob/4e5523c98597a417eb43555933b1075d18ec5f8b/configure#L7443
    if (${arch} STREQUAL "arm64")
        set(BUILD_FFMPEG_NV_CODEC_HEADERS OFF)
    endif()
elseif(APPLE)
    set(BUILD_FFMPEG_AMF OFF)
    set(BUILD_FFMPEG_MF OFF)
    set(BUILD_FFMPEG_NV_CODEC_HEADERS OFF)
    set(BUILD_FFMPEG_LIBVA OFF)
elseif(FREEBSD)
    set(BUILD_FFMPEG_AMF OFF)
    set(BUILD_FFMPEG_MF OFF)
    if(${arch} STREQUAL "aarch64")
        set(BUILD_FFMPEG_NV_CODEC_HEADERS OFF)
    endif()
elseif(UNIX)
    set(BUILD_FFMPEG_MF OFF)
endif()

set(_original_cmake_install_prefix ${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_PREFIX ${FFMPEG_INSTALL_PREFIX})

if(BUILD_FFMPEG_AMF)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/amf.cmake)
endif()

if(BUILD_FFMPEG_MF)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/mf.cmake)
endif()

if(BUILD_FFMPEG_NV_CODEC_HEADERS)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/nv_codec_headers.cmake)
endif()

if(BUILD_FFMPEG_SVT_AV1)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/svt_av1.cmake)
endif()

if(BUILD_FFMPEG_LIBVA)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/libva.cmake)
endif()

if(BUILD_FFMPEG_X264)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/x264.cmake)
endif()

if(BUILD_FFMPEG_X265)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/x265.cmake)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ffmpeg/ffmpeg.cmake)

set(CMAKE_INSTALL_PREFIX ${_original_cmake_install_prefix})
