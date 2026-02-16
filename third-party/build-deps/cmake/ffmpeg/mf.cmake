if(BUILD_FFMPEG_ALL_PATCHES OR BUILD_FFMPEG_MF_PATCHES)
    file(GLOB FFMPEG_MF_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/FFmpeg/MF/*.patch)

    foreach(patch_file ${FFMPEG_MF_FILES})
        APPLY_GIT_PATCH(${FFMPEG_GENERATED_SRC_PATH} ${patch_file})
    endforeach()
endif()
