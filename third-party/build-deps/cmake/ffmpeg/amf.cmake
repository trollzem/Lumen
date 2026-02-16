set(AMF_GENERATED_SRC_PATH ${CMAKE_CURRENT_BINARY_DIR}/FFmpeg/AMF)

if(BUILD_FFMPEG_ALL_PATCHES OR BUILD_FFMPEG_AMF_PATCHES)
    file(GLOB FFMPEG_AMF_PATCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/patches/FFmpeg/FFmpeg/AMF/*.patch)

    foreach(patch_file ${FFMPEG_AMF_PATCH_FILES})
        APPLY_GIT_PATCH(${FFMPEG_GENERATED_SRC_PATH} ${patch_file})
    endforeach()
endif()

set(AMF_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/usr/local/include/AMF)
add_custom_target(amf ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${AMF_GENERATED_SRC_PATH}/amf/public/include"
            "${AMF_TARGET_DIR}"
        COMMENT "Copying AMF headers"
)
add_dependencies(${CMAKE_PROJECT_NAME} amf)
install(DIRECTORY ${AMF_TARGET_DIR}
        DESTINATION ${FFMPEG_INSTALL_PREFIX}/include
)
