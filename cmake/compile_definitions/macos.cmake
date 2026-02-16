# macos specific compile definitions

add_compile_definitions(SUNSHINE_PLATFORM="macos")

set(MACOS_LINK_DIRECTORIES
        /opt/homebrew/lib
        /opt/local/lib
        /usr/local/lib)

foreach(dir ${MACOS_LINK_DIRECTORIES})
    if(EXISTS ${dir})
        link_directories(${dir})
    endif()
endforeach()

if(NOT BOOST_USE_STATIC AND NOT FETCH_CONTENT_BOOST_USED)
    ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
endif()

# ScreenCaptureKit for system audio capture (macOS 12.3+)
FIND_LIBRARY(SCREEN_CAPTURE_KIT_LIBRARY ScreenCaptureKit)

# IOKit for virtual HID gamepad (IOHIDUserDevice)
FIND_LIBRARY(IOKIT_LIBRARY IOKit)

list(APPEND SUNSHINE_EXTERNAL_LIBRARIES
        ${APP_KIT_LIBRARY}
        ${APP_SERVICES_LIBRARY}
        ${AV_FOUNDATION_LIBRARY}
        ${CORE_MEDIA_LIBRARY}
        ${CORE_VIDEO_LIBRARY}
        ${FOUNDATION_LIBRARY}
        ${VIDEO_TOOLBOX_LIBRARY}
        ${SCREEN_CAPTURE_KIT_LIBRARY}
        ${IOKIT_LIBRARY})

set(APPLE_PLIST_FILE "${SUNSHINE_SOURCE_ASSETS_DIR}/macos/assets/Info.plist")

set(PLATFORM_TARGET_FILES
        "${CMAKE_SOURCE_DIR}/src/platform/macos/av_audio.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/av_audio.m"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/av_img_t.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/av_video.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/av_video.m"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/sc_capture.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/sc_capture.m"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/sc_audio.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/sc_audio.m"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/gamepad.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/gamepad.m"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/hid_gamepad.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/hid_gamepad.m"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/virtual_display.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/virtual_display.m"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/display.mm"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/input.mm"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/microphone.mm"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/misc.mm"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/misc.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/nv12_zero_device.cpp"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/nv12_zero_device.h"
        "${CMAKE_SOURCE_DIR}/src/platform/macos/publish.cpp"
        "${CMAKE_SOURCE_DIR}/third-party/TPCircularBuffer/TPCircularBuffer.c"
        "${CMAKE_SOURCE_DIR}/third-party/TPCircularBuffer/TPCircularBuffer.h"
        ${APPLE_PLIST_FILE})

# virtual_display.m uses ARC (required for CGVirtualDisplay private API lifecycle management)
set_source_files_properties(
        "${CMAKE_SOURCE_DIR}/src/platform/macos/virtual_display.m"
        PROPERTIES COMPILE_FLAGS "-fobjc-arc")

if(SUNSHINE_ENABLE_TRAY)
    list(APPEND SUNSHINE_EXTERNAL_LIBRARIES
            ${COCOA})
    list(APPEND PLATFORM_TARGET_FILES
            "${CMAKE_SOURCE_DIR}/third-party/tray/src/tray_darwin.m")
endif()

# Build vd_helper: standalone subprocess for creating CGVirtualDisplay
add_executable(vd_helper "${CMAKE_SOURCE_DIR}/src/platform/macos/vd_helper.m")
set_source_files_properties(
        "${CMAKE_SOURCE_DIR}/src/platform/macos/vd_helper.m"
        PROPERTIES COMPILE_FLAGS "-fobjc-arc")
target_link_libraries(vd_helper PRIVATE
        "-framework Foundation"
        "-framework AppKit"
        "-framework CoreGraphics"
        "-F/System/Library/PrivateFrameworks"
        "-framework SkyLight")
# Place vd_helper next to the sunshine binary
set_target_properties(vd_helper PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
