# This a shared common library for other libraries
add_subdirectory(common)

# This is a platform-specific library
if(WIN32)
    add_subdirectory(windows)
elseif(APPLE)
    add_library(libdisplaydevice_macos_dummy INTERFACE)
    add_library(libdisplaydevice::platform ALIAS libdisplaydevice_macos_dummy)
    message(WARNING "MacOS is not supported yet.")
elseif(UNIX)
    add_library(libdisplaydevice_linux_dummy INTERFACE)
    add_library(libdisplaydevice::platform ALIAS libdisplaydevice_linux_dummy)
    message(WARNING "Linux is not supported yet.")
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

# Create a target that links to everything
add_library(libdisplaydevice_display_device INTERFACE)
target_link_libraries(libdisplaydevice_display_device INTERFACE
        libdisplaydevice::common
        libdisplaydevice::platform)

# Create an alias for the main target
add_library(libdisplaydevice::display_device ALIAS libdisplaydevice_display_device)
