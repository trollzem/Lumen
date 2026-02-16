# General platform-agnostic tests (or without hard platform dependencies)
add_subdirectory(general)

# Platform specific tests
if(WIN32)
    add_subdirectory(windows)
elseif(APPLE)
    message(WARNING "MacOS is not supported yet.")
elseif(UNIX)
    message(WARNING "Linux is not supported yet.")
else()
    message(FATAL_ERROR "Unsupported platform")
endif()
