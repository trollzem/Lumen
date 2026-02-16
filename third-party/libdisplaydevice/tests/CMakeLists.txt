#
# Setup google test
#
set(INSTALL_GTEST OFF)
set(INSTALL_GMOCK OFF)
include(GoogleTest)
add_subdirectory("${PROJECT_SOURCE_DIR}/third-party/googletest" "third-party/googletest")

if (WIN32)
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "Always use msvcrt.dll" FORCE)  # cmake-lint: disable=C0103
endif ()

# A helper function to setup the dependencies for the test executable
function(add_dd_test_dir)
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs ADDITIONAL_LIBRARIES ADDITIONAL_SOURCES)
    cmake_parse_arguments(FN_VARS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Get the current sources and libraries
    get_property(sources GLOBAL PROPERTY DD_TEST_SOURCES)
    get_property(libraries GLOBAL PROPERTY DD_TEST_LIBRARIES)

    # Gather new data
    file(GLOB test_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")

    list(APPEND sources ${test_files})
    list(APPEND libraries ${FN_VARS_ADDITIONAL_LIBRARIES})

    foreach (source_pattern ${FN_VARS_ADDITIONAL_SOURCES})
        file(GLOB source_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${source_pattern}")
        foreach (source_file ${source_files})
            list(APPEND sources ${source_file})
        endforeach ()
    endforeach ()

    # Update the global variables
    set_property(GLOBAL PROPERTY DD_TEST_SOURCES "${sources}")
    set_property(GLOBAL PROPERTY DD_TEST_LIBRARIES "${libraries}")
endfunction()

#
# Add subdirectories
#
add_subdirectory(fixtures)
add_subdirectory(unit)

#
# Setup the final test binary
#
set(TEST_BINARY test_libdisplaydevice)
get_property(sources GLOBAL PROPERTY DD_TEST_SOURCES)
get_property(libraries GLOBAL PROPERTY DD_TEST_LIBRARIES)

add_executable(${TEST_BINARY} ${sources})
target_link_libraries(${TEST_BINARY}
        PUBLIC
        gmock_main  # if we use this we don't need our own main function
        libdisplaydevice::display_device  # this target includes common + platform specific targets
        libfixtures # these are our fixtures/helpers for the tests
        ${libraries} # additional libraries if needed
)

# Add the test to CTest
gtest_discover_tests(${TEST_BINARY})
