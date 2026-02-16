cmake_minimum_required(VERSION 3.13)
# https://github.com/google/oss-policies-info/blob/main/foundational-cxx-support-matrix.md#foundational-c-support

project(test_tray)

include_directories("${CMAKE_SOURCE_DIR}")

# Add GoogleTest directory to the project
set(GTEST_SOURCE_DIR "${CMAKE_SOURCE_DIR}/third-party/googletest")
set(INSTALL_GTEST OFF)
set(INSTALL_GMOCK OFF)
add_subdirectory("${GTEST_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/googletest")
include_directories("${GTEST_SOURCE_DIR}/googletest/include" "${GTEST_SOURCE_DIR}")

# if windows
if (WIN32)
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # cmake-lint: disable=C0103
endif ()

file(GLOB_RECURSE TEST_SOURCES
        ${CMAKE_SOURCE_DIR}/tests/conftest.cpp
        ${CMAKE_SOURCE_DIR}/tests/utils.cpp
        ${CMAKE_SOURCE_DIR}/tests/test_*.cpp)

add_executable(${PROJECT_NAME}
        ${TEST_SOURCES}
        ${TRAY_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
target_link_directories(${PROJECT_NAME} PRIVATE ${TRAY_EXTERNAL_DIRECTORIES})
target_link_libraries(${PROJECT_NAME}
        ${TRAY_EXTERNAL_LIBRARIES}
        gtest
        gtest_main)  # if we use this we don't need our own main function
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAY_DEFINITIONS} ${TEST_DEFINITIONS})
target_compile_options(${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${TRAY_COMPILE_OPTIONS}>)
target_link_options(${PROJECT_NAME} PRIVATE)

add_test(NAME ${PROJECT_NAME} COMMAND tray_test)
