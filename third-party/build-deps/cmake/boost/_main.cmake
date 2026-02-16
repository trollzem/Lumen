CPMGetPackage(Boost)

if(BUILD_BOOST_SUNSHINE)
    set(BUILD_BOOST_LIBDISPLAYDEVICE ON)
endif()

list(APPEND BOOST_COMPONENTS_BASE
        asio
        crc
        format
        process
        property_tree
)

if(BUILD_BOOST_LIBDISPLAYDEVICE)
    list(APPEND BOOST_COMPONENTS
            algorithm
            preprocessor
            scope
            uuid
    )
endif()
if(BUILD_BOOST_SUNSHINE)
    list(APPEND BOOST_COMPONENTS
            filesystem
            locale
            log
            program_options
            system
    )
endif()

set(BOOST_ENABLE_CMAKE ON)  # Use experimental superproject to pull library dependencies recursively
set(BOOST_INCLUDE_LIBRARIES ${BOOST_COMPONENTS_BASE} ${BOOST_COMPONENTS})
set(BOOST_INSTALL_INCLUDE_SUBDIR "")  # boost uses a versioned directory by default on Windows
set(BOOST_SKIP_INSTALL_RULES OFF)  # disabled to allow installation of Boost libraries
set(Boost_USE_STATIC_LIBS ON)  # cmake-lint: disable=C0103

message(STATUS "Boost_BINARY_DIR: ${Boost_BINARY_DIR}")
message(STATUS "Boost_SOURCE_DIR: ${Boost_SOURCE_DIR}")

if(WIN32)
    # Windows build is failing to create header file in this directory
    file(MAKE_DIRECTORY ${Boost_BINARY_DIR}/libs/log/src/windows)
endif()

# https://github.com/boostorg/context/issues/311
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "ppc64le")
    set(BOOST_CONTEXT_ARCHITECTURE "ppc64")
endif ()

set(_original_cmake_install_prefix ${CMAKE_INSTALL_PREFIX})
set(_original_cmake_install_includedir ${CMAKE_INSTALL_INCLUDEDIR})
set(_original_cmake_install_libdir ${CMAKE_INSTALL_LIBDIR})

set(CMAKE_INSTALL_PREFIX ${BOOST_INSTALL_PREFIX})
set(CMAKE_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_PREFIX}/include")
set(CMAKE_INSTALL_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")

add_subdirectory(${Boost_SOURCE_DIR} ${Boost_BINARY_DIR} SYSTEM)

set(CMAKE_INSTALL_PREFIX ${_original_cmake_install_prefix})
set(CMAKE_INSTALL_INCLUDEDIR ${_original_cmake_install_includedir})
set(CMAKE_INSTALL_LIBDIR ${_original_cmake_install_libdir})
