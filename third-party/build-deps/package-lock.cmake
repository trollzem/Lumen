# CPM Package Lock
# This file should be committed to version control

# The first argument of CPMDeclarePackage can be freely chosen and is used as argument in CPMGetPackage.
# The NAME argument should be package name that would also be used in a find_package call.
# Ideally, both are the same, which might not always be possible: https://github.com/cpm-cmake/CPM.cmake/issues/603
# This is needed to support CPM_USE_LOCAL_PACKAGES

# TODO: update dependencies with renovate
# https://joht.github.io/johtizen/automation/2022/08/03/keep-your-cpp-dependencies-up-to-date.html

set(PATCH_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/patches")

# Boost
CPMDeclarePackage(Boost
        NAME Boost
        VERSION 1.87.0
        URL https://github.com/boostorg/boost/releases/download/boost-1.87.0/boost-1.87.0-cmake.tar.xz
        URL_HASH SHA256=7da75f171837577a52bbf217e17f8ea576c7c246e4594d617bfde7fafd408be5
        PATCHES
            "${PATCH_DIRECTORY}/boost/01-fix-arm64-asm-compile.patch"
            "${PATCH_DIRECTORY}/boost/02-no-link-libatomic-clang-windows.patch"
        DOWNLOAD_ONLY YES
)

# libva
CPMDeclarePackage(libva
        NAME libva
        VERSION 2.23.0
        GIT_REPOSITORY https://github.com/intel/libva.git
        GIT_TAG 2.23.0
        DOWNLOAD_ONLY YES
)
