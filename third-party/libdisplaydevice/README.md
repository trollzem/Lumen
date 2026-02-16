# Overview

[![GitHub Workflow Status (CI)](https://img.shields.io/github/actions/workflow/status/lizardbyte/libdisplaydevice/ci.yml.svg?branch=master&label=CI%20build&logo=github&style=for-the-badge)](https://github.com/LizardByte/libdisplaydevice/actions/workflows/ci.yml?query=branch%3Amaster)
[![Codecov](https://img.shields.io/codecov/c/gh/LizardByte/libdisplaydevice?token=goyvmDl6J5&style=for-the-badge&logo=codecov&label=codecov)](https://codecov.io/gh/LizardByte/libdisplaydevice)
[![GitHub stars](https://img.shields.io/github/stars/lizardbyte/libdisplaydevice.svg?logo=github&style=for-the-badge)](https://github.com/LizardByte/libdisplaydevice)

## About

LizardByte has the full documentation hosted on [Read the Docs](https://libdisplaydevice.readthedocs.io/).

libdisplaydevice is a WIP library that provides a common interface for interacting with display devices.
It is intended to be used by applications that need to interact with displays, such as screen capture software,
remote desktop software, and video players.

Initial support is planned for Windows, but could be expanded to other platforms in the future.

## Build

### Clone

Ensure [git](https://git-scm.com/) is installed and run the following:

```bash
git clone https://github.com/lizardbyte/libdisplaydevice.git --recurse-submodules
cd libdisplaydevice
mkdir -p build
```

### Windows

#### Requirements

First you need to install [MSYS2](https://www.msys2.org), then startup "MSYS2 UCRT64" and execute the following
commands.

Update all packages:
```bash
pacman -Syu
```

Install dependencies:
```bash
pacman -S \
  doxygen \
  mingw-w64-ucrt-x86_64-binutils \
  mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-graphviz \
  mingw-w64-ucrt-x86_64-ninja \
  mingw-w64-ucrt-x86_64-toolchain \
  mingw-w64-ucrt-x86_64-boost \
  mingw-w64-ucrt-x86_64-nlohmann-json
```

### Build

```bash
cmake -G Ninja -B build -S .
ninja -C build
```

### Test

```bash
./build/tests/test_libdisplaydevice
```

## Support

Our support methods are listed in our [LizardByte Docs](https://lizardbyte.readthedocs.io/latest/about/support.html).

<details style="display: none;">
  <summary></summary>
  [TOC]
</details>
