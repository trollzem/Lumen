# build-deps

[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/lizardbyte/build-deps/ci.yml.svg?branch=master&label=build&logo=github&style=for-the-badge)](https://github.com/LizardByte/build-deps/actions/workflows/ci.yml?query=branch%3Amaster)

This is a common set of pre-compiled dependencies for [LizardByte/Sunshine](https://github.com/LizardByte/Sunshine).

- [Boost](https://www.boost.org)
- [FFmpeg](https://ffmpeg.org)

## Usage

1. Add this repository as a submodule to your project.

   ```bash
   git submodule add https://github.com/LizardByte/build-deps.git third-party/build-deps
   cd third-party/build-deps
   git submodule update --init --recursive
   checkout dist
   ```

## License

This repo is licensed under the MIT License, but this does not cover submodules or patches.
Please see the individual projects for their respective licenses.


## Build

### Checkout

```bash
git clone --recurse-submodules https://github.com/LizardByte/build-deps.git
```

You can reduce the size of the repo by setting the depth to 1:

```bash
git clone --recurse-submodules --depth 1 https://github.com/LizardByte/build-deps.git
```

ℹ️ If you have already cloned the repository without submodules, you can initialize them with the following command:

```bash
cd build-deps
git submodule update --init --recursive
```

### Line Endings

ℹ️ On Windows, you must copy the `.gitattributes` file to `.git/modules/third-party/FFmpeg/x264/info/attributes`,
see https://stackoverflow.com/a/23671157/11214013 for more info.

Then run the following commands:
```bash
cd third-party/FFmpeg/x264
git checkout HEAD -- .
```

### Dependencies

#### FreeBSD

```bash
pkg install -y \
  devel/autoconf \
  devel/automake \
  devel/cmake \
  devel/git \
  devel/gmake \
  devel/nasm \
  devel/ninja \
  devel/pkgconf \
  multimedia/libass \
  multimedia/libv4l \
  multimedia/libva \
  multimedia/v4l_compat \
  print/freetype2 \
  security/gnutls \
  shells/bash \
  x11/libxcb \
  x11/libX11 \
  x11/libXfixes
```

#### Linux

```bash
sudo apt install -y \
autoconf \
    automake \
    build-essential \
    cmake \
    git-core \
    libass-dev \
    libfreetype6-dev \
    libgnutls28-dev \
    libmp3lame-dev \
    libnuma-dev \
    libopus-dev \
    libsdl2-dev \
    libtool \
    libvorbis-dev \
    libxcb1-dev \
    libxcb-shm0-dev \
    libxcb-xfixes0-dev \
    make \
    meson \
    nasm \
    ninja-build \
    pkg-config \
    texinfo \
    wget \
    zlib1g-dev
```

#### macOS

```bash
brew install \
    automake \
    fdk-aac \
    git \
    lame \
    libass \
    libtool \
    libvorbis \
    libvpx \
    nasm \
    ninja \
    opus \
    pkg-config \
    sdl \
    shtool \
    texi2html \
    theora \
    wget \
    xvid
```

#### Windows

ℹ️ Cross-compilation is not supported on Windows. You must build on the target architecture.

First, install [MSYS2](https://www.msys2.org/).

##### x86_64 / amd64

Open the UCRT64 shell and run the following commands:

```bash
pacman -Syu
pacman -S \
    diffutils \
    git \
    make \
    patch \
    pkg-config \
    mingw-w64-ucrt-x86_64-binutils \
    mingw-w64-ucrt-x86_64-cmake \
    mingw-w64-ucrt-x86_64-gcc \
    mingw-w64-ucrt-x86_64-make \
    mingw-w64-ucrt-x86_64-nasm \
    mingw-w64-ucrt-x86_64-ninja \
    mingw-w64-ucrt-x86_64-onevpl
```

##### aarch64 / arm64

Open the CLANGARM64 shell and run the following commands:

```bash
pacman -Syu
pacman -S \
    diffutils \
    git \
    make \
    patch \
    pkg-config \
    mingw-w64-clang-aarch64-binutils \
    mingw-w64-clang-aarch64-cmake \
    mingw-w64-clang-aarch64-gcc \
    mingw-w64-clang-aarch64-make \
    mingw-w64-clang-aarch64-nasm \
    mingw-w64-clang-aarch64-ninja \
    mingw-w64-clang-aarch64-onevpl
```

### Configure

Use the `Unix Makefiles` generator for Linux and macOS, and the `MSYS Makefiles` generator for Windows.

#### Standard

```bash
mkdir -p ./build/dist
cmake \
    -B ./build \
    -S . \
    -G "<generator>" \
    -DCMAKE_INSTALL_PREFIX=./build/dist
```

#### Cross Compile

```bash
mkdir -p ./build/dist
cmake \
    -B ./build \
    -S . \
    -G "<generator>" \
    -DCMAKE_INSTALL_PREFIX=./build/dist \
    -DCMAKE_TOOLCHAIN_FILE=./cmake/toolchains/<target>.cmake
```

#### Windows

ℹ️ On Windows, the environment is sometimes not properly passed to the `make` subprocesses. To account for this, there
are three options. If the default does not work, you can try passing in the following flags:

```bash
-DMSYS2_OPTION=3
```

Valid options are, 1, 2, and 3. The default is 1.

### Build

ℹ️ On FreeBSD, use `gmake` instead of `make`.

```bash
make -C build
```

### Install

⚠️ It is critical that the `-DCMAKE_INSTALL_PREFIX` is set to the path where you want to install the dependencies.

```bash
make -C build install
```
