# Lumen

**Native macOS game streaming, built for Apple Silicon.**

Lumen is a fork of [Sunshine](https://github.com/LizardByte/Sunshine) that fixes macOS support from the ground up. Stream your Mac's display to any [Moonlight](https://moonlight-stream.org/) client — TV, phone, tablet, another PC — with native system audio, automatic virtual display management, and hardware-accelerated encoding.

Tested on **M4 Mac Mini (16GB RAM)** — **1ms streaming latency** over local network with H.264 VideoToolbox encoding.

---

## Why Lumen?

Upstream Sunshine has significant issues on macOS:

| Problem | Sunshine (upstream) | Lumen |
|---------|-------------------|-------|
| **Build on macOS** | Fails with C++ toolchain errors on modern Xcode/CLT | Builds cleanly with automated dependency management |
| **System audio** | No capture — requires BlackHole virtual audio device | Native ScreenCaptureKit audio — zero-config, no extra software |
| **Virtual displays** | Manual setup with BetterDisplay ($15 app) | Automatic — creates/destroys virtual displays on connect/disconnect |
| **Gamepad support** | None on macOS | Virtual HID gamepad via IOHIDUserDevice (works with Dolphin, Steam, etc.) |
| **H.264 encoding** | All-IDR bug on Apple Silicon (every frame is a keyframe) | Fixed — proper P-frame generation, 3x bandwidth reduction |
| **Encoder performance** | Capture and encode on same thread | Parallel capture/encode pipeline |

---

## Features

- **Zero-config system audio** — ScreenCaptureKit captures all desktop audio natively. No BlackHole, no Soundflower, no virtual audio devices to install or configure.

- **Automatic virtual displays** — When a Moonlight client connects, Lumen creates a virtual display matching the client's requested resolution and refresh rate (e.g., 4K@60Hz). When the last client disconnects, the virtual display is destroyed. No third-party display managers needed.

- **Hardware-accelerated encoding** — VideoToolbox H.264 and HEVC encoding with Apple Silicon hardware acceleration. H.264 at 1080p60 encodes in ~15ms on M4 (fits within the 16.67ms frame budget). HEVC available for higher quality at the cost of slightly higher latency (~18ms on M4).

- **Virtual gamepad** — Creates a system-wide virtual HID gamepad that appears as a real controller to any application. Works with SDL-based games, Dolphin Emulator, Steam, Ryujinx, and more. Requires one-time security configuration (see [Gamepad Setup](#gamepad-setup-optional)).

- **Low latency** — Measured 1ms encode-to-network latency on local network with H.264 VideoToolbox on M4 Mac Mini.

---

## Requirements

- **macOS 14 (Sonoma) or later** — required for CGVirtualDisplay API
- **Apple Silicon Mac** (M1/M2/M3/M4) — ARM64 only
- **Moonlight client** on your target device — [moonlight-stream.org](https://moonlight-stream.org/)

---

## Quick Install

```bash
git clone https://github.com/trollzem/Lumen.git
cd Lumen
./install.sh
```

The install script handles everything:
1. Checks macOS version and architecture
2. Installs Homebrew (if not present)
3. Installs all build dependencies with correct versions:
   - `cmake` — build system
   - `boost` — C++ utility libraries (Asio, Log, Process, Locale)
   - `pkg-config` — library path resolution
   - `openssl@3` — TLS/SSL for HTTPS web UI and RTSP
   - `opus` — audio codec for streaming
   - `llvm` — Clang/LLVM toolchain
   - `doxygen` — documentation generation (build requirement)
   - `graphviz` — documentation graphs (build requirement)
   - `node` — web UI build (Vue 3 + Vite)
   - `icu4c@78` — Unicode support (Boost.Locale dependency)
   - `miniupnpc` — UPnP port mapping for NAT traversal
4. Detects the correct macOS SDK path and C++ header location
5. Configures cmake with all necessary flags (see [macOS Build Fixes](#macos-build-fixes) for why this is needed)
6. Builds from source with all CPU cores
7. Installs the binary, virtual display helper, and assets to `~/.local/share/lumen/`
8. Sets up default configuration in `~/.config/sunshine/`
9. Creates a `lumen` launch command in `~/.local/bin/`

After installation, grant these macOS permissions when prompted:
- **Screen Recording** (System Settings > Privacy & Security > Screen Recording)
- **Accessibility** (System Settings > Privacy & Security > Accessibility)

---

## Usage

### Start Lumen

```bash
lumen
```

Or if `~/.local/bin` isn't in your PATH:

```bash
~/.local/bin/lumen
```

### Pair with Moonlight

1. Open the Lumen web UI at **https://localhost:47990**
2. Set a username and password on first launch
3. Open Moonlight on your client device
4. Moonlight will discover Lumen automatically via mDNS
5. Enter the PIN shown in Moonlight into the Lumen web UI
6. Connect — a virtual display is created automatically at your client's resolution

### Stop Lumen

Press `Ctrl+C` in the terminal, or quit from the system tray icon.

---

## Configuration

Config files live in `~/.config/sunshine/`:

| File | Purpose |
|------|---------|
| `sunshine.conf` | Runtime settings (bitrate, audio source, encoder, etc.) |
| `apps.json` | Applications visible in Moonlight's app list |
| `credentials/` | Pairing credentials (auto-generated) |

### Key Settings (sunshine.conf)

```ini
# Audio source — "system" uses ScreenCaptureKit (recommended)
audio_sink = system

# Maximum streaming bitrate in kbps
max_bitrate = 80000

# Virtual display — "enabled" creates displays on-demand (recommended)
virtual_display = enabled

# UPnP port mapping for remote access
upnp = enabled
```

### Adding Apps (apps.json)

Apps appear in Moonlight's launcher. Example for Dolphin Emulator:

```json
{
  "name": "Dolphin Emulator",
  "detached": [
    "~/.config/sunshine/scripts/launch_dolphin.sh"
  ],
  "prep-cmd": [
    {
      "do": "",
      "undo": "osascript -e 'tell application \"Dolphin\" to quit'"
    }
  ]
}
```

Launch scripts can use the virtual display position file (`/tmp/sunshine_vd_id`) to move app windows to the streaming display automatically. See `scripts/launch_dolphin.sh` for an example.

---

## Gamepad Setup (Optional)

Virtual gamepad support requires a one-time security configuration because it uses Apple's IOHIDUserDevice API with a restricted entitlement. **This is only needed if you want gamepad/controller support.** Keyboard and mouse input work without this step.

### Why This Is Needed

macOS restricts the creation of virtual HID devices to prevent malicious software from injecting fake input. Lumen creates a virtual gamepad that appears as a real USB controller to the system — this requires the `com.apple.developer.hid.virtual.device` entitlement, which Apple only allows with AMFI (Apple Mobile File Integrity) disabled.

**SIP (System Integrity Protection) does NOT need to be disabled** — only AMFI. This is a less invasive change that specifically allows ad-hoc signed binaries to use restricted entitlements.

### Steps

1. **Shut down your Mac** completely (not restart)

2. **Boot into Recovery Mode:**
   - **Apple Silicon:** Hold the **power button** until "Loading startup options" appears
   - Select **Options** > **Continue**

3. **Open Terminal** from the **Utilities** menu in Recovery Mode

4. **Disable AMFI** (allows restricted entitlements on ad-hoc signed binaries):
   ```bash
   nvram boot-args="amfi_get_out_of_my_way=1"
   ```

5. **Restart** your Mac normally

6. **Sign the Lumen binaries** with the HID entitlement:
   ```bash
   codesign --sign - --entitlements ~/.local/share/lumen/hid_entitlements.plist --force ~/.local/share/lumen/sunshine
   codesign --sign - --force ~/.local/share/lumen/vd_helper
   ```

That's it. The gamepad will now appear in any application as a generic USB controller. You can verify it's working by connecting from Moonlight with a controller and checking System Information > USB.

### Important Notes

- **AMFI disable persists across reboots** — you only need to do this once
- **Re-sign after every rebuild** — if you rebuild from source, run the `codesign` commands again
- **The `lumen` launcher auto-signs on every launch** — the manual step above is only needed if you bypass the launcher
- **To re-enable AMFI later:** boot into Recovery Mode and run `nvram -d boot-args`
- **Without AMFI disabled, Lumen still works fully** — you just won't have gamepad support. Keyboard, mouse, virtual displays, audio, and all other features work normally.
- **Security note:** Disabling AMFI reduces one layer of macOS security. Only do this if you understand the implications and need gamepad support.

---

## Building From Source

If you want to build manually instead of using `install.sh`:

### Prerequisites

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install all build dependencies
brew install cmake boost pkg-config openssl@3 opus llvm doxygen graphviz node icu4c@78 miniupnpc
```

### Build

```bash
cd Lumen

# Detect macOS SDK path
SDK_PATH=$(xcrun --show-sdk-path)

mkdir -p build && cd build

cmake -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WERROR=ON \
  -DHOMEBREW_ALLOW_FETCHCONTENT=ON \
  -DOPENSSL_ROOT_DIR=$(brew --prefix openssl@3) \
  -DSUNSHINE_ASSETS_DIR=sunshine/assets \
  -DSUNSHINE_BUILD_HOMEBREW=ON \
  -DSUNSHINE_ENABLE_TRAY=ON \
  -DBOOST_USE_STATIC=OFF \
  -DCMAKE_OSX_SYSROOT="$SDK_PATH" \
  -DCMAKE_CXX_FLAGS="-nostdinc++ -cxx-isystem $SDK_PATH/usr/include/c++/v1 -std=gnu++2b -I$(brew --prefix openssl@3)/include" \
  -DCMAKE_C_FLAGS="-I$(brew --prefix openssl@3)/include" \
  ..

make sunshine -j$(sysctl -n hw.ncpu)
```

### Run

```bash
SUNSHINE_ASSETS_DIR=./assets ./sunshine
```

---

## Networking

| Port | Protocol | Purpose |
|------|----------|---------|
| 47984-47990 | TCP | Control, RTSP, Web UI (HTTPS) |
| 47998-48010 | UDP | Video/audio streaming |
| 47990 | HTTPS | Web configuration UI |

Lumen supports **UPnP** for automatic port mapping. For manual port forwarding, open the ports above on your router.

mDNS/DNS-SD is used for automatic discovery on the local network — Moonlight will find Lumen without any configuration.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No screen capture permission" | Grant Screen Recording in System Settings > Privacy & Security |
| No audio in stream | Ensure `audio_sink = system` in sunshine.conf and Screen Recording permission is granted |
| Black screen / no video | Check that the virtual display was created (look for "Created virtual display" in logs) |
| Gamepad not detected | Ensure AMFI is disabled and binary is signed (see [Gamepad Setup](#gamepad-setup-optional)) |
| High latency (>5ms) | Use H.264 instead of HEVC — H.264 encodes faster on Apple Silicon (~15ms vs ~18ms for 1080p) |
| Build fails with C++ include errors | The install script handles this automatically. If building manually, ensure `-nostdinc++` and `-cxx-isystem` flags point to your SDK's C++ headers (see [macOS Build Fixes](#macos-build-fixes)). |
| `libminiupnpc` not found | Run `brew install miniupnpc` |
| App opens on wrong display | Launch scripts should use `/tmp/sunshine_vd_id` to find the virtual display position. See `scripts/launch_dolphin.sh`. |

### Viewing Logs

Lumen outputs logs to the terminal. To save logs to a file:

```bash
lumen 2>&1 | tee ~/lumen.log
```

---

## Technical Details

### Architecture Overview

```
Moonlight Client connects (e.g. 1920x1080@60Hz)
  │
  ├─ Virtual Display created via CGVirtualDisplay (private API, macOS 14+)
  │    └─ vd_helper subprocess holds display reference
  │    └─ Display appears in System Settings > Displays
  │
  ├─ Video Pipeline
  │    └─ AVFoundation (AVCaptureScreenInput) captures virtual display at fixed 60fps
  │    └─ CVPixelBuffer (NV12/BGRA) → VideoToolbox H.264/HEVC hardware encoder
  │    └─ Parallel encode pipeline (capture thread decoupled from encode thread)
  │    └─ Network stream → Moonlight client
  │
  ├─ Audio Pipeline
  │    └─ ScreenCaptureKit creates minimal SCStream (64x64@1fps video + audio)
  │    └─ System audio captured as Float32 PCM (48kHz stereo)
  │    └─ Non-interleaved → interleaved conversion
  │    └─ TPCircularBuffer → Opus encoding → Network stream
  │
  ├─ Input Pipeline
  │    └─ Keyboard: Moonlight keycodes → macOS virtual keycodes → CGEventPost
  │    └─ Mouse: Absolute/relative → CGWarpMouseCursorPosition (virtual display coords)
  │    └─ Gamepad: IOHIDUserDevice virtual HID reports → SDL/Game Controller framework
  │
  └─ Client disconnects
       └─ Virtual display destroyed (vd_helper SIGTERM'd)
```

### What We Changed (Complete List)

#### New Files (7)

| File | Purpose |
|------|---------|
| `src/platform/macos/sc_audio.h` / `.m` | ScreenCaptureKit system audio capture — creates minimal SCStream for audio, handles non-interleaved→interleaved conversion, uses TPCircularBuffer |
| `src/platform/macos/virtual_display.h` / `.m` | Virtual display management — spawns vd_helper subprocess, manages lifecycle (create/destroy/get_id) |
| `src/platform/macos/vd_helper.m` | Standalone subprocess for CGVirtualDisplay — uses private API to create display, SkyLight functions to activate and position it |
| `src/platform/macos/hid_gamepad.h` / `.m` | Virtual HID gamepad via IOHIDUserDevice — creates system-wide controller with generic VID/PID for SDL compatibility |

#### Modified Files (13)

| File | Change |
|------|--------|
| `src/platform/macos/av_audio.m` | Updated AVFoundation device discovery API for macOS 14+ (`AVCaptureDeviceTypeMicrophone`, `AVCaptureDeviceTypeExternal`) with backward compatibility |
| `src/platform/macos/microphone.mm` | Unified audio source selection: ScreenCaptureKit → BlackHole → AVFoundation fallback chain |
| `src/platform/macos/display.mm` | Virtual display detection and preferential capture; synthetic dummy frame for encoder probing (avoids capture timeout) |
| `src/platform/macos/input.mm` | Dynamic virtual display targeting — mouse/keyboard input redirected to virtual display coordinates |
| `src/platform/macos/sc_capture.h` / `.m` | Added frame caching and re-delivery for ScreenCaptureKit idle-frame handling |
| `src/video.cpp` | Removed `max_ref_frames=1` for H.264 VT (fixes all-IDR bug); enabled `PARALLEL_ENCODING` flag |
| `src/config.h` / `.cpp` | Added `virtual_display` config option (`enabled`/`disabled`) |
| `src/display_device.cpp` / `.h` | Virtual display create/destroy hooks in session lifecycle |
| `src/platform/common.h` | Declared `virtual_display_create/destroy/get_id` platform functions |
| `cmake/compile_definitions/macos.cmake` | Added ScreenCaptureKit, IOKit linking; sc_audio, hid_gamepad, virtual_display build targets; vd_helper compilation with ARC |
| `cmake/dependencies/common.cmake` | Fixed Opus include path (parent directory for `opus/opus_multistream.h`) |

### Virtual Display System (CGVirtualDisplay)

Lumen uses Apple's private `CGVirtualDisplay` API (available on macOS 14+) to create virtual displays on demand. This eliminates the need for third-party tools like BetterDisplay.

**Why a subprocess?** CGVirtualDisplay doesn't work when created directly in the Sunshine process. The TCC (Transparency, Consent, and Control) framework and WindowServer registration require a clean process context. Lumen spawns `vd_helper` as a subprocess that:

1. Creates a `CGVirtualDisplayDescriptor` with the requested resolution
2. Creates display modes — both native and retina variants (e.g., 3840x2160 native + 1920x1080@2x)
3. Creates a `CGVirtualDisplay` object and applies the modes
4. Activates the display via `SLSConfigureDisplayEnabled` (SkyLight private function)
5. Forces extend mode — macOS may auto-mirror new displays, hiding them from `CGGetActiveDisplayList`
6. Switches to native 1x mode via `CGDisplaySetDisplayMode` to avoid retina 2x scaling at 4K
7. Writes the display ID to stdout (read by Sunshine) and `/tmp/sunshine_vd_id` (read by app launch scripts)
8. Stays alive holding the display reference until SIGTERM

**Physical dimensions matter.** CGVirtualDisplay rejects displays where the pixel density exceeds a threshold relative to the declared physical size. Lumen uses a fixed 27-inch equivalent (597x336mm) which supports up to 4K resolution without rejection.

### ScreenCaptureKit Audio Capture

macOS has no public API for directly capturing system audio output. Previously, this required routing audio through a virtual loopback device like BlackHole. Lumen uses ScreenCaptureKit's audio capture capability instead:

1. Creates a minimal SCStream targeting a 64x64 pixel region at 1fps (minimal CPU/GPU usage)
2. Enables `capturesAudio = YES` on the stream configuration
3. Sets `excludesCurrentProcessAudio = YES` to avoid feedback loops
4. Receives non-interleaved Float32 stereo PCM at 48kHz
5. Interleaves the L/R channels and writes to a TPCircularBuffer
6. The Opus encoder reads from the ring buffer for network delivery

This approach requires **Screen Recording permission** (which is needed for video capture anyway) and works on macOS 12.3+.

### VideoToolbox H.264 Fix

On Apple Silicon (confirmed on M4), setting `ReferenceBufferCount=1` (Sunshine's `max_ref_frames=1`) on VideoToolbox's H.264 encoder causes catastrophic behavior: **every frame becomes an IDR keyframe**. P-frames are never produced.

The impact:
- Each frame is ~300KB instead of ~30KB (10x larger)
- Actual bitrate is 2-3x the configured maximum
- Moonlight reports 30%+ "frames dropped by network"
- Stuttering and frame drops despite low latency

Lumen removes this option for H.264. HEVC is unaffected and retains `max_ref_frames=1`. This single fix dramatically improves streaming quality on Apple Silicon.

### Virtual HID Gamepad

Lumen creates a virtual USB gamepad using `IOHIDUserDeviceCreateWithProperties()`. The device appears as a generic USB gamepad with:

- **VID/PID:** `0x1209`/`0x5853` (generic, not matching any known controller)
- **Usage Page:** Generic Desktop (0x01), Usage: Joystick (0x04)
- **Buttons:** 16 digital buttons mapped to standard gamepad layout
- **Axes:** 4 analog axes (2 sticks) + 2 triggers

**Why generic VID/PID?** SDL (used by most games/emulators) has a list of known Xbox/PlayStation VID/PIDs. When SDL sees a known VID/PID, it assumes the macOS Game Controller framework handles it and skips its own IOKit HID backend. Since virtual HID devices get `IOHIDEventDummyService` (not recognized by Game Controller framework), this creates a dead zone — SDL won't see the controller through either path. Using a generic VID/PID forces SDL to use its IOKit HID backend directly. Games and emulators like Dolphin can then manually map the buttons.

### macOS Build Fixes

Building upstream Sunshine on modern macOS (15+) with Command Line Tools fails due to several issues:

1. **Empty C++ include directory:** The Command Line Tools install at `/Library/Developer/CommandLineTools/usr/include/c++/v1/` contains only a `__cxx_version` marker file. The actual C++ standard library headers are in the SDK at `$SDK_PATH/usr/include/c++/v1/`.

2. **Fix:** CMake flags `-nostdinc++ -cxx-isystem $SDK_PATH/usr/include/c++/v1` tell the compiler to ignore the empty default path and use the SDK headers instead.

3. **OpenSSL headers:** Homebrew installs OpenSSL to a non-standard location (`/opt/homebrew/opt/openssl@3/`). The upstream CMake configuration doesn't add this to the include path on macOS, requiring explicit `-I` flags in both `CMAKE_CXX_FLAGS` and `CMAKE_C_FLAGS`.

4. **Opus include path:** The code uses `#include <opus/opus_multistream.h>` but the Opus pkg-config reports the directory containing the headers directly (without the `opus/` subdirectory). Fixed in `common.cmake` by using the parent directory.

5. **`CMAKE_OSX_SYSROOT`:** Must be set explicitly to the current SDK path to avoid picking up a stale or mismatched SDK.

The install script detects all of these automatically and configures the build correctly.

### Parallel Encoding Pipeline

Lumen enables `PARALLEL_ENCODING` for the VideoToolbox encoder, which decouples the capture and encode threads. Without this flag, frame capture blocks until the previous frame finishes encoding. With it enabled:

- Capture thread delivers frames to a queue continuously
- Encode thread processes frames from the queue independently
- This eliminates frame drops caused by encoder stalls and improves overall throughput

---

## Acknowledgments

- [LizardByte/Sunshine](https://github.com/LizardByte/Sunshine) — the upstream project this is forked from
- [Moonlight](https://moonlight-stream.org/) — the open-source game streaming client
- [TPCircularBuffer](https://github.com/michaeltyson/TPCircularBuffer) — lock-free ring buffer used for audio delivery

---

## License

Lumen is licensed under the same terms as Sunshine (GPLv3). See [LICENSE](LICENSE) for details.
