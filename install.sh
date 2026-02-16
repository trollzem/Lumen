#!/bin/bash
set -e

# ─── Lumen Installer ───────────────────────────────────────────────────────────
# One-click build and install for macOS Apple Silicon.
# Installs all dependencies, builds from source, and sets up configuration.
# ────────────────────────────────────────────────────────────────────────────────

LUMEN_DIR="$(cd "$(dirname "$0")" && pwd)"
INSTALL_DIR="$HOME/.local/share/lumen"
BIN_DIR="$HOME/.local/bin"
CONFIG_DIR="$HOME/.config/sunshine"
BUILD_DIR="$LUMEN_DIR/build"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()  { echo -e "${BLUE}[INFO]${NC} $1"; }
ok()    { echo -e "${GREEN}[OK]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

echo ""
echo "  ╦   ╦ ╦╔╦╗╔═╗╔╗╔"
echo "  ║   ║ ║║║║║╣ ║║║"
echo "  ╩═╝╚═╝╩ ╩╚═╝╝╚╝"
echo "  Native macOS Game Streaming"
echo ""

# ─── Pre-flight checks ─────────────────────────────────────────────────────────

info "Running pre-flight checks..."

# Check macOS version (need 14+ for CGVirtualDisplay)
MACOS_MAJOR=$(sw_vers -productVersion | cut -d. -f1)
if [ "$MACOS_MAJOR" -lt 14 ]; then
    error "Lumen requires macOS 14 (Sonoma) or later. You have macOS $(sw_vers -productVersion)."
fi
ok "macOS $(sw_vers -productVersion)"

# Check Apple Silicon
ARCH=$(uname -m)
if [ "$ARCH" != "arm64" ]; then
    error "Lumen only supports Apple Silicon (arm64). Detected: $ARCH"
fi
ok "Apple Silicon ($ARCH)"

# Check for Xcode Command Line Tools
if ! xcode-select -p &>/dev/null; then
    info "Installing Xcode Command Line Tools..."
    xcode-select --install
    echo "Please complete the Xcode CLT installation, then run this script again."
    exit 1
fi
ok "Xcode Command Line Tools"

# Check for Homebrew
if ! command -v brew &> /dev/null; then
    info "Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    # Add Homebrew to PATH for this session
    eval "$(/opt/homebrew/bin/brew shellenv)"
fi
ok "Homebrew $(brew --version | head -1 | awk '{print $2}')"

# ─── Install dependencies ──────────────────────────────────────────────────────

info "Installing build dependencies via Homebrew..."
info "(This may take a few minutes on first run)"

DEPS=(
    cmake           # Build system generator
    boost           # C++ utility libraries (Asio, Log, Process, Locale, etc.)
    pkg-config      # Library path resolution for build system
    openssl@3       # TLS/SSL for HTTPS web UI and RTSP streaming
    opus            # Audio codec for low-latency streaming
    llvm            # Clang/LLVM toolchain (required by Sunshine build)
    doxygen         # Documentation generation (build requirement)
    graphviz        # Documentation graphs (build requirement)
    node            # Web UI build toolchain (Vue 3 + Vite)
    icu4c@78        # Unicode support (Boost.Locale dependency)
    miniupnpc       # UPnP port mapping for automatic NAT traversal
)

for dep in "${DEPS[@]}"; do
    if brew list "$dep" &>/dev/null; then
        ok "$dep (already installed)"
    else
        info "Installing $dep..."
        brew install "$dep" 2>&1 | tail -1
        ok "$dep"
    fi
done

# ─── Detect SDK path ───────────────────────────────────────────────────────────

info "Detecting macOS SDK..."

SDK_PATH=$(xcrun --show-sdk-path 2>/dev/null)
if [ -z "$SDK_PATH" ] || [ ! -d "$SDK_PATH" ]; then
    # Try Command Line Tools SDK directly
    SDK_PATH="/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk"
    if [ ! -d "$SDK_PATH" ]; then
        error "Could not find macOS SDK. Install Xcode Command Line Tools: xcode-select --install"
    fi
fi

# Verify C++ headers exist in the SDK (this is the key build fix for macOS 15+)
CXX_HEADERS="$SDK_PATH/usr/include/c++/v1"
if [ ! -f "$CXX_HEADERS/__config" ]; then
    # Try a versioned SDK
    LATEST_SDK=$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX*.sdk 2>/dev/null | sort -V | tail -1)
    if [ -n "$LATEST_SDK" ] && [ -f "$LATEST_SDK/usr/include/c++/v1/__config" ]; then
        SDK_PATH="$LATEST_SDK"
        CXX_HEADERS="$SDK_PATH/usr/include/c++/v1"
    else
        error "C++ headers not found. Install or update Xcode Command Line Tools: xcode-select --install"
    fi
fi

ok "SDK: $SDK_PATH"
ok "C++ headers: $CXX_HEADERS"

# ─── Build ──────────────────────────────────────────────────────────────────────

info "Building Lumen from source..."

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

OPENSSL_PREFIX=$(brew --prefix openssl@3)
NUM_CORES=$(sysctl -n hw.ncpu)

info "Running cmake configuration..."
cmake -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WERROR=ON \
  -DOPENSSL_ROOT_DIR="$OPENSSL_PREFIX" \
  -DSUNSHINE_ASSETS_DIR="$INSTALL_DIR/assets" \
  -DSUNSHINE_BUILD_HOMEBREW=ON \
  -DSUNSHINE_ENABLE_TRAY=ON \
  -DBOOST_USE_STATIC=OFF \
  -DCMAKE_OSX_SYSROOT="$SDK_PATH" \
  -DCMAKE_CXX_FLAGS="-nostdinc++ -cxx-isystem $CXX_HEADERS -std=gnu++2b -I$OPENSSL_PREFIX/include" \
  -DCMAKE_C_FLAGS="-I$OPENSSL_PREFIX/include" \
  ..

info "Compiling with $NUM_CORES cores (this may take several minutes)..."
make sunshine -j"$NUM_CORES"

ok "Build complete"

# Build get_display_origin helper (used by app launch scripts to find virtual display position)
info "Building display helper tools..."
clang -framework CoreGraphics -o "$BUILD_DIR/get_display_origin" \
  "$LUMEN_DIR/src/platform/macos/get_display_origin.m" 2>/dev/null && \
  ok "get_display_origin" || warn "get_display_origin build failed (non-critical)"

# ─── Install ────────────────────────────────────────────────────────────────────

info "Installing to $INSTALL_DIR..."

mkdir -p "$INSTALL_DIR"
mkdir -p "$BIN_DIR"
mkdir -p "$CONFIG_DIR/scripts"

# Copy binary (follow symlinks)
cp -fL "$BUILD_DIR/sunshine" "$INSTALL_DIR/sunshine" 2>/dev/null || \
  cp -f "$BUILD_DIR/sunshine-"* "$INSTALL_DIR/sunshine" 2>/dev/null

# Copy helper binaries
for helper in vd_helper get_display_origin; do
    if [ -f "$BUILD_DIR/$helper" ]; then
        cp -f "$BUILD_DIR/$helper" "$INSTALL_DIR/$helper"
        ok "Installed $helper"
    fi
done

# Copy assets
ASSETS_SRC=""
if [ -d "$BUILD_DIR/sunshine/assets" ]; then
    ASSETS_SRC="$BUILD_DIR/sunshine/assets"
elif [ -d "$BUILD_DIR/assets" ]; then
    ASSETS_SRC="$BUILD_DIR/assets"
fi

if [ -n "$ASSETS_SRC" ]; then
    rm -rf "$INSTALL_DIR/assets"
    cp -Rf "$ASSETS_SRC" "$INSTALL_DIR/assets"
    ok "Installed assets"
fi

# Create HID entitlements plist (for gamepad support)
cat > "$INSTALL_DIR/hid_entitlements.plist" << 'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>com.apple.developer.hid.virtual.device</key>
    <true/>
</dict>
</plist>
PLIST

# Copy example launch scripts
if [ -d "$LUMEN_DIR/scripts" ]; then
    cp -f "$LUMEN_DIR/scripts/"*.sh "$CONFIG_DIR/scripts/" 2>/dev/null
    chmod +x "$CONFIG_DIR/scripts/"*.sh 2>/dev/null
    ok "Installed example launch scripts"
fi

# Clean slate: always write fresh config and apps.json.
# Old configs from previous Sunshine installs can have invalid options
# (e.g. min_bitrate, wrong output_name) that cause confusing warnings.
cat > "$CONFIG_DIR/sunshine.conf" << 'CONF'
# Lumen Configuration
# See https://github.com/trollzem/Lumen for documentation

# Audio: "system" uses ScreenCaptureKit for native system audio capture
# No extra software needed — captures all desktop audio directly.
audio_sink = system

# Maximum streaming bitrate (kbps)
# 80000 (80 Mbps) is good for 4K. Use 40000 for 1080p.
max_bitrate = 80000

# Virtual display: "enabled" creates a display matching client resolution on connect.
# The display is destroyed when the last client disconnects.
# Set to "disabled" to use a physical display or BetterDisplay instead.
virtual_display = enabled

# UPnP: automatic port mapping for remote access through NAT
upnp = enabled

# Encoder: videotoolbox uses Apple Silicon hardware acceleration.
# Falls back to software (libx264) if VT is unavailable.
# encoder = videotoolbox
CONF
ok "Config written to $CONFIG_DIR/sunshine.conf"

cat > "$CONFIG_DIR/apps.json" << 'APPS'
{
  "env": {
    "PATH": "$(PATH):$(HOME)/.local/bin"
  },
  "apps": [
    {
      "name": "Desktop"
    }
  ]
}
APPS
ok "Created apps.json"

# Always set up Web UI credentials during install.
# Previous Sunshine installs may have left credentials the user doesn't remember.
echo ""
info "Setting up Web UI credentials..."
echo "  Choose a username and password for the Lumen web interface."
echo "  (You'll use these to log in at https://localhost:47990)"
echo ""
printf "  Username: "
read -r LUMEN_USER
printf "  Password: "
read -rs LUMEN_PASS
echo ""
if [ -n "$LUMEN_USER" ] && [ -n "$LUMEN_PASS" ]; then
    if "$INSTALL_DIR/sunshine" --creds "$LUMEN_USER" "$LUMEN_PASS" >/dev/null 2>&1; then
        ok "Web UI credentials saved"
    else
        warn "Failed to save credentials. Set them manually: lumen --creds username password"
    fi
else
    warn "Skipped — you can set credentials later at https://localhost:47990"
fi

# Create launcher script that auto-signs for gamepad support on every launch
cat > "$BIN_DIR/lumen" << 'LAUNCHER'
#!/bin/bash
INSTALL_DIR="$HOME/.local/share/lumen"
ENTITLEMENTS="$INSTALL_DIR/hid_entitlements.plist"
BINARY="$INSTALL_DIR/sunshine"

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

# Pass through --creds and other CLI flags directly to sunshine
if [ "${1:-}" = "--creds" ]; then
    "$BINARY" "$@"
    exit $?
fi

# Sign the binary for gamepad support (only if AMFI is disabled).
# With AMFI enabled, restricted entitlements cause macOS to kill the process.
# Check AMFI status by looking at boot-args.
AMFI_OFF=false
if nvram boot-args 2>/dev/null | grep -q "amfi_get_out_of_my_way=1"; then
    AMFI_OFF=true
fi

if [ "$AMFI_OFF" = true ] && [ -f "$ENTITLEMENTS" ] && [ -f "$BINARY" ]; then
    codesign --sign - --entitlements "$ENTITLEMENTS" --force "$BINARY" 2>/dev/null
fi

# First-run permission guide.
# Use a flag file since TCC.db queries are unreliable on newer macOS versions.
PERM_FLAG="$INSTALL_DIR/.permissions_configured"

if [ ! -f "$PERM_FLAG" ]; then
    echo ""
    echo -e "${YELLOW}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║  macOS permissions required (first run only)                ║${NC}"
    echo -e "${YELLOW}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}║                                                              ║${NC}"
    echo -e "${YELLOW}║  Lumen needs Screen Recording and Accessibility permissions. ║${NC}"
    echo -e "${YELLOW}║  macOS will prompt you, or grant them manually:              ║${NC}"
    echo -e "${YELLOW}║                                                              ║${NC}"
    echo -e "${YELLOW}║  1. Screen Recording (required for video + audio)            ║${NC}"
    echo -e "${YELLOW}║  2. Accessibility (required for keyboard/mouse input)        ║${NC}"
    echo -e "${YELLOW}║                                                              ║${NC}"
    echo -e "${YELLOW}║  Opening System Settings now...                              ║${NC}"
    echo -e "${YELLOW}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    # Open directly to Screen Recording privacy pane
    open "x-apple.systempreferences:com.apple.preference.security?Privacy_ScreenCapture" 2>/dev/null
    echo -e "  Grant ${GREEN}Screen Recording${NC} to '${GREEN}Terminal${NC}' (the app running Lumen)."
    echo -e "  Press Enter when done..."
    read -r
    # Open Accessibility pane
    open "x-apple.systempreferences:com.apple.preference.security?Privacy_Accessibility" 2>/dev/null
    echo -e "  Grant ${GREEN}Accessibility${NC} to '${GREEN}Terminal${NC}'."
    echo -e "  Press Enter to start Lumen..."
    read -r
    # Mark permissions as configured so we don't show this again
    touch "$PERM_FLAG"
fi

echo -e "${GREEN}Starting Lumen...${NC}"
echo "  Web UI: https://localhost:47990"
echo ""

exec "$BINARY" "$@"
LAUNCHER
chmod +x "$BIN_DIR/lumen"

ok "Installed launcher to $BIN_DIR/lumen"

# ─── Post-install ───────────────────────────────────────────────────────────────

# Check if AMFI is disabled (for gamepad support info)
AMFI_STATUS="unknown"
BOOT_ARGS=$(nvram boot-args 2>/dev/null || echo "")
if echo "$BOOT_ARGS" | grep -q "amfi_get_out_of_my_way=1"; then
    AMFI_STATUS="disabled"
else
    AMFI_STATUS="enabled"
fi

echo ""
echo "  ────────────────────────────────────────────────────"
echo -e "  ${GREEN}Lumen installed successfully!${NC}"
echo "  ────────────────────────────────────────────────────"
echo ""
echo "  Start Lumen:"
echo -e "    ${GREEN}lumen${NC}"
echo ""
echo "  Or if ~/.local/bin isn't in your PATH:"
echo "    export PATH=\"\$HOME/.local/bin:\$PATH\""
echo "    lumen"
echo ""
echo "  Web UI: https://localhost:47990"
echo ""
echo -e "  ${GREEN}macOS permissions:${NC} The launcher will walk you through granting"
echo "    Screen Recording and Accessibility on first run."
echo ""

if [ "$AMFI_STATUS" = "disabled" ]; then
    echo -e "  ${GREEN}Gamepad support: READY${NC}"
    echo "    AMFI is disabled. The launcher auto-signs with HID entitlements."
    echo "    Gamepad will work automatically when you connect from Moonlight."
else
    echo -e "  ${YELLOW}Gamepad support: NOT CONFIGURED${NC}"
    echo "    To enable gamepad/controller support, you need to disable AMFI (one-time):"
    echo ""
    echo "    1. Shut down your Mac completely"
    echo "    2. Hold the power button until 'Loading startup options' appears"
    echo "    3. Select Options > Continue"
    echo "    4. Open Terminal from the Utilities menu"
    echo "    5. Run: nvram boot-args=\"amfi_get_out_of_my_way=1\""
    echo "    6. Restart normally"
    echo ""
    echo "    After this one-time setup, gamepad support works automatically."
    echo "    See README for full details."
fi

echo ""
echo "  Config:  $CONFIG_DIR/sunshine.conf"
echo "  Logs:    lumen 2>&1 | tee ~/lumen.log"
echo ""
