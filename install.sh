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

DEPS=(
    cmake           # Build system
    boost           # C++ utility libraries (Asio, Log, Process, etc.)
    pkg-config      # Library path resolution
    openssl@3       # TLS/SSL for HTTPS web UI and RTSP
    opus            # Audio codec for streaming
    llvm            # Clang/LLVM toolchain
    doxygen         # Documentation generation (build requirement)
    graphviz        # Documentation graphs (build requirement)
    node            # Web UI build (Vue 3 + Vite)
    icu4c@78        # Unicode support (Boost.Locale dependency)
    miniupnpc       # UPnP port mapping for NAT traversal
)

for dep in "${DEPS[@]}"; do
    if brew list "$dep" &>/dev/null; then
        ok "$dep (already installed)"
    else
        info "Installing $dep..."
        brew install "$dep"
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

# Verify C++ headers exist in the SDK
CXX_HEADERS="$SDK_PATH/usr/include/c++/v1"
if [ ! -f "$CXX_HEADERS/__config" ]; then
    error "C++ headers not found at $CXX_HEADERS. Install or update Xcode Command Line Tools."
fi

ok "SDK: $SDK_PATH"

# ─── Build ──────────────────────────────────────────────────────────────────────

info "Building Lumen from source..."

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

OPENSSL_PREFIX=$(brew --prefix openssl@3)
NUM_CORES=$(sysctl -n hw.ncpu)

cmake -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WERROR=ON \
  -DHOMEBREW_ALLOW_FETCHCONTENT=ON \
  -DOPENSSL_ROOT_DIR="$OPENSSL_PREFIX" \
  -DSUNSHINE_ASSETS_DIR=sunshine/assets \
  -DSUNSHINE_BUILD_HOMEBREW=ON \
  -DSUNSHINE_ENABLE_TRAY=ON \
  -DBOOST_USE_STATIC=OFF \
  -DCMAKE_OSX_SYSROOT="$SDK_PATH" \
  -DCMAKE_CXX_FLAGS="-nostdinc++ -cxx-isystem $CXX_HEADERS -std=gnu++2b -I$OPENSSL_PREFIX/include" \
  -DCMAKE_C_FLAGS="-I$OPENSSL_PREFIX/include" \
  ..

info "Compiling with $NUM_CORES cores..."
make sunshine -j"$NUM_CORES"

ok "Build complete"

# ─── Install ────────────────────────────────────────────────────────────────────

info "Installing to $INSTALL_DIR..."

mkdir -p "$INSTALL_DIR"
mkdir -p "$BIN_DIR"
mkdir -p "$CONFIG_DIR/scripts"

# Copy binary and helpers
cp -f "$BUILD_DIR/sunshine" "$INSTALL_DIR/sunshine"
if [ -f "$BUILD_DIR/vd_helper" ]; then
    cp -f "$BUILD_DIR/vd_helper" "$INSTALL_DIR/vd_helper"
fi
if [ -f "$BUILD_DIR/get_display_origin" ]; then
    cp -f "$BUILD_DIR/get_display_origin" "$INSTALL_DIR/get_display_origin"
fi

# Copy assets
if [ -d "$BUILD_DIR/assets" ]; then
    cp -Rf "$BUILD_DIR/assets" "$INSTALL_DIR/assets"
elif [ -d "$BUILD_DIR/sunshine/assets" ]; then
    mkdir -p "$INSTALL_DIR/assets"
    cp -Rf "$BUILD_DIR/sunshine/assets/"* "$INSTALL_DIR/assets/"
fi

# Copy HID entitlements plist
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

# Create default config if it doesn't exist
if [ ! -f "$CONFIG_DIR/sunshine.conf" ]; then
    cat > "$CONFIG_DIR/sunshine.conf" << 'CONF'
# Lumen Configuration
# See https://github.com/trollzem/Lumen for documentation

# Audio: "system" uses ScreenCaptureKit for native system audio capture
audio_sink = system

# Maximum streaming bitrate (kbps)
max_bitrate = 80000

# Virtual display: creates a display matching client resolution on connect
virtual_display = enabled

# UPnP: automatic port mapping
upnp = enabled
CONF
    ok "Created default config at $CONFIG_DIR/sunshine.conf"
fi

# Create default apps.json if it doesn't exist
if [ ! -f "$CONFIG_DIR/apps.json" ]; then
    cat > "$CONFIG_DIR/apps.json" << 'APPS'
{
  "env": {
    "PATH": "$(PATH):$(HOME)/.local/bin"
  },
  "apps": [
    {
      "name": "Desktop",
      "image-path": "desktop.png"
    }
  ]
}
APPS
    ok "Created default apps.json"
fi

# Create launcher script
cat > "$BIN_DIR/lumen" << LAUNCHER
#!/bin/bash
SUNSHINE_ASSETS_DIR="$INSTALL_DIR/assets" exec "$INSTALL_DIR/sunshine" "\$@"
LAUNCHER
chmod +x "$BIN_DIR/lumen"

ok "Installed to $INSTALL_DIR"

# ─── Post-install ───────────────────────────────────────────────────────────────

echo ""
echo "  ────────────────────────────────────────────────────"
echo -e "  ${GREEN}Lumen installed successfully!${NC}"
echo "  ────────────────────────────────────────────────────"
echo ""
echo "  Start Lumen:"
echo "    lumen"
echo ""
echo "  Or if ~/.local/bin isn't in your PATH:"
echo "    ~/.local/bin/lumen"
echo ""
echo "  Web UI: https://localhost:47990"
echo ""
echo -e "  ${YELLOW}Required macOS permissions:${NC}"
echo "    1. Screen Recording  (System Settings > Privacy & Security)"
echo "    2. Accessibility     (System Settings > Privacy & Security)"
echo ""
echo -e "  ${YELLOW}Optional — Gamepad support:${NC}"
echo "    Requires AMFI disable (one-time, see README for instructions)."
echo "    Then run:"
echo "      codesign --sign - --entitlements $INSTALL_DIR/hid_entitlements.plist --force $INSTALL_DIR/sunshine"
echo ""
echo "  Config: $CONFIG_DIR/sunshine.conf"
echo "  Logs:   lumen 2>&1 | tee ~/lumen.log"
echo ""
