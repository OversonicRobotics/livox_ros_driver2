#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_PKG_DIR="$(dirname "$SCRIPT_DIR")"
SDK_DEST="$ROS_PKG_DIR/livox_sdk"

# ── 1. Validate Livox-SDK2 path ──────────────────────────────────────────────
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <path-to-Livox-SDK2>"
    echo "  e.g. $0 ~/oversonic/extra/Livox-SDK2"
    exit 1
fi

SDK_SRC="$(realpath "$1")"

if [[ ! -f "$SDK_SRC/CMakeLists.txt" ]]; then
    echo "ERROR: $SDK_SRC/CMakeLists.txt not found — not a valid Livox-SDK2 root"
    exit 1
fi

echo "[1/4] Using Livox-SDK2 source: $SDK_SRC"

# ── 2. Build ─────────────────────────────────────────────────────────────────
echo "[2/4] Building Livox-SDK2..."

BUILD_DIR="$SDK_SRC/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make -j"$(nproc)"

# ── 3. Install system-wide ───────────────────────────────────────────────────
echo "[3/4] Installing (sudo make install)..."
sudo make install
sudo ldconfig

# ── 4. Copy headers + libs into ROS package ─────────────────────────────────
echo "[4/4] Updating $SDK_DEST ..."

ARCH="$(uname -m)"   # x86_64 or aarch64

# Headers — grab from installed location or build tree
INSTALLED_INC="/usr/local/include/livox_lidar_api.h"
if [[ -f "$INSTALLED_INC" ]]; then
    HDR_SRC="/usr/local/include"
else
    HDR_SRC="$(find "$SDK_SRC" -name "livox_lidar_api.h" | head -1 | xargs dirname)"
fi

if [[ -z "$HDR_SRC" ]]; then
    echo "ERROR: headers not found after install"
    exit 1
fi

mkdir -p "$SDK_DEST/include"
cp -v "$HDR_SRC"/livox_lidar_api.h \
      "$HDR_SRC"/livox_lidar_cfg.h \
      "$HDR_SRC"/livox_lidar_def.h \
      "$SDK_DEST/include/"

# Shared lib
SHARED_INSTALLED="/usr/local/lib/liblivox_lidar_sdk_shared.so"
if [[ -f "$SHARED_INSTALLED" ]]; then
    LIB_SHARED="$SHARED_INSTALLED"
else
    LIB_SHARED="$(find "$BUILD_DIR" -name "liblivox_lidar_sdk_shared.so" | head -1)"
fi

# Static lib
STATIC_INSTALLED="/usr/local/lib/liblivox_lidar_sdk_static.a"
if [[ -f "$STATIC_INSTALLED" ]]; then
    LIB_STATIC="$STATIC_INSTALLED"
else
    LIB_STATIC="$(find "$BUILD_DIR" -name "liblivox_lidar_sdk_static.a" | head -1)"
fi

if [[ -z "$LIB_SHARED" || -z "$LIB_STATIC" ]]; then
    echo "ERROR: one or more libs not found:"
    echo "  shared: ${LIB_SHARED:-<missing>}"
    echo "  static: ${LIB_STATIC:-<missing>}"
    exit 1
fi

mkdir -p "$SDK_DEST/lib/$ARCH"
cp -v "$LIB_SHARED" "$SDK_DEST/lib/$ARCH/"
cp -v "$LIB_STATIC" "$SDK_DEST/lib/$ARCH/"

echo ""
echo "Done. Updated files in $SDK_DEST:"
find "$SDK_DEST" -type f | sort | sed 's/^/  /'
echo ""
echo "Rebuild ROS package:"
echo "  cd $(dirname "$ROS_PKG_DIR") && colcon build --packages-select livox_ros_driver2"
