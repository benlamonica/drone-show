#!/usr/bin/env bash
# One-shot toolchain setup for this port: Pico SDK 2.x + the Raspberry Pi
# OpenOCD fork (upstream 0.12.0 has no RP2350 target) + picotool.
set -euo pipefail
PICO_DIR="${PICO_DIR:-$HOME/pico}"
mkdir -p "$PICO_DIR"

if [ ! -d "$PICO_DIR/pico-sdk/.git" ]; then
  git clone -b master --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/raspberrypi/pico-sdk.git "$PICO_DIR/pico-sdk"
fi

if command -v brew >/dev/null; then
  brew install autoconf automake libtool pkg-config texinfo libusb hidapi picotool || true
fi

if [ ! -x "$PICO_DIR/openocd-install/bin/openocd" ]; then
  [ -d "$PICO_DIR/openocd/.git" ] || git clone --depth 1 -b sdk-2.0.0 \
      https://github.com/raspberrypi/openocd.git "$PICO_DIR/openocd"
  cd "$PICO_DIR/openocd"
  ./bootstrap
  # --disable-capstone: OpenOCD 0.12 does not match Homebrew capstone 5's
  # header layout, and nothing here needs disassembly.
  ./configure --prefix="$PICO_DIR/openocd-install" --enable-cmsis-dap \
              --disable-werror --disable-capstone
  make -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc)"
  make install
fi

echo
echo "Toolchain ready. Add to your shell:"
echo "  export PICO_SDK_PATH=$PICO_DIR/pico-sdk"
echo "  export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:\$PATH"
"$PICO_DIR/openocd-install/bin/openocd" --version 2>&1 | head -1
