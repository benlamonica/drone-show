#!/usr/bin/env bash
# Start an OpenOCD GDB server for the target. Then, in another shell:
#   arm-none-eabi-gdb build/pico/drone-show-controller.elf -ex 'target extended-remote :3333'
set -euo pipefail
OPENOCD="${OPENOCD:-$HOME/pico/openocd-install/bin/openocd}"
SCRIPTS="${OPENOCD_SCRIPTS:-$HOME/pico/openocd-install/share/openocd/scripts}"
exec "$OPENOCD" -s "$SCRIPTS" \
  -f interface/cmsis-dap.cfg -c "adapter speed 5000" \
  -f target/rp2350.cfg
