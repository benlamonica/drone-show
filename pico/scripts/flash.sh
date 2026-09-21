#!/usr/bin/env bash
# Flash an ELF to the target over SWD via the debugprobe.
#   ./pico/scripts/flash.sh build/pico/drone-show-controller.elf
set -euo pipefail
ELF="${1:?usage: flash.sh <file.elf>}"
OPENOCD="${OPENOCD:-$HOME/pico/openocd-install/bin/openocd}"
SCRIPTS="${OPENOCD_SCRIPTS:-$HOME/pico/openocd-install/share/openocd/scripts}"
exec "$OPENOCD" -s "$SCRIPTS" \
  -f interface/cmsis-dap.cfg -c "adapter speed 5000" \
  -f target/rp2350.cfg \
  -c "program $ELF verify reset exit"
