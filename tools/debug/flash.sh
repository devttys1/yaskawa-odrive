#!/bin/sh
# Build (unless --no-build) and flash the firmware over ST-Link.
#
#   ./tools/debug/flash.sh              build, flash, reset and run
#   ./tools/debug/flash.sh --no-build   flash whatever is already built
set -e
cd "$(dirname "$0")/../.."
. ./tools/debug/env.sh
check_tools

[ "$1" = "--no-build" ] || make

[ -f "$ELF" ] || die "$ELF not found -- run make first"

"$PROGRAMMER" -c port=SWD mode=UR -w "$ELF" -v -rst
