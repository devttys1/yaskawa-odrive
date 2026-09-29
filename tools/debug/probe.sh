#!/bin/sh
# Reports whether an ST-Link and target are visible, without flashing.
# Run this first when something is not working.
cd "$(dirname "$0")/../.."
. ./tools/debug/env.sh
check_tools

echo "=== ST-Link probe ==="
"$PROGRAMMER" -c port=SWD mode=UR -q 2>&1 | sed -n '1,40p'
