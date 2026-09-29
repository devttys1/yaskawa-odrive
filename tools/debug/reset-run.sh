#!/bin/sh
# Reset the target and let it run free (no breakpoints).
cd "$(dirname "$0")/../.."
exec ./tools/debug/gdb.sh "monitor reset" "continue &" "detach"
