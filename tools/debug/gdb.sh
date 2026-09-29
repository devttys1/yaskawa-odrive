#!/bin/sh
# Runs GDB commands against the target in batch mode and prints the output.
#
# This is the entry point intended for scripted or assistant-driven debugging:
# every invocation is non-interactive, so it always terminates and its output
# can be read back.
#
#   ./tools/debug/gdb.sh "info registers"
#   ./tools/debug/gdb.sh "break yask_read" "continue" "bt"
#   ./tools/debug/gdb.sh -f script.gdb
#
# The gdbserver is started automatically if it is not already running.
set -e
cd "$(dirname "$0")/../.."
. ./tools/debug/env.sh
check_tools

[ -f "$ELF" ] || die "$ELF not found -- run make first"

./tools/debug/gdbserver.sh start >/dev/null

CMDFILE=$(mktemp /tmp/yask-gdb.XXXXXX)
trap 'rm -f "$CMDFILE"' EXIT

# Common preamble: quiet, no pagination (batch mode must never block on a
# pager prompt), and connect to the running server.
cat > "$CMDFILE" <<EOF
set confirm off
set pagination off
set print pretty on
target extended-remote localhost:$GDB_PORT
EOF

if [ "$1" = "-f" ]; then
    [ -f "$2" ] || die "script file $2 not found"
    cat "$2" >> "$CMDFILE"
else
    for cmd in "$@"; do
        echo "$cmd" >> "$CMDFILE"
    done
fi

# Note: avoid "next"/"step" in batch scripts.  Source-level stepping needs
# several stop/go round trips with the gdbserver and regularly hangs a batch
# session; read registers and variables instead, or set another breakpoint.
#
# -batch exits when the script ends, even if the target is still running.
"$GDB" -batch -x "$CMDFILE" "$ELF" 2>&1
