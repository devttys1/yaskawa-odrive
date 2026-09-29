#!/bin/sh
# Locates the STM32CubeIDE-bundled toolchain and ST-Link utilities.
# Sourced by the other scripts in this directory; picks the newest plugin
# version when several are installed side by side.

CUBE_IDE="${CUBE_IDE:-/Applications/STM32CubeIDE.app}"
PLUGINS="$CUBE_IDE/Contents/Eclipse/plugins"

_newest() {
    # $1 = filename to find under the plugins tree.  Sort so the highest
    # plugin version wins rather than whichever find happens to hit first.
    find "$PLUGINS" -name "$1" -type f 2>/dev/null | sort -V | tail -1
}

GDB="${GDB:-$(_newest arm-none-eabi-gdb)}"
GDBSERVER="${GDBSERVER:-$(_newest ST-LINK_gdbserver)}"
PROGRAMMER="${PROGRAMMER:-$(_newest STM32_Programmer_CLI)}"
OBJDUMP="${OBJDUMP:-$(_newest arm-none-eabi-objdump)}"

# The gdbserver needs the CubeProgrammer directory for its flash loaders.
CUBEPROG_DIR="$(dirname "$PROGRAMMER" 2>/dev/null)"

ELF="${ELF:-build/yaskawa-encoder.elf}"
GDB_PORT="${GDB_PORT:-61234}"

die() { echo "error: $*" >&2; exit 1; }

check_tools() {
    [ -x "$GDB" ]        || die "arm-none-eabi-gdb not found (set GDB=)"
    [ -x "$GDBSERVER" ]  || die "ST-LINK_gdbserver not found (set GDBSERVER=)"
    [ -x "$PROGRAMMER" ] || die "STM32_Programmer_CLI not found (set PROGRAMMER=)"
}
