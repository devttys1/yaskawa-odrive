#!/bin/sh
# Starts the ST-Link GDB server in the background and waits for it to listen.
#
#   ./tools/debug/gdbserver.sh start   launch (idempotent)
#   ./tools/debug/gdbserver.sh stop    kill it
#   ./tools/debug/gdbserver.sh status  is it listening?
#
# Logs to /tmp/stlink-gdbserver.log, PID in /tmp/stlink-gdbserver.pid.
cd "$(dirname "$0")/../.."
. ./tools/debug/env.sh

PIDFILE=/tmp/stlink-gdbserver.pid
LOGFILE=/tmp/stlink-gdbserver.log

is_listening() {
    # lsof is more reliable than parsing the log, which buffers.
    lsof -nP -iTCP:"$GDB_PORT" -sTCP:LISTEN >/dev/null 2>&1
}

case "${1:-start}" in
start)
    if is_listening; then
        echo "gdbserver already listening on port $GDB_PORT"
        exit 0
    fi
    check_tools
    # -p  GDB listen port (-z is the SWO port, not this one)
    # -d  SWD wire protocol (the Nucleo's on-board ST-Link)
    # -g  attach to the running target instead of halting under reset
    # -cp path to the CubeProgrammer flash loaders
    "$GDBSERVER" -p "$GDB_PORT" -d -g -cp "$CUBEPROG_DIR" \
        > "$LOGFILE" 2>&1 &
    echo $! > "$PIDFILE"

    # Wait up to 5 s for the listen socket, so callers can proceed safely.
    i=0
    while [ $i -lt 50 ]; do
        if is_listening; then
            echo "gdbserver listening on port $GDB_PORT (pid $(cat $PIDFILE))"
            exit 0
        fi
        sleep 0.1
        i=$((i + 1))
    done
    echo "gdbserver failed to start; log follows:" >&2
    cat "$LOGFILE" >&2
    exit 1
    ;;
stop)
    [ -f "$PIDFILE" ] && kill "$(cat "$PIDFILE")" 2>/dev/null
    rm -f "$PIDFILE"
    echo "gdbserver stopped"
    ;;
status)
    if is_listening; then
        echo "listening on port $GDB_PORT"
    else
        echo "not running"
        exit 1
    fi
    ;;
*)
    echo "usage: $0 {start|stop|status}" >&2
    exit 2
    ;;
esac
