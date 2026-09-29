#!/usr/bin/env python3
"""Read the ST-Link virtual COM port.

Uses termios directly rather than stty+cat: on macOS the tty must be
configured and then held open by the *same* process, otherwise the settings
are reset between the stty call and the read, and characters are lost
mid-line.

    ./tools/debug/serial.py                      follow until Ctrl-C
    ./tools/debug/serial.py 50                   stop after 50 lines
    ./tools/debug/serial.py -o run.txt           capture to a file, Ctrl-C to stop
    ./tools/debug/serial.py 2000 -o run.txt      capture 2000 lines then stop
    ./tools/debug/serial.py --port /dev/cu.usbmodemXXXX
"""
import os
import sys
import glob
import fcntl
import array
import termios

BAUD = 2000000

# macOS has no termios constant above B230400, and tcsetattr rejects a raw
# integer speed.  Non-standard rates go through this ioctl instead.
IOSSIOSPEED = 0x80045402


def main() -> int:
    argv = sys.argv[1:]

    out_path = None
    for flag in ("-o", "--out"):
        if flag in argv:
            i = argv.index(flag)
            out_path = argv[i + 1]
            del argv[i:i + 2]

    port = None
    if "--port" in argv:
        i = argv.index("--port")
        port = argv[i + 1]
        del argv[i:i + 2]

    args = [a for a in argv if not a.startswith("-")]
    lines = int(args[0]) if args else 0

    if port is None:
        found = sorted(glob.glob("/dev/cu.usbmodem*"))
        if found:
            port = found[0]
    if not port:
        print("error: no /dev/cu.usbmodem* found -- is the Nucleo plugged in?",
              file=sys.stderr)
        return 1

    # A second reader on the same tty silently steals bytes, which looks
    # exactly like firmware corruption: lines arrive interleaved and with
    # chunks missing mid-field.  STM32CubeIDE's built-in serial terminal is
    # the usual culprit.
    try:
        import subprocess
        held = subprocess.run(["lsof", port], capture_output=True, text=True).stdout
        others = [l for l in held.splitlines()[1:] if l.strip()]
        if others:
            names = {l.split()[0] for l in others}
            print(f"error: {port} is already open by: {', '.join(sorted(names))}",
                  file=sys.stderr)
            print("       Two readers split the byte stream, so this would "
                  "either hang or capture", file=sys.stderr)
            print("       corrupted lines. Close that reader first "
                  "(in STM32CubeIDE: the serial", file=sys.stderr)
            print("       terminal tab), then run this again. "
                  "Use --force to read anyway.", file=sys.stderr)
            if "--force" not in sys.argv:
                return 2
    except Exception:
        pass

    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
    try:
        attrs = termios.tcgetattr(fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        # 8N1, raw, no flow control, receiver enabled, ignore modem lines.
        iflag = 0
        oflag = 0
        lflag = 0
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
        cc = list(cc)
        cc[termios.VMIN] = 1
        cc[termios.VTIME] = 0
        # Set a standard speed first, then override via IOSSIOSPEED.
        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag,
                           termios.B9600, termios.B9600, cc])
        fcntl.ioctl(fd, IOSSIOSPEED, array.array("i", [BAUD]))
        termios.tcflush(fd, termios.TCIFLUSH)

        sink = open(out_path, "w") if out_path else None
        if sink:
            print(f"capturing {port} -> {out_path}"
                  + (f" ({lines} lines)" if lines else " (Ctrl-C to stop)"),
                  file=sys.stderr)
        buf = b""
        count = 0
        try:
            while True:
                chunk = os.read(fd, 4096)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("ascii", "replace").rstrip("\r")
                    if sink:
                        sink.write(text + "\n")
                        sink.flush()
                        count += 1
                        if count % 200 == 0:
                            print(f"  {count} lines", file=sys.stderr)
                    else:
                        sys.stdout.write(text + "\n")
                        sys.stdout.flush()
                        count += 1
                    if lines and count >= lines:
                        return 0
        finally:
            if sink:
                sink.close()
                print(f"wrote {count} lines to {out_path}", file=sys.stderr)
    except KeyboardInterrupt:
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
