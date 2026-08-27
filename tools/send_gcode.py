#!/usr/bin/env python3
"""Stream a G-code file to the ME306 plotter over USB serial.

Why this exists rather than pasting the file into a serial monitor:

The firmware only reads serial while the FSM is in HOLD or FAULT (see
FSM::doHold / FSM::doFault). During a G1 or G28 it never drains the input,
so anything sent mid-move piles up in the Mega's 64-byte RX buffer and the
tail of the program is silently lost. Lines therefore have to go one at a
time, each one held back until the machine is idle again.

There is no "ok" acknowledgement in the firmware, so idleness is inferred
from the telemetry instead: G1 prints a CSV row every 20 ms and G28 prints
a LEFT=/BOTTOM=/PHASE= line every dispatch, while HOLD prints nothing at
all. A quiet gap on the serial line therefore means the move finished and
the FSM is back in HOLD, ready for the next line.

Usage:
    pip install pyserial
    python3 tools/send_gcode.py /dev/ttyACM0 tools/heart.gcode
    python3 tools/send_gcode.py COM3 tools/heart.gcode        # Windows
"""

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed. Run:  pip install pyserial")

G1_RE = re.compile(r"^G1 X(-?\d+) Y(-?\d+)(?: F(\d+))?$")


def strip_comment(line):
    return line.split(";")[0].strip()


def load_program(path, invert_xy):
    out = []
    for raw in open(path):
        line = strip_comment(raw)
        if not line:
            continue
        if invert_xy:
            m = G1_RE.match(line)
            if m:
                dx, dy = -int(m.group(1)), -int(m.group(2))
                f = f" F{m.group(3)}" if m.group(3) else ""
                line = f"G1 X{dx} Y{dy}{f}"
        out.append(line)
    return out


def drain(ser, quiet_s, timeout_s, echo, label):
    """Read until the port has been silent for `quiet_s`. Returns the text.

    Raises RuntimeError if the machine reported an error or stayed busy for
    longer than `timeout_s`.
    """
    buf = []
    last_rx = time.monotonic()
    started = last_rx

    while True:
        chunk = ser.read(ser.in_waiting or 1)
        now = time.monotonic()

        if chunk:
            text = chunk.decode("utf-8", "replace")
            buf.append(text)
            if echo:
                sys.stdout.write(text)
                sys.stdout.flush()
            last_rx = now
        elif (now - last_rx) >= quiet_s:
            return "".join(buf)

        if (now - started) >= timeout_s:
            raise RuntimeError(
                f"{label}: machine still busy after {timeout_s:.0f}s - "
                f"it may be stalled, stuck against a limit switch, or in FAULT")


def check_reply(reply, line):
    """Fail loudly on the firmware's error paths instead of ploughing on."""
    if "Error: machine in FAULT" in reply:
        raise RuntimeError(
            f"{line!r}: machine is in FAULT (a limit switch latched).\n"
            f"Send M999 to clear it, then re-home with G28 before retrying.")
    for bad in ("Error: Unknown command", "Error: No G/M command",
                "Error: G1 requires", "Error: F "):
        if bad in reply:
            raise RuntimeError(f"{line!r} was rejected by the parser:\n{reply.strip()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="serial port, e.g. /dev/ttyACM0 or COM3")
    ap.add_argument("gcode", help="path to the .gcode file")
    ap.add_argument("--baud", type=int, default=115200,
                    help="must match cfg::SERIAL_BAUD (default 115200)")
    ap.add_argument("--quiet-ms", type=float, default=400.0,
                    help="silence that counts as 'move finished' (default 400)")
    ap.add_argument("--timeout", type=float, default=90.0,
                    help="seconds to wait for one command (default 90)")
    ap.add_argument("--invert-xy", action="store_true",
                    help="negate every X/Y delta before sending. Use this if "
                         "the first test move runs the wrong way (see README).")
    ap.add_argument("--echo", action="store_true",
                    help="print the telemetry stream as it arrives")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would be sent and exit; opens no port")
    args = ap.parse_args()

    program = load_program(args.gcode, args.invert_xy)
    if not program:
        sys.exit(f"{args.gcode} contains no commands")

    if args.dry_run:
        for line in program:
            print(line)
        print(f"\n-- {len(program)} commands, nothing sent (--dry-run)",
              file=sys.stderr)
        return

    quiet_s = args.quiet_ms / 1000.0

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        # Opening the port pulls DTR and resets the Mega. Give the bootloader
        # time to hand over before expecting anything.
        time.sleep(2.0)

        # setup() prints BOOT and then blocks on `while (Serial.available()
        # == 0)`, so it needs a byte before it will do anything at all.
        print("waiting for BOOT ...")
        ser.reset_input_buffer()
        ser.write(b"\n")
        ser.flush()
        banner = drain(ser, quiet_s, 15.0, args.echo, "boot")
        if "BOOT" in banner:
            print("board booted")
        else:
            # Harmless: the board was probably already past setup().
            print("no BOOT banner seen - continuing anyway")

        for i, line in enumerate(program, 1):
            print(f"[{i}/{len(program)}] {line}")
            ser.write((line + "\n").encode())
            ser.flush()

            # G28 sweeps the whole axis twice, so give it a longer leash.
            timeout = max(args.timeout, 180.0) if line == "G28" else args.timeout
            reply = drain(ser, quiet_s, timeout, args.echo, line)
            check_reply(reply, line)

    print(f"\ndone - {len(program)} commands sent")


if __name__ == "__main__":
    main()
