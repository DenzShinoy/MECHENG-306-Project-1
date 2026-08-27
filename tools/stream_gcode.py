"""
Stream a G-code file to the ME306 plotter, one line at a time.

The firmware only reads serial while the FSM sits in HOLD (see
FSM::doHold), and the AVR's receive buffer is 64 bytes, so the file
cannot simply be dumped at the port -- anything sent during a move is
lost. This streamer therefore sends one command and waits for the board
to announce it has come back to HOLD before sending the next.

The handshake rides on the FSM's state banners:

    -> G1 X10 Y0
    <- STATE: G1
    <- G1: moving to 10 0 at 1200 feed rate
       ...the move runs...
    <- STATE: HOLD          # ready for the next line

Every rejection now reports itself, always as a single ERR: line:

    -> G7
    <- ERR: unknown command G7, known: G1 G28 G333 M999
    -> G1 X250 Y0
    <- ERR: out of bounds: at X100 Y20 + X250 Y0 = X350 Y20, limits ...

so a rejected line aborts the stream immediately with the board's own
reason. A line that is genuinely LOST still produces nothing, and is
caught by the accept timeout below. Either way the stream aborts rather
than silently skipping part of the drawing.

Opening the port resets the Mega, so the connection is held open for the
whole plot.
"""

import argparse
import sys
import time

import serial


# The board leaves HOLD within a control loop or two of accepting a line.
# Anything longer means the parser threw the line away.
ACCEPT_TIMEOUT = 3.0

# A G1 is bounded by its length plus the settle window; G28 sweeps the
# whole envelope twice looking for switches.
MOVE_TIMEOUT = 90.0
HOME_TIMEOUT = 240.0


class Fault(Exception):
    pass


class Rejected(Exception):
    def __init__(self, command, reason=None):
        super().__init__(command)
        self.reason = reason


def read_lines(port, deadline, log):
    """Yield decoded lines from the board until `deadline`."""

    buf = b""

    while time.time() < deadline:
        chunk = port.read(256)
        if not chunk:
            continue

        buf += chunk

        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if line:
                log(line)
                yield line


def send(port, command, log, verbose):
    """Send one command and wait for the board to return to HOLD."""

    is_home = command.startswith("G28")

    port.reset_input_buffer()
    port.write((command + "\n").encode("ascii"))
    port.flush()

    # Phase 1: the board must LEAVE hold, which is its only acknowledgement
    # that the line parsed and validated.
    state = None
    deadline = time.time() + ACCEPT_TIMEOUT

    for line in read_lines(port, deadline, log if verbose else lambda s: None):
        if line.startswith("STATE: FAULT"):
            raise Fault(command)
        if line.startswith("ERR:"):
            # The board said why. No need to sit out the accept timeout.
            raise Rejected(command, line)
        if line.startswith("STATE: ") and not line.startswith("STATE: HOLD"):
            state = line
            break

    if state is None:
        raise Rejected(command)

    # Phase 2: wait out the move.
    deadline = time.time() + (HOME_TIMEOUT if is_home else MOVE_TIMEOUT)

    for line in read_lines(port, deadline, log if verbose else lambda s: None):
        if line.startswith("STATE: FAULT"):
            raise Fault(command)
        if line.startswith("STATE: HOLD"):
            return

    raise TimeoutError(command)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--limit", type=int, default=0,
                    help="send only the first N commands (0 = all)")
    ap.add_argument("--start", type=int, default=0,
                    help="skip the first N commands (to resume a plot)")
    ap.add_argument("--quiet", action="store_true",
                    help="hide the board's own chatter")
    args = ap.parse_args()

    with open(args.file) as f:
        commands = []
        for raw in f:
            line = raw.split(";")[0].strip()
            if line:
                commands.append(line)

    if args.start:
        commands = commands[args.start:]
    if args.limit:
        commands = commands[:args.limit]

    def log(s):
        print("      | %s" % s, flush=True)

    print("port     : %s @ %d" % (args.port, args.baud), flush=True)
    print("file     : %s" % args.file, flush=True)
    print("commands : %d" % len(commands), flush=True)

    port = serial.Serial(args.port, args.baud, timeout=0.1)

    started = time.time()
    done = 0

    try:
        # Opening the port pulls DTR and resets the board; wait out the
        # bootloader, then clear the banner it prints on the way up.
        time.sleep(2.5)
        port.reset_input_buffer()

        for i, command in enumerate(commands, 1):
            elapsed = time.time() - started
            print("[%4d/%d] %-24s  t=%5.1fs" % (i, len(commands), command,
                                                elapsed), flush=True)
            send(port, command, log, not args.quiet)
            done = i

    except Rejected as e:
        print("\nABORTED: the board rejected %r." % str(e), flush=True)
        if e.reason:
            print("  %s" % e.reason, flush=True)
        else:
            print("It never acknowledged the line and said nothing at all.",
                  flush=True)
            print("The board reports every rejection it makes, so silence",
                  flush=True)
            print("means the line never arrived -- check the cable/port.",
                  flush=True)
        print("Completed %d commands." % done, flush=True)
        return 1

    except Fault as e:
        print("\nABORTED: the machine FAULTED at %r." % str(e), flush=True)
        print("A limit switch latched. Send M999 to clear, then resume with",
              flush=True)
        print("  --start %d" % (args.start + done), flush=True)
        return 1

    except TimeoutError as e:
        print("\nABORTED: %r started but never finished." % str(e), flush=True)
        print("Completed %d commands." % done, flush=True)
        return 1

    except KeyboardInterrupt:
        print("\nInterrupted after %d commands. Resume with --start %d"
              % (done, args.start + done), flush=True)
        return 1

    finally:
        port.close()

    print("\nDone: %d commands in %.1f s." % (done, time.time() - started),
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
