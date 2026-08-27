# Host-side tooling — drawing a heart from the centre

Two scripts: one generates the heart program, one streams it to the board.
Neither touches the firmware — `src/` is unchanged.

| File | What it is |
|---|---|
| `gen_heart.py` | Generates `heart.gcode` from a parametric heart curve |
| `heart.gcode` | The generated program: `G28`, travel to centre, draw the heart |
| `send_gcode.py` | Streams a `.gcode` file to the Mega with proper flow control |

---

## Running it

```bash
# 1. Flash the firmware (from the repo root, board plugged in)
pio run -t upload -e megaatmega2560

# 2. Close every serial monitor, then stream the program
pip install pyserial
python3 tools/send_gcode.py /dev/ttyACM0 tools/heart.gcode
```

Use `COM3` (or whatever Device Manager shows) instead of `/dev/ttyACM0` on
Windows. `--echo` adds the live telemetry stream, `--dry-run` prints the
commands without opening the port.

**Check the direction before you run the whole thing.** See
[Which way is +X?](#which-way-is-x) below — it is the one thing that can
drive the machine into its own limit switches.

---

## What it draws

`G28` homes to the LEFT switch, then the BOTTOM switch, and zeroes the
origin in that corner. From there the program travels to the centre of the
200 × 200 mm envelope and draws an 80 × 72 mm heart centred on it:

```
              ******                     ******
         ******    ******           ******    ******
       ***              ***       ***              ***
     ***                  ***   ***                  ***
    **                      ** **                      **
   **                        ***                        **
   *                          *                          *
   *                                                     *
   **                                                   **
    **                                                 **
     **                                               **
      **                                             **
       **                     *                     **
        ***                   *                   ***
          ****                *                ****
             ***              *              ***
                ***           *           ***
                  ***         *         ***
                    ***       *       ***
                      ***     *     ***
                         ***  *  ***
                           ** * **
                            *****
                             ***
                              *
```

43 moves, roughly 30 s at F600. The pen starts at the centre, runs down to
the bottom cusp, traces the outline, and comes back up to the centre.

**There is no pen lift in this firmware**, so every move draws. Two marks
are unavoidable consequences of that:

- the diagonal lead-in from the home corner out to the centre, and
- the vertical line from the centre down to the bottom cusp.

The lead-in and lead-out both run along the heart's axis of symmetry, so
the return stroke retraces the entry stroke exactly — you get one line
through the middle, not two.

To start the heart somewhere else, or to size it differently:

```bash
python3 tools/gen_heart.py --scale 2.0 --centre-x 80 --centre-y 90 --feed 800
```

The generator refuses to emit a program whose waypoints fall outside the
envelope, so a too-large `--scale` fails loudly rather than at the frame.

---

## Which way is +X?

`G1::beginMove()` negates the commanded target before running the
kinematics:

```cpp
long x = -target_x;
long y = -target_y;
```

Whether that lands as physical +X also depends on the encoder polarity and
on `MotorDriver`'s `invert` flag, which is `true` for both motors. That
cannot be resolved by reading the code alone — it needs one move on the
real machine.

Because `G28` homes into the bottom-left corner and calls it `(0, 0)`, the
whole workspace has to lie in +X / +Y from there. If the sign is backwards,
the first move drives straight back into the switches it just homed against.

So check it first, with the machine homed and a hand near the power:

```
G28
G1 X20 Y20 F600
```

- Moves **away** from the corner (up and to the right) → correct, run the
  heart as-is.
- Moves **into** the corner, or latches a limit fault → add `--invert-xy`:

  ```bash
  python3 tools/send_gcode.py /dev/ttyACM0 tools/heart.gcode --invert-xy
  ```

  That negates every delta on the host, so nothing has to be recompiled or
  re-flashed. If you would rather fix it in firmware, drop the two minus
  signs in `G1::beginMove()` instead — but then do *not* pass `--invert-xy`.

If a limit switch latches mid-run the machine enters FAULT and rejects
everything until `M999` clears it. `send_gcode.py` detects that and stops
rather than sending the rest of the program into a faulted machine.

---

## Why streaming needs a script

The firmware only reads serial while the FSM is in `HOLD` or `FAULT`
(`FSM::doHold` / `FSM::doFault`). During a `G1` or `G28` it never drains the
input, so anything pasted mid-move overflows the Mega's 64-byte RX buffer
and the rest of the program is lost silently.

There is no `ok` acknowledgement to synchronise on, so `send_gcode.py`
infers readiness from the telemetry: `G1` prints a CSV row every 20 ms and
`G28` prints a `LEFT=/BOTTOM=/PHASE=` line every dispatch, while `HOLD`
prints nothing. A quiet gap on the line means the move is done. Tune it
with `--quiet-ms` if the default 400 ms is too twitchy.

The script also handles the boot handshake — `setup()` prints `BOOT` and
then blocks on `while (Serial.available() == 0)`, so it needs a byte before
it will start.

---

## Constraints the generated G-code respects

From `src/GCodeParser.cpp` and `src/G1.cpp`:

- **Integers only.** `parseIntToken` rejects a decimal point outright, for
  every token including `X`, `Y` and `F`.
- **`G1` needs both `X` and `Y`** on the line or it is rejected.
- **Moves are relative.** `beginMove()` zeroes the encoders each move, so
  `X`/`Y` are displacements, not absolute positions.
- **No zero-length moves.** With `pathLength_ == 0` the profile's `s_` can
  never reach `1.0`, so the move never completes and the FSM hangs in `G1`.
  The generator drops any waypoint that rounds onto its predecessor.
- **`F` is sticky** but must appear on the first move.

`gen_heart.py` rounds absolute waypoints to whole mm *first* and takes the
deltas between the rounded absolutes, so rounding error stays bounded at
±0.5 mm instead of accumulating around the outline.
