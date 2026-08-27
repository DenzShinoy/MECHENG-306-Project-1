# ME306 Project 1 — CoreXY X-Y Plotter

Modular Arduino (C++) firmware for an **Arduino Mega 2560 + DFRobot L298P**
shield driving a 2-axis **CoreXY** pen plotter.

PID, encoder decoding, debouncing, the FSM, G-code parsing and trajectory
generation are all **our own modules** (no third-party libraries), as required
by the brief.

Hardware, pin map and bring-up order: [`ME306_plotter_pinout.md`](ME306_plotter_pinout.md).

---

## Design rules

- **Non-blocking everywhere.** No `delay()` beyond a few µs. Every module is
  polled: `update(nowMs)` / `execute()` are handed the current time and must
  return quickly. `loop()` runs a super-loop; the control math is gated to
  `cfg::CONTROL_PERIOD_MS`.
- **Counts internally, mm at the edge.** The encoders, PID and kinematics all
  work in **integer encoder counts** (`long`). The only place millimetres
  appear is the G-code boundary (`GCodeParser` in, and `Kinematics::mmToCounts`
  converting to the count domain).
- **No dynamic allocation, minimal STL.** Fixed buffers, static storage,
  `constexpr` config. Suits the ATmega2560.
- **Single responsibility per module**, one `.h`/`.cpp` each.
- **Dependency injection.** Concrete modules are constructed once in
  `main.ino` and injected by reference. Only `main` knows real pin numbers.
- **One place prints.** The FSM owns state reporting, once per transition.
  The exception is `GCodeParser`, which reports every line it rejects as a
  single `ERR: …` line. The FSM never sees those: a rejected line causes
  no transition, so silence from the board means a line was lost, never
  that it was refused.

---

## Modules

| # | Module | Responsibility | Owns state? |
|---|--------|----------------|-------------|
| 1 | `Pins.h` | All pins + machine constants (`constexpr`, header only) | — |
| 2 | `Encoder` | Quadrature decode + position (counts), ISR-friendly | count |
| 3 | `LimitSwitch` | Non-blocking debounce of one switch | debounce timer |
| 4 | `MotorDriver` | dir + PWM wrapper over one shield channel | — |
| 5 | `PID` | Position PID with a bounded integral | integrator |
| 6 | `Kinematics` | CoreXY ↔ XY transforms + counts↔mm (static) | — |
| 7 | `updateVelocityProfile1` | One trapezoidal path-velocity step (stateless) | — |
| 8 | `GCodeParser` | Serial line → `GCodeCommand` → FSM event | line buffer |
| 9 | `Manager` | Shared state: position, current command, switches, fault | position + fault |
| 10 | `G1` | One coordinated straight-line move (profile + 2 PIDs) | move state |
| 11 | `G28` | Homing: seek LEFT, back off, seek BOTTOM, back off | homing phase |
| 12 | `FSM` (`fsm_1`) | HOLD / G1 / G28 / FAULT + all serial reporting | current state |
| 13 | `main.ino` | Composition root: construct, wire ISRs, tick, fault path | static instances |

---

## Module graph (who includes / uses whom)

```
                        ┌──────────────┐
                        │   main.ino   │  composition root: builds every
                        └──────┬───────┘  module, wires ISRs, ticks loop()
                               │ constructs + injects
                               ▼
                        ┌──────────────┐
                        │     FSM      │  HOLD / G1 / G28 / FAULT
                        └──┬────┬────┬─┘
              ┌────────────┘    │    └────────────┐
              ▼                 ▼                 ▼
        ┌──────────┐      ┌──────────┐     ┌────────────┐
        │    G1    │      │   G28    │     │GCodeParser │
        └─┬──┬──┬──┘      └─┬──┬─────┘     └──────┬─────┘
          │  │  │           │  │                  │
          ▼  ▼  ▼           ▼  ▼                  ▼
    ┌───────┐ ┌───┐ ┌───────────┐ ┌───────────┐ ┌─────────┐
    │Encoder│ │PID│ │MotorDriver│ │Kinematics │ │ Manager │
    └───────┘ └───┘ └───────────┘ └───────────┘ └────┬────┘
                                                     ▼
                                               ┌───────────┐
                                               │LimitSwitch│ ×4
                                               └───────────┘
```

`Pins.h` is included by nearly everything and depends on nothing. `Kinematics`
is stateless (static methods) and is the shared owner of the counts↔mm scale.
`Manager` is the shared blackboard: the parser writes the command into it, the
motion classes read it and write back the achieved position.

---

## Data flow (one command, end to end)

```
 Host (USB Serial, mm / mm-min)
        │  "G1 X50 Y30 F1200\n"
        ▼
 FSM::doHold ──► GcodeParserFull ──► GCodeCommand {mm} ──► Manager::setCommand
        │                                  │  (bounds + max-feed checked here)
        │                                  ▼
        │                            FSM event (1 = G1, 2 = G28, 0 = HOLD, -1 = FAULT)
        ▼
 FSM::doG1 ──► G1::execute(x, y, F)
        │            │  Kinematics::mmToCounts ──► xyToAB ──► A/B target [counts]
        │            │
        │            │  ── every control tick (cfg::CONTROL_PERIOD_MS) ──
        │            ▼
        │   updateVelocityProfile1 ──► path progress s ──► moving A/B reference
        │            │
        │            ▼
        │   Encoder::position() ──► PID::update ──► MotorDriver::setSpeed ──► L298P ──► motors
        │                                                                          │
        ▼                                                                          │
 Manager::setCurrentPosition ◄── encoder feedback (ISR: Encoder::handleEdge) ◄──────┘
```

Feed handling: `F` is the true tool feed in mm/min. The parser throttles any
`F` above `cfg::MAX_FEED_MM_PER_MIN`, and `G1` slows a move further if the
dominant motor would have to exceed `cfg::MAX_TRACK_CPS` — an over-fast move
slows down instead of bowing off the straight line.

Homing (`G28`): seek LEFT at full PWM, back off slowly until the switch
releases, seek BOTTOM, back off, then zero both encoders and the Manager's
position. G28 is exempt from the limit fault path, since it presses switches
on purpose.

Fault path: three layers — an ISR per switch (a *hint* only, because PWM noise
couples into the harness), the `LimitSwitch` debouncer, and a per-switch
confirmation window in `loop()`. A fault latches only when the debounced state
confirms the ISR's hint. `M999` clears it. See the comment block at the top of
`main.ino`.

---

## Layout

```
platformio.ini            build config (env:megaatmega2560)
ME306_plotter_pinout.md   hardware pin map + bring-up notes
src/
  Pins.h                    1  config (header only)
  Encoder.{h,cpp}           2
  LimitSwitch.{h,cpp}       3
  MotorDriver.{h,cpp}       4
  PID.{h,cpp}               5
  Kinematics.{h,cpp}        6
  updateVelocityProfile1.{h,cpp}  7
  GCodeParser.{h,cpp}       8
  manager.{h,cpp}           9
  G1.{h,cpp}               10
  G28.{h,cpp}              11
  fsm_1.{h,cpp}            12
  main.ino                 13  setup() / loop()
test/
  LimitSwitchTest.ino     standalone bring-up sketch (not part of the build)
tools/                    host-side Python: G-code generators + serial streamer
gcode/                    generated G-code + preview renders
velocity_logger/          MATLAB velocity capture + the estimator it pairs with
```

Build: `pio run` — targets the Mega. Upload: `pio run -t upload`.
Serial monitor: `pio device monitor` (115200 baud).
Stream a drawing: `python tools/stream_gcode.py gcode/banana.gcode --port COM5`.
