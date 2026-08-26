# ME306 Project 1 — CoreXY X-Y Plotter (firmware scaffold)

Modular Arduino (C++) firmware for an **Arduino Mega 2560 + DFRobot L298P**
shield driving a 2-axis **CoreXY** pen plotter.

> **Status: scaffold only.** This tree defines the file/class structure and
> public interfaces. Method bodies are `// TODO` stubs — there is no working
> control logic yet. PID, encoder decoding, debouncing, the FSM, G-code
> parsing and trajectory generation are all scaffolded as **our own modules**
> (no third-party libraries), as required by the brief.

Hardware, pin map and bring-up order: [`ME306_plotter_pinout.md`](ME306_plotter_pinout.md).

---

## Design rules

- **Non-blocking everywhere.** No `delay()` beyond a few µs. Every module is
  polled: `update(nowMs)` / `compute(dt)` are handed the current time and must
  return quickly. `loop()` runs a super-loop; the control math is gated to
  `cfg::CONTROL_PERIOD_MS`.
- **Counts internally, mm at the edge.** The encoders, PID, kinematics and
  trajectory planner all work in **integer encoder counts** (`long`). The only
  place millimetres appear is the G-code boundary (`GCodeParser` in, and
  `Kinematics::mmToCounts` converting to the count domain). This keeps the fast
  loop free of float drift.
- **No dynamic allocation, minimal STL.** Fixed buffers, static storage,
  `constexpr` config. Suits the ATmega2560.
- **Single responsibility per module**, one `.h`/`.cpp` each.
- **Dependency injection.** Concrete modules are constructed once in
  `main.ino` and injected by reference into `PlotterController`. Only `main`
  knows real pin numbers; everything else is testable in isolation.

---

## Modules

| # | Module | Responsibility | Owns state? |
|---|--------|----------------|-------------|
| 1 | `Pins.h` | All pins + machine constants (`constexpr`, header only) | — |
| 2 | `Encoder` | Quadrature decode + position (counts), ISR-friendly | count |
| 3 | `LimitSwitch` | Non-blocking debounce of one active-LOW switch | debounce timer |
| 4 | `MotorDriver` | dir + PWM wrapper over one shield channel | — |
| 5 | `PIDController` | Generic position PID with anti-windup | integrator |
| 6 | `TrajectoryPlanner` | Trapezoidal profile, coordinated 2-axis | profile + clock |
| 7 | `Kinematics` | CoreXY ↔ XY transforms + counts↔mm (static) | — |
| 8 | `GCodeParser` | Parse a line → `GCodeCommand` (G1, G28) | — |
| 9 | `StateMachine` | FSM: IDLE / HOMING / MOVING / FAULT | current state |
| 10 | `PlotterController` | Orchestrates all of the above | scheduling + line buffer |
| 11 | `main.ino` | Composition root: construct, inject, tick | static instances |

---

## Module graph (who includes / uses whom)

```
                        ┌──────────────┐
                        │   main.ino   │  composition root: builds every
                        └──────┬───────┘  module, wires ISRs, ticks loop()
                               │ constructs + injects
                               ▼
                    ┌────────────────────────┐
                    │   PlotterController     │  orchestrator
                    └──┬───┬───┬────┬────┬───┬┘
          ┌───────────┘   │   │    │    │   └────────────┐
          ▼               ▼   ▼    ▼    ▼                ▼
     ┌─────────┐   ┌──────────┐ ┌───────────┐ ┌───────────────────┐ ┌────────────┐
     │ Encoder │   │LimitSwitch│ │MotorDriver│ │ TrajectoryPlanner │ │GCodeParser │
     └─────────┘   └──────────┘ └───────────┘ └─────────┬─────────┘ └────────────┘
          ▲                                             │ uses
          │ used by                                     ▼
     ┌─────────┐   ┌───────────────┐            ┌───────────────┐
     │StateMach│   │ PIDController │            │  Kinematics   │◄── also used by
     └─────────┘   └───────────────┘            └───────────────┘    controller & parser edge
```

`Pins.h` is included by nearly everything and depends on nothing. `Kinematics`
is stateless (static methods) and is the shared owner of the counts↔mm scale.

---

## Data flow (one command, end to end)

```
 Host (USB Serial, mm / mm-min)
        │  "G1 X50 Y30 F1200\n"
        ▼
 PlotterController::pumpSerial ── line ──► GCodeParser::parseLine ──► GCodeCommand {mm}
        │                                                                     │
        │  Kinematics::mmToCounts                                             │
        ▼                                                                     ▼
 StateMachine.dispatch(MOVE_CMD) ─────────────────► TrajectoryPlanner::plan(start,target) [counts]
        │                                                                     │
        │            ── every control tick (cfg::CONTROL_PERIOD_MS) ──        │
        ▼                                                                     ▼
 Encoder::position() [counts] ──► [error] ◄── Kinematics::xyToAB( planner.setpoint() ) [A/B counts]
        │                            │
        ▼                            ▼
   (measurement)            PIDController::compute ──► MotorDriver::setSpeed ──► L298P ──► motors
                                                                                            │
        ▲───────────────────────── encoder feedback (ISR: Encoder::handleEdge) ────────────┘
```

Homing (`G28`): the FSM enters `HOMING`; the controller jogs each axis at
`cfg::HOMING_PWM` until the relevant `LimitSwitch::justPressed()` latches, then
zeroes that encoder and dispatches `HOMED` → back to `IDLE`. An **unexpected**
switch press during normal motion drives the FSM to `FAULT` and stops the motors.

---

## Layout

```
platformio.ini            build config (env:megaatmega2560)
ME306_plotter_pinout.md   hardware pin map + bring-up notes
src/
  Pins.h                  1  config (header only)
  Encoder.{h,cpp}         2
  LimitSwitch.{h,cpp}     3
  MotorDriver.{h,cpp}     4
  PIDController.{h,cpp}    5
  TrajectoryPlanner.{h,cpp} 6
  Kinematics.{h,cpp}      7
  GCodeParser.{h,cpp}     8
  StateMachine.{h,cpp}    9
  PlotterController.{h,cpp} 10
  main.ino                11  setup() / loop()
```

Build: `pio run` — targets the Mega. Upload: `pio run -t upload`.
Serial monitor: `pio device monitor` (115200 baud).

---

## Next steps (filling the stubs)

Suggested order, bottom-up so each layer can be bench-tested before the next:

1. `Pins.h` — set `PULLEY_CIRCUM_MM` and envelope from the real machine.
2. `Encoder` + ISR wiring → confirm counts change by hand (pinout §10.3).
3. `LimitSwitch` debounce → confirm each switch reads pressed.
4. `MotorDriver` → confirm directions against pinout §9, capped PWM.
5. `PIDController` → single-axis hold, then tune gains.
6. `Kinematics` + `TrajectoryPlanner` → coordinated straight-line moves.
7. `GCodeParser` + `StateMachine` + `PlotterController` → full G1/G28 path.
