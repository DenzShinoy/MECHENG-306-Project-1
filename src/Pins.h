#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 1 — Config / Pins  (header only)
// ---------------------------------------------------------------------
//  Single source of truth for every pin number and machine constant.
//  Nothing here allocates or executes; it is pure configuration so that
//  no other module hard-codes a pin or a scale factor. Values are drawn
//  from ME306_plotter_pinout.md.
//
//  Convention (see README): the control loop works in ENCODER COUNTS
//  (int32 / long). Millimetres appear only at the G-code boundary. The
//  counts<->mm scale lives here so both edges agree.
// =====================================================================

namespace pins {

// --- Motor shield, DFRobot L298P in PWM jumper mode -------------------
constexpr uint8_t M1_DIR = 4;  // Left motor direction
constexpr uint8_t M1_PWM = 5;  // Left motor PWM  (Timer3, E1)
constexpr uint8_t M2_PWM = 6;  // Right motor PWM (Timer4, E2)
constexpr uint8_t M2_DIR = 7;  // Right motor direction

// --- Encoders --------------------------------------------------------
constexpr uint8_t ENC_L_A = 2;   // Left  A — INT4, attachInterrupt
constexpr uint8_t ENC_L_B = 30;  // Left  B — plain digital, read in ISR
constexpr uint8_t ENC_R_A = 3;   // Right A — INT5, attachInterrupt
constexpr uint8_t ENC_R_B = 32;  // Right B — plain digital, read in ISR

// --- Limit switches, INPUT_PULLUP, active LOW ------------------------
constexpr uint8_t SW_TOP = 18;
constexpr uint8_t SW_BOTTOM = 19;
constexpr uint8_t SW_LEFT = 20;
constexpr uint8_t SW_RIGHT = 21;

}  // namespace pins

namespace cfg {

// --- Serial ----------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200;

// --- Encoder resolution ----------------------------------------------
//  2x decode (A-channel CHANGE) per the current wiring.
//  48 CPR * 171.79 gear / 2  -> counts per output-shaft revolution.
constexpr float MOTOR_CPR = 48.0f;  // encoder counts/rev, 4x max
constexpr float GEAR_RATIO = 171.79f;
constexpr uint8_t DECODE_FACTOR = 2;  // 4x decode in use
constexpr float COUNTS_PER_REV = (MOTOR_CPR * GEAR_RATIO) / static_cast<float>(4 / DECODE_FACTOR);

// --- Mechanics -------------------------------------------------------
//  TODO: set from the measured pulley pitch diameter / belt pitch.
//  counts_per_mm = COUNTS_PER_REV / (pulley circumference in mm)
constexpr float PULLEY_CIRCUM_MM =
    44.872f;  // calibrated: 50 mm cmd -> 56 mm measured
constexpr float COUNTS_PER_MM = COUNTS_PER_REV / PULLEY_CIRCUM_MM;

// --- Motion limits, in COUNTS (planner + PID work in counts) ---------
//  TODO: tune during bring-up.
constexpr float MAX_ACC_CPS2 = 5000.0f;  // counts per second^2

// Highest speed ONE motor can be asked to track and still hold its
// reference (bench-proven: the 40/-50 test ran its dominant motor at
// ~1990 counts/s and drew straight). G1 caps every move so the dominant
// motor never exceeds this — the axes stay in ratio and the line stays
// straight; a too-fast F slows down instead of bowing. Raise only after
// verifying straightness on the plotter at the new value.
constexpr float MAX_TRACK_CPS = 2000.0f;

// Hard ceiling on the commanded tool feed, mm/min. Any F above this is
// throttled down to it at the parser (see SendToController); G1 may slow
// a move further for straightness (see the guard in G1::execute).
constexpr float MAX_FEED_MM_PER_MIN = 1200.0f;

// "Close enough" band for declaring a move finished, in counts.
// ~10 counts ≈ 0.1 mm at the current scale. Tune during bring-up.
constexpr long POS_TOLERANCE_COUNTS = 5;

// How long G1 will chase the last few counts after its reference has
// arrived before calling the move done anyway. Belt stretch, backlash and
// stiction mean the tolerance band above is not always reachable, and
// without a bound the FSM sits in G1 hunting long after the machine has
// visibly stopped. The end position is read from the encoders either way,
// so finishing on the timeout does not lose track of where the tool is.
constexpr uint16_t SETTLE_TIMEOUT_MS = 300;

// --- PID default gains (per axis, position loop) ---------------------
//  TODO: tune. Output clamps to the PWM range below.
constexpr float PID_KP = 10.0f;
constexpr float PID_KI = 1.0f;
constexpr float PID_KD = 0.1f;

// --- Actuator limits -------------------------------------------------
//  Cap PWM during bring-up (supply 1.25 A, stall 2.2 A/motor).
constexpr int16_t PWM_LIMIT = 250;  // bring-up ceiling, raise once safe
constexpr int16_t PWM_HOLD = 60;
// --- Timing ----------------------------------------------------------
constexpr uint16_t CONTROL_PERIOD_MS = 2;  // ~500 Hz control loop
constexpr uint16_t DEBOUNCE_MS = 5;        // limit-switch debounce window

// --- Limit-switch electrical sense ------------------------------------
//  Level the pin sits at while the switch is PRESSED.
//  Measured on the bench (2026-08-26, pin-sweep sketch): with INPUT_PULLUP
//  enabled, all four pins D18-D21 idle LOW and go HIGH when a switch is
//  pressed. The switches are wired NORMALLY-CLOSED to GND: the closed
//  contact grounds the pin at rest, and pressing opens the contact so the
//  pullup takes the line HIGH. (Bonus: a broken wire reads as "pressed",
//  which fails safe.)
constexpr uint8_t SW_PRESSED_LEVEL = HIGH;

// Edge that corresponds to a press, derived from the level above so the
// ISRs and the polled debouncer can never disagree.
constexpr int SW_PRESS_EDGE = (SW_PRESSED_LEVEL == HIGH) ? RISING : FALLING;

// --- Limit-switch noise filter ---------------------------------------
//  On the 9 V motor supply, PWM noise couples into the switch harness and
//  fires the limit ISRs mid-G1. An ISR is therefore only a HINT: it opens
//  a confirmation window, and the fault latches only if the DEBOUNCED
//  LimitSwitch state confirms a real press inside that window. A glitch a
//  few microseconds wide can never hold the pin for DEBOUNCE_MS, so it is
//  discarded when the window expires.
constexpr uint16_t LIMIT_CONFIRM_MS = 25;  // must exceed DEBOUNCE_MS

}  // namespace cfg
