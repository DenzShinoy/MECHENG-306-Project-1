#pragma once
#include <Arduino.h>

// Every pin number and machine constant lives in here and nowhere else,
// so nothing downstream ever hard-codes a pin or a scale factor. The pin
// numbers come from ME306_plotter_pinout.md.
//
// Everything past the parser works in encoder counts, not mm. The
// counts/mm scale is in here too, so both ends agree on it.

namespace pins {

// Motor shield: DFRobot L298P, PWM jumper mode.
constexpr uint8_t M1_DIR = 4;  // Left motor direction
constexpr uint8_t M1_PWM = 5;  // Left motor PWM  (Timer3, E1)
constexpr uint8_t M2_PWM = 6;  // Right motor PWM (Timer4, E2)
constexpr uint8_t M2_DIR = 7;  // Right motor direction

// Encoders.
constexpr uint8_t ENC_L_A = 2;   // left A, INT4, gets the interrupt
constexpr uint8_t ENC_L_B = 30;  // left B, plain digital, read in the ISR
constexpr uint8_t ENC_R_A = 3;   // right A, INT5, gets the interrupt
constexpr uint8_t ENC_R_B = 32;  // right B, plain digital, read in the ISR

// Limit switches. All four run on INPUT_PULLUP.
constexpr uint8_t SW_TOP = 18;
constexpr uint8_t SW_BOTTOM = 19;
constexpr uint8_t SW_LEFT = 20;
constexpr uint8_t SW_RIGHT = 21;

}  // namespace pins

namespace cfg {

// Serial.
constexpr uint32_t SERIAL_BAUD = 115200;

// Encoder resolution. We only watch the A channel (on CHANGE), so we see
// 2 edges per cycle instead of 4.
// 48 CPR * 171.79 gearbox / 2 = counts per turn of the output shaft.
constexpr float MOTOR_CPR = 48.0f;  // encoder counts/rev, 4x max
constexpr float GEAR_RATIO = 171.79f;
constexpr uint8_t DECODE_FACTOR = 2;  // A channel only, so 2 not 4
constexpr float COUNTS_PER_REV = (MOTOR_CPR * GEAR_RATIO) / static_cast<float>(4 / DECODE_FACTOR);

// Mechanics. counts_per_mm = COUNTS_PER_REV / pulley circumference.
// The circumference below is not the CAD number. We asked for 50 mm, got
// 56 mm on the ruler, and scaled it until it matched.
constexpr float PULLEY_CIRCUM_MM = 44.872f;
constexpr float COUNTS_PER_MM = COUNTS_PER_REV / PULLEY_CIRCUM_MM;

// Motion limits, in counts, since the planner and the PIDs both work in
// counts.
constexpr float MAX_ACC_CPS2 = 5000.0f;  // counts per second^2

// Fastest we can drive one motor and still have it keep up with its
// setpoint. The 40/-50 test ran its faster motor at about 1990 counts/s
// and came out straight, which is where the number is from. G1 caps every
// move against it, so too big an F just makes the move slower instead of
// bending the line. Don't raise it without redrawing that test and
// checking the line is still straight.
constexpr float MAX_TRACK_CPS = 2000.0f;

// Ceiling on the commanded feed, mm/min. The parser clamps anything
// bigger (SendToController). G1 can still slow a move further than this
// on its own.
constexpr float MAX_FEED_MM_PER_MIN = 1000.0f;

// Close enough to call a move finished, in counts. Roughly 0.05 mm at
// the current scale.
constexpr long POS_TOLERANCE_COUNTS = 5;

// How long G1 keeps chasing the last few counts before giving up and
// calling the move done anyway. Between belt stretch, backlash and
// stiction the band above isn't always reachable, and without this the
// FSM sat in G1 twitching long after the machine had visibly stopped.
// The end position gets read off the encoders either way, so giving up
// early doesn't lose track of where the pen is.
constexpr uint16_t SETTLE_TIMEOUT_MS = 300;

// PID gains. Same set for both axes, position loop. The output gets
// clamped to the PWM range below.
constexpr float PID_KP = 10.0f;
constexpr float PID_KI = 1.0f;
constexpr float PID_KD = 0.1f;

// Actuator limits. PWM is capped because the supply only gives 1.25 A and
// each motor pulls 2.2 A at stall.
constexpr int16_t PWM_LIMIT = 250;
constexpr int16_t PWM_HOLD = 60;  // floor for the slower axis of a move
// Timing.
constexpr uint16_t CONTROL_PERIOD_MS = 2;  // ~500 Hz control loop
constexpr uint16_t DEBOUNCE_MS = 5;        // limit-switch debounce window

// What the pin actually reads while a switch is PRESSED.
// Checked on the bench with a pin-sweep sketch (26/08): with the pullups
// on, D18-D21 all idle LOW and go HIGH when you push a switch. They're
// wired normally-closed to GND, so the contact grounds the pin at rest,
// and pressing it opens the circuit and lets the pullup win. Handy side
// effect: a broken wire also reads as pressed, which is the safe way
// round to get it wrong.
constexpr uint8_t SW_PRESSED_LEVEL = HIGH;

// The press edge, worked out from the level above rather than written
// down separately, so the ISRs and the debouncer can't drift apart.
constexpr int SW_PRESS_EDGE = (SW_PRESSED_LEVEL == HIGH) ? RISING : FALLING;

// Noise filter. On the 9 V supply the motor PWM couples into the switch
// loom and sets the limit ISRs off in the middle of a G1. So an ISR is
// only ever a hint: it opens a window, and the fault only latches if the
// debounced state agrees inside that window. A glitch a few microseconds
// wide can't hold the pin for DEBOUNCE_MS, so it gets binned.
constexpr uint16_t LIMIT_CONFIRM_MS = 25;  // must exceed DEBOUNCE_MS

}  // namespace cfg
