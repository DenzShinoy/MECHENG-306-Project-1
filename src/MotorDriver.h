#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 4 — MotorDriver
// ---------------------------------------------------------------------
//  Thin wrapper over one channel of the L298P shield (dir pin + PWM pin,
//  PWM jumper mode). Turns a signed command into a direction level and a
//  PWM magnitude. Knows nothing about control or kinematics — it just
//  drives the two pins. The `invert` flag flips direction in software so
//  a mirrored axis can be corrected without rewiring (see pinout §9).
// =====================================================================

class MotorDriver {
public:
  MotorDriver(uint8_t dirPin, uint8_t pwmPin, bool invert = false);

  // Configure pin modes and force a stopped state. Call from setup().
  void begin();

  // Signed command: sign -> direction, magnitude -> PWM duty.
  // Magnitude is clamped to [0, cfg::PWM_LIMIT] inside the wrapper.
  void setSpeed(int16_t speed);

  // Coast to stop (PWM = 0).
  void stop();

private:
  const uint8_t _dirPin;
  const uint8_t _pwmPin;
  const bool    _invert;
};
