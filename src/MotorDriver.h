#pragma once
#include <Arduino.h>

// One channel of the L298P shield: a direction pin and a PWM pin, with
// the shield in PWM jumper mode. Takes a signed number and splits it into
// a direction level and a duty cycle. It has no idea about control loops
// or kinematics, it just writes the two pins. The invert flag flips the
// direction in software, so an axis wired backwards can be fixed without
// pulling wires out (pinout §9).

class MotorDriver {
public:
  MotorDriver(uint8_t dirPin, uint8_t pwmPin, bool invert = false);

  // Pin modes, and make sure we start stopped. Call from setup().
  void begin();

  // The sign picks the direction, the magnitude is the duty. Magnitude
  // gets clamped to cfg::PWM_LIMIT in here, so callers don't have to.
  void setSpeed(int16_t speed);

  // Coast. Just drops the PWM to 0, doesn't brake.
  void stop();

private:
  const uint8_t _dirPin;
  const uint8_t _pwmPin;
  const bool    _invert;
};
