#include "MotorDriver.h"

#include "Pins.h"

MotorDriver::MotorDriver(uint8_t dirPin, uint8_t pwmPin, bool invert)
    : _dirPin(dirPin), _pwmPin(pwmPin), _invert(invert) {}

void MotorDriver::begin() {
  pinMode(_dirPin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
  // Start stopped so nothing moves before the first command.
  digitalWrite(_dirPin, LOW);
  analogWrite(_pwmPin, 0);
}

void MotorDriver::setSpeed(int16_t speed) {
  // Re-enable the output pins before driving so later moves still work after
  // a stop() call.
  pinMode(_dirPin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);

  // Sign -> direction, magnitude -> PWM duty. _invert flips the
  // sense in software so a mirrored axis can be fixed without
  // rewiring (pinout §9).
  bool forward = (speed >= 0);
  if (_invert) {
    forward = !forward;
  }

  int16_t mag = (speed < 0) ? static_cast<int16_t>(-speed) : speed;
  if (mag > cfg::PWM_LIMIT) {
    mag = cfg::PWM_LIMIT;  // bring-up ceiling (supply vs stall current)
  }

  digitalWrite(_dirPin, forward ? HIGH : LOW);
  analogWrite(_pwmPin, mag);
}

void MotorDriver::stop() {
  // Coast: drop PWM, but keep the pins configured as outputs so the next
  // move can be commanded without reinitializing the driver.

  analogWrite(_pwmPin, 0);
}
