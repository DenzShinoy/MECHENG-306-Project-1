#include "MotorDriver.h"
#include "Pins.h"

MotorDriver::MotorDriver(uint8_t dirPin, uint8_t pwmPin, bool invert)
    : _dirPin(dirPin), _pwmPin(pwmPin), _invert(invert) {}

void MotorDriver::begin() {
  // TODO: pinMode both pins OUTPUT; write PWM 0 so we start stopped.
}

void MotorDriver::setSpeed(int16_t speed) {
  // TODO: apply _invert; digitalWrite(_dirPin) from the sign;
  // analogWrite(_pwmPin, clamp(|speed|, 0, cfg::PWM_LIMIT)).
  (void)speed;
}

void MotorDriver::stop() {
  // TODO: analogWrite(_pwmPin, 0).
}
