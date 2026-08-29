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
  // Set the pins up again before driving. Cheap insurance so a move after
  // a stop() still goes.
  pinMode(_dirPin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);

  // _invert flips the sense in software, for when an axis turns out to be
  // wired backwards (pinout §9).
  bool forward = (speed >= 0);
  if (_invert) {
    forward = !forward;
  }

  int16_t mag = (speed < 0) ? static_cast<int16_t>(-speed) : speed;
  if (mag > cfg::PWM_LIMIT) {
    mag = cfg::PWM_LIMIT;  // the supply can't feed both motors at stall
  }

  digitalWrite(_dirPin, forward ? HIGH : LOW);
  analogWrite(_pwmPin, mag);
}

void MotorDriver::stop() {
  // Coast rather than brake. The pins stay outputs so the next move
  // doesn't have to set the driver up again.
  analogWrite(_pwmPin, 0);
}
