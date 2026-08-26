
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

#include "Encoder.h"
#include "LimitSwitch.h"
#include "MotorDriver.h"
#include "Pins.h"

#define HOMING_MOTOR_SPEED 10
#define HOMING_MOTOR_SPEED_REDUCTION 0.5f

namespace {
struct AxisHomingState {
  bool firstPassDone;
  bool secondPassDone;
  long measuredCounts;
};

bool homeAxis(MotorDriver& motor, LimitSwitch& firstSwitch,
              LimitSwitch& secondSwitch, Encoder& encoder,
              int16_t firstPassSpeed, int16_t secondPassSpeed,
              int16_t firstDirection, int16_t secondDirection,
              AxisHomingState& axisState) {
  if (!axisState.firstPassDone) {
    motor.setSpeed(firstDirection * firstPassSpeed);
    if (firstSwitch.justPressed()) {
      motor.stop();
      encoder.reset();
      axisState.firstPassDone = true;
    }
    return false;
  }

  if (!axisState.secondPassDone) {
    motor.setSpeed(secondDirection * secondPassSpeed);
    if (secondSwitch.justPressed()) {
      motor.stop();
      axisState.measuredCounts = encoder.position();
      axisState.secondPassDone = true;
      return true;
    }
  }

  return false;
}
}  // namespace

bool first_pass = true;
bool left_home = false;
bool right_home = false;
bool top_home = false;
bool bottom_home = false;

int current_motor_speed =
    first_pass
        ? HOMING_MOTOR_SPEED
        : static_cast<int>(HOMING_MOTOR_SPEED * HOMING_MOTOR_SPEED_REDUCTION);

void runHomingSequence(MotorDriver& motL, MotorDriver& motR, Encoder& encL,
                       Encoder& encR, LimitSwitch& swTop, LimitSwitch& swBottom,
                       LimitSwitch& swLeft, LimitSwitch& swRight) {
  static AxisHomingState xHoming{false, false, 0};
  static AxisHomingState yHoming{false, false, 0};
  static bool done = false;

  if (done) {
    return;
  }

  if (!xHoming.firstPassDone) {
    if (homeAxis(motL, swLeft, swRight, encL, cfg::HOMING_PWM,
                 cfg::HOMING_PWM / 2, -1, 1, xHoming)) {
      left_home = true;
      right_home = true;
    }
  }

  if (!yHoming.firstPassDone) {
    if (homeAxis(motR, swTop, swBottom, encR, cfg::HOMING_PWM,
                 cfg::HOMING_PWM / 2, -1, 1, yHoming)) {
      top_home = true;
      bottom_home = true;
    }
  }

  if (xHoming.secondPassDone && yHoming.secondPassDone) {
    done = true;
    Serial.print(F("X span counts: "));
    Serial.println(xHoming.measuredCounts);
    Serial.print(F("Y span counts: "));
    Serial.println(yHoming.measuredCounts);
  }
}
