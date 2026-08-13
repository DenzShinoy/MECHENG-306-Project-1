#include "G28.h"

#include <Arduino.h>

#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "Timer.h"
#include "manager.h"

// Constructor for the G1 class, initializing motor drivers and encoders
G28::G28(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
       Encoder& encoderR, Manager& manager)
    : motorL_(motorL),
      motorR_(motorR),
      encoderL_(encoderL),
      encoderR_(encoderR),
      manager_(manager) {}

// Calculate the maximum speed for each axis based on the target and current
// positions
void G28::getMaxSpeed(const AxisPair& target, const AxisPair& current,
                     int16_t& a, int16_t& b) {
  // Scale so the LONGER move runs at PWM_LIMIT and the shorter one is
  // slowed proportionally — both axes finish together.

  const long deltaA = labs(target.a - current.a);
  const long deltaB = labs(target.b - current.b);
  const long workingSpeed =
      cfg::PWM_LIMIT -
      cfg::PWM_HOLD;  // leave a little headroom for PID overshoot

  if (deltaA == 0 && deltaB == 0) {
    a = 0;
    b = 0;
    return;
  }  // no move

  if (deltaA >= deltaB) {
    a = cfg::PWM_LIMIT;
    b = static_cast<int16_t>((deltaB * workingSpeed) / deltaA) +
        cfg::PWM_HOLD;  // shorter axis scaled DOWN
  } else {
    b = cfg::PWM_LIMIT;
    a = static_cast<int16_t>((deltaA * workingSpeed) / deltaB) +
        cfg::PWM_HOLD;  // shorter axis scaled UP
  }
}


// Execute a G28 command to move to the specified target position (target_x,
// target_y)
void G28::execute(long target_x, long target_y) {
  encoderL_.reset();
  encoderR_.reset();

  long x = target_x;
  long y = target_y;
  long a = 0;
  long b = 0;

  x *= -1;
  y *= -1;

  AxisPair currentPos = {a, b};  // Current position in counts (A, B)

  // Convert target position from mm to counts and then to motor coordinates (A,
  // B)
  Point targetPoint = {
      Kinematics::mmToCounts(x),
      Kinematics::mmToCounts(y)};  // Target position in counts (X, Y)
  // Convert target position from counts (X, Y) to motor coordinates (A, B)
  AxisPair motorTarget =
      Kinematics::xyToAB(targetPoint);  // Current position in counts (A, B)

  int16_t speedL = 0;
  int16_t speedR = 0;

  bool moving = true;  // Flag to indicate if the motors are still moving

  // Calculate the maximum speed for each axis based on the target and current
  getMaxSpeed(motorTarget, currentPos, speedL, speedR);

  // Initialize PID controllers for each motor with the calculated target
  // positions and speeds
  PID pidL(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget.a, speedL);
  PID pidR(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget.b, speedR);

  timing::Interval control(cfg::CONTROL_PERIOD_MS);

  // Serial at 500 Hz would blow the control cadence — throttle to 10 Hz.
  timing::Interval report(100);

  moving = true;

  long startA = encoderL_.position();
  long startB = encoderR_.position();
  long dA = motorTarget.a - startA;
  long dB = motorTarget.b - startB;

  static unsigned long lastMicros = micros();

  const float FEED_CPS = 2000.0f;  // path speed, counts/s
  float len = sqrt((float)dA * dA + (float)dB * dB);
  unsigned long moveMs = (unsigned long)(1000.0f * len / FEED_CPS);
  unsigned long tStart = millis();

  while (moving) {
    if (control.ready()) {
      long currentPosL = encoderL_.position();
      long currentPosR = encoderR_.position();

      long errL = motorTarget.a - currentPosL;
      long errR = motorTarget.b - currentPosR;

      float s = (float)(millis() - tStart) / moveMs;
      if (s > 1.0f) s = 1.0f;
      pidL.setSetpoint(startA + lround(s * dA));
      pidR.setSetpoint(startB + lround(s * dB));

      if (report.ready()) {
        Serial.print(F("seg "));
        Serial.print(F(" | L cur "));
        Serial.print(currentPosL);
        Serial.print(F(" tgt "));
        Serial.print(motorTarget.a);
        Serial.print(F(" err "));
        Serial.print(errL);
        Serial.print(F(" | R cur "));
        Serial.print(currentPosR);
        Serial.print(F(" tgt "));
        Serial.print(motorTarget.b);
        Serial.print(F(" err "));
        Serial.print(errR);
        Serial.print(F(" | pwm "));
        Serial.print(speedL);
        Serial.print(',');
        Serial.println(speedR);
      }

      // Done only when BOTH axes are within tolerance
      if (labs(errL) <= cfg::POS_TOLERANCE_COUNTS &&
          labs(errR) <= cfg::POS_TOLERANCE_COUNTS) {

        /*AxisPair currentAB = {currentPosL, currentPosR};
        Point currentXY = Kinematics::abToXY(currentAB);
        long currentX = Kinematics::countsToMm(currentXY.x);
        long currentY = Kinematics::countsToMm(currentXY.y);*/
        manager_.resetXY();
        
        moving = false;
      } else {
        const unsigned long nowUs = micros();
        const double dt = (nowUs - lastMicros) * 1e-6;
        lastMicros = nowUs;

        speedL = lround(pidL.update(currentPosL, dt));
        speedR = lround(pidR.update(currentPosR, dt));

        motorL_.setSpeed(speedL);
        motorR_.setSpeed(speedR);
      }
    }
  }
  motorL_.stop();  // fix #7b folded in: don't leave PWM driving
  motorR_.stop();  // fix #7b folded in: don't leave loop running

}
