#include <Arduino.h>

#include "Encoder.h"
#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "Timer.h"

// Set up encoder and motor driver objects with the correct pins. The encoder
// ISRs are wired in main() to call the handleEdge() method on each object.
static Encoder encoderL(pins::ENC_L_A, pins::ENC_L_B);
static Encoder encoderR(pins::ENC_R_A, pins::ENC_R_B);

static MotorDriver motorL(pins::M1_DIR, pins::M1_PWM, true);
static MotorDriver motorR(pins::M2_DIR, pins::M2_PWM, true);

void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);
  Serial.println(F("BOOT"));
  while (Serial.available() == 0) {
  }  // wait for any input
  while (Serial.available() > 0) Serial.read();  // clear buffer

  encoderL.begin();
  encoderR.begin();
  attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A), isrEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A), isrEncoderR, CHANGE);
  motorL.begin();
  motorR.begin();
}

void getMaxSpeed(const AxisPair& target, const AxisPair& current, int16_t& a,
                 int16_t& b) {
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

void loop() {  // Current position in counts (A, B)

  long x = -150;
  long y = 90;
  long a = 0;
  long b = 0;

  x *= -1;
  y *= -1;

  AxisPair currentPos = {a, b};
  Point targetPoint = {
      Kinematics::mmToCounts(x),
      Kinematics::mmToCounts(y)};  // Target position in counts (X, Y)
  AxisPair motorTarget =
      Kinematics::xyToAB(targetPoint);  // Current position in counts (A, B)

  int16_t speedL = 0;
  int16_t speedR = 0;

  bool moving = true;

  getMaxSpeed(motorTarget, currentPos, speedL, speedR);
  // long targetErrDiff = (motorTarget.a == 0 || motorTarget.b == 0) ? 0 :
  // motorTarget.a / motorTarget.b;

  PID pidL(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget.a, speedL);
  PID pidR(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget.b, speedR);

  timing::Interval control(cfg::CONTROL_PERIOD_MS);

  // Serial at 500 Hz would blow the control cadence — throttle to 10 Hz.
  timing::Interval report(100);

  moving = true;

  long startA = encoderL.position();
  long startB = encoderR.position();
  long dA = motorTarget.a - startA;
  long dB = motorTarget.b - startB;

  static unsigned long lastMicros = micros();

  const float FEED_CPS = 2000.0f;  // path speed, counts/s
  float len = sqrt((float)dA * dA + (float)dB * dB);
  unsigned long moveMs = (unsigned long)(1000.0f * len / FEED_CPS);
  unsigned long tStart = millis();

  while (moving) {
    if (control.ready()) {
      long currentPosL = encoderL.position();
      long currentPosR = encoderR.position();

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
        moving = false;
      } else {
        const unsigned long nowUs = micros();
        const double dt = (nowUs - lastMicros) * 1e-6;
        lastMicros = nowUs;

        speedL = lround(pidL.update(currentPosL, dt));
        speedR = lround(pidR.update(currentPosR, dt));

        motorL.setSpeed(speedL);
        motorR.setSpeed(speedR);
      }
    }
  }
  motorL.stop();  // fix #7b folded in: don't leave PWM driving
  motorR.stop();  // fix #7b folded in: don't leave loop running

  while (true) {
    // Do nothing, just stop the loop after completing the circle
  }
}