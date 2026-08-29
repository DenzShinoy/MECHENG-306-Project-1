#include "G1.h"

#include <Arduino.h>
#include <math.h>

#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "manager.h"
#include "updateVelocityProfile1.h"

G1::G1(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
       Encoder& encoderR, Manager& manager)
    : motorL_(motorL),
      motorR_(motorR),
      encoderL_(encoderL),
      encoderR_(encoderR),
      manager_(manager),
      pidL_(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, 0, 0),
      pidR_(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, 0, 0) {}

// Per-move PWM ceilings. Whichever motor has further to go gets the full
// limit and the other is scaled down in proportion, so the two finish
// together instead of one arriving early and waiting around.

void G1::getMaxSpeed(const AxisPair& target, const AxisPair& current,
                     int16_t& a, int16_t& b) {
  const long deltaA = labs(target.a - current.a);
  const long deltaB = labs(target.b - current.b);

  const long workingSpeed = cfg::PWM_LIMIT - cfg::PWM_HOLD;

  if (deltaA == 0 && deltaB == 0) {
    a = 0;
    b = 0;
    return;
  }

  if (deltaA >= deltaB) {
    a = cfg::PWM_LIMIT;

    b = static_cast<int16_t>((deltaB * workingSpeed) / deltaA) + cfg::PWM_HOLD;
  } else {
    b = cfg::PWM_LIMIT;

    a = static_cast<int16_t>((deltaA * workingSpeed) / deltaB) + cfg::PWM_HOLD;
  }
}

// Set up one move.

void G1::beginMove(long target_x, long target_y) {
  encoderL_.reset();
  encoderR_.reset();

  // Both axes get negated here, and again when we write the position back
  // at the end. The machine runs the opposite way round to the G-code.
  long x = -target_x;
  long y = -target_y;

  AxisPair currentPos = {0, 0};

  // Convert XY mm target to encoder counts.
  Point targetPoint = {Kinematics::mmToCounts(x), Kinematics::mmToCounts(y)};

  // ...then out of Cartesian and into CoreXY motor space.
  motorTarget_ = Kinematics::xyToAB(targetPoint);

  // Work out this move's PWM ceilings (see getMaxSpeed above).
  getMaxSpeed(motorTarget_, currentPos, maxSpeedL_, maxSpeedR_);

  // Fresh PIDs for the new move.
  pidL_ =
      PID(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget_.a, maxSpeedL_);

  pidR_ =
      PID(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget_.b, maxSpeedR_);

  startA_ = encoderL_.position();
  startB_ = encoderR_.position();

  dA_ = motorTarget_.a - startA_;
  dB_ = motorTarget_.b - startB_;

  pathLength_ = sqrtf(static_cast<float>(dA_) * static_cast<float>(dA_) +
                      static_cast<float>(dB_) * static_cast<float>(dB_));

  // Start the trapezoidal profile from rest.
  pathVelocity_ = 0.0f;
  s_ = 0.0f;

  settling_ = false;
  settleStartMs_ = 0;

  lastControlMs_ = millis();
  lastMicros_ = micros();

  // Nothing to move. Call it done now rather than waiting on a progress
  // fraction that is never going to get anywhere.
  if (pathLength_ <= 0.0f) {
    active_ = false;
    complete_ = true;
    return;
  }

  active_ = true;
  complete_ = false;
}

// One tick of the move. Never blocks.

void G1::execute(long target_x, long target_y, long feed_rate) {
  // First call just sets the move up, then we come back next tick.
  if (!active_ && !complete_) {
    beginMove(target_x, target_y);
    return;
  }

  if (!active_ || complete_) {
    return;
  }

  const uint32_t nowMs = millis();

  // Only run the loop every CONTROL_PERIOD_MS. No delay() anywhere.
  if ((nowMs - lastControlMs_) < cfg::CONTROL_PERIOD_MS) {
    return;
  }

  lastControlMs_ = nowMs;

  // Same dt for the profile and the PIDs.
  const unsigned long nowUs = micros();

  const float dt = (nowUs - lastMicros_) * 1.0e-6f;

  lastMicros_ = nowUs;

  const long currentPosL = encoderL_.position();

  const long currentPosR = encoderR_.position();

  // Trapezoidal reference trajectory.
  //
  // F is the feed at the pen, in mm/min. On a CoreXY the A/B path is
  // sqrt(2) longer than the Cartesian one, so the path speed has to be
  // sqrt(2) higher for the pen itself to move at F.
  float FEED_CPS = 1.41421356f * (static_cast<float>(feed_rate) / 60.0f) *
                   cfg::COUNTS_PER_MM;

  // The line only stays straight while both PIDs are keeping up. Ask the
  // faster motor for more than it can do and it rails while the other one
  // tracks fine, and the line bows out. So cap the path speed to keep the
  // faster motor under cfg::MAX_TRACK_CPS: too big an F just runs the
  // move slower instead of bending it.
  const float domCounts =
      fmaxf(fabsf(static_cast<float>(dA_)), fabsf(static_cast<float>(dB_)));
  if (domCounts > 0.0f) {
    const float maxPathVel = cfg::MAX_TRACK_CPS * pathLength_ / domCounts;
    if (FEED_CPS > maxPathVel) {
      FEED_CPS = maxPathVel;
    }
  }

  float remainingDistance = pathLength_ * (1.0f - s_);

  pathVelocity_ = updateVelocityProfile1(dt, FEED_CPS, cfg::MAX_ACC_CPS2,
                                         pathVelocity_, remainingDistance);

  // Turn speed into progress along the path.
  if (pathLength_ > 0.0f) {
    s_ += (pathVelocity_ * dt) / pathLength_;
  }

  if (s_ > 1.0f) {
    s_ = 1.0f;
  }

  // The integration sometimes stalls a hair short of s = 1. Once we've
  // braked all the way to a stop, just snap it to the end.
  if (pathVelocity_ <= 0.0f && remainingDistance > 0.0f && s_ > 0.0f) {
    s_ = 1.0f;
  }

  // Where the reference sits right now, in motor counts.
  const long referenceL = startA_ + lroundf(s_ * dA_);

  const long referenceR = startB_ + lroundf(s_ * dB_);

  pidL_.setSetpoint(referenceL);
  pidR_.setSetpoint(referenceR);

  speedL_ = lroundf(pidL_.update(currentPosL, dt));

  speedR_ = lroundf(pidR_.update(currentPosR, dt));

  motorL_.setSpeed(speedL_);
  motorR_.setSpeed(speedR_);

  const long errL = motorTarget_.a - currentPosL;

  const long errR = motorTarget_.b - currentPosR;

  // Reference is at the end, so from here it's just regulation. Each PID's
  // integral is holding the drive that kept the cruise going, which is
  // feedforward under another name, and it's now wrong. It grows with how
  // long the move ran, so on a long move unwinding it against a few counts
  // of error kept G1 twitching for seconds after the machine had visibly
  // stopped. Dump it once, on the way in.
  if (s_ >= 1.0f && !settling_) {
    settling_ = true;
    settleStartMs_ = nowMs;

    pidL_.resetIntegral();
    pidR_.resetIntegral();
  }

  const bool inTolerance = labs(errL) <= cfg::POS_TOLERANCE_COUNTS &&
                           labs(errR) <= cfg::POS_TOLERANCE_COUNTS;

  // Stop chasing the last few counts rather than jam up the FSM. The
  // position below is read off the encoders either way, so ending on the
  // timeout doesn't lose track of where we actually are.
  const bool settleExpired =
      settling_ && (nowMs - settleStartMs_) >= cfg::SETTLE_TIMEOUT_MS;

  if (settling_ && (inTolerance || settleExpired)) {
    motorL_.stop();
    motorR_.stop();

    AxisPair currentAB = {currentPosL, currentPosR};

    Point currentXY = Kinematics::abToXY(currentAB);

    const long currentX = lroundf(Kinematics::countsToMm(currentXY.x));

    const long currentY = lroundf(Kinematics::countsToMm(currentXY.y));

    manager_.setCurrentPosition(-currentX, -currentY);

    active_ = false;
    complete_ = true;
  }
}

bool G1::isComplete() const { return complete_; }

// Back to a clean slate for the next command.

void G1::reset() {
  motorL_.stop();
  motorR_.stop();

  active_ = false;
  complete_ = false;

  pathVelocity_ = 0.0f;
  pathLength_ = 0.0f;
  s_ = 0.0f;

  settling_ = false;
  settleStartMs_ = 0;

  speedL_ = 0;
  speedR_ = 0;

  lastControlMs_ = 0;
  lastMicros_ = 0;
}