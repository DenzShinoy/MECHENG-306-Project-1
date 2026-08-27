#include "G1.h"

#include <Arduino.h>
#include <math.h>

#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "manager.h"
#include "updateVelocityProfile1.h"

// =====================================================================
// Constructor
// =====================================================================

G1::G1(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
       Encoder& encoderR, Manager& manager)
    : motorL_(motorL),
      motorR_(motorR),
      encoderL_(encoderL),
      encoderR_(encoderR),
      manager_(manager),
      pidL_(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, 0, 0),
      pidR_(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, 0, 0) {}

// =====================================================================
// Maximum PWM calculation
// =====================================================================

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

// =====================================================================
// Initialise one G1 move
// =====================================================================

void G1::beginMove(long target_x, long target_y) {
  encoderL_.reset();
  encoderR_.reset();

  // Keep the existing coordinate sign convention.
  long x = -target_x;
  long y = -target_y;

  AxisPair currentPos = {0, 0};

  // Convert XY mm target to encoder counts.
  Point targetPoint = {Kinematics::mmToCounts(x), Kinematics::mmToCounts(y)};

  // Convert Cartesian XY target to CoreXY A/B target.
  motorTarget_ = Kinematics::xyToAB(targetPoint);

  // Per-move PWM ceilings: the dominant axis gets the full limit and the
  // other is scaled to it, so both finish together.
  getMaxSpeed(motorTarget_, currentPos, maxSpeedL_, maxSpeedR_);

  // Reinitialise the PID controllers for the new move.
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

  // A zero-length move has no path to follow: report it complete now
  // rather than waiting on a progress fraction that can never advance.
  if (pathLength_ <= 0.0f) {
    active_ = false;
    complete_ = true;
    return;
  }

  active_ = true;
  complete_ = false;
}

// =====================================================================
// Non-blocking G1 update
// =====================================================================

void G1::execute(long target_x, long target_y, long feed_rate) {
  // Initialise the move only on the first call.
  if (!active_ && !complete_) {
    beginMove(target_x, target_y);
    return;
  }

  if (!active_ || complete_) {
    return;
  }

  const uint32_t nowMs = millis();

  // Non-blocking control period.
  if ((nowMs - lastControlMs_) < cfg::CONTROL_PERIOD_MS) {
    return;
  }

  lastControlMs_ = nowMs;

  // Use one dt for both trajectory generation and PID.
  const unsigned long nowUs = micros();

  const float dt = (nowUs - lastMicros_) * 1.0e-6f;

  lastMicros_ = nowUs;

  const long currentPosL = encoderL_.position();

  const long currentPosR = encoderR_.position();

  // =================================================================
  // Trapezoidal reference trajectory
  // =================================================================

  // F is the true tool feed in mm/min. The A/B (motor-space) path is
  // sqrt(2) longer than the Cartesian path on a CoreXY, so the path
  // cruise speed must be sqrt(2) higher for the tool to move at F.
  float FEED_CPS = 1.41421356f * (static_cast<float>(feed_rate) / 60.0f) *
                   cfg::COUNTS_PER_MM;

  // Straightness guard: the line stays straight only while BOTH PIDs can
  // track their reference; once the dominant motor is asked for more
  // speed than it can deliver, it rails while the other keeps up and the
  // path bows. Cap the path cruise so the dominant motor never exceeds
  // cfg::MAX_TRACK_CPS — an over-fast F slows down instead of bending.
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

  // Integrate path velocity into path progress.
  if (pathLength_ > 0.0f) {
    s_ += (pathVelocity_ * dt) / pathLength_;
  }

  if (s_ > 1.0f) {
    s_ = 1.0f;
  }

  // Discrete integration can occasionally stop slightly short of s = 1.
  // Snap the reference to the final target once braking reaches zero.
  if (pathVelocity_ <= 0.0f && remainingDistance > 0.0f && s_ > 0.0f) {
    s_ = 1.0f;
  }

  // =================================================================
  // Moving position reference
  // =================================================================

  const long referenceL = startA_ + lroundf(s_ * dA_);

  const long referenceR = startB_ + lroundf(s_ * dB_);

  pidL_.setSetpoint(referenceL);
  pidR_.setSetpoint(referenceR);

  // =================================================================
  // PID
  // =================================================================

  speedL_ = lroundf(pidL_.update(currentPosL, dt));

  speedR_ = lroundf(pidR_.update(currentPosR, dt));

  motorL_.setSpeed(speedL_);
  motorR_.setSpeed(speedR_);

  // =================================================================
  // Final position check
  // =================================================================

  const long errL = motorTarget_.a - currentPosL;

  const long errR = motorTarget_.b - currentPosR;

  // The reference has arrived; what is left is pure regulation. Each PID's
  // integral currently holds the drive the cruise needed to sustain speed
  // — feedforward by another name — and that term is now wrong. It scales
  // with how long the move ran, so on a long move unwinding it against a
  // few counts of error kept G1 hunting for seconds after the machine had
  // stopped. Zero it once, on entry to the settle.
  if (s_ >= 1.0f && !settling_) {
    settling_ = true;
    settleStartMs_ = nowMs;

    pidL_.resetIntegral();
    pidR_.resetIntegral();
  }

  const bool inTolerance = labs(errL) <= cfg::POS_TOLERANCE_COUNTS &&
                           labs(errR) <= cfg::POS_TOLERANCE_COUNTS;

  // Give up chasing the last counts rather than stalling the FSM: the
  // position recorded below comes from the encoders, so a move that ends
  // on the timeout still leaves the machine's idea of where it is correct.
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

// =====================================================================
// Motion status
// =====================================================================

bool G1::isComplete() const { return complete_; }

// =====================================================================
// Reset for next command
// =====================================================================

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