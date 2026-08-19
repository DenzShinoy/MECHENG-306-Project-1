#include "G1.h"

#include <Arduino.h>
#include <math.h>

#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "manager.h"
#include "updateVelocityProfile1.h"
#include "velocityEstimater.h"


// =====================================================================
// Constructor
// =====================================================================

G1::G1(
    MotorDriver& motorL,
    MotorDriver& motorR,
    Encoder& encoderL,
    Encoder& encoderR,
    Manager& manager
)
    : motorL_(motorL),
      motorR_(motorR),
      encoderL_(encoderL),
      encoderR_(encoderR),
      manager_(manager),
      pidL_(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, 0, 0),
      pidR_(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, 0, 0)
{
}


// =====================================================================
// Maximum PWM calculation
// =====================================================================

void G1::getMaxSpeed(
    const AxisPair& target,
    const AxisPair& current,
    int16_t& a,
    int16_t& b
)
{
    const long deltaA = labs(target.a - current.a);
    const long deltaB = labs(target.b - current.b);

    const long workingSpeed =
        cfg::PWM_LIMIT - cfg::PWM_HOLD;

    if (deltaA == 0 && deltaB == 0)
    {
        a = 0;
        b = 0;
        return;
    }

    if (deltaA >= deltaB)
    {
        a = cfg::PWM_LIMIT;

        b = static_cast<int16_t>((deltaB * workingSpeed) / deltaA) + cfg::PWM_HOLD;
    }
    else
    {
        b = cfg::PWM_LIMIT;

        a = static_cast<int16_t>((deltaA * workingSpeed) / deltaB) + cfg::PWM_HOLD;
    }
}


// =====================================================================
// Initialise one G1 move
// =====================================================================

void G1::beginMove(long target_x, long target_y)
{
    encoderL_.reset();
    encoderR_.reset();


    // Keep the existing coordinate sign convention.
    long x = -target_x;
    long y = -target_y;


    AxisPair currentPos = {0, 0};


    // Convert XY mm target to encoder counts.
    Point targetPoint = {
        Kinematics::mmToCounts(x),
        Kinematics::mmToCounts(y)
    };


    // Convert Cartesian XY target to CoreXY A/B target.
    motorTarget_ = Kinematics::xyToAB(targetPoint);


    getMaxSpeed(
        motorTarget_,
        currentPos,
        speedL_,
        speedR_
    );


    // Reinitialise PID controllers for the new move.
    pidL_ = PID(
        cfg::PID_KP,
        cfg::PID_KI,
        cfg::PID_KD,
        motorTarget_.a,
        speedL_
    );

    pidR_ = PID(
        cfg::PID_KP,
        cfg::PID_KI,
        cfg::PID_KD,
        motorTarget_.b,
        speedR_
    );


    startA_ = encoderL_.position();
    startB_ = encoderR_.position();

    previousLeftCount_ = startA_;
    previousRightCount_ = startB_;
    lastVelocityMicros_ = micros();

    dA_ = motorTarget_.a - startA_;
    dB_ = motorTarget_.b - startB_;


    pathLength_ =
        sqrtf(
            static_cast<float>(dA_) * static_cast<float>(dA_) +
            static_cast<float>(dB_) * static_cast<float>(dB_)
        );


    // Start the trapezoidal profile from rest.
    pathVelocity_ = 0.0f;
    s_ = 0.0f;


    lastControlMs_ = millis();
    lastMicros_ = micros();


    active_ = true;
    complete_ = false;
}


// =====================================================================
// Non-blocking G1 update
// =====================================================================

void G1::execute(long target_x, long target_y)
{
    // Initialise the move only on the first call.
    if (!active_ && !complete_)
    {
        beginMove(target_x, target_y);
        return;
    }


    if (!active_ || complete_)
    {
        return;
    }


    const uint32_t nowMs = millis();


    // Non-blocking control period.
    if ((nowMs - lastControlMs_) < cfg::CONTROL_PERIOD_MS)
    {
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

    const float FEED_CPS = 2000.0f;


    float remainingDistance = pathLength_ * (1.0f - s_);


    pathVelocity_ =
        updateVelocityProfile1(
            dt,
            FEED_CPS,
            cfg::MAX_ACC_CPS2,
            pathVelocity_,
            remainingDistance
        );


    // Integrate path velocity into path progress.
    if (pathLength_ > 0.0f)
    {
        s_ += (pathVelocity_ * dt) / pathLength_;
    }


    if (s_ > 1.0f)
    {
        s_ = 1.0f;
    }


    // Discrete integration can occasionally stop slightly short of s = 1.
    // Snap the reference to the final target once braking reaches zero.
    if (
        pathVelocity_ <= 0.0f &&
        remainingDistance > 0.0f &&
        s_ > 0.0f
    )
    {
        s_ = 1.0f;
    }


    // =================================================================
    // Moving position reference
    // =================================================================

    const long referenceL = startA_ + lroundf(s_ * dA_);

    const long referenceR =startB_ + lroundf(s_ * dB_);

    pidL_.setSetpoint(referenceL);
    pidR_.setSetpoint(referenceR);


    // =================================================================
    // PID
    // =================================================================

    speedL_ = lroundf(pidL_.update(currentPosL, dt));

    speedR_ = lroundf(pidR_.update(currentPosR, dt));

    motorL_.setSpeed(speedL_);
    motorR_.setSpeed(speedR_);


    // =====================================================
// Velocity CSV output
// =====================================================

const unsigned long velocityTime = micros();

if ((velocityTime - lastVelocityMicros_) >= 20000)
{
    const float velocityDt =
        (velocityTime - lastVelocityMicros_) * 1.0e-6f;

    const VelocityData velocity =
        estimateCoreXYVelocity(
            velocityDt,
            previousLeftCount_,
            currentPosL,
            previousRightCount_,
            currentPosR,
            cfg::COUNTS_PER_MM
        );

    previousLeftCount_ = currentPosL;
    previousRightCount_ = currentPosR;
    lastVelocityMicros_ = velocityTime;

    const float referenceVelocity =
        pathVelocity_ /
        (sqrtf(2.0f) * cfg::COUNTS_PER_MM);


    Serial.print(velocityTime * 1.0e-6f, 4);
    Serial.print(",");
    Serial.print(velocity.motor1, 4);
    Serial.print(",");
    Serial.print(velocity.motor2, 4);
    Serial.print(",");
    Serial.print(velocity.x, 4);
    Serial.print(",");
    Serial.print(velocity.y, 4);
    Serial.print(",");
    Serial.print(velocity.entity, 4);
    Serial.print(",");
    Serial.println(referenceVelocity, 4);
}


    // =================================================================
    // Final position check
    // =================================================================

    const long errL = motorTarget_.a - currentPosL;

    const long errR = motorTarget_.b - currentPosR;


    if (
        s_ >= 1.0f &&
        labs(errL) <= cfg::POS_TOLERANCE_COUNTS &&
        labs(errR) <= cfg::POS_TOLERANCE_COUNTS
    )
    {
        motorL_.stop();
        motorR_.stop();


        AxisPair currentAB = { currentPosL, currentPosR};


        Point currentXY = Kinematics::abToXY(currentAB);


        const long currentX = lroundf(Kinematics::countsToMm(currentXY.x));

        const long currentY = lroundf(Kinematics::countsToMm(currentXY.y));


        manager_.setCurrentPosition(currentX, currentY);


        active_ = false;
        complete_ = true;
    }
}


// =====================================================================
// Motion status
// =====================================================================

bool G1::isComplete() const
{
    return complete_;
}


// =====================================================================
// Reset for next command
// =====================================================================

void G1::reset()
{
    motorL_.stop();
    motorR_.stop();

    active_ = false;
    complete_ = false;

    pathVelocity_ = 0.0f;
    pathLength_ = 0.0f;
    s_ = 0.0f;

    speedL_ = 0;
    speedR_ = 0;

    lastControlMs_ = 0;
    lastMicros_ = 0;
}