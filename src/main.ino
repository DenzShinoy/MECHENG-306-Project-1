#include <Arduino.h>
#include "Pins.h"
#include "Encoder.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Kinematics.h"
#include "Timer.h"

static Encoder encoderL(pins::ENC_L_A, pins::ENC_L_B);
static Encoder encoderR(pins::ENC_R_A, pins::ENC_R_B);

static MotorDriver motorL(pins::M1_DIR, pins::M1_PWM, false);
static MotorDriver motorR(pins::M2_DIR, pins::M2_PWM, false);

void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

void setup()
{
    encoderL.begin();
    encoderR.begin();
    attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A), isrEncoderL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A), isrEncoderR, CHANGE);
    motorL.begin();
    motorR.begin();
}

void getMaxSpeed(const AxisPair &target, int16_t &a, int16_t &b)
{
    // Scale so the LONGER move runs at PWM_LIMIT and the shorter one is
    // slowed proportionally — both axes finish together.
    const long absA = labs(target.a);
    const long absB = labs(target.b);

    if (absA == 0 && absB == 0)
    {
        a = 0;
        b = 0;
        return;
    } // no move

    if (absA >= absB)
    {
        a = cfg::PWM_LIMIT;
        b = static_cast<int16_t>((absB * cfg::PWM_LIMIT) / absA); // shorter axis scaled DOWN
    }
    else
    {
        b = cfg::PWM_LIMIT;
        a = static_cast<int16_t>((absA * cfg::PWM_LIMIT) / absB);
    }
}

void loop()
{
    long x = 10;
    long y = 10;
    long a = 0;
    long b = 0;

    Point targetPoint = {Kinematics::mmToCounts(x), Kinematics::mmToCounts(y)}; // Target position in counts (X, Y)
    AxisPair motorTarget = Kinematics::xyToAB(targetPoint);                     // Current position in counts (A, B)

    int16_t speedL = 0;
    int16_t speedR = 0;

    getMaxSpeed(motorTarget, speedL, speedR);

    PID pidL(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget.a, speedL);
    PID pidR(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, motorTarget.b, speedR);

    timing::Interval control(cfg::CONTROL_PERIOD_MS);

    bool moving = true;

    static unsigned long lastMicros = micros();

    while (moving)
    {
        if (control.ready())
        {
            long currentPosL = encoderL.position();
            long currentPosR = encoderR.position();

            long errL = motorTarget.a - currentPosL;
            long errR = motorTarget.b - currentPosR;

            // Done only when BOTH axes are within tolerance
            if (labs(errL) <= cfg::POS_TOLERANCE_COUNTS &&
                labs(errR) <= cfg::POS_TOLERANCE_COUNTS)
            {
                moving = false;
            }
            else
            {
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
    motorL.stop(); // fix #7b folded in: don't leave PWM driving
    motorR.stop(); // fix #7b folded in: don't leave loop running
}