#include "G28.h"

#include <Arduino.h>

#include "Kinematics.h"
#include "MotorDriver.h"
#include "LimitSwitch.h"
#include "PID.h"
#include "Pins.h"
#include "Timer.h"
#include "manager.h"


G28::G28(
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
      manager_(manager)
{
}


void G28::execute()
{
    const uint32_t nowMs = millis();

    // Always update the debounced limit switch states.
    manager_.updateLimits(nowMs);


    // Initialise homing only once when G28 starts.
    if (phase_ == HomingPhase::IDLE)
    {
        encoderL_.reset();
        encoderR_.reset();

        phase_ = HomingPhase::SEEK_LEFT;
        lastControlMs_ = nowMs;

        return;
    }


    // Non-blocking control period.
    if ((nowMs - lastControlMs_) < cfg::CONTROL_PERIOD_MS)
    {
        return;
    }

    lastControlMs_ = nowMs;


    switch (phase_)
    {
        case HomingPhase::SEEK_LEFT:
        {
            if (manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::BACKOFF_LEFT;
            }
            else
            {
                // Move left.
                motorL_.setSpeed(250);
                motorR_.setSpeed(250);
            }

            break;
        }


        case HomingPhase::BACKOFF_LEFT:
        {
            if (!manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::SEEK_BOTTOM;
            }
            else
            {
                // Move slowly away from the left switch.
                motorL_.setSpeed(-100);
                motorR_.setSpeed(-100);
            }

            break;
        }


        case HomingPhase::SEEK_BOTTOM:
        {
            if (manager_.bottomPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::BACKOFF_BOTTOM;
            }
            else
            {
                // Move down.
                motorL_.setSpeed(250);
                motorR_.setSpeed(-250);
            }

            break;
        }


        case HomingPhase::BACKOFF_BOTTOM:
        {
            if (!manager_.bottomPressed())
            {
                motorL_.stop();
                motorR_.stop();

                encoderL_.reset();
                encoderR_.reset();

                manager_.resetXY();

                phase_ = HomingPhase::COMPLETE;
            }
            else
            {
                // Move slowly away from the bottom switch.
                motorL_.setSpeed(-100);
                motorR_.setSpeed(100);
            }

            break;
        }


        case HomingPhase::COMPLETE:
        {
            motorL_.stop();
            motorR_.stop();
            break;
        }


        case HomingPhase::IDLE:
        {
            break;
        }
    }
}


bool G28::isComplete() const
{
    return phase_ == HomingPhase::COMPLETE;
}


void G28::reset()
{
    motorL_.stop();
    motorR_.stop();

    phase_ = HomingPhase::IDLE;
    lastControlMs_ = 0;
}