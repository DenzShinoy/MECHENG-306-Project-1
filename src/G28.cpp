#include "G28.h"

#include <Arduino.h>

#include "MotorDriver.h"
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

bool G28::isExpectedLimit(LimitId id) const
{
    switch (phase_)
    {
        case HomingPhase::SEEK_LEFT:
        case HomingPhase::BACKOFF_LEFT:
        case HomingPhase::ENGAGE_LEFT:
        case HomingPhase::DISENGAGE_LEFT:
            return id == LimitId::LEFT;

        case HomingPhase::SEEK_BOTTOM:
        case HomingPhase::BACKOFF_BOTTOM:
        case HomingPhase::ENGAGE_BOTTOM:
        case HomingPhase::DISENGAGE_BOTTOM:
            return id == LimitId::BOTTOM;

        default:
            return false;
    }
}

void G28::execute()
{
    // Initialise homing only once when G28 starts.
    if (phase_ == HomingPhase::IDLE)
    {
        phase_ = HomingPhase::SEEK_LEFT;

        return;
    }

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

        case HomingPhase::ENGAGE_LEFT:
        {
            // normal operation
            if (manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::DISENGAGE_LEFT;
            }
            else
            {
                // Move slowly away from the left switch.
                motorL_.setSpeed(80);
                motorR_.setSpeed(80);
            }

            break;
        }


        case HomingPhase::BACKOFF_LEFT:
        {
            // normal operation
            if (!manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::ENGAGE_LEFT;
            }
            else
            {
                // Move slowly away from the left switch.
                motorL_.setSpeed(-100);
                motorR_.setSpeed(-100);
            }

            break;
        }

        case HomingPhase::DISENGAGE_LEFT:
        {
            // normal operation
            if (!manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::SEEK_BOTTOM;
            }
            else
            {
                // Move slowly away from the left switch.
                motorL_.setSpeed(-55);
                motorR_.setSpeed(-55);
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
            // phase 4 bottom switch polled
            if (!manager_.bottomPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::ENGAGE_BOTTOM;
            }
            else
            {
                // Move slowly away from the bottom switch.
                motorL_.setSpeed(-100);
                motorR_.setSpeed(100);
            }

            break;
        }

        case HomingPhase::ENGAGE_BOTTOM:
        {
            // normal operation
            if (manager_.bottomPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::DISENGAGE_BOTTOM;
            }
            else
            {
                // Move slowly to the bottom switch.
                motorL_.setSpeed(50);
                motorR_.setSpeed(-50);
            }

            break;
        }

        case HomingPhase::DISENGAGE_BOTTOM:
        {
            // normal operation
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
                // Move slowly to the bottom switch.
                motorL_.setSpeed(-50);
                motorR_.setSpeed(50);
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
}
