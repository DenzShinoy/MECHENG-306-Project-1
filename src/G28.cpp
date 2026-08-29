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


// The limit ISRs stay live through G28, so main.ino needs to know which
// switch we're allowed to be pressing. Anything else that gets confirmed
// while we're in here is a genuine fault.

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


// Left switch first for X, then the bottom one for Y. Both go in fast,
// back off, then creep back in, which is what makes the trigger point
// repeatable.
//
// Non-blocking: one check and one motor write per call, then back to the
// main loop. Debouncing and fault confirmation happen elsewhere.

void G28::execute()
{
    // First call in just picks the starting phase.
    if (phase_ == HomingPhase::IDLE)
    {
        phase_ = HomingPhase::SEEK_LEFT;

        return;
    }

    switch (phase_)
    {
        // X axis

        case HomingPhase::SEEK_LEFT:
        {
            // Fast run at the left switch.
            if (manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::BACKOFF_LEFT;
            }
            else
            {
                motorL_.setSpeed(250);
                motorR_.setSpeed(250);
            }

            break;
        }


        case HomingPhase::BACKOFF_LEFT:
        {
            // Back off until it lets go.
            if (!manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::ENGAGE_LEFT;
            }
            else
            {
                motorL_.setSpeed(-100);
                motorR_.setSpeed(-100);
            }

            break;
        }


        case HomingPhase::ENGAGE_LEFT:
        {
            // Creep back in. This second contact is the one we trust.
            if (manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::DISENGAGE_LEFT;
            }
            else
            {
                motorL_.setSpeed(80);
                motorR_.setSpeed(80);
            }

            break;
        }


        case HomingPhase::DISENGAGE_LEFT:
        {
            // Get off the switch before we start on Y.
            if (!manager_.leftPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::SEEK_BOTTOM;
            }
            else
            {
                motorL_.setSpeed(-55);
                motorR_.setSpeed(-55);
            }

            break;
        }


        // Y axis

        case HomingPhase::SEEK_BOTTOM:
        {
            // Same again on Y. Fast run at the bottom switch.
            if (manager_.bottomPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::BACKOFF_BOTTOM;
            }
            else
            {
                motorL_.setSpeed(250);
                motorR_.setSpeed(-250);
            }

            break;
        }


        case HomingPhase::BACKOFF_BOTTOM:
        {
            // Back off until it lets go.
            if (!manager_.bottomPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::ENGAGE_BOTTOM;
            }
            else
            {
                motorL_.setSpeed(-100);
                motorR_.setSpeed(100);
            }

            break;
        }


        case HomingPhase::ENGAGE_BOTTOM:
        {
            // Slow approach again, for the reference we actually keep.
            if (manager_.bottomPressed())
            {
                motorL_.stop();
                motorR_.stop();

                phase_ = HomingPhase::DISENGAGE_BOTTOM;
            }
            else
            {
                motorL_.setSpeed(50);
                motorR_.setSpeed(-50);
            }

            break;
        }


        case HomingPhase::DISENGAGE_BOTTOM:
        {
            // Pull clear, and call wherever we end up the origin, for both
            // the encoders and the Manager's idea of where we are.
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
                motorL_.setSpeed(-50);
                motorR_.setSpeed(50);
            }

            break;
        }


        case HomingPhase::COMPLETE:
        {
            // Nothing to do but sit still.
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
    // Motors off and back to IDLE, so the next G28 starts from scratch.
    motorL_.stop();
    motorR_.stop();

    phase_ = HomingPhase::IDLE;
}
