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


// =====================================================================
// Expected limit switch for each homing phase
// ---------------------------------------------------------------------
// Limit-switch interrupts remain active during G28. The switch that is
// intentionally used by the current homing phase is allowed, while any
// other confirmed switch press is treated as a fault by main.cpp.
//
// Each axis uses four stages:
//   SEEK      - approach the switch quickly until first contact
//   BACKOFF   - move away until the switch releases
//   ENGAGE    - approach again slowly for a repeatable homing point
//   DISENGAGE - move clear of the switch before changing axis
// =====================================================================

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


// =====================================================================
// G28 homing sequence
// ---------------------------------------------------------------------
// Home X against the left switch first, then home Y against the bottom
// switch. Both axes use a fast first approach followed by a slow second
// engagement to improve repeatability.
//
// This state machine is non-blocking: execute() performs one check and
// motor update per call, then returns immediately to the main loop.
// Limit-switch debouncing and fault confirmation are handled outside
// this class.
// =====================================================================

void G28::execute()
{
    // Start homing by searching for the left limit.
    if (phase_ == HomingPhase::IDLE)
    {
        phase_ = HomingPhase::SEEK_LEFT;

        return;
    }

    switch (phase_)
    {
        // -------------------------------------------------------------
        // X-axis homing
        // -------------------------------------------------------------

        case HomingPhase::SEEK_LEFT:
        {
            // First approach: move quickly until the left switch is hit.
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
            // Move away from the first contact until the switch releases.
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
            // Re-approach at a lower speed for a more repeatable trigger
            // position than the initial high-speed contact.
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
            // Clear the left switch before starting Y-axis homing.
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


        // -------------------------------------------------------------
        // Y-axis homing
        // -------------------------------------------------------------

        case HomingPhase::SEEK_BOTTOM:
        {
            // First approach: move quickly until the bottom switch is hit.
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
            // Move away from the first contact until the switch releases.
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
            // Re-approach slowly to obtain a repeatable bottom reference.
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
            // Clear the bottom switch, then define this released position
            // as the machine origin for both encoder and Manager position.
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


        // -------------------------------------------------------------
        // Homing complete
        // -------------------------------------------------------------

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


// =====================================================================
// Motion interface
// =====================================================================

bool G28::isComplete() const
{
    return phase_ == HomingPhase::COMPLETE;
}


void G28::reset()
{
    // Stop both motors and return the homing state machine to its
    // initial state so a future G28 command can start from the beginning.
    motorL_.stop();
    motorR_.stop();

    phase_ = HomingPhase::IDLE;
}
