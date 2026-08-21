#include <math.h>
#include "updateVelocityProfile1.h"

static MotionPhase currentPhase = MotionPhase::ACCEL;

void resetVelocityProfile1()
{
    currentPhase = MotionPhase::ACCEL;
}

float updateVelocityProfile1(
    float dt, float cruiseSpeed,
    float acceleration,
    float pathVelocity,
    float remainingDistance
){
    if (dt <= 0.0f || cruiseSpeed < 0.0f || acceleration <= 0.0f)
    {
        return 0.0f;
    }

    if (remainingDistance < 0.0f)
    {
        remainingDistance = 0.0f;
    }

    const float brakingDistance = pathVelocity * pathVelocity / (2.0f * acceleration);

    
    switch (currentPhase)
    {
        case MotionPhase::ACCEL:
        {
            // Start decelerating early for a short movement
            if (remainingDistance <= brakingDistance)
            {
                currentPhase = MotionPhase::DECEL;
                break;
            }

            pathVelocity += acceleration * dt;

            if (pathVelocity >= cruiseSpeed)
            {
                pathVelocity = cruiseSpeed;
                currentPhase = MotionPhase::CRUISE;
            }

            break;
        }

        case MotionPhase::CRUISE:
        {
            pathVelocity = cruiseSpeed;

            if (remainingDistance <= brakingDistance)
            {
                currentPhase = MotionPhase::DECEL;
            }

            break;
        }

        case MotionPhase::DECEL:
        {
            pathVelocity -= acceleration * dt;

            if (pathVelocity <= 0.0f)
            {
                pathVelocity = 0.0f;
                currentPhase = MotionPhase::HOLD;
            }

            break;
        }

        case MotionPhase::HOLD:
        case MotionPhase::COMPLETE:
        {
            pathVelocity = 0.0f;
            break;
        }
    }

    return pathVelocity;

}