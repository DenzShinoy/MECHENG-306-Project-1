#include "updateVelocityProfile1.h"


// Trapezoidal profile for the path velocity G1 follows.
//
// Three things can happen on any tick: ramp up by a*dt while there's room
// to stop, hold the cruise speed once we reach it, or ramp back down when
// what's left is less than the distance we need to stop in. The braking
// check is the part that lets it start slowing early enough to land on
// the target, without ever blocking the loop.

float updateVelocityProfile1(
    float dt,
    float cruiseSpeed,
    float acceleration,
    float pathVelocity,
    float remainingDistance
)
{
    // Nonsense dt or acceleration. Nothing sensible to do, so hand back
    // what we were given.
    if (dt <= 0.0f || acceleration <= 0.0f)
        return pathVelocity;

    // Nothing left to travel.
    if (remainingDistance <= 0.0f)
        return 0.0f;

    // Distance needed to stop from here at constant deceleration:
    //
    //     d = v^2 / (2a)
    //
    // Once there's no more path left than that, start slowing down.
    const float brakingDistance =
        pathVelocity * pathVelocity / (2.0f * acceleration);

    if (remainingDistance <= brakingDistance)
    {
        // Slow down, but don't let it go negative and start reversing.
        pathVelocity -= acceleration * dt;

        if (pathVelocity < 0.0f)
            pathVelocity = 0.0f;
    }
    else
    {
        // Still room to stop, so speed up, capped at the cruise speed.
        pathVelocity += acceleration * dt;

        if (pathVelocity > cruiseSpeed)
            pathVelocity = cruiseSpeed;
    }

    return pathVelocity;
}
