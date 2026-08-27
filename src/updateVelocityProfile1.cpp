#include "updateVelocityProfile1.h"


// =====================================================================
//  Trapezoidal path-velocity profile
// ---------------------------------------------------------------------
//  Updates the scalar path velocity used by G1.
//
//  The profile has three possible behaviours:
//
//    ACCEL  - increase velocity by a * dt while there is enough distance
//             remaining to accelerate safely
//
//    CRUISE - hold the commanded cruise speed once it has been reached
//
//    DECEL  - reduce velocity when the remaining distance becomes smaller
//             than the distance required to stop
//
//  The braking-distance test allows the move to begin decelerating early
//  enough to reach zero velocity near the target without blocking the
//  control loop.
// =====================================================================

float updateVelocityProfile1(
    float dt,
    float cruiseSpeed,
    float acceleration,
    float pathVelocity,
    float remainingDistance
)
{
    // Invalid timing or acceleration cannot produce a meaningful update,
    // so preserve the current path velocity.
    if (dt <= 0.0f || acceleration <= 0.0f)
        return pathVelocity;

    // No path remains: command zero velocity.
    if (remainingDistance <= 0.0f)
        return 0.0f;

    // Distance required to stop from the current path velocity under the
    // specified constant deceleration:
    //
    //     d = v^2 / (2a)
    //
    // Once the remaining path is no greater than this value, the profile
    // must begin slowing down.
    const float brakingDistance =
        pathVelocity * pathVelocity / (2.0f * acceleration);

    if (remainingDistance <= brakingDistance)
    {
        // DECEL: reduce the path velocity at the configured acceleration
        // rate, but never allow the velocity magnitude to become negative.
        pathVelocity -= acceleration * dt;

        if (pathVelocity < 0.0f)
            pathVelocity = 0.0f;
    }
    else
    {
        // ACCEL / CRUISE: increase velocity while sufficient stopping
        // distance remains, then clamp it at the requested cruise speed.
        pathVelocity += acceleration * dt;

        if (pathVelocity > cruiseSpeed)
            pathVelocity = cruiseSpeed;
    }

    // Return the updated scalar speed for the next trajectory step.
    return pathVelocity;
}
