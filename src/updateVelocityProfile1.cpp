#include "updateVelocityProfile1.h"

float updateVelocityProfile1(
    float dt,
    float cruiseSpeed,
    float acceleration,
    float pathVelocity,
    float remainingDistance
)
{
    if (dt <= 0.0f || acceleration <= 0.0f)
        return pathVelocity;

    if (remainingDistance <= 0.0f)
        return 0.0f;

    const float brakingDistance =
        pathVelocity * pathVelocity / (2.0f * acceleration);

    if (remainingDistance <= brakingDistance)
    {
        // DECEL
        pathVelocity -= acceleration * dt;

        if (pathVelocity < 0.0f)
            pathVelocity = 0.0f;
    }
    else
    {
        // ACCEL / CRUISE
        pathVelocity += acceleration * dt;

        if (pathVelocity > cruiseSpeed)
            pathVelocity = cruiseSpeed;
    }

    return pathVelocity;
}
