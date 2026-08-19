#include "velocityEstimater.h"

#include <math.h>

VelocityData estimateCoreXYVelocity(
    float durationSeconds,
    long previousEncoder1,
    long currentEncoder1,
    long previousEncoder2,
    long currentEncoder2,
    float countsPerMm
)
{
    VelocityData velocity{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    if (durationSeconds <= 0.0f || countsPerMm <= 0.0f)
    {
        return velocity;
    }

    const long deltaCount1 = currentEncoder1 - previousEncoder1;
    const long deltaCount2 = currentEncoder2 - previousEncoder2;

    velocity.motor1 =
        static_cast<float>(deltaCount1) / countsPerMm / durationSeconds;

    velocity.motor2 =
        static_cast<float>(deltaCount2) / countsPerMm / durationSeconds;

    velocity.x = (velocity.motor1 + velocity.motor2) * 0.5f;
    velocity.y = (velocity.motor1 - velocity.motor2) * 0.5f;

    velocity.entity = sqrtf(
        velocity.x * velocity.x +
        velocity.y * velocity.y
    );

    return velocity;
}