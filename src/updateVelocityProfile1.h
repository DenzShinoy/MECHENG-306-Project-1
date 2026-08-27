#ifndef UPDATE_VELOCITY_PROFILE1_H
#define UPDATE_VELOCITY_PROFILE1_H

/**
 * Update the scalar path velocity using a trapezoidal velocity profile.
 *
 * @param dt Time step between successive updates
 * @param cruiseSpeed Maximum commanded path speed
 * @param acceleration Acceleration and deceleration rate
 * @param pathVelocity Current path velocity
 * @param remainingDistance Distance remaining to the target
 * @return Updated path velocity
 *
 * Stateless: the caller owns pathVelocity and feeds the returned value
 * back in on the next tick (see G1::execute).
 */
float updateVelocityProfile1(
    float dt, float cruiseSpeed,
    float acceleration,
    float pathVelocity,
    float remainingDistance
);

#endif // UPDATE_VELOCITY_PROFILE1_H
