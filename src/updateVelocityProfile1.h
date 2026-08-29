#ifndef UPDATE_VELOCITY_PROFILE1_H
#define UPDATE_VELOCITY_PROFILE1_H

/**
 * Trapezoidal profile for the scalar path velocity.
 *
 * Stateless on purpose: G1 owns pathVelocity and feeds last tick's return
 * value straight back in.
 *
 * @param dt                time since the last call
 * @param cruiseSpeed       speed to hold once we get up to it
 * @param acceleration      used for the ramp up and the ramp down
 * @param pathVelocity      where we are now
 * @param remainingDistance how much path is left
 * @return the new path velocity
 */
float updateVelocityProfile1(
    float dt, float cruiseSpeed,
    float acceleration,
    float pathVelocity,
    float remainingDistance
);

#endif // UPDATE_VELOCITY_PROFILE1_H
