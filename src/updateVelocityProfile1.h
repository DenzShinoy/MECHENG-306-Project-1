#ifndef UPDATE_VELOCITY_PROFILE1_H
#define UPDATE_VELOCITY_PROFILE1_H

/** Function to update the velocity profile based on the current phase of motion
 * @param dt Time step depends on pid controller iteration period
 * @param cruiseSpeed Maximum speed
 * @param acceleration Acceleration rate
 * @param pathVelocity Current velocity
 * @param remainingDistance Distance left to target
 * @return Updated velocity
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

#endif // UPDATE_VELOCITY_PROFILE_H
