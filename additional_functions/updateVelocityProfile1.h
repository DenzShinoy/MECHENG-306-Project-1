#ifndef UPDATE_VELOCITY_PROFILE1_H
#define UPDATE_VELOCITY_PROFILE1_H

#include "motionPhase.h"

// Function to reset the velocity profile to its initial state
void resetVelocityProfile1();


/** Function to update the velocity profile based on the current phase of motion
 * @param dt Time step depends on pid controller iteration period
 * @param cruiseSpeed Maximum speed
 * @param acceleration Acceleration rate
 * @param pathVelocity Current velocity
 * @param remainingDistance Distance left to target
 * @return Updated velocity
 */
/** Example of use
 * dt = 0.01; // 10 ms
 * cruiseSpeed = 10.0; // 10 mm/s
 * acceleration = 0.5; // 0.5 mm/s^2
 * pathVelocity = 0.0; // Starting from rest
 * remainingDistance = 5.0; // 5 millimeters to target
 * --------------------------------------------------------------------
 *  resetVelocityProfile(); -> needed for each new movement command to reset the internal state of the velocity profile
 *  pathVelocity = 0.0f;
 *  .
 *  .
 *  .
 *  .
 *  float updatedVelocity = updateVelocityProfile(dt, cruiseSpeed, acceleration, pathVelocity, remainingDistance);
 *  analogWrite(velocityPin, updatedVelocity * 255.0 / cruiseSpeed); // Scale to PWM range
 */


float updateVelocityProfile1(
    float dt, float cruiseSpeed,
    float acceleration,
    float pathVelocity,
    float remainingDistance
);

#endif // UPDATE_VELOCITY_PROFILE_H