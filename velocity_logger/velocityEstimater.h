#pragma once

// velocityEstimater.h

/** * Function to update the velocity profile based on the current phase of motion
 * @param durationSeconds Time duration of the movement
 * @param previousEncoder1 Previous encoder count for motor 1
 * @param currentEncoder1 Current encoder count for motor 1
 * @param previousEncoder2 Previous encoder count for motor 2
 * @param currentEncoder2 Current encoder count for motor 2
 * @param countsPerMm Counts per millimeter for the encoders
 * @return Measured velocity data
                velocity.motor1
                velocity.motor2
                velocity.x
                velocity.y
                velocity.entity
    
 * 
 * 
 * Example of use
 * durationSeconds = 10; // 10 ms
 * previousEncoder1 = 1000; // Previous encoder count for motor 1
 * currentEncoder1 = 1100; // Current encoder count for motor 1
 * previousEncoder2 = 2000; // Previous encoder count for motor 2
 * currentEncoder2 = 2100; // Current encoder count for motor 2
 * countsPerMm = 171.79 (from Pin.h); // Counts per millimeter for the encoders
 * --------------------------------------------------------------------
 * -----Global variables-----
 *      long previousLeftCount = 0;
 *      long previousRightCount = 0;
 *      unsigned long previousVelocityTime = 0;
 * 
 * 
 * -----End of the setup-----
 *      previousLeftCount = leftEncoder.position();
 *      previousRightCount = rightEncoder.position();
 *      previousVelocityTime = micros();
 * 
 * 
 * -----After read the encoder counts (leftCount, rightCount)------
 *      const unsigned long velocityTime = micros();

        const float velocityDt =
        (velocityTime - previousVelocityTime) * 1.0e-6f; -> convert microseconds to seconds

        const VelocityData velocity = estimateCoreXYVelocity(
            velocityDt,
            previousLeftCount,
            leftCount,
            previousRightCount,
            rightCount,
            cfg::COUNTS_PER_MM -> from cfg
        );

        previousLeftCount = leftCount;
        previousRightCount = rightCount;
        previousVelocityTime = velocityTime;
 * 
 * 
 * -----Serial Print section-----
 *      Serial.print(currentTime * 1.0e-6f, 4);
        Serial.print(",");
        Serial.print(velocity.motor1, 4);
        Serial.print(",");
        Serial.print(velocity.motor2, 4);
        Serial.print(",");
        Serial.print(velocity.x, 4);
        Serial.print(",");
        Serial.print(velocity.y, 4);
        Serial.print(",");
        Serial.println(velocity.entity, 4);
 * 
 * Printing format:
 * ||time,motor1,motor2,x,y,entity                 ||
 * ||1.2500,-6.4200,-6.3100,-6.3650,-0.0550,6.3652 ||
 * 
 * 
 * -----After settlement-----
 *      Serial.println("END");
 * 
 * Then the matlab will automatically finish the recording from the serial port
 * and 
 */

// struct to hold the measured velocity data
struct VelocityData
{
    float motor1;
    float motor2;
    float x;
    float y;
    float entity;
};

// Function to estimate the velocity of a CoreXY system based on encoder counts and time duration
VelocityData estimateCoreXYVelocity(
    float durationSeconds,
    long previousEncoder1,
    long currentEncoder1,
    long previousEncoder2,
    long currentEncoder2,
    float countsPerMm
);