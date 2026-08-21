# Velocity Estimator Usage

This module estimates the velocity of each CoreXY motor and the Cartesian velocity of the moving platform from encoder count changes.

## 1. Required Files

```text
VelocityEstimator.h
VelocityEstimator.cpp
```

Include the header in the main control file:

```cpp
#include "VelocityEstimator.h"
```

If `build_src_filter` is used in `platformio.ini`, also include:

```ini
+<VelocityEstimator.cpp>
```

---

## 2. Velocity Estimator Interface

```cpp
struct VelocityData
{
    float motor1;  // Motor A velocity in mm/s
    float motor2;  // Motor B velocity in mm/s
    float x;       // Cartesian X velocity in mm/s
    float y;       // Cartesian Y velocity in mm/s
    float entity;  // Overall platform speed in mm/s
};

VelocityData estimateCoreXYVelocity(
    float durationSeconds,
    long previousEncoder1,
    long currentEncoder1,
    long previousEncoder2,
    long currentEncoder2,
    float countsPerMm
);
```

The estimator uses:

```text
velocity = encoder count change / counts per mm / elapsed time
```

For CoreXY:

```text
vX = (vA + vB) / 2
vY = (vA - vB) / 2
platform speed = sqrt(vX² + vY²)
```

---

## 3. Variables Required in the Main File

Add these variables globally:

```cpp
long previousLeftCount = 0;
long previousRightCount = 0;

unsigned long previousVelocityTime = 0;
```

---

## 4. Initialise the Estimator

In `setup()`, initialise the previous encoder counts and time:

```cpp
previousLeftCount = leftEncoder.position();
previousRightCount = rightEncoder.position();
previousVelocityTime = micros();
```

This only establishes the initial reference values. The variables must also be updated after every velocity calculation.

For a new independent movement, initialise them again before the motion starts.

---

## 5. Calculate Velocity in the Control Loop

Read the current encoder counts:

```cpp
const long leftCount = leftEncoder.position();
const long rightCount = rightEncoder.position();
```

Measure the actual elapsed time:

```cpp
const unsigned long velocityTime = micros();

const float velocityDt =
    (velocityTime - previousVelocityTime) * 1.0e-6f;
```

`micros()` returns microseconds. Multiplying by `1.0e-6f` converts the duration to seconds.

Call the estimator:

```cpp
const VelocityData velocity = estimateCoreXYVelocity(
    velocityDt,
    previousLeftCount,
    leftCount,
    previousRightCount,
    rightCount,
    cfg::COUNTS_PER_MM
);
```

The returned values are:

```cpp
velocity.motor1  // Motor A velocity
velocity.motor2  // Motor B velocity
velocity.x       // Cartesian X velocity
velocity.y       // Cartesian Y velocity
velocity.entity  // Overall platform speed
```

Update the previous values after the calculation:

```cpp
previousLeftCount = leftCount;
previousRightCount = rightCount;
previousVelocityTime = velocityTime;
```

The update must happen after the estimator call so the next iteration measures only the new encoder change.

---

## 6. Fixed-Duration Alternative

When the estimator is always called at an accurately controlled period, the configured duration can be used directly:

```cpp
const VelocityData velocity = estimateCoreXYVelocity(
    DT,
    previousLeftCount,
    leftCount,
    previousRightCount,
    rightCount,
    cfg::COUNTS_PER_MM
);
```

For example:

```cpp
constexpr float DT = 0.010f;  // 10 ms
```

Using the measured `micros()` duration is usually more accurate because Serial output and other calculations may slightly change the real loop duration.

---

## 7. Serial Output for MATLAB

Send one comma-separated row for each sample:

```cpp
Serial.print(velocityTime * 1.0e-6f, 4);
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
```

Output format:

```text
time_s,motor1_mm_s,motor2_mm_s,x_mm_s,y_mm_s,entity_mm_s
```

Example data:

```text
0.0500,-1.2000,-1.1800,-1.1900,-0.0100,1.1900
0.1000,-2.1000,-2.0500,-2.0750,-0.0250,2.0752
```

When the movement is complete, send:

```cpp
Serial.println("END");
```

MATLAB can use `END` to stop receiving data, retain the completed plot, and automatically save the collected data as a CSV file.

Avoid printing unrelated messages such as `MOVING` between CSV rows while MATLAB is recording. Additional text can be ignored by a robust MATLAB parser, but a numeric-only stream is easier to debug.

---

## 8. MATLAB Procedure

1. Upload the Arduino program.
2. Close the VS Code or PlatformIO Serial Monitor.
3. Set the correct COM port and baud rate in MATLAB.
4. Run the MATLAB logging script.
5. Reset the Arduino to start the movement.
6. MATLAB receives the Serial data and updates the plot in real time.
7. Arduino sends `END` after motion completion.
8. MATLAB stops recording and saves the CSV file automatically.

The VS Code Serial Monitor and MATLAB cannot normally use the same COM port at the same time.

---

## 9. Recommended Sampling

The PID control loop may remain at 10 ms:

```cpp
constexpr float DT = 0.010f;
```

Velocity calculated from only 10 ms of encoder counts can be noisy because the count change may be small.

A practical option is:

```text
PID calculation: every 10 ms
Velocity logging: every 50 ms
```

This does not require changing the PID period. It only reduces the Serial logging frequency and produces a clearer plot.

---

## 10. Important Notes

- `cfg::COUNTS_PER_MM` must match the encoder decoding method used by the project.
- The current encoder implementation reads channel A on `CHANGE`, which is 2x decoding.
- Encoder direction signs must match the motor command convention.
- `velocity.entity` is always a non-negative speed magnitude.
- `velocity.motor1`, `velocity.motor2`, `velocity.x`, and `velocity.y` retain direction through their signs.
- This estimator is independent of PID, feedforward, motor PWM, and the velocity profile generator.
- The same estimator can therefore be used with different controller implementations.