#ifndef G1_H
#define G1_H

#include "Encoder.h"
#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "manager.h"

class G1 {
 public:
  G1(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
     Encoder& encoderR, Manager& manager);
  void execute(long target_x, long target_y);

  bool isComplete() const;

  void reset();
  void stop();

 private:
  void beginMove(long target_x, long target_y);

  void getMaxSpeed(const AxisPair& target, const AxisPair& current, int16_t& a,
                   int16_t& b);

  long previousLeftCount_ = 0;
  long previousRightCount_ = 0;

  unsigned long lastVelocityMicros_ = 0;

  static constexpr uint32_t REPORT_INTERVAL_US = 20000;

  MotorDriver& motorL_;
  MotorDriver& motorR_;

  Encoder& encoderL_;
  Encoder& encoderR_;

  Manager& manager_;

  // Persistent PID controllers.
  PID pidL_;
  PID pidR_;

  // Final A/B target.
  AxisPair motorTarget_ = {0, 0};

  // Starting encoder positions.
  long startA_ = 0;
  long startB_ = 0;

  // Total A/B displacement.
  long dA_ = 0;
  long dB_ = 0;

  // Current motor output.
  int16_t speedL_ = 0;
  int16_t speedR_ = 0;

  // Trapezoidal trajectory state.
  float pathLength_ = 0.0f;
  float pathVelocity_ = 0.0f;
  float s_ = 0.0f;

  // Timing.
  uint32_t lastControlMs_ = 0;
  unsigned long lastMicros_ = 0;

  bool active_ = false;
  bool complete_ = false;

  int16_t maxSpeedL_ = 0;
  int16_t maxSpeedR_ = 0;
};

#endif  // G1_H