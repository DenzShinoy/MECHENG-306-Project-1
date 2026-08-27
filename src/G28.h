#ifndef G28_H
#define G28_H

#include "Encoder.h"
#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "manager.h"

class G28 {
 public:
  G28(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
      Encoder& encoderR, Manager& manager);

  void execute();

  bool isComplete() const;
  
  void reset();

  bool isExpectedLimit(LimitId id) const;

 private:
  enum class HomingPhase {
      IDLE,
      SEEK_LEFT,
      BACKOFF_LEFT,
      ENGAGE_LEFT,
      DISENGAGE_LEFT,
      SEEK_BOTTOM,
      BACKOFF_BOTTOM,
      ENGAGE_BOTTOM,
      DISENGAGE_BOTTOM,
      COMPLETE
  };

  MotorDriver& motorL_;
  MotorDriver& motorR_;
  Encoder& encoderL_;
  Encoder& encoderR_;
  Manager& manager_;

  HomingPhase phase_ = HomingPhase::IDLE;
};

#endif  // G28_H
