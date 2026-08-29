#ifndef G28_H
#define G28_H

#include "Encoder.h"
#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "manager.h"


// Homing, as a non-blocking state machine.
//
// X goes into the LEFT switch first, then Y into the BOTTOM one. Each
// axis does the same four steps:
//
//   SEEK      drive at it quickly until it trips
//   BACKOFF   reverse until it lets go again
//   ENGAGE    come back in slowly, which is the repeatable bit
//   DISENGAGE pull clear before moving on
//
// The fault handling itself lives in main.ino. All this class does for it
// is answer isExpectedLimit(), i.e. "is that switch meant to be pressed
// right now".

class G28 {
 public:
  G28(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
      Encoder& encoderR, Manager& manager);

  // One step of the sequence. Returns straight away, never blocks.
  void execute();

  // True once both axes are done.
  bool isComplete() const;
  
  // Stop the motors and go back to IDLE.
  void reset();

  // True if this is the switch we're deliberately driving into right now.
  // main.ino uses it to tell a normal homing contact from a real fault.
  bool isExpectedLimit(LimitId id) const;

 private:
  // X:  SEEK_LEFT -> BACKOFF_LEFT -> ENGAGE_LEFT -> DISENGAGE_LEFT
  // Y:  SEEK_BOTTOM -> BACKOFF_BOTTOM -> ENGAGE_BOTTOM -> DISENGAGE_BOTTOM
  //
  // COMPLETE means both axes are done and the origin has been set.
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

  // Shared with everything else, we don't own any of it.
  MotorDriver& motorL_;
  MotorDriver& motorR_;
  Encoder& encoderL_;
  Encoder& encoderR_;
  Manager& manager_;

  // Current phase of the homing sequence.
  HomingPhase phase_ = HomingPhase::IDLE;
};

#endif  // G28_H
