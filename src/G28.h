#ifndef G28_H
#define G28_H

#include "Encoder.h"
#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "manager.h"


// =====================================================================
//  G28 homing controller
// ---------------------------------------------------------------------
//  Implements the non-blocking homing state machine used by the plotter.
//
//  Homing is performed in two axes:
//
//    1. X axis homes against the LEFT limit switch.
//    2. Y axis homes against the BOTTOM limit switch.
//
//  Each axis uses a four-stage sequence:
//
//    SEEK      - approach the switch quickly until first contact
//    BACKOFF   - move away until the switch releases
//    ENGAGE    - approach again slowly for a repeatable trigger point
//    DISENGAGE - move clear of the switch before continuing
//
//  Limit-switch fault detection itself is handled by main.cpp. During
//  G28, isExpectedLimit() tells main.cpp which switch is intentionally
//  involved in the current homing phase.
// =====================================================================

class G28 {
 public:
  // Construct the homing controller using the shared motor, encoder,
  // and Manager objects.
  G28(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
      Encoder& encoderR, Manager& manager);

  // Advance the homing state machine by one step.
  // This function is non-blocking and returns after each phase update.
  void execute();

  // True once the full homing sequence has reached COMPLETE.
  bool isComplete() const;
  
  // Stop both motors and return the homing state machine to IDLE.
  void reset();

  // Return true when the supplied limit switch is intentionally allowed
  // in the current homing phase. main.cpp uses this to distinguish a
  // normal homing contact from an unexpected limit-switch fault.
  bool isExpectedLimit(LimitId id) const;

 private:
  // Internal homing sequence.
  //
  // X axis:
  //   SEEK_LEFT -> BACKOFF_LEFT -> ENGAGE_LEFT -> DISENGAGE_LEFT
  //
  // Y axis:
  //   SEEK_BOTTOM -> BACKOFF_BOTTOM -> ENGAGE_BOTTOM -> DISENGAGE_BOTTOM
  //
  // COMPLETE is entered after both axes have been homed and the final
  // machine origin has been established.
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

  // Shared hardware / state objects.
  MotorDriver& motorL_;
  MotorDriver& motorR_;
  Encoder& encoderL_;
  Encoder& encoderR_;
  Manager& manager_;

  // Current phase of the homing sequence.
  HomingPhase phase_ = HomingPhase::IDLE;
};

#endif  // G28_H
