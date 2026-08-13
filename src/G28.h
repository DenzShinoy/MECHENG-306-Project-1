#ifndef G28_H
#define G82_H

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
  void execute(long target_x, long target_y);
  // get the target from the G1, void setTarget(int x, int y);
  void setFeedRate(int rate);
  void setEvent(int event);
  void reset();
  int getEvent() const;
  void getMaxSpeed(const AxisPair& target, const AxisPair& current, int16_t& a,
                   int16_t& b);

 private:
  int feed_rate;
  int curr_event;
  MotorDriver& motorL_;
  MotorDriver& motorR_;
  Encoder& encoderL_;
  Encoder& encoderR_;
  Manager& manager_;
};

#endif  // G8_H
