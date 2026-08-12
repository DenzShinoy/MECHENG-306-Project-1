#ifndef G1_H
#define G1_H

#include "Encoder.h"
#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"

class G1 {
 public:
  G1(MotorDriver& motorL, MotorDriver& motorR, Encoder& encoderL,
     Encoder& encoderR);
  void execute(long target_x, long target_y);
  void setTarget(int x, int y);
  void setFeedRate(int rate);
  void setEvent(int event);
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
};

#endif  // G1_H