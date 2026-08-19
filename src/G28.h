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
    G28(
        MotorDriver& motorL,
        MotorDriver& motorR,
        Encoder& encoderL,
        Encoder& encoderR,
        Manager& manager
    );

    void execute();

    bool isComplete() const;
    void reset();

private:
    enum class HomingPhase {
        IDLE,
        SEEK_LEFT,
        BACKOFF_LEFT,
        SEEK_BOTTOM,
        BACKOFF_BOTTOM,
        COMPLETE
    };

    MotorDriver& motorL_;
    MotorDriver& motorR_;
    Encoder& encoderL_;
    Encoder& encoderR_;
    Manager& manager_;

    HomingPhase phase_ = HomingPhase::IDLE;

    uint32_t lastControlMs_ = 0;
};

#endif  // G8_H
