#include <Arduino.h>

#include "Encoder.h"
#include "Kinematics.h"
#include "MotorDriver.h"
#include "PID.h"
#include "Pins.h"
#include "Timer.h"
#include "fsm_1.h"
#include "manager.h"

// Set up encoder and motor driver objects with the correct pins. The encoder
// ISRs are wired in main() to call the handleEdge() method on each object.
static Encoder encoderL(pins::ENC_L_A, pins::ENC_L_B);
static Encoder encoderR(pins::ENC_R_A, pins::ENC_R_B);

static MotorDriver motorL(pins::M1_DIR, pins::M1_PWM, true);
static MotorDriver motorR(pins::M2_DIR, pins::M2_PWM, true);
static G1 g1(motorL, motorR, encoderL, encoderR);

FSM fsm;
Manager manager;

void isrEncoderL() { encoderL.handleEdge(); }
void isrEncoderR() { encoderR.handleEdge(); }

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);
  Serial.println(F("BOOT"));
  while (Serial.available() == 0) {
  }  // wait for any input
  while (Serial.available() > 0) Serial.read();  // clear buffer

  encoderL.begin();
  encoderR.begin();
  attachInterrupt(digitalPinToInterrupt(pins::ENC_L_A), isrEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_R_A), isrEncoderR, CHANGE);
  motorL.begin();
  motorR.begin();

  fsm.setMotion(g1);
}

void loop() {
  while (Serial.available() == 0) {
    // wait for a serial command
  }

  fsm.handleEvent(1);
  fsm.dispatch();
}