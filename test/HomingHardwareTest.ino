#include <Arduino.h>

#include "Pins.h"

volatile bool topHit = false;
volatile bool bottomHit = false;
volatile bool leftHit = false;
volatile bool rightHit = false;

volatile long leftEncoderCounts = 0;
volatile long rightEncoderCounts = 0;

void handleSwitchInterrupt(uint8_t pin) {
  switch (pin) {
    case pins::SW_TOP:
      topHit = true;
      break;
    case pins::SW_BOTTOM:
      bottomHit = true;
      break;
    case pins::SW_LEFT:
      leftHit = true;
      break;
    case pins::SW_RIGHT:
      rightHit = true;
      break;
  }
}

void isrTop() { handleSwitchInterrupt(pins::SW_TOP); }
void isrBottom() { handleSwitchInterrupt(pins::SW_BOTTOM); }
void isrLeft() { handleSwitchInterrupt(pins::SW_LEFT); }
void isrRight() { handleSwitchInterrupt(pins::SW_RIGHT); }

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);

  pinMode(pins::M1_DIR, OUTPUT);
  pinMode(pins::M1_PWM, OUTPUT);
  pinMode(pins::M2_DIR, OUTPUT);
  pinMode(pins::M2_PWM, OUTPUT);

  pinMode(pins::SW_TOP, INPUT_PULLUP);
  pinMode(pins::SW_BOTTOM, INPUT_PULLUP);
  pinMode(pins::SW_LEFT, INPUT_PULLUP);
  pinMode(pins::SW_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pins::SW_TOP), isrTop, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_BOTTOM), isrBottom, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_LEFT), isrLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_RIGHT), isrRight, FALLING);

  Serial.println("Full homing test started");
  Serial.println(
      "The sketch will run a first pass, reset on first hit, then run a slower "
      "second pass and print the span.");
}

void driveMotor(uint8_t dirPin, uint8_t pwmPin, int speed) {
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

void stopMotors() {
  analogWrite(pins::M1_PWM, 0);
  analogWrite(pins::M2_PWM, 0);
}

void printEncoderState(const char* label, long left, long right) {
  Serial.print(label);
  Serial.print(" left=");
  Serial.print(left);
  Serial.print(" right=");
  Serial.println(right);
}

void loop() {
  static enum State { IDLE, FIRST_PASS, SECOND_PASS, DONE } state = FIRST_PASS;

  static bool firstPassDone = false;
  static bool secondPassDone = false;
  static long firstPassLeft = 0;
  static long firstPassRight = 0;
  static long secondPassLeft = 0;
  static long secondPassRight = 0;

  if (state == DONE) {
    return;
  }

  if (state == FIRST_PASS) {
    driveMotor(pins::M1_DIR, pins::M1_PWM, 80);
    driveMotor(pins::M2_DIR, pins::M2_PWM, 80);

    if (leftHit) {
      leftHit = false;
      firstPassLeft = leftEncoderCounts;
      Serial.println("First-pass LEFT hit");
      firstPassDone = true;
      leftEncoderCounts = 0;
    }

    if (rightHit) {
      rightHit = false;
      firstPassRight = rightEncoderCounts;
      Serial.println("First-pass RIGHT hit");
      firstPassDone = true;
      rightEncoderCounts = 0;
    }

    if (firstPassDone) {
      state = SECOND_PASS;
      stopMotors();
      Serial.println("Switched to second pass");
      delay(500);
    }
  }

  if (state == SECOND_PASS) {
    driveMotor(pins::M1_DIR, pins::M1_PWM, 60);
    driveMotor(pins::M2_DIR, pins::M2_PWM, 60);

    if (leftHit) {
      leftHit = false;
      secondPassLeft = leftEncoderCounts;
      Serial.println("Second-pass LEFT hit");
      secondPassDone = true;
    }

    if (rightHit) {
      rightHit = false;
      secondPassRight = rightEncoderCounts;
      Serial.println("Second-pass RIGHT hit");
      secondPassDone = true;
    }

    if (secondPassDone) {
      stopMotors();
      Serial.println("Homing complete");
      printEncoderState("First pass", firstPassLeft, firstPassRight);
      printEncoderState("Second pass", secondPassLeft, secondPassRight);
      Serial.print("Measured span left=");
      Serial.print(secondPassLeft);
      Serial.print(" right=");
      Serial.println(secondPassRight);
      state = DONE;
    }
  }

  delay(10);
}
