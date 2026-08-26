#include <Arduino.h>

#include "LimitSwitch.h"
#include "Pins.h"

// Simple hardware test that polls the debounced LimitSwitch wrappers
// and prints events to Serial. Press a switch to see a one-shot
// "JUST PRESSED" message; a periodic status line reports current
// debounced states to help verify wiring and bouncing behaviour.

LimitSwitch swTop(pins::SW_TOP);
LimitSwitch swBottom(pins::SW_BOTTOM);
LimitSwitch swLeft(pins::SW_LEFT);
LimitSwitch swRight(pins::SW_RIGHT);

// ISR flags — set from interrupt context, read/cleared in loop().
volatile bool topHit = false;
volatile bool bottomHit = false;
volatile bool leftHit = false;
volatile bool rightHit = false;

void isrTop() { topHit = true; }
void isrBottom() { bottomHit = true; }
void isrLeft() { leftHit = true; }
void isrRight() { rightHit = true; }

const uint32_t PRINT_INTERVAL_MS = 200;
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);
  while (!Serial) {
    ;
  }

  swTop.begin();
  swBottom.begin();
  swLeft.begin();
  swRight.begin();

  // Attach interrupts after pins are configured by begin().
  attachInterrupt(digitalPinToInterrupt(pins::SW_TOP), isrTop, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_BOTTOM), isrBottom, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_LEFT), isrLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::SW_RIGHT), isrRight, FALLING);

  Serial.println("Limit switch test started");
  Serial.println("Press any limit switch to see a 'JUST PRESSED' event");
}

void printStates() {
  Serial.print("States: TOP=");
  Serial.print(swTop.isPressed() ? "P" : "-");
  Serial.print(" BTM=");
  Serial.print(swBottom.isPressed() ? "P" : "-");
  Serial.print(" LFT=");
  Serial.print(swLeft.isPressed() ? "P" : "-");
  Serial.print(" RGT=");
  Serial.println(swRight.isPressed() ? "P" : "-");
}

void loop() {
  const uint32_t now = millis();

  // Poll the debouncers regularly
  swTop.update(now);
  swBottom.update(now);
  swLeft.update(now);
  swRight.update(now);

  // One-shot events on transition to pressed
  if (swTop.justPressed()) {
    Serial.println("TOP JUST PRESSED");
  }
  if (swBottom.justPressed()) {
    Serial.println("BOTTOM JUST PRESSED");
  }
  if (swLeft.justPressed()) {
    Serial.println("LEFT JUST PRESSED");
  }
  if (swRight.justPressed()) {
    Serial.println("RIGHT JUST PRESSED");
  }

  // ISR-driven immediate notifications
  if (topHit) {
    topHit = false;
    Serial.println("TOP ISR HIT");
  }
  if (bottomHit) {
    bottomHit = false;
    Serial.println("BOTTOM ISR HIT");
  }
  if (leftHit) {
    leftHit = false;
    Serial.println("LEFT ISR HIT");
  }
  if (rightHit) {
    rightHit = false;
    Serial.println("RIGHT ISR HIT");
  }

  // Periodic summary to confirm stable states (avoids flooding)
  if ((now - lastPrintMs) >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printStates();
  }
}
