#include <Arduino.h>

#include "LimitSwitch.h"
#include "Pins.h"

// Bench test for the limit switches. Polls the debounced wrappers and
// prints what it sees. Push a switch and you should get one "JUST
// PRESSED" line; the status line every 200 ms is there for checking the
// wiring and seeing how much the contacts actually bounce.

LimitSwitch swTop(pins::SW_TOP);
LimitSwitch swBottom(pins::SW_BOTTOM);
LimitSwitch swLeft(pins::SW_LEFT);
LimitSwitch swRight(pins::SW_RIGHT);

// Set in the ISRs, read and cleared in loop().
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

  // Interrupts go on after begin() has sorted the pin modes out.
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

  // Debouncers need polling every pass.
  swTop.update(now);
  swBottom.update(now);
  swLeft.update(now);
  swRight.update(now);

  // These only fire once each, on the press edge.
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

  // Raw ISR hits, printed as they come in.
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

  // Status line, throttled so it doesn't drown out everything else.
  if ((now - lastPrintMs) >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printStates();
  }
}
