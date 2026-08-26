#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 3 — LimitSwitch
// ---------------------------------------------------------------------
//  Debounced read of one limit switch on INPUT_PULLUP. The switches are
//  wired normally-closed to GND (see cfg::SW_PRESSED_LEVEL in Pins.h):
//  the pin idles LOW and reads HIGH while pressed. This class never
//  hard-codes that — it compares against cfg::SW_PRESSED_LEVEL, so the
//  polarity lives in exactly one place.
//
//  Our own non-blocking debouncer (project brief 2.1.4): update() is
//  polled each loop with the current millis(); no delay(). A reading
//  only becomes the committed state after it has held unchanged for
//  cfg::DEBOUNCE_MS — a PWM glitch a few microseconds wide can never
//  survive that, which is what makes this the arbiter for the fault
//  path's ISR confirmation.
// =====================================================================

class LimitSwitch {
 public:
  explicit LimitSwitch(uint8_t pin);

  // pinMode(INPUT_PULLUP). Call from setup().
  void begin();

  // Sample the pin and run the debounce timer. Call every loop with the
  // current millis(). Non-blocking.
  void update(uint32_t nowMs);

  // Debounced state: true = pressed.
  bool isPressed() const;

  // True for one call only, on the debounced transition to pressed.
  // Useful for latching a homing datum exactly once.
  bool justPressed();

 private:
  const uint8_t _pin;
  bool _stable;         // last debounced state (true = pressed)
  bool _lastRaw;        // last raw sample, for detecting change
  bool _edge;           // pending press edge for justPressed()
  uint32_t _lastChangeMs;  // when _lastRaw last changed
};
