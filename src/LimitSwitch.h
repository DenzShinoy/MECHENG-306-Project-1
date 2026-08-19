#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 3 — LimitSwitch
// ---------------------------------------------------------------------
//  Debounced read of one active-LOW switch wired to INPUT_PULLUP. Our
//  own non-blocking debouncer (project brief 2.1.4): update() is polled
//  each loop with the current millis(); no delay(). A stable reading is
//  one that has held for cfg::DEBOUNCE_MS.
// =====================================================================

class LimitSwitch {
public:
  explicit LimitSwitch(uint8_t pin);

  // pinMode(INPUT_PULLUP). Call from setup().
  void begin();

  // Sample the pin and run the debounce timer. Call every loop with the
  // current millis(). Non-blocking.
  void update(uint32_t nowMs);

  // Debounced state: true = pressed (electrically LOW).
  bool isPressed() const;

  // True for one update() only, on the transition to pressed. Useful for
  // latching a homing datum exactly once.
  bool justPressed();

private:
  const uint8_t _pin;
  bool     _stable;      // last debounced state (true = pressed)
  bool     _lastRaw;     // last raw sample, for detecting change
  bool     _edge;        // pending rising (pressed) edge for justPressed()
  uint32_t _lastChangeMs;// when _lastRaw last changed
};