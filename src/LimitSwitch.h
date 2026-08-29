#pragma once
#include <Arduino.h>

// Debounced read of one limit switch on INPUT_PULLUP. The switches are
// wired normally-closed to GND (see cfg::SW_PRESSED_LEVEL in Pins.h), so
// the pin idles LOW and reads HIGH while pressed. Nothing in here
// hard-codes that; it compares against cfg::SW_PRESSED_LEVEL so the
// polarity is written down in one place only.
//
// Our own debouncer, non-blocking as the brief asks (2.1.4). update()
// gets polled every loop with the current millis(), no delay() anywhere.
// A reading only becomes the committed state once it has held for
// cfg::DEBOUNCE_MS, which a PWM glitch a few microseconds wide has no
// chance of doing. That's why the fault path trusts this over the raw
// ISR.

class LimitSwitch {
 public:
  explicit LimitSwitch(uint8_t pin);

  // pinMode(INPUT_PULLUP). Call from setup().
  void begin();

  // Sample the pin and tick the debounce timer. Call every loop with the
  // current millis(). Doesn't block.
  void update(uint32_t nowMs);

  // Debounced state: true = pressed.
  bool isPressed() const;

  // True once, on the debounced release-to-press edge, then false again.
  // Handy for latching a homing datum exactly once.
  bool justPressed();

 private:
  const uint8_t _pin;
  bool _stable;         // last debounced state (true = pressed)
  bool _lastRaw;        // last raw sample, for detecting change
  bool _edge;           // pending press edge for justPressed()
  uint32_t _lastChangeMs;  // when _lastRaw last changed
};
