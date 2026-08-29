#include "LimitSwitch.h"

#include "Pins.h"

LimitSwitch::LimitSwitch(uint8_t pin)
    : _pin(pin),
      _stable(false),
      _lastRaw(false),
      _edge(false),
      _lastChangeMs(0) {}

void LimitSwitch::begin() { pinMode(_pin, INPUT_PULLUP); }

void LimitSwitch::update(uint32_t nowMs) {
  // Normally-closed to GND: the contact holds the pin LOW at rest, and
  // pressing opens it so the pullup takes the line to
  // cfg::SW_PRESSED_LEVEL (HIGH). Compare against cfg so this and the ISR
  // edge over in main.ino can't drift apart.
  const bool raw = (digitalRead(_pin) == cfg::SW_PRESSED_LEVEL);

  if (raw != _lastRaw) {
    // Reading moved. Could be a real press, a bounce, or motor noise, so
    // just restart the window. The subtraction below is unsigned, so it
    // survives millis() wrapping.
    _lastRaw = raw;
    _lastChangeMs = nowMs;
    return;
  }

  // Held steady since _lastChangeMs. Commit it once it has outlasted the
  // window and is actually different from what we already have.
  if ((nowMs - _lastChangeMs) >= cfg::DEBOUNCE_MS && raw != _stable) {
    if (raw) {
      _edge = true;  // released -> pressed: latch the one-shot
    }
    _stable = raw;
  }
}

bool LimitSwitch::isPressed() const { return _stable; }

bool LimitSwitch::justPressed() {
  // One-shot: hand back the pending edge, then clear it.
  const bool edge = _edge;
  _edge = false;
  return edge;
}
