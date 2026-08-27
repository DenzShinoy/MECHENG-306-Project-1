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
  // Normally-closed to GND: the closed contact holds the pin LOW at rest
  // and pressing opens it, so the pullup takes the line to
  // cfg::SW_PRESSED_LEVEL (HIGH). The comparison is against cfg so this
  // debouncer and the ISR edge in main.ino can never disagree.
  const bool raw = (digitalRead(_pin) == cfg::SW_PRESSED_LEVEL);

  if (raw != _lastRaw) {
    // Reading just moved (real edge, a bounce, or motor noise) —
    // (re)start the window. Unsigned subtraction below makes this
    // wrap-safe across millis().
    _lastRaw = raw;
    _lastChangeMs = nowMs;
    return;
  }

  // raw has held steady since _lastChangeMs; commit once it outlasts the
  // debounce window and actually differs from the committed state.
  if ((nowMs - _lastChangeMs) >= cfg::DEBOUNCE_MS && raw != _stable) {
    if (raw) {
      _edge = true;  // released -> pressed: latch the one-shot
    }
    _stable = raw;
  }
}

bool LimitSwitch::isPressed() const { return _stable; }

bool LimitSwitch::justPressed() {
  // Consume the one-shot: report the pending press edge, then clear it.
  const bool edge = _edge;
  _edge = false;
  return edge;
}