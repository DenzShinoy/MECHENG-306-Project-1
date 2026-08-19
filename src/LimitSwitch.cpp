#include "LimitSwitch.h"
#include "Pins.h"

LimitSwitch::LimitSwitch(uint8_t pin)
    : _pin(pin), _stable(false), _lastRaw(false), _edge(false),
      _lastChangeMs(0) {}

void LimitSwitch::begin() {
  pinMode(_pin, INPUT_PULLUP);
}

void LimitSwitch::update(uint32_t nowMs) {
  // Active-LOW: the pin sits HIGH via the pullup and is pulled LOW when
  // the switch closes, so a LOW reading means "pressed".
  const bool raw = (digitalRead(_pin) == LOW);

  if (raw != _lastRaw) {
    // Reading just moved (real edge or a bounce) — (re)start the window.
    // Unsigned subtraction below makes this wrap-safe across millis().
    _lastRaw = raw;
    _lastChangeMs = nowMs;
    return;
  }

  // raw has held steady since _lastChangeMs; commit once it outlasts the
  // debounce window and actually differs from the committed state.
  if ((nowMs - _lastChangeMs) >= cfg::DEBOUNCE_MS && raw != _stable) {
    if (raw) {
      _edge = true;   // released -> pressed: latch the one-shot
    }
    _stable = raw;
  }
}

bool LimitSwitch::isPressed() const {
  return _stable;
}

bool LimitSwitch::justPressed() {
  // Consume the one-shot: report the pending press edge, then clear it.
  const bool edge = _edge;
  _edge = false;
  return edge;
}  