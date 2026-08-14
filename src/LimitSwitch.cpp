#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(uint8_t pin)
    : _pin(pin), _stable(false), _lastRaw(false), _edge(false),
      _lastChangeMs(0) {}

void LimitSwitch::begin() {
  // TODO: pinMode(_pin, INPUT_PULLUP).
}

void LimitSwitch::update(uint32_t nowMs) {
  // TODO: read pin (LOW == pressed); if raw changed, restart the timer;
  // once it has held DEBOUNCE_MS, commit to _stable and flag _edge on a
  // low->high (released->pressed) transition.
  (void)nowMs;
}

bool LimitSwitch::isPressed() const {
  return _stable;
}

bool LimitSwitch::justPressed() {
  // TODO: return _edge then clear it (consume the one-shot).
  return false;
}
