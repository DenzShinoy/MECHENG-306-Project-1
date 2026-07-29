#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : _pinA(pinA), _pinB(pinB), _count(0), _lastSample(0) {}

void Encoder::begin() {
  // Pullups are harmless on the push-pull encoder outputs and cover the
  // case of an open line. attachInterrupt(_pinA, CHANGE) is wired in main.
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
}

void Encoder::handleEdge() {
  // 2x decode: this runs on every edge of A. Comparing A and B at the
  // instant of the edge gives direction — when they match we are turning
  // one way, when they differ the other. Kept tiny for the ISR; sign
  // convention is corrected in wiring (pinout §9) if an axis runs mirrored.
  const bool a = digitalRead(_pinA);
  const bool b = digitalRead(_pinB);
  if (a == b) {
    _count++;
  } else {
    _count--;
  }
}

long Encoder::position() const {
  // _count is 4 bytes; an 8-bit AVR cannot read it in one instruction, so
  // guard against an ISR landing mid-read. Called from the main loop where
  // interrupts are enabled, so re-enabling afterwards is correct.
  noInterrupts();
  const long v = _count;
  interrupts();
  return v;
}

void Encoder::reset() {
  noInterrupts();
  _count = 0;
  interrupts();
  _lastSample = 0;
}

long Encoder::consumeDelta() {
  const long now = position();
  const long delta = now - _lastSample;
  _lastSample = now;
  return delta;
}
