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
  // guard against an ISR landing mid-read. Save/restore the interrupt state
  // rather than blindly re-enabling: this may be called before interrupts
  // are on (setup) or from inside another critical section, where forcing
  // them back on would corrupt the caller.
  const uint8_t sreg = SREG;
  noInterrupts();
  const long v = _count;
  SREG = sreg;
  return v;
}

// Reset the encoder count to zero. This is a critical section because
// the ISR may be running and changing _count at the same time.
void Encoder::reset() {
  const uint8_t sreg = SREG;
  noInterrupts();
  _count = 0;
  SREG = sreg;
  _lastSample = 0;
}

// Return the number of counts since the previous call. This is a crude
// velocity estimate; the caller is responsible for timing. The first call
// after reset() returns the total counts since reset.
long Encoder::consumeDelta() {
  const long now = position();
  const long delta = now - _lastSample;
  _lastSample = now;
  return delta;
}
