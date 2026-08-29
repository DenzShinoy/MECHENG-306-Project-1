#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : _pinA(pinA), _pinB(pinB), _count(0) {}

void Encoder::begin() {
  // The pullups don't bother the encoder's push-pull outputs, and they
  // mean a disconnected line reads as something instead of floating.
  // attachInterrupt(_pinA, CHANGE) happens in main.
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
}

void Encoder::handleEdge() {
  // Runs on every edge of A, so 2x decode. Sampling A and B at the same
  // instant tells us the direction: same means one way, different means
  // the other. If an axis ends up mirrored, fix it in the wiring
  // (pinout §9) rather than flipping the sign in here.
  const bool a = digitalRead(_pinA);
  const bool b = digitalRead(_pinB);
  if (a == b) {
    _count++;
  } else {
    _count--;
  }
}

long Encoder::position() const {
  // _count is 4 bytes and this is an 8-bit AVR, so the read takes several
  // instructions and the ISR can cut it in half. Save and restore SREG
  // rather than just calling interrupts() at the end: this gets called
  // from setup() before interrupts are even on, and from inside other
  // critical sections, and switching them back on there would break the
  // caller.
  const uint8_t sreg = SREG;
  noInterrupts();
  const long v = _count;
  SREG = sreg;
  return v;
}

// Same critical section as position(): the ISR could be part way through
// writing _count while we clear it.
void Encoder::reset() {
  const uint8_t sreg = SREG;
  noInterrupts();
  _count = 0;
  SREG = sreg;
}
