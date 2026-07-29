#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : _pinA(pinA), _pinB(pinB), _count(0), _lastSample(0) {}

void Encoder::begin() {
  // TODO: pinMode(_pinA, INPUT) / INPUT_PULLUP as wiring requires.
  // TODO: pinMode(_pinB, INPUT). attachInterrupt wired in main.
}

void Encoder::handleEdge() {
  // TODO: read _pinB; step _count +/- 1 based on A vs B phase (2x decode).
}

long Encoder::position() const {
  // TODO: return _count with interrupts briefly disabled for an atomic read.
  return 0;
}

void Encoder::reset() {
  // TODO: atomically clear _count and _lastSample.
}

long Encoder::consumeDelta() {
  // TODO: return position() - _lastSample, then update _lastSample.
  return 0;
}
