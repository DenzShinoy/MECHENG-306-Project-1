#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 2 — Encoder
// ---------------------------------------------------------------------
//  Quadrature decoding + position tracking for one motor. Our own code
//  (no Encoder.h library). Designed to be driven from a hardware ISR on
//  the A channel: the ISR calls handleEdge(), which reads the B channel
//  to resolve direction and updates a volatile count. Everything the ISR
//  touches is minimal and volatile so it stays interrupt-safe.
//
//  Units: raw encoder COUNTS (int32). mm conversion happens elsewhere.
// =====================================================================

class Encoder {
public:
  // pinA must be an interrupt-capable pin; pinB is read inside the ISR.
  Encoder(uint8_t pinA, uint8_t pinB);

  // Configure pin modes. Call from setup(); attachInterrupt is wired in
  // main (a free ISR function forwards to handleEdge()).
  void begin();

  // Called from the A-channel ISR on every edge (2x decode). Reads B and
  // increments/decrements the count. Keep this tiny — no Serial, no math.
  void handleEdge();

  // Current position in counts. Reads the volatile count atomically.
  long position() const;

  // Zero the position (e.g. after homing sets the datum).
  void reset();

private:
  const uint8_t _pinA;
  const uint8_t _pinB;
  volatile long _count;       // updated in ISR, read in main
};
