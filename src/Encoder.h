#pragma once
#include <Arduino.h>

// Quadrature decode and position for one motor. Written from scratch, no
// Encoder library. A hardware interrupt on the A channel drives it: the
// ISR calls handleEdge(), which reads B to work out which way we're
// turning and bumps the count. Anything the ISR touches is volatile and
// kept as short as possible.
//
// Raw counts only. mm conversion happens over in Kinematics.

class Encoder {
public:
  // pinA must be an interrupt-capable pin; pinB is read inside the ISR.
  Encoder(uint8_t pinA, uint8_t pinB);

  // Sets the pin modes. Call from setup(). attachInterrupt happens in
  // main, where a plain function forwards to handleEdge().
  void begin();

  // Called from the A-channel ISR on every edge. Reads B, then adds or
  // subtracts one. Keep it tiny: no Serial, no maths.
  void handleEdge();

  // Position in counts, read without an ISR landing halfway through.
  long position() const;

  // Zero the count, e.g. once homing has found the datum.
  void reset();

private:
  const uint8_t _pinA;
  const uint8_t _pinB;
  volatile long _count;       // updated in ISR, read in main
};
