#pragma once
#include <Arduino.h>

// CoreXY to Cartesian and back, written from scratch. There's no state,
// so everything is static and there's nothing to construct. The counts/mm
// helpers live in here too, so the G-code side and the control loop use
// the same scale.
//
//     dA = dX + dY          dX = (dA + dB) / 2
//     dB = dX - dY          dY = (dA - dB) / 2
//
// A is the left motor, B the right (pinout §9). Counts everywhere unless
// the name says mm.

struct AxisPair { long a; long b; };   // motor-space (A, B)
struct Point    { long x; long y; };   // Cartesian counts

class Kinematics {
public:
  // Cartesian (counts) -> motor (counts).  a = x + y ; b = x - y
  static AxisPair xyToAB(const Point& xy);

  // Motor (counts) -> Cartesian (counts).  x = (a+b)/2 ; y = (a-b)/2
  static Point    abToXY(const AxisPair& ab);

  // Scale helpers shared with the G-code boundary.
  static long mmToCounts(float mm);
  static float countsToMm(long counts);
  
};
