#pragma once
#include <Arduino.h>

// =====================================================================
//  Module 7 — Kinematics
// ---------------------------------------------------------------------
//  Pure CoreXY <-> Cartesian transforms, our own code. Stateless, so the
//  methods are static — there is nothing to construct. Also holds the
//  counts<->mm helpers so the G-code edge and the control loop share one
//  definition of scale.
//
//      dA = dX + dY          dX = (dA + dB) / 2
//      dB = dX - dY          dY = (dA - dB) / 2
//
//  A = left motor, B = right motor (see pinout §9). All axis quantities
//  here are in COUNTS unless a name says mm.
// =====================================================================

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
