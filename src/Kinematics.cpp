#include "Kinematics.h"
#include "Pins.h"

AxisPair Kinematics::xyToAB(const Point& xy) {
  // CoreXY forward: a = x + y, b = x - y.
  return AxisPair{ xy.x + xy.y, xy.x - xy.y };
}

Point Kinematics::abToXY(const AxisPair& ab) {
  // Inverse: x = (a+b)/2, y = (a-b)/2. Anything that came out of xyToAB
  // has (a+b) and (a-b) even, so the integer divide loses nothing.
  return Point{ (ab.a + ab.b) / 2, (ab.a - ab.b) / 2 };
}

long Kinematics::mmToCounts(float mm) {
  return lroundf(mm * cfg::COUNTS_PER_MM);
}

float Kinematics::countsToMm(long counts) {
  return static_cast<float>(counts) / cfg::COUNTS_PER_MM;
}
