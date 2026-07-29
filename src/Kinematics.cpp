#include "Kinematics.h"
#include "Pins.h"

AxisPair Kinematics::xyToAB(const Point& xy) {
  // CoreXY forward: a = x + y, b = x - y.
  return AxisPair{ xy.x + xy.y, xy.x - xy.y };
}

Point Kinematics::abToXY(const AxisPair& ab) {
  // CoreXY inverse: x = (a+b)/2, y = (a-b)/2. For any pose reachable via
  // xyToAB, (a+b) and (a-b) are even, so the integer division is exact.
  return Point{ (ab.a + ab.b) / 2, (ab.a - ab.b) / 2 };
}

long Kinematics::mmToCounts(float mm) {
  return lroundf(mm * cfg::COUNTS_PER_MM);
}

float Kinematics::countsToMm(long counts) {
  return static_cast<float>(counts) / cfg::COUNTS_PER_MM;
}
