#include "Kinematics.h"
#include "Pins.h"

AxisPair Kinematics::xyToAB(const Point& xy) {
  // TODO: return { xy.x + xy.y, xy.x - xy.y }.
  (void)xy;
  return AxisPair{0, 0};
}

Point Kinematics::abToXY(const AxisPair& ab) {
  // TODO: return { (ab.a + ab.b) / 2, (ab.a - ab.b) / 2 }.
  (void)ab;
  return Point{0, 0};
}

long Kinematics::mmToCounts(float mm) {
  // TODO: lroundf(mm * cfg::COUNTS_PER_MM).
  (void)mm;
  return 0;
}

float Kinematics::countsToMm(long counts) {
  // TODO: counts / cfg::COUNTS_PER_MM.
  (void)counts;
  return 0.0f;
}
