#ifndef GLINESEGMENT2D_H
#define GLINESEGMENT2D_H

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <utility>

#include "apriltags/GLine2D.h"
#include "apriltags/XYWeight.h"

namespace AprilTags {

//! A 2D line with endpoints.
class GLineSegment2D {
public:
  GLineSegment2D(const std::pair<float,float> &p0Arg, const std::pair<float,float> &p1Arg);
  static GLineSegment2D lsqFitXYW(const std::vector<XYWeight>& xyweight);
  std::pair<float,float> getP0() const { return p0; }
  std::pair<float,float> getP1() const { return p1; }

private:
  GLine2D line;
  std::pair<float,float> p0;
  std::pair<float,float> p1;
  int weight;
};

} // namespace

#endif
