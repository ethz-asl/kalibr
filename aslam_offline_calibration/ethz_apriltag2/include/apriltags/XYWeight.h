#ifndef XYWeight_H_
#define XYWeight_H_

namespace AprilTags {

//! Represents a triple holding an x value, y value, and weight value.
struct XYWeight {
  float x;
  float y;
  float weight;

  XYWeight(float xval, float yval, float weightval) :
    x(xval), y(yval), weight(weightval) {}

};

} // namespace

#endif
