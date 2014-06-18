//-*-c++-*-

#ifndef GRAYMODEL_H
#define GRAYMODEL_H

#include <Eigen/Dense>
#include <vector>

namespace AprilTags {

//! Fits a grayscale model over an area of pixels.
/*! The model is of the form: c1*x + c2*y + c3*x*y + c4 = value
 *
 * We use this model to compute spatially-varying thresholds for
 * reading bits.
 */
class GrayModel {
public:
  GrayModel();

  void addObservation(float x, float y, float gray);

  inline int getNumObservations() { return nobs; }

  float interpolate(float x, float y);

private:
  void compute();

// We're solving Av = b.  
//
// For each observation, we add a row to A of the form [x y xy 1]
// and to b of the form gray*[x y xy 1].  v is the vector [c1 c2 c3 c4].
//
// The least-squares solution to the system is v = inv(A'A)A'b

  Eigen::Matrix4d A;
  Eigen::Vector4d v;
  Eigen::Vector4d b;
  int nobs;
  bool dirty;  //!< True if we've added an observation and need to recompute v
};

} // namespace

#endif
