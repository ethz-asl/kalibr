#include <iostream>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "apriltags/GrayModel.h"

namespace AprilTags {

GrayModel::GrayModel() : A(), v(), b(), nobs(0), dirty(false) {
  A.setZero();
  v.setZero();
  b.setZero();
}

void GrayModel::addObservation(float x, float y, float gray) {
  float xy = x*y;

  // update only upper-right elements. A'A is symmetric,
  // we'll fill the other elements in later.
  A(0,0) += x*x;
  A(0,1) += x*y;
  A(0,2) += x*xy;
  A(0,3) += x;
  A(1,1) += y*y;
  A(1,2) += y*xy;
  A(1,3) += y;
  A(2,2) += xy*xy;
  A(2,3) += xy;
  A(3,3) += 1;
  
  b[0] += x*gray;
  b[1] += y*gray;
  b[2] += xy*gray;
  b[3] += gray;

  nobs++;
  dirty = true;
}

float GrayModel::interpolate(float x, float y) {
  if (dirty) compute();
  return v[0]*x + v[1]*y + v[2]*x*y + v[3];
}

void GrayModel::compute() {
  // we really only need 4 linearly independent observations to fit our answer, but we'll be very
  // sensitive to noise if we don't have an over-determined system. Thus, require at least 6
  // observations (or we'll use a constant model below).

  dirty = false;
  if (nobs >= 6) {
    // make symmetric
    Eigen::Matrix4d Ainv;
    for (int i = 0; i < 4; i++)
      for (int j = i+1; j < 4; j++)
        A(j,i) = A(i,j);

    //    try {
    //      Ainv = A.inverse();
    bool invertible;
    double det_unused;
    A.computeInverseAndDetWithCheck(Ainv, det_unused, invertible);
    if (invertible) {
      v = Ainv * b;
      return;
    }
    std::cerr << "AprilTags::GrayModel::compute() has underflow in matrix inverse\n";
    //    }
    //    catch (std::underflow_error&) {
    //      std::cerr << "AprilTags::GrayModel::compute() has underflow in matrix inverse\n";
    //    }
  }

  // If we get here, either nobs < 6 or the matrix inverse generated
  // an underflow, so use a constant model.
  v.setZero();   // need the cast to avoid operator= ambiguity wrt. const-ness
  v[3] = b[3] / nobs;      
}

} // namespace
