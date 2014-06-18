// src
// Luc Oth 04/12
// othlu@student.ethz.ch

#include <aslam/cameras/PinholeRSCameraGeometry.hpp>
#include <aslam/Exceptions.hpp>
#include <Eigen/Dense>

namespace aslam {
namespace cameras {

PinholeRSCameraGeometry::PinholeRSCameraGeometry() {
  *this = createTestGeometry();
}

PinholeRSCameraGeometry::PinholeRSCameraGeometry(double focalLengthU,
                                                 double focalLengthV,
                                                 double imageCenterU,
                                                 double imageCenterV,
                                                 int resolutionU,
                                                 int resolutionV,
                                                 double lineDelay)
    : _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV),
      _enableDistortion(0),
      _recip_fu(1.0 / _fu),
      _recip_fv(1.0 / _fv),
      _fu_over_fv(_fu / _fv),
      _lineDelay(lineDelay) {
  // 0
}

PinholeRSCameraGeometry::PinholeRSCameraGeometry(double focalLengthU,
                                                 double focalLengthV,
                                                 double imageCenterU,
                                                 double imageCenterV,
                                                 int resolutionU,
                                                 int resolutionV,
                                                 double lineDelay, double k1,
                                                 double k2, double p1,
                                                 double p2)
    : _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV),
      _k1(k1),
      _k2(k2),
      _p1(p1),
      _p2(p2),
      _enableDistortion(1),
      _recip_fu(1.0 / _fu),
      _recip_fv(1.0 / _fv),
      _fu_over_fv(_fu / _fv),
      _lineDelay(lineDelay) {
  // 0
}

PinholeRSCameraGeometry::~PinholeRSCameraGeometry() {

}

// This updates the intrinsic parameters with a small step: i <-- i + di
// The Jacobians above are with respect to this update function.
void PinholeRSCameraGeometry::updateIntrinsicsOplus(double * di) {
  SM_THROW(NotImplementedException,
           "The pinhole camera object does not support optimizing intrinsics");
}

// The amount of time elapsed between the start of the image and the
// keypoint. For a global shutter camera, this can return Duration(0).
Duration PinholeRSCameraGeometry::temporalOffset(
    const keypoint_t & keypoint) const {

  // time offset only depends on the line => keypoint(1)
  return Duration(_lineDelay * keypoint(1));

}

PinholeRSCameraGeometry::keypoint_t PinholeRSCameraGeometry::maxKeypoint() const {
  return keypoint_t(_ru, _rv);
}

PinholeRSCameraGeometry::keypoint_t PinholeRSCameraGeometry::minKeypoint() const {
  return keypoint_t(0, 0);
}

std::string PinholeRSCameraGeometry::typeName() const {
  return "PinholeRSCamera";
}

PinholeRSCameraGeometry PinholeRSCameraGeometry::createTestGeometry() {
  return PinholeRSCameraGeometry(400, 400, 320, 240, 640, 480, 0.0005);
}

PinholeRSCameraGeometry PinholeRSCameraGeometry::createDistortedTestGeometry() {
  return PinholeRSCameraGeometry(400, 400, 320, 240, 640, 480, 0.0005, 0.2, 0.2,
                                 0.2, 0.2);
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *  
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void PinholeRSCameraGeometry::distortion(double mx_u, double my_u, double *dx_u,
                                         double *dy_u) const {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mx_u * mx_u;
  my2_u = my_u * my_u;
  mxy_u = mx_u * my_u;
  rho2_u = mx2_u + my2_u;
  rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;
  *dx_u = mx_u * rad_dist_u + 2 * _p1 * mxy_u + _p2 * (rho2_u + 2 * mx2_u);
  *dy_u = my_u * rad_dist_u + 2 * _p2 * mxy_u + _p1 * (rho2_u + 2 * my2_u);
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate jacobian
 *
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void PinholeRSCameraGeometry::distortion(double mx_u, double my_u, double *dx_u,
                                         double *dy_u, double *dxdmx,
                                         double *dydmx, double *dxdmy,
                                         double *dydmy) const {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mx_u * mx_u;
  my2_u = my_u * my_u;
  mxy_u = mx_u * my_u;
  rho2_u = mx2_u + my2_u;
  rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;
  *dx_u = mx_u * rad_dist_u + 2 * _p1 * mxy_u + _p2 * (rho2_u + 2 * mx2_u);
  *dy_u = my_u * rad_dist_u + 2 * _p2 * mxy_u + _p1 * (rho2_u + 2 * my2_u);

  *dxdmx = 1 + rad_dist_u + _k1 * 2 * mx2_u + _k2 * rho2_u * 4 * mx2_u
      + 2 * _p1 * my_u + 6 * _p2 * mx_u;
  *dydmx = _k1 * 2 * mx_u * my_u + _k2 * 4 * rho2_u * mx_u * my_u
      + _p1 * 2 * mx_u + 2 * _p2 * my_u;
  *dxdmy = *dydmx;
  *dydmy = 1 + rad_dist_u + _k1 * 2 * my2_u + _k2 * rho2_u * 4 * my2_u
      + 6 * _p1 * my_u + 2 * _p2 * mx_u;
}

// Use Gauss-Newton to undistort.
void PinholeRSCameraGeometry::undistortGN(double u_d, double v_d, double * u,
                                          double * v) const {
  *u = u_d;
  *v = v_d;

  double ubar = u_d;
  double vbar = v_d;
  const int n = 5;
  Eigen::Matrix2d F;

  double hat_u_d;
  double hat_v_d;

  for (int i = 0; i < n; i++) {
    distortion(ubar, vbar, &hat_u_d, &hat_v_d, &F(0, 0), &F(1, 0), &F(0, 1),
               &F(1, 1));

    Eigen::Vector2d e(u_d - ubar - hat_u_d, v_d - vbar - hat_v_d);
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;

    ubar += du[0];
    vbar += du[1];

    if (e.dot(e) < 1e-15)
      break;

  }
  *u = ubar;
  *v = vbar;
}

void PinholeRSCameraGeometry::setIntrinsicsVectorImplementation(
    const PinholeRSCameraGeometry::traits_t::intrinsics_t & intrinsics) {
  _fu = intrinsics(0);
  _fv = intrinsics(1);
  _cu = intrinsics(2);
  _cv = intrinsics(3);
  _k1 = intrinsics(4);
  _k2 = intrinsics(5);
  _p1 = intrinsics(6);
  _p2 = intrinsics(7);
  _recip_fu = 1.0 / _fu;
  _recip_fv = 1.0 / _fv;
  _fu_over_fv = _fu / _fv;

}

PinholeRSCameraGeometry::traits_t::intrinsics_t PinholeRSCameraGeometry::getIntrinsicsVectorImplementation() const {
  traits_t::intrinsics_t intrinsics;
  intrinsics(0) = _fu;
  intrinsics(1) = _fv;
  intrinsics(2) = _cu;
  intrinsics(3) = _cv;
  intrinsics(4) = _k1;
  intrinsics(5) = _k2;
  intrinsics(6) = _p1;
  intrinsics(7) = _p2;
  return intrinsics;
}

}  // namespace cameras  

}  // namespace aslam
