#include <aslam/cameras/DepthCameraGeometry.hpp>
#include <aslam/Exceptions.hpp>

namespace aslam {
namespace cameras {

DepthCameraGeometry::DepthCameraGeometry() {
  *this = createTestGeometry();
}

DepthCameraGeometry::DepthCameraGeometry(double focalLengthU,
                                         double focalLengthV,
                                         double imageCenterU,
                                         double imageCenterV, int resolutionU,
                                         int resolutionV)
    : _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV),
      _recip_fu(1.0 / _fu),
      _recip_fv(1.0 / _fv),
      _fu_over_fv(_fu / _fv) {
  // 0
}

DepthCameraGeometry::~DepthCameraGeometry() {

}

// This updates the intrinsic parameters with a small step: i <-- i + di
// The Jacobians above are with respect to this update function.
void DepthCameraGeometry::updateIntrinsicsOplus(double * di) {
  SM_THROW(NotImplementedException,
           "The pinhole camera object does not support optimizing intrinsics");
}

// The amount of time elapsed between the start of the image and the
// keypoint. For a global shutter camera, this can return Duration(0).
Duration DepthCameraGeometry::temporalOffset(
    const keypoint_t & keypoint) const {
  return Duration(0);
}

DepthCameraGeometry::keypoint_t DepthCameraGeometry::maxKeypoint() const {
  return keypoint_t(_ru, _rv, 1.0e3);  // that's kind of ridiculously a too large distance, but numerics don't allow smaller distances...
}

DepthCameraGeometry::keypoint_t DepthCameraGeometry::minKeypoint() const {
  return keypoint_t(0, 0, 1.0e-12);
}

std::string DepthCameraGeometry::typeName() const {
  return "PinholeCamera";
}

DepthCameraGeometry DepthCameraGeometry::createTestGeometry() {
  return DepthCameraGeometry(400, 400, 320, 240, 640, 480);
}

}  // namespace cameras  

}  // namespace aslam
