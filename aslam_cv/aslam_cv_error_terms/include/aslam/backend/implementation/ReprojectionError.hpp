namespace aslam {
namespace backend {

template<typename F>
ReprojectionError<F>::ReprojectionError() {

}

template<typename F>
ReprojectionError<F>::ReprojectionError(
    const frame_t * frame, int keypointIndex, HomogeneousExpression point,
    CameraDesignVariable<camera_geometry_t> camera)
    : _point(point),
      _camera(camera) {
  SM_ASSERT_TRUE(Exception, frame != NULL, "The frame must not be null");
  _y = frame->keypoint(keypointIndex).y();
  parent_t::setInvR(frame->keypoint(keypointIndex).invR());
  JacobianContainer::set_t dvs;
  point.getDesignVariables(dvs);	// point dv's
  camera.getDesignVariables(dvs);  // camera dv's

  parent_t::setDesignVariablesIterator(dvs.begin(), dvs.end());

}

template<typename F>
ReprojectionError<F>::ReprojectionError(
    const measurement_t & measurement,
    const inverse_covariance_t & inverseCovariance,
    const HomogeneousExpression & point,
    CameraDesignVariable<camera_geometry_t> camera)
    : _y(measurement),
      _point(point),
      _camera(camera) {
  parent_t::setInvR(inverseCovariance);
  JacobianContainer::set_t dvs;
  point.getDesignVariables(dvs);	// point dv's
  camera.getDesignVariables(dvs);  // camera dv's

  parent_t::setDesignVariablesIterator(dvs.begin(), dvs.end());

}

template<typename F>
ReprojectionError<F>::~ReprojectionError() {

}

template<typename F>
double ReprojectionError<F>::evaluateErrorImplementation() {
  const camera_geometry_t & cam = *_camera.camera();

  Eigen::Vector4d p = _point.toHomogeneous();
  measurement_t hat_y;
  cam.homogeneousToKeypoint(p, hat_y);

  parent_t::setError(_y - hat_y);

  return parent_t::error().dot(parent_t::invR() * parent_t::error());
}

template<typename F>
void ReprojectionError<F>::evaluateJacobiansImplementation(
    aslam::backend::JacobianContainer & _jacobians) const {
  //const keypoint_t & k = _frame->keypoint(_keypointIndex);
  const camera_geometry_t & cam = *_camera.camera();

  Eigen::Vector4d p = _point.toHomogeneous();
  typename camera_geometry_t::jacobian_homogeneous_t J;
  measurement_t hat_y;
  cam.homogeneousToKeypoint(p, hat_y, J);

  _point.evaluateJacobians(_jacobians, -J);

  _camera.evaluateJacobians(_jacobians, p);

}

template<typename F>
void ReprojectionError<F>::updateMeasurement(
    const measurement_t & measurement) {
  _y = measurement;
}

template<typename F>
void ReprojectionError<F>::updateMeasurementAndCovariance(
    const measurement_t & measurement, const inverse_covariance_t & invR) {
  _y = measurement;
  parent_t::setInvR(invR);
}

template<typename F>
typename ReprojectionError<F>::measurement_t ReprojectionError<F>::getMeasurement() const {
  return _y;
}

template<typename F>
typename ReprojectionError<F>::measurement_t ReprojectionError<F>::getPredictedMeasurement() {
  const camera_geometry_t & cam = *_camera.camera();

  Eigen::Vector4d p = _point.toHomogeneous();
  measurement_t hat_y;
  cam.homogeneousToKeypoint(p, hat_y);

  return hat_y;
}

}  // namespace backend
}  // namespace aslam
