#include <kalibr_errorterms/AccelerometerError.hpp>

namespace kalibr_errorterms {

// error = C_b_w(g - alpha) + bias - measurement;
AccelerometerError::AccelerometerError(
    const Eigen::Vector3d & measurement, const Eigen::Matrix3d invR,
    aslam::backend::RotationExpression C_b_w,
    aslam::backend::EuclideanExpression acceleration_w,
    aslam::backend::EuclideanExpression bias,
    aslam::backend::EuclideanExpression g_w)
    : _measurement(measurement),
      _predictedMeasurement(NULL) {

  setInvR(invR);
  _predictedMeasurement = C_b_w * (acceleration_w - g_w) + bias;

  aslam::backend::DesignVariable::set_t dvs;
  _predictedMeasurement.getDesignVariables(dvs);
  setDesignVariablesIterator(dvs.begin(), dvs.end());
}

AccelerometerError::~AccelerometerError() {
}

/// \brief evaluate the error term and return the weighted squared error e^T invR e
double AccelerometerError::evaluateErrorImplementation() {
  setError(_predictedMeasurement.toEuclidean() - _measurement);
  return evaluateChiSquaredError();
}

/// \brief evaluate the jacobian
void AccelerometerError::evaluateJacobiansImplementation(
    aslam::backend::JacobianContainer & _jacobians) {
  _predictedMeasurement.evaluateJacobians(_jacobians);
}

/// \brief return predicted measurement
Eigen::Vector3d AccelerometerError::getPredictedMeasurement() {
  return _predictedMeasurement.evaluate();
}

/// \brief return measurement
Eigen::Vector3d AccelerometerError::getMeasurement() {
  return _measurement;
}

}  // namespace kalibr_errorterms
