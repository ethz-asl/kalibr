#include <kalibr_errorterms/GyroscopeError.hpp>

namespace kalibr_errorterms {

GyroscopeError::GyroscopeError(
    const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR,
    const aslam::backend::EuclideanExpression & angularVelocity,
    const aslam::backend::EuclideanExpression & bias)
    : _measurement(measurement),
      _predictedMeasurement(NULL){

  //measurement_hat =  omega + bias
  setInvR(invR);
  _predictedMeasurement = angularVelocity + bias;

  aslam::backend::DesignVariable::set_t dvs;
  _predictedMeasurement.getDesignVariables(dvs);
  setDesignVariablesIterator(dvs.begin(), dvs.end());
}

GyroscopeError::~GyroscopeError() {

}

double GyroscopeError::evaluateErrorImplementation() {
  // e =  _predictedMeasurement - measurement
  setError(_predictedMeasurement.toEuclidean() - _measurement);
  return evaluateChiSquaredError();
}

void GyroscopeError::evaluateJacobiansImplementation(
    aslam::backend::JacobianContainer & _jacobians) {
  _predictedMeasurement.evaluateJacobians(_jacobians);
}

/// \brief return predicted measurement
Eigen::Vector3d GyroscopeError::getPredictedMeasurement() {
  return _predictedMeasurement.evaluate();
}

/// \brief return measurement
Eigen::Vector3d GyroscopeError::getMeasurement() {
  return _measurement;
}

/////////////////////////////////////////////////////////////////////////////////////////////

GyroscopeNoBiasError::GyroscopeNoBiasError(
    const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR,
    const aslam::backend::EuclideanExpression & angularVelocity)
    : _measurement(measurement),
      _angularVelocity(angularVelocity) {
  setInvR(invR);

  aslam::backend::DesignVariable::set_t dvs;
  _angularVelocity.getDesignVariables(dvs);
  setDesignVariablesIterator(dvs.begin(), dvs.end());
}

GyroscopeNoBiasError::~GyroscopeNoBiasError() {

}

double GyroscopeNoBiasError::evaluateErrorImplementation() {
  // e =  omega + bias - measurement
  setError(_angularVelocity.toEuclidean() - _measurement);

  return evaluateChiSquaredError();
}

void GyroscopeNoBiasError::evaluateJacobiansImplementation(
    aslam::backend::JacobianContainer & _jacobians) {
  _angularVelocity.evaluateJacobians(_jacobians);
}

}  // namespace kalibr_errorterms
