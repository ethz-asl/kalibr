#include <kalibr_errorterms/EuclideanError.hpp>

namespace kalibr_errorterms {

  EuclideanError::EuclideanError(const Eigen::Vector3d & measurement,
                 const Eigen::Matrix3d invR,
                 const aslam::backend::EuclideanExpression & predicted_measurement)
    : _measurement(measurement), _predictedMeasurement(predicted_measurement) {

    setInvR(invR);

    aslam::backend::DesignVariable::set_t dvs;
    _predictedMeasurement.getDesignVariables(dvs);
    setDesignVariablesIterator(dvs.begin(), dvs.end());
  }

/// \brief evaluate the error term and return the weighted squared error e^T invR e
double EuclideanError::evaluateErrorImplementation() {
	setError(_predictedMeasurement.toEuclidean() - _measurement);
	return evaluateChiSquaredError();
}

/// \brief evaluate the jacobian
void EuclideanError::evaluateJacobiansImplementation(
		aslam::backend::JacobianContainer & _jacobians) const {
	_predictedMeasurement.evaluateJacobians(_jacobians);
}

/// \brief return predicted measurement
Eigen::Vector3d EuclideanError::getPredictedMeasurement() {
	return _predictedMeasurement.evaluate();
}

/// \brief return measurement
Eigen::Vector3d EuclideanError::getMeasurement() {
	return _measurement;
}
}  // namespace kalibr_errorterms

