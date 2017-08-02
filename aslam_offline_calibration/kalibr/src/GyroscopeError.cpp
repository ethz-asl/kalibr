#include <kalibr_errorterms/GyroscopeError.hpp>

namespace kalibr_errorterms {

GyroscopeError::GyroscopeError(
		const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR,
		const aslam::backend::EuclideanExpression & angularVelocity,
		const aslam::backend::EuclideanExpression & bias) {

	_measurement = measurement;
	setInvR(invR);
	_predictedMeasurement = angularVelocity + bias;

	aslam::backend::DesignVariable::set_t dvs;
	_predictedMeasurement.getDesignVariables(dvs);
	setDesignVariablesIterator(dvs.begin(), dvs.end());
}

GyroscopeError::~GyroscopeError() {

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
		aslam::backend::JacobianContainer & _jacobians) const {
	_angularVelocity.evaluateJacobians(_jacobians);
}

GyroscopeErrorEccentric::GyroscopeErrorEccentric(
		const Eigen::Vector3d & measurement,
		const Eigen::Matrix3d & invR,
		const aslam::backend::MatrixExpression & M,
		const aslam::backend::MatrixExpression & Ma,
		const aslam::backend::RotationExpression & C_b_w,
		const aslam::backend::EuclideanExpression & acceleration_w,
		const aslam::backend::EuclideanExpression & angularVelocity_b,
		const aslam::backend::EuclideanExpression & angularAcceleration_b,
		const aslam::backend::RotationExpression & C_i_b,
		const aslam::backend::EuclideanExpression & r_b,
		const aslam::backend::EuclideanExpression & bias,
		const aslam::backend::EuclideanExpression & g_w
) {

	_measurement = measurement;
	setInvR(invR);
	_predictedMeasurement = M * (C_i_b * angularVelocity_b +
			Ma * (C_i_b * (C_b_w * ( acceleration_w - g_w)
					+ angularAcceleration_b.cross(r_b)
					+ angularVelocity_b.cross(angularVelocity_b.cross(r_b))))) + bias;

	aslam::backend::DesignVariable::set_t dvs;
	_predictedMeasurement.getDesignVariables(dvs);
	setDesignVariablesIterator(dvs.begin(), dvs.end());
}

GyroscopeErrorEccentric::~GyroscopeErrorEccentric() {

}

}  // namespace kalibr_errorterms
