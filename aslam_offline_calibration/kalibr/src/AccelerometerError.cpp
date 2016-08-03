#include <kalibr_errorterms/AccelerometerError.hpp>

namespace kalibr_errorterms {

// error = C_b_w(g - alpha) + bias - measurement;
AccelerometerError::AccelerometerError(
		const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR,
		const aslam::backend::RotationExpression & C_b_w,
		const aslam::backend::EuclideanExpression & acceleration_w,
		const aslam::backend::EuclideanExpression & bias,
		const aslam::backend::EuclideanExpression & g_w) {
	_measurement = measurement;
	setInvR(invR);
	_predictedMeasurement = C_b_w * (acceleration_w - g_w) + bias;

	aslam::backend::DesignVariable::set_t dvs;
	_predictedMeasurement.getDesignVariables(dvs);
	setDesignVariablesIterator(dvs.begin(), dvs.end());
}

AccelerometerError::~AccelerometerError() {
}

AccelerometerErrorEccentric::AccelerometerErrorEccentric(
		const Eigen::Vector3d & measurement,
		const Eigen::Matrix3d & invR,
		const aslam::backend::MatrixExpression & M,
		const aslam::backend::RotationExpression & C_b_w,
		const aslam::backend::EuclideanExpression & acceleration_w,
		const aslam::backend::EuclideanExpression & angularVelocity_b,
		const aslam::backend::EuclideanExpression & angularAcceleration_b,
		const aslam::backend::RotationExpression & C_i_b,
		const aslam::backend::EuclideanExpression & rx_b,
		const aslam::backend::EuclideanExpression & ry_b,
		const aslam::backend::EuclideanExpression & rz_b,
		const aslam::backend::EuclideanExpression & bias,
		const aslam::backend::EuclideanExpression & g_w) : _Ix(aslam::backend::MatrixBasic(Eigen::Vector3d(1.,0.,0.).asDiagonal(), Eigen::Matrix3i::Zero())),
				_Iy(aslam::backend::MatrixBasic(Eigen::Vector3d(0.,1.,0.).asDiagonal(), Eigen::Matrix3i::Zero())),
				_Iz(aslam::backend::MatrixBasic(Eigen::Vector3d(0.,0.,1.).asDiagonal(), Eigen::Matrix3i::Zero()))
{
	_measurement = measurement;
	setInvR(invR);
	_predictedMeasurement = M * (C_i_b * (C_b_w * ( acceleration_w - g_w)
			+ _Ix.toExpression() * (angularAcceleration_b.cross(rx_b) + angularVelocity_b.cross(angularVelocity_b.cross(rx_b)))
			+ _Iy.toExpression() * (angularAcceleration_b.cross(ry_b) + angularVelocity_b.cross(angularVelocity_b.cross(ry_b)))
			+ _Iz.toExpression() * (angularAcceleration_b.cross(rz_b) + angularVelocity_b.cross(angularVelocity_b.cross(rz_b)))
	)) + bias;

	aslam::backend::DesignVariable::set_t dvs;
	_predictedMeasurement.getDesignVariables(dvs);
	setDesignVariablesIterator(dvs.begin(), dvs.end());
}

AccelerometerErrorEccentric::~AccelerometerErrorEccentric() {
}

}  // namespace kalibr_errorterms

