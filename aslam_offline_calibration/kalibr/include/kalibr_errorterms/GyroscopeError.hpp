#ifndef KALIBR_IMU_CAM_GYROSCOPE_ERROR_HPP
#define KALIBR_IMU_CAM_GYROSCOPE_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/MatrixExpression.hpp>

#include <kalibr_errorterms/EuclideanError.hpp>

namespace kalibr_errorterms {

class GyroscopeError : public EuclideanError {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GyroscopeError(const Eigen::Vector3d & measurement,
                 const Eigen::Matrix3d & invR,
                 const aslam::backend::EuclideanExpression & angularVelocity,
                 const aslam::backend::EuclideanExpression & bias);
  virtual ~GyroscopeError();
};

class GyroscopeErrorEccentric : public EuclideanError {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GyroscopeErrorEccentric(
    const Eigen::Vector3d & measurement,
    const Eigen::Matrix3d & invR,
	const aslam::backend::MatrixExpression & M,
	const aslam::backend::MatrixExpression & Ma,
	const aslam::backend::RotationExpression & C_b_w,
	const aslam::backend::EuclideanExpression & acceleration_w,
	const aslam::backend::EuclideanExpression & angularVelocity_b,
	const aslam::backend::EuclideanExpression & angularAcceleration_b,
	const aslam::backend::RotationExpression & C_i_b,
	const  aslam::backend::EuclideanExpression & r_b,
	const aslam::backend::EuclideanExpression & bias,
	const aslam::backend::EuclideanExpression & g_w
    );

  virtual ~GyroscopeErrorEccentric();
};


class GyroscopeNoBiasError : public aslam::backend::ErrorTermFs<3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GyroscopeNoBiasError(
      const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR,
      const aslam::backend::EuclideanExpression & angularVelocity);
  ~GyroscopeNoBiasError();

 protected:
  /// \brief evaluate the error term and return the weighted squared error e^T invR e
  virtual double evaluateErrorImplementation();

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(
      aslam::backend::JacobianContainer & _jacobians) const;
 private:
  Eigen::Vector3d _measurement;
  aslam::backend::EuclideanExpression _angularVelocity;
};

} //namespace kalibr_errorterms

#endif /* KALIBR_IMU_CAM_GYROSCOPE_ERROR_HPP */
