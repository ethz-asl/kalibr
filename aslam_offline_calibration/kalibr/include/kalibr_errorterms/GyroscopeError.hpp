#ifndef KALIBR_IMU_CAM_GYROSCOPE_ERROR_HPP
#define KALIBR_IMU_CAM_GYROSCOPE_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>

namespace kalibr_errorterms {

class GyroscopeError : public aslam::backend::ErrorTermFs<3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GyroscopeError(const Eigen::Vector3d & measurement,
                 const Eigen::Matrix3d & invR,
                 const aslam::backend::EuclideanExpression & angularVelocity,
                 const aslam::backend::EuclideanExpression & bias);
  virtual ~GyroscopeError();

  /// \brief return predicted measurement
  Eigen::Vector3d getPredictedMeasurement();

  /// \brief return measurement
  Eigen::Vector3d getMeasurement();

 protected:
  /// \brief evaluate the error term and return the weighted squared error e^T invR e
  virtual double evaluateErrorImplementation();

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(
      aslam::backend::JacobianContainer & _jacobians);

 private:
  Eigen::Vector3d _measurement;
  aslam::backend::EuclideanExpression _predictedMeasurement;
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
      aslam::backend::JacobianContainer & _jacobians);
 private:
  Eigen::Vector3d _measurement;
  aslam::backend::EuclideanExpression _angularVelocity;
};

} //namespace kalibr_errorterms

#endif /* KALIBR_IMU_CAM_GYROSCOPE_ERROR_HPP */
