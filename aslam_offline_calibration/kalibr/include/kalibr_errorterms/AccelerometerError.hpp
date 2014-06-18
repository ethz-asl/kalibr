#ifndef KALIBR_IMU_CAM_ACCELEROMETER_ERROR_HPP
#define KALIBR_IMU_CAM_ACCELEROMETER_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>

namespace kalibr_errorterms {

class AccelerometerError : public aslam::backend::ErrorTermFs<3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // error = C_b_w(g - alpha) + bias - measurement;
  AccelerometerError(const Eigen::Vector3d & measurement,
                     const Eigen::Matrix3d invR,
                     aslam::backend::RotationExpression C_b_w,
                     aslam::backend::EuclideanExpression acceleration_w,
                     aslam::backend::EuclideanExpression bias,
                     aslam::backend::EuclideanExpression g_w);
  virtual ~AccelerometerError();

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

}  // namespace kalibr_errorterms

#endif /* KALIBR_IMU_CAM_ACCELEROMETER_ERROR_HPP */
