#ifndef KALIBR_IMU_CAM_EUCLIDEAN_ERROR_HPP
#define KALIBR_IMU_CAM_EUCLIDEAN_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>

namespace kalibr_errorterms {

class EuclideanError : public aslam::backend::ErrorTermFs<3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EuclideanError() : _predictedMeasurement(NULL) {}

  EuclideanError(const Eigen::Vector3d & measurement,
                 const Eigen::Matrix3d invR,
                 const aslam::backend::EuclideanExpression & predicted_measurement);
  virtual ~EuclideanError() {}

  /// \brief return predicted measurement
  Eigen::Vector3d getPredictedMeasurement();

  /// \brief return measurement
  Eigen::Vector3d getMeasurement();

 protected:
  /// \brief evaluate the error term and return the weighted squared error e^T invR e
  virtual double evaluateErrorImplementation();

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(
      aslam::backend::JacobianContainer & _jacobians) const;

  Eigen::Vector3d _measurement;
  aslam::backend::EuclideanExpression _predictedMeasurement;
};

} //namespace kalibr_errorterms

#endif /* KALIBR_IMU_CAM_EUCLIDEAN_ERROR_HPP */
