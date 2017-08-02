#ifndef ASLAM_BACKEND_CAMERA_REPROJECTION_ERROR_HPP
#define ASLAM_BACKEND_CAMERA_REPROJECTION_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <boost/shared_ptr.hpp>
#include <aslam/Frame.hpp>

namespace aslam {
namespace backend {

template<typename CAMERA_GEOMETRY_T>
class ReprojectionError : public ErrorTermFs<
    CAMERA_GEOMETRY_T::KeypointDimension> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef CAMERA_GEOMETRY_T camera_geometry_t;

  enum {
    KeypointDimension = camera_geometry_t::KeypointDimension /*!< The dimension of the keypoint associated with this geometry policy */
  };

  typedef Eigen::Matrix<double, KeypointDimension, 1> measurement_t;
  typedef Eigen::Matrix<double, KeypointDimension, KeypointDimension> inverse_covariance_t;
  typedef ErrorTermFs<KeypointDimension> parent_t;
  typedef aslam::Frame<camera_geometry_t> frame_t;

  ReprojectionError();
  ReprojectionError(const measurement_t & measurement,
                    const inverse_covariance_t & inverseCovariance,
                    const HomogeneousExpression & point,
                    CameraDesignVariable<camera_geometry_t> camera);
  ReprojectionError(const frame_t * frame, int keypointIndex,
                    HomogeneousExpression point,
                    CameraDesignVariable<camera_geometry_t> camera);

  virtual ~ReprojectionError();

  void updateMeasurement(const measurement_t & measurement);
  void updateMeasurementAndCovariance(const measurement_t & measurement,
                                      const inverse_covariance_t & invR);
  measurement_t getMeasurement() const;
  measurement_t getPredictedMeasurement();
 protected:
  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation();

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(
      aslam::backend::JacobianContainer & J) const;

  measurement_t _y;

  /// \brief the homogeneous point expressed in the camera frame
  HomogeneousExpression _point;

  CameraDesignVariable<camera_geometry_t> _camera;

};
}  // namespace backend
}  // namespace aslam

#include "implementation/ReprojectionError.hpp"

#endif /* ASLAM_BACKEND_CAMERA_REPROJECTION_ERROR_HPP */
