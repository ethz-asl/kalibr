#ifndef ASLAM_BACKEND_SIMPLE_REPROJECTION_ERROR_HPP
#define ASLAM_BACKEND_SIMPLE_REPROJECTION_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <boost/shared_ptr.hpp>

namespace aslam {
namespace backend {

template<typename FRAME_T>
class SimpleReprojectionError : public ErrorTermFs<FRAME_T::KeypointDimension> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef FRAME_T frame_t;
  typedef typename frame_t::keypoint_t keypoint_t;
  typedef typename frame_t::camera_geometry_t camera_geometry_t;
  enum {
    KeypointDimension = frame_t::KeypointDimension /*!< The dimension of the keypoint associated with this geometry policy */
  };

  typedef Eigen::Matrix<double, KeypointDimension, 1> measurement_t;
  typedef Eigen::Matrix<double, KeypointDimension, KeypointDimension> inverse_covariance_t;
  typedef ErrorTermFs<KeypointDimension> parent_t;

  SimpleReprojectionError();
  SimpleReprojectionError(const frame_t * frame, int keypointIndex,
                          HomogeneousExpression point);
  SimpleReprojectionError(const measurement_t & y,
                          const inverse_covariance_t & invR,
                          HomogeneousExpression point,
                          const camera_geometry_t & geometry);

  virtual ~SimpleReprojectionError();

 protected:
  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation();

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(
      aslam::backend::JacobianContainer & _jacobians) const;

  /// \brief the frame that this measurement comes from.
  measurement_t _y;

  /// \brief The camera geometry
  const camera_geometry_t * _geometry;

  /// \brief the homogeneous point expressed in the camera frame
  HomogeneousExpression _point;

};
}  // namespace backend
}  // namespace aslam

#include "implementation/SimpleReprojectionError.hpp"

#endif /* ASLAM_BACKEND_SIMPLE_REPROJECTION_ERROR_HPP */
