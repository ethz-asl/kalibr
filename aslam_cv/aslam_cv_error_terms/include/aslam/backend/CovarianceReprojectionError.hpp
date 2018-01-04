#ifndef ASLAM_BACKEND_COVARIANCE_REPROJECTION_ERROR_HPP
#define ASLAM_BACKEND_COVARIANCE_REPROJECTION_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <boost/shared_ptr.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/kinematics/transformations.hpp>
#include <aslam/backend/Scalar.hpp>

namespace aslam {
  namespace backend {

    template<typename FRAME_T>
    class CovarianceReprojectionError : public ErrorTermFs< FRAME_T::KeypointDimension >
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);


      typedef FRAME_T frame_t;
      typedef typename frame_t::keypoint_t keypoint_t;
      typedef typename frame_t::camera_geometry_t camera_geometry_t;
      typedef aslam::splines::BSplinePoseDesignVariable spline_t;

      enum {
	KeypointDimension = frame_t::KeypointDimension /*!< The dimension of the keypoint associated with this geometry policy */
      };

      typedef Eigen::Matrix<double, KeypointDimension, 1> measurement_t;
      typedef Eigen::Matrix<double, KeypointDimension, KeypointDimension> inverse_covariance_t;
      typedef ErrorTermFs< KeypointDimension > parent_t;

      CovarianceReprojectionError();
      CovarianceReprojectionError(
        const frame_t * frame,
        int keypointIndex,
        const HomogeneousExpression & point,
        CameraDesignVariable<camera_geometry_t> camera,
        spline_t* spline = NULL
      );
      virtual ~CovarianceReprojectionError();

      double observationTime();

      Eigen::MatrixXd covarianceMap();

    protected:
      /// \brief evaluate the error term
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(JacobianContainer & J) const;

      /// \brief the frame that this measurement comes from.
      const frame_t * _frame;

      /// \brief the keypoint index within the frame.
      int _keypointIndex;

      /// \brief the homogeneous point expressed in the camera frame
      HomogeneousExpression _point;

      CameraDesignVariable<camera_geometry_t> _camera;

      spline_t * _spline;
    };
  } // namespace backend
} // namespace aslam

#include "implementation/CovarianceReprojectionError.hpp"

#endif /* ASLAM_BACKEND_COVARIANCE_REPROJECTION_ERROR_HPP */
