/*!
 * \file ErrorTermTransformation.hpp
 *
 * \author diemf
 *
 * A Pose ErrorTerm
 */
#ifndef ASLAM_BACKEND_ERROR_TRANSFORMATION_HPP
#define ASLAM_BACKEND_ERROR_TRANSFORMATION_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <aslam/backend/TransformationExpression.hpp>

namespace aslam {
  namespace backend {

    /*!
    * \class ErrorTermTransformation
    *
    * \brief An ErrorTerm implementation for the deviation of a Pose w.r.t. a given prior
    */

    class ErrorTermTransformation : public aslam::backend::ErrorTermFs<6>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ErrorTermTransformation(aslam::backend::TransformationExpression T, sm::kinematics::Transformation prior, Eigen::Matrix<double,6,6> N, int debug=0);
        ErrorTermTransformation(aslam::backend::TransformationExpression T, sm::kinematics::Transformation prior, double weightRotation, double weightTranslation);
      
      virtual ~ErrorTermTransformation();

    protected:
      /// This is the interface required by ErrorTermFs<>

      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(JacobianContainer & J) const;

    private:
      aslam::backend::TransformationExpression _T;
      sm::kinematics::Transformation _prior;
      Eigen::Matrix4d _errorMatrix;
      int _debug;
    };

  } // namespace vc
} // namespace aslam


#endif /* ASLAM_VC_ERROR_TRANSFORMATION_HPP */
