#ifndef ASLAM_BACKEND_TUTORIAL_ERROR_MOTION_HPP
#define ASLAM_BACKEND_TUTORIAL_ERROR_MOTION_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/VectorExpression.hpp>


namespace aslam {
  namespace backend {
    
    // An error term implementing our observation model.
    // This class derives from ErrorTermFs<1> because the
    // errors are of dimension 1.
    class ErrorTermMotionBST : public ErrorTermFs<1>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ErrorTermMotionBST(aslam::backend::VectorExpression<1> robotVelocity, double u, double sigma2_u);
      virtual ~ErrorTermMotionBST();
    private:
      /// This is the inteface required by ErrorTermFs<>
      
      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians);

      aslam::backend::ScalarExpression _motionErrorTerm;
    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_TUTORIAL_ERROR_MOTION_HPP */
