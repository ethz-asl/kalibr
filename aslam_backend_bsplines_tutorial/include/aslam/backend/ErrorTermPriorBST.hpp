#ifndef ASLAM_BACKEND_TUTORIAL_ERROR_PRIOR_HPP
#define ASLAM_BACKEND_TUTORIAL_ERROR_PRIOR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/VectorExpression.hpp>


namespace aslam {
  namespace backend {
    class ErrorTermPriorBST : public ErrorTermFs<1>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ErrorTermPriorBST(aslam::backend::VectorExpression<1> robotPos, double hat_x, double sigma2_x);
      virtual ~ErrorTermPriorBST();

    private:
      /// This is the inteface required by ErrorTermFs<>
      
      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians);

      aslam::backend::VectorExpression<1> _robotPos;
      double _hat_x;

    };
  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_TUTORIAL_ERROR_PRIOR_HPP */
