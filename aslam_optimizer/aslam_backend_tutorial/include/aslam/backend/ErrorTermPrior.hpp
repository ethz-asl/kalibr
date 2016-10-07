#ifndef ASLAM_BACKEND_TUTORIAL_ERROR_PRIOR_HPP
#define ASLAM_BACKEND_TUTORIAL_ERROR_PRIOR_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include "ScalarDesignVariable.hpp"


namespace aslam {
  namespace backend {
    class ErrorTermPrior : public ErrorTermFs<1>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ErrorTermPrior(ScalarDesignVariable * x, double hat_x, double sigma2_x);
      virtual ~ErrorTermPrior();

    private:
      /// This is the inteface required by ErrorTermFs<>
      
      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & J);

      ScalarDesignVariable * _x;
      double _hat_x;

    };
  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_TUTORIAL_ERROR_PRIOR_HPP */
