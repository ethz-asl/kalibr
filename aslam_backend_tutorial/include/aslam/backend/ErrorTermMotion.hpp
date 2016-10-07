#ifndef ASLAM_BACKEND_TUTORIAL_ERROR_MOTION_HPP
#define ASLAM_BACKEND_TUTORIAL_ERROR_MOTION_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include "ScalarDesignVariable.hpp"

namespace aslam {
  namespace backend {
    
    // An error term implementing our observation model.
    // This class derives from ErrorTermFs<1> because the
    // errors are of dimension 1.
    class ErrorTermMotion : public ErrorTermFs<1>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ErrorTermMotion(ScalarDesignVariable * x_k, ScalarDesignVariable * x_kp1, double u, double sigma2_u);
      virtual ~ErrorTermMotion();
    private:
      /// This is the inteface required by ErrorTermFs<>
      
      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & J);

      double _u;
      ScalarDesignVariable * _x_k;
      ScalarDesignVariable * _x_kp1;
    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_TUTORIAL_ERROR_MOTION_HPP */
