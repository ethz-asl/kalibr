#include <aslam/backend/ErrorTermMotion.hpp>

namespace aslam {
  namespace backend {
    
    ErrorTermMotion::ErrorTermMotion(ScalarDesignVariable * x_k, ScalarDesignVariable * x_kp1, double u, double sigma2_u) :
      _u(u), _x_k(x_k), _x_kp1(x_kp1)
    {

        Eigen::Matrix<double,1,1> invR;
        invR(0,0) = (1.0/sigma2_u);
      // Fill in the inverse covariance. In this scalar case, this is just an inverse variance.
        setInvR( invR );

      // Tell the super class about the design variables:
      setDesignVariables(x_k, x_kp1);
    }

    ErrorTermMotion::~ErrorTermMotion()
    {

    }

        
    /// \brief evaluate the error term and return the weighted squared error e^T invR e
    double ErrorTermMotion::evaluateErrorImplementation()
    {
        error_t error;
        error(0) = _x_kp1->value() - _x_k->value() - _u;
        setError(error);
        return evaluateChiSquaredError();
    }


    /// \brief evaluate the jacobian
    void ErrorTermMotion::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians)
    {
      _jacobians.add(_x_k, -Eigen::MatrixXd::Identity(1,1));
      _jacobians.add(_x_kp1, Eigen::MatrixXd::Identity(1,1));
    }

  } // namespace backend
} // namespace aslam
