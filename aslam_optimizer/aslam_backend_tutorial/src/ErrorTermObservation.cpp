#include <aslam/backend/ErrorTermObservation.hpp>

namespace aslam {
  namespace backend {
    
    
    ErrorTermObservation::ErrorTermObservation(ScalarDesignVariable * x_k, ScalarDesignVariable * w, double y, double sigma2_n) :
      _x_k(x_k), _w(w), _y(y)
    {
        Eigen::Matrix<double,1,1> invR;
        invR(0,0) = (1.0/sigma2_n);
      // Fill in the inverse covariance. In this scalar case, this is just an inverse variance.
        setInvR( invR );

      // Tell the super class about the design variables:
      setDesignVariables(_x_k, w);
    }

    ErrorTermObservation::~ErrorTermObservation()
    {

    }


    /// \brief evaluate the error term and return the weighted squared error e^T invR e
    double ErrorTermObservation::evaluateErrorImplementation()
    {
      // Build the error from the measurment _y and the design variables
        error_t error;
        error(0) = _y - 1.0/(_w->value() - _x_k->value());
        setError(error);
        return evaluateChiSquaredError();
    }


    /// \brief evaluate the jacobians
    void ErrorTermObservation::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians)
    {
      double hat_y = -1.0/(_w->value() - _x_k->value());
      Eigen::MatrixXd hat_y2(1,1);
      hat_y2(0,0) = hat_y * hat_y;
      _jacobians.add(_x_k, -hat_y2);
      _jacobians.add(_w, hat_y2);
    }


  } // namespace backend
} // namespace aslam
