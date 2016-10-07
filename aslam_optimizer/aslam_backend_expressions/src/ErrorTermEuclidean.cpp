#include <aslam/backend/ErrorTermEuclidean.hpp>

#include <Eigen/LU>

namespace aslam {
  namespace backend {

    
  ErrorTermEuclidean::ErrorTermEuclidean(
		  const aslam::backend::EuclideanExpression& t,
		  const Eigen::Vector3d& prior,
		  const Eigen::Matrix<double,3,3>& N,
		  int debug) :
    _t(t), _prior(prior), _debug(debug)
  {
    // Fill in the inverse covariance.
    setInvR(N.inverse());

    // Tell the super class about the design variables:
    aslam::backend::DesignVariable::set_t dv;
    _t.getDesignVariables(dv);
    setDesignVariablesIterator(dv.begin(), dv.end());
  }

  ErrorTermEuclidean::ErrorTermEuclidean(
		  const aslam::backend::EuclideanExpression& t,
		  const Eigen::Vector3d& prior,
		  double weight,
		  int /* debug */) :
    _t(t), _prior(prior), _debug(0)
      {
          Eigen::Matrix<double, 3, 1> W;
          W << weight, weight, weight;
          
          // Fill in the inverse covariance.
          setInvR(Eigen::Matrix<double,3,3>(W.asDiagonal()));
          
          // Tell the super class about the design variables:
          aslam::backend::DesignVariable::set_t dv;
          _t.getDesignVariables(dv);
          setDesignVariablesIterator(dv.begin(), dv.end());
      }


  ErrorTermEuclidean::~ErrorTermEuclidean()
  {
  }


  /// \brief evaluate the error term and return the weighted squared error e^T invR e
  double ErrorTermEuclidean::evaluateErrorImplementation()
  {
    Eigen::VectorXd errorVector = _t.toEuclidean() - _prior;

    setError(errorVector);

    return evaluateChiSquaredError();
  }


  /// \brief evaluate the jacobians
  void ErrorTermEuclidean::evaluateJacobiansImplementation(JacobianContainer & _jacobians)
  {
     _t.evaluateJacobians(_jacobians);
  }

  } // namespace backend
} // namespace aslam
