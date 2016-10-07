#include <aslam/backend/ErrorTermPriorBST.hpp>

namespace aslam {
  namespace backend {
    ErrorTermPriorBST::ErrorTermPriorBST(aslam::backend::VectorExpression<1> robotPos, double hat_x, double sigma2_x)
      : _robotPos(robotPos), _hat_x(hat_x)
    {
      // Fill in the inverse covariance. In this scalar case, this is just an inverse variance.
        Eigen::Matrix<double,1,1> invR = Eigen::Matrix<double,1,1>::Identity();
        invR(0,0) = ( 1.0/sigma2_x);
        setInvR( invR );

      // Tell the super class about the design variables:
        DesignVariable::set_t dvs;
        robotPos.getDesignVariables(dvs);
        ErrorTermFs<1>::setDesignVariablesIterator(dvs.begin(), dvs.end());

    }

    ErrorTermPriorBST::~ErrorTermPriorBST()
    {

    }

      
    /// \brief evaluate the error term and return the weighted squared error e^T invR e
    double ErrorTermPriorBST::evaluateErrorImplementation()
    {

    	double robotPos = _robotPos.evaluate()(0);

    	Eigen::VectorXd err = Eigen::VectorXd(1);
    	err(0) = robotPos - _hat_x;
        //std::cout << "err: " << err << std::endl;
        setError(err);
        return evaluateChiSquaredError();
    }


    /// \brief evaluate the jacobian
    void ErrorTermPriorBST::evaluateJacobiansImplementation(JacobianContainer & outJacobians)
    {
      _robotPos.evaluateJacobians(outJacobians);
    }


  } // namespace backend
} // namespace aslam
