#include <aslam/backend/ErrorTermMotionBST.hpp>

namespace aslam {
  namespace backend {
    
    ErrorTermMotionBST::ErrorTermMotionBST(aslam::backend::VectorExpression<1> robotVelocity, double u, double sigma2_u) :
      _motionErrorTerm(ScalarExpression(boost::shared_ptr<ScalarExpressionNode>(new ScalarExpressionNodeConstant(u))) - robotVelocity.toScalarExpression())
    {

        Eigen::Matrix<double,1,1> invR;
        invR(0,0) = (1.0/sigma2_u);
      // Fill in the inverse covariance. In this scalar case, this is just an inverse variance.
        setInvR( invR );

      // Tell the super class about the design variables:
        DesignVariable::set_t dvs;
        robotVelocity.getDesignVariables(dvs);
        ErrorTermFs<1>::setDesignVariablesIterator(dvs.begin(), dvs.end());
    }

    ErrorTermMotionBST::~ErrorTermMotionBST()
    {

    }

        
    /// \brief evaluate the error term and return the weighted squared error e^T invR e
    double ErrorTermMotionBST::evaluateErrorImplementation()
    {
        error_t error;
        error(0) = _motionErrorTerm.toScalar();
        //std::cout << "The motion error is: " << error(0) << std::endl;
        setError(error);
        return evaluateChiSquaredError();
    }


    /// \brief evaluate the jacobian
    void ErrorTermMotionBST::evaluateJacobiansImplementation(JacobianContainer & outJacobians)
    {
      _motionErrorTerm.evaluateJacobians(outJacobians);
    }

  } // namespace backend
} // namespace aslam
