#include <aslam/backend/ErrorTermTransformation.hpp>

namespace aslam {
  namespace backend {

    
  ErrorTermTransformation::ErrorTermTransformation(aslam::backend::TransformationExpression T, sm::kinematics::Transformation prior, Eigen::Matrix<double,6,6> N, int debug) : 
    _T(T), _prior(prior), _debug(debug)
  {
    // Fill in the inverse covariance.
    setInvR(N.inverse());

    // Tell the super class about the design variables:
    aslam::backend::DesignVariable::set_t dv;
    _T.getDesignVariables(dv);
    setDesignVariablesIterator(dv.begin(), dv.end());
  }

      ErrorTermTransformation::ErrorTermTransformation(aslam::backend::TransformationExpression T, sm::kinematics::Transformation prior, double weightRotation, double weightTranslation) :
    _T(T), _prior(prior), _debug(0)
      {
          Eigen::Matrix<double, 6, 1> W;
          W << weightTranslation, weightTranslation, weightTranslation, weightRotation, weightRotation, weightRotation;
          
          // Fill in the inverse covariance.
          setInvR(Eigen::Matrix<double,6,6>(W.asDiagonal()));
          
          // Tell the super class about the design variables:
          aslam::backend::DesignVariable::set_t dv;
          _T.getDesignVariables(dv);
          setDesignVariablesIterator(dv.begin(), dv.end());
      }


  ErrorTermTransformation::~ErrorTermTransformation()
  {

  }


  /// \brief evaluate the error term and return the weighted squared error e^T invR e
  double ErrorTermTransformation::evaluateErrorImplementation()
  {
    _errorMatrix = _T.toTransformationMatrix()*_prior.inverse().T();

    Eigen::VectorXd errorVector = Eigen::VectorXd::Zero(6);
    errorVector.block(0,0,3,1) = _errorMatrix.block(0,3,3,1);
    errorVector.block(3,0,3,1) = sm::kinematics::R2AxisAngle(_errorMatrix.block(0,0,3,3));
    setError(errorVector);

    if (_debug == 1)
    {
      std::cout << "\n\n\nPriorEstim:\n" << _T.toTransformationMatrix() << "\n";
      std::cout << "PriorGiven:\n" << _prior.T() << "\n";
      std::cout << "invPrior:\n" << _prior.inverse().T() << "\n";

      std::cout << "errorMatrix:\n" << _errorMatrix << "\n";
      std::cout << "error:\n" << error() <<  "\ninvR:\n" << invR() << "\n\n\n";
    }

    return evaluateChiSquaredError();
  }


  /// \brief evaluate the jacobians
  void ErrorTermTransformation::evaluateJacobiansImplementation(JacobianContainer & _jacobians) const
  {
    sm::kinematics::RotationVector rotVector;
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6,6);
    J.block(0,3,3,3) = sm::kinematics::crossMx(_errorMatrix.block(0,3,3,1));
    J.block(3,3,3,3) = rotVector.parametersToInverseSMatrix(sm::kinematics::R2AxisAngle(_errorMatrix.block(0,0,3,3)));

    if (_debug == 1)
    {
      std::cout << "errorMatrix:\n" << _errorMatrix << "\n";
      std::cout << "error:\n" << error() << "\n";
      std::cout << "J:\n" << J << "\n";
    }

    _T.evaluateJacobians(_jacobians, J);
  }

  } // namespace vc
} // namespace aslam
