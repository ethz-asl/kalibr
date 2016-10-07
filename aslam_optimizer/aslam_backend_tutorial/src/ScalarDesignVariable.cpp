#include <aslam/backend/ScalarDesignVariable.hpp>

namespace aslam {
  namespace backend {
    
    // initialize the variable with some initial guess
    // Also initialize the previous version. This is not strictly necessary
    // but it seems like the right thing to do.
    ScalarDesignVariable::ScalarDesignVariable(double initialValue) :
      _value(initialValue), _p_value(initialValue)
    {

    }

    ScalarDesignVariable::~ScalarDesignVariable()
    {

    }

      
    /// \brief get the current value
    double ScalarDesignVariable::value()
    {
      return _value;
    }


    /// \brief what is the number of dimensions of the perturbation variable.
    int ScalarDesignVariable::minimalDimensionsImplementation() const
    {
      return 1;
    }

      
    /// \brief Update the design variable.
  void ScalarDesignVariable::updateImplementation(const double * dp, int /* size */)
    {
      // Backup the value so we can revert if necessary
      _p_value = _value;

      // Update the value
      _value += *dp;
    }

      
    /// \brief Revert the last state update.
    void ScalarDesignVariable::revertUpdateImplementation()
    {
      _value = _p_value;
    }

    void ScalarDesignVariable::getParametersImplementation(
        Eigen::MatrixXd& value) const {
      Eigen::Matrix<double, 1, 1> valueMat;
      valueMat << _value;
      value = valueMat;
    }


    void ScalarDesignVariable::setParametersImplementation(
        const Eigen::MatrixXd& value) {
      _p_value = _value;
      _value = value(0, 0);
    }

    void ScalarDesignVariable::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
    	outDifference = Eigen::VectorXd(1);
    	outDifference(0) = _value - xHat(0,0);
    }

    void ScalarDesignVariable::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
    {
    	minimalDifferenceImplementation(xHat, outDifference);
    	outJacobian = Eigen::MatrixXd::Identity(1,1);
    }


  } // namespace backend
} // namespace aslam
