/*
 * MarginalizationPriorErrorTerm.cpp
 *
 *  Created on: Apr 26, 2013
 *      Author: mbuerki
 */

#include <aslam/backend/MarginalizationPriorErrorTerm.hpp>
#include <Eigen/Dense>
#include <sm/assert_macros.hpp>

using namespace std;

namespace aslam {
namespace backend {

MarginalizationPriorErrorTerm::MarginalizationPriorErrorTerm(const std::vector<aslam::backend::DesignVariable*>& designVariables,
    const Eigen::VectorXd& d, const Eigen::MatrixXd& R)
: aslam::backend::ErrorTermDs(R.rows()), _designVariables(designVariables), _d(d), _R(R), _dimensionDesignVariables(R.cols())
{
	SM_ASSERT_GT(aslam::InvalidArgumentException, designVariables.size(), 0, "The prior error term doesn't make much sense with zero design variables.");
  // PTF: Hrm...here is a big weakness of the current optimizer code. We should have
  //      different base classes for different uncertainty types (scalar, diagonal, matrix, none)
  //      to avoid big matrix multiplications during optimization.
  setInvR(Eigen::MatrixXd::Identity(R.rows(), R.rows()));

  //_M = Eigen::MatrixXd::Identity(R.cols(), R.cols());

  // set all design variables
  for(vector<aslam::backend::DesignVariable*>::iterator it = _designVariables.begin(); it != _designVariables.end(); ++it)
  {
    Eigen::MatrixXd values;
    (*it)->getParameters(values);
    _designVariableValuesAtMarginalization.push_back(values);
  }
  setDesignVariables(designVariables);

}

MarginalizationPriorErrorTerm::~MarginalizationPriorErrorTerm() {
  // TODO Auto-generated destructor stub
}

double MarginalizationPriorErrorTerm::evaluateErrorImplementation()
{
  Eigen::VectorXd diff = getDifferenceSinceMarginalization();
  SM_ASSERT_EQ(aslam::Exception, diff.rows(), _R.cols(), "Dimension of R and the minimal difference vector mismatch!");
  SM_ASSERT_EQ(aslam::Exception, _d.rows(), _R.rows(), "Dimension of R and the d mismatch!");
  Eigen::VectorXd currentError(_dimensionErrorTerm);
  currentError.setZero();
  currentError = -(_d - _R*diff);
  setError(currentError);
  return evaluateChiSquaredError();

}

// Computes the difference vector of all design variables between the linearization point at marginalization and the current guess, on the tangent space (i.e. log(x_bar - x))
Eigen::VectorXd MarginalizationPriorErrorTerm::getDifferenceSinceMarginalization()
{
  Eigen::VectorXd diff = Eigen::VectorXd(_dimensionDesignVariables);
  diff.setZero();
  int index = 0;
  std::vector<Eigen::MatrixXd>::iterator it_marg = _designVariableValuesAtMarginalization.begin();
  std::vector<aslam::backend::DesignVariable*>::iterator it_current = _designVariables.begin();
  for(;it_current != _designVariables.end(); ++it_current, ++it_marg)
  {
	  // retrieve current value (xbar) and value at marginalization(xHat)
      Eigen::MatrixXd xHat = *it_marg;
      //get minimal difference in tangent space
      Eigen::VectorXd diffVector;
      (*it_current)->minimalDifference(xHat, diffVector);
      //int base = index;
      int dim = diffVector.rows();
      diff.segment(index, dim) = diffVector;
      index += dim;
  }
  return diff;
}

void MarginalizationPriorErrorTerm::evaluateJacobiansImplementation(JacobianContainer & outJ) const
{
  int colIndex = 0;
  std::vector<Eigen::MatrixXd>::const_iterator it_marg = _designVariableValuesAtMarginalization.begin();
  for(vector<aslam::backend::DesignVariable*>::const_iterator it = _designVariables.begin(); it != _designVariables.end(); ++it, ++it_marg)
  {
    int dimDesignVariable = (*it)->minimalDimensions();
    Eigen::MatrixXd M;
    Eigen::VectorXd diff;
    (*it)->minimalDifferenceAndJacobian(*it_marg, diff, M);
    SM_ASSERT_EQ(aslam::Exception, M.rows(), dimDesignVariable, "Minimal difference jacobian and design variable dimension mismatch!");
    outJ.add(*it, _R.block(0, colIndex, _dimensionErrorTerm, dimDesignVariable)*M);
    colIndex += dimDesignVariable;
  }

}

aslam::backend::DesignVariable* MarginalizationPriorErrorTerm::getDesignVariable(int i)
{
  return _designVariables[i];
}

} /* namespace backend */
} /* namespace aslam */
