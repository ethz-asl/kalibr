#ifndef ASLAM_BACKEND_GENERIC_SCALAR_POINT_HPP
#define ASLAM_BACKEND_GENERIC_SCALAR_POINT_HPP

#include <cstdint>
#include <aslam/backend/DesignVariable.hpp>
#include "GenericScalarExpressionNode.hpp"
#include "GenericScalarExpression.hpp"

namespace aslam {
namespace backend {

template <typename Scalar_>
class GenericScalar : public GenericScalarExpressionNode<Scalar_>, public DesignVariable {
 public:
  typedef GenericScalarExpressionNode<Scalar_> Base;
  typedef Scalar_ Scalar;

  constexpr static int DesignVariableDimension = 1;
  constexpr static int MinimalDimension = 1;

  GenericScalar(Scalar p);
  virtual ~GenericScalar();


  GenericScalarExpression<Scalar> toExpression();

  Eigen::MatrixXd getParameters();
  using DesignVariable::getParameters;

 protected:
  /// \brief Revert the last state update.
  virtual void revertUpdateImplementation();

  /// \brief Update the design variable.
  virtual void updateImplementation(const double * dp, int size);

  /// \brief the size of an update step
  virtual inline int minimalDimensionsImplementation() const { return MinimalDimension; }
 private:
  virtual Scalar toScalarImplementation() const;
  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule) const;
  virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

  /// Returns the content of the design variable
  virtual void getParametersImplementation(Eigen::MatrixXd& value) const;

  /// Sets the content of the design variable
  virtual void setParametersImplementation(const Eigen::MatrixXd& value);

  /// Computes the minimal distance in tangent space between the current value of the DV and xHat
  virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const;

  /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
  virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const;

  /// \brief The current value of the design variable.
  Scalar _p;

  /// \brief The previous version of the design variable.
  Scalar _p_p;
};

}  // namespace backend
}  // namespace aslam

#include "implementation/GenericScalar.hpp"

#endif /* ASLAM_BACKEND_GENERIC_SCALAR_POINT_HPP */
