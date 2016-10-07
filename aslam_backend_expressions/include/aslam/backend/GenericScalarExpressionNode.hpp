#ifndef ASLAM_BACKEND_GENERIC_SCALAR_EXPRESSION_NODE_HPP
#define ASLAM_BACKEND_GENERIC_SCALAR_EXPRESSION_NODE_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/backend/VectorExpressionNode.hpp>

namespace aslam {
namespace backend {

/**
 * \class GenericScalarExpressionNode
 * \brief The superclass of all classes representing generic scalar values.
 */
template<typename Scalar_>
class GenericScalarExpressionNode {
 public:
  typedef Scalar_ Scalar;
  typedef Scalar value_t;
  typedef boost::shared_ptr<GenericScalarExpressionNode> SharedNodePointer;

  virtual ~GenericScalarExpressionNode(){}

  /// \brief Evaluate the scalar matrix.
  inline Scalar toScalar() const { return evaluateImplementation(); }

  /// \brief Evaluate the Jacobians
  void evaluateJacobians(JacobianContainer & outJacobians) const { evaluateJacobiansImplementation(outJacobians); }

  /// \brief Evaluate the Jacobians and apply the chain rule.
  template <typename DERIVED>
  EIGEN_ALWAYS_INLINE void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixBase<DERIVED> & applyChainRule) const {
    evaluateJacobians(outJacobians.apply(applyChainRule));
  }

  void getDesignVariables(DesignVariable::set_t & designVariables) const { getDesignVariablesImplementation(designVariables); }
 protected:
  // These functions must be implemented by child classes.
  virtual Scalar evaluateImplementation() const = 0;
  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
  virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;
};

}  // namespace backend
}  // namespace aslam

#endif /* ASLAM_BACKEND_EUCLIDEAN_EXPRESSION_NODE_HPP */
