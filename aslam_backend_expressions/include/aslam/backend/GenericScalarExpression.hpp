#ifndef ASLAM_GENERIC_SCALAR_EXPRESSION_HPP
#define ASLAM_GENERIC_SCALAR_EXPRESSION_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <set>


namespace aslam {
  namespace backend {
    template <typename Scalar_>
    class GenericScalarExpressionNode;
    
    template <typename Scalar_>
    class GenericScalarExpression
    {
    public:
      typedef Scalar_ Scalar;
      typedef GenericScalarExpressionNode<Scalar> NodeType;
      typedef boost::shared_ptr<NodeType> SharedNodePointer;

      GenericScalarExpression() = default;
      GenericScalarExpression(Scalar value);
      GenericScalarExpression(NodeType * node, bool expressionOwnsNode = false);
      GenericScalarExpression(SharedNodePointer node) { _root = node; }
      GenericScalarExpression(const GenericScalarExpression& /* other */) = default;

      template <typename OtherScalar, typename = decltype(Scalar_(OtherScalar()))>
      inline GenericScalarExpression(const GenericScalarExpression<OtherScalar> & other);

      ~GenericScalarExpression(){}

      Scalar toScalar() const;
      Scalar toValue() const { return toScalar(); }
      Scalar evaluate() const { return toScalar(); }

      void evaluateJacobians(JacobianContainer & outJacobians) const;
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      void getDesignVariables(DesignVariable::set_t & designVariables) const;

      const SharedNodePointer & root() { return _root; }

      bool empty() const { return !_root;}

      GenericScalarExpression operator-();

      GenericScalarExpression operator+(const GenericScalarExpression & s);
      GenericScalarExpression operator-(const GenericScalarExpression & s);
      GenericScalarExpression operator*(const GenericScalarExpression & s);
      GenericScalarExpression operator/(const GenericScalarExpression & s);
    private:
      SharedNodePointer _root;
      template <typename OtherScalar> friend class GenericScalarExpression;
    };
    
  } // namespace backend
} // namespace aslam

#include "implementation/GenericScalarExpression.hpp"

#endif /* ASLAM_GENERIC_SCALAR_EXPRESSION_HPP */
