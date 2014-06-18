#ifndef ASLAM_BACKEND_VECTOR_EXPRESSION_TO_GME_NODE_HPP
#define ASLAM_BACKEND_VECTOR_EXPRESSION_TO_GME_NODE_HPP

#include "GenericMatrixExpression.hpp"
#include "VectorExpression.hpp"
#include "VectorExpressionNode.hpp"

namespace aslam {
namespace backend {
namespace internal {

template<typename TExpression>
struct GenericMatrixTraits {
  typedef void gme_expression_type;
};
} // namespace internal

template<typename TExpression>
typename internal::GenericMatrixTraits<TExpression>::gme_expression_type convertToGME(const TExpression & t) {
  return typename internal::GenericMatrixTraits<TExpression>::gme_expression_type(const_cast<TExpression &>(t).root());
}

namespace internal {
template<int D>
struct GenericMatrixNodeTraits<VectorExpressionNode<D> > {
  typedef VectorExpressionNode<D> node_t;
  typedef typename boost::shared_ptr<node_t> node_ptr_t;
  typedef typename node_t::vector_t vector_t;
  typedef typename node_t::vector_t matrix_t;
  typedef vector_t value_t;
  typedef typename node_t::vector_t tangent_vector_t;
  typedef Differential<tangent_vector_t, double> differential_t;

  class Constant : VectorExpressionNode<D> {
   public:
    typedef typename VectorExpressionNode<D>::vector_t vector_t;

    Constant(int rows = D, int cols = 1) {
      if (D != Eigen::Dynamic){
        SM_ASSERT_EQ_DBG(std::runtime_error, rows, D, "dynamic size has to equal static size");
      }
      SM_ASSERT_EQ_DBG(std::runtime_error, cols, 1, "there is only one column supported as vector expression.");
    }
   private:
    virtual vector_t evaluateImplementation() const {
      return value;
    }
    virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    }
    virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
    }
    virtual void evaluateJacobiansImplementationWithDifferential(JacobianContainer & outJacobians, const differential_t & chainRuleDifferentail) const {
    }
    virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    }
   private:
    vector_t value;
  };

  typedef Constant constant_t;
};

template<int D>
struct GenericMatrixTraits<VectorExpression<D> > {
  typedef GenericMatrixExpression<D, 1, double, VectorExpressionNode<D> > gme_expression_type;
};

}  // namespace internal

}  // namespace backend
}  // namespace aslam

#endif
