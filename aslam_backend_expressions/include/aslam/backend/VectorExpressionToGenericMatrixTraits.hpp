#ifndef ASLAM_BACKEND_VECTOR_EXPRESSION_TO_GME_NODE_HPP
#define ASLAM_BACKEND_VECTOR_EXPRESSION_TO_GME_NODE_HPP

#include "GenericMatrixExpression.hpp"
#include "VectorExpression.hpp"
#include "VectorExpressionNode.hpp"
#include "EuclideanExpression.hpp"
#include "EuclideanExpressionNode.hpp"

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

  typedef ConstantVectorExpressionNode<D> constant_t;
};

template<int D>
struct GenericMatrixTraits<VectorExpression<D> > {
  typedef GenericMatrixExpression<D, 1, double, VectorExpressionNode<D> > gme_expression_type;
};

template <>
struct GenericMatrixTraits<EuclideanExpression > {
  typedef GenericMatrixExpression<3, 1, double, EuclideanExpressionNode > gme_expression_type;
};

}  // namespace internal

}  // namespace backend
}  // namespace aslam

#endif
