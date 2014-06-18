/*
 * QuaternionExpression.hpp
 *
 *  Created on: Apr 24, 2013
 *      Author: hannes
 */

#ifndef QUATERNIONEXPRESSION_HPP_
#define QUATERNIONEXPRESSION_HPP_

#include "sm/assert_macros.hpp"
#include "GenericMatrixExpression.hpp"
#include "VectorExpression.hpp"

namespace aslam {
namespace backend {
namespace quaternion {

enum class QuaternionMode {
  FIRST_IS_REAL_AND_TRADITIONAL_MULT_ORDER = 0,
  FIRST_IS_REAL_AND_OPPOSITE_MULT_ORDER = 1,
  LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER = 2,
  LAST_IS_REAL_AND_OPPOSITE_MULT_ORDER = 3,
}const DefaultQuaternionMode = QuaternionMode::LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER;

enum class UnitQuaternionGeometry {
  LEFT_TRANSLATED,
  RIGHT_TRANSLATED
} const DefaultUnitQuaternionGeometry  = UnitQuaternionGeometry::RIGHT_TRANSLATED;


namespace internal {
enum {
  OPPOSITE_MULT_ORDER_MASK = 1,
  REAL_IS_LAST_MASK = 2,
};

inline constexpr bool isRealFirst(const QuaternionMode mode) {
  return ((int) mode & internal::REAL_IS_LAST_MASK) == 0;
}

inline constexpr bool isTraditionalMultOrder(const QuaternionMode mode) {
  return ((int) mode & internal::OPPOSITE_MULT_ORDER_MASK) == 0;
}
} // namespace internal

template<typename TScalar = double, enum QuaternionMode EMode = DefaultQuaternionMode, typename TNode = typename GenericMatrixExpression<4, 1, TScalar>::node_t>
class QuaternionExpression : public GenericMatrixExpression<4, 1, TScalar, TNode> {
 public:
  enum {
    RealIsFirst = internal::isRealFirst(EMode),
    IsTraditionalMultOrder = internal::isTraditionalMultOrder(EMode),
  };

  typedef GenericMatrixExpression<4, 1, TScalar, TNode> base_t;
  typedef QuaternionExpression self_t;
  typedef QuaternionExpression<TScalar, EMode, typename GenericMatrixExpression<4, 1, TScalar>::node_t> self_with_default_node_t;
  typedef typename base_t::value_t value_t;
  typedef value_t vector_t;

  using base_t::evaluateJacobians;

  inline value_t evaluate() const {
    return base_t::evaluate();
  }

  QuaternionExpression(const base_t & v)
      : base_t(v) {
  }
  QuaternionExpression(TScalar r, TScalar i, TScalar j, TScalar k)
      : base_t(createQuatVal(i, j, k, r)) {
  }
  QuaternionExpression(TScalar i, TScalar j, TScalar k)
      : base_t(createQuatVal(i, j, k)) {
  }

  template<typename TOtherNode>
  inline self_with_default_node_t operator *(const QuaternionExpression<TScalar, EMode, TOtherNode> & other) const;

  template<typename TOtherNode> inline self_with_default_node_t operator +(const QuaternionExpression<TScalar, EMode, TOtherNode> & other) const {
    return self_with_default_node_t(base_t::operator+(other));
  }
  template<typename TOtherNode> inline self_with_default_node_t operator -(const QuaternionExpression<TScalar, EMode, TOtherNode> & other) const {
    return self_with_default_node_t(base_t::operator-(other));
  }
  inline self_with_default_node_t operator -() const {
    return self_with_default_node_t(base_t::operator-());
  }

  inline self_with_default_node_t inverse() const;
  inline self_with_default_node_t conjugate() const;

  inline GenericMatrixExpression<3, 1, TScalar> imaginaryPart() const;

  inline base_t toVectorExpression() {
    return base_t(this->root());
  }

 protected:
  inline static value_t createQuatVal(TScalar i, TScalar j, TScalar k, TScalar r = 0) {
    return RealIsFirst ? value_t(r, i, j, k) : value_t(i, j, k, r);
  }
};

template<typename TScalar = double, enum QuaternionMode EMode = DefaultQuaternionMode, typename TNode = typename GenericMatrixExpression<4, 1, TScalar>::node_t>
class UnitQuaternionExpression : public QuaternionExpression<TScalar, EMode, TNode> {
 public:
  typedef QuaternionExpression<TScalar, EMode, TNode> base_t;
  typedef UnitQuaternionExpression self_t;
  typedef UnitQuaternionExpression<TScalar, EMode, typename GenericMatrixExpression<4, 1, TScalar>::node_t> self_with_default_node_t;
  typedef typename base_t::value_t value_t;
  typedef  GenericMatrixExpression<3, 1, TScalar> tangent_vector_expression_t;

  UnitQuaternionExpression(const base_t & q)
      : base_t(q) {
  }
  UnitQuaternionExpression(const typename base_t::base_t & q)
      : base_t(q) {
  }
  UnitQuaternionExpression(TScalar r, TScalar i, TScalar j, TScalar k)
      : base_t(assertUnitNorm(this->createQuatVal(i, j, k, r))) {
  }
  UnitQuaternionExpression(TScalar i, TScalar j, TScalar k)
      : base_t(assertUnitNorm(this->createQuatVal(i, j, k))) {
  }

  using base_t::operator *;

  inline value_t evaluate() const {
    return assertUnitNorm(base_t::evaluate());
  }

  template<typename TOtherNode>
  inline self_with_default_node_t operator *(const UnitQuaternionExpression<TScalar, EMode, TOtherNode> & other) const {
    return self_with_default_node_t(base_t::operator*(other));
  }

  inline self_with_default_node_t operator -() const {
    return self_with_default_node_t(base_t::operator-());
  }
  using base_t::operator -;

  inline self_with_default_node_t inverse() const {
    return conjugate();
  }
  inline self_with_default_node_t conjugate() const {
    return self_with_default_node_t(base_t::conjugate());
  }

  template<typename TOtherNode>
  static tangent_vector_expression_t log(const UnitQuaternionExpression<TScalar, EMode, TOtherNode> & other);

  template<typename TOtherNode>
  static self_with_default_node_t exp(const GenericMatrixExpression<3, 1, TScalar, TOtherNode> & tangentVector);

  template <enum UnitQuaternionGeometry EGeometry = DefaultUnitQuaternionGeometry, typename TOtherNode>
  inline tangent_vector_expression_t geoLog(const UnitQuaternionExpression<TScalar, EMode, TOtherNode> & other) const;

  template <enum UnitQuaternionGeometry EGeometry = DefaultUnitQuaternionGeometry, typename TOtherNode>
  inline self_with_default_node_t geoExp(const GenericMatrixExpression<3, 1, TScalar, TOtherNode> & tangentVector) const;

  template<typename TOtherNode>
  inline GenericMatrixExpression<3, 1, TScalar> rotate3Vector(const GenericMatrixExpression<3, 1, TScalar, TOtherNode> & vector) const;

  GenericMatrixExpression<3, 3, TScalar> toMatrixExpression() const;
 private:
  inline static const value_t & assertUnitNorm(const value_t & val) {
    SM_ASSERT_NEAR_DBG(std::runtime_error, 1, val.norm(), sqrt(std::numeric_limits<TScalar>::epsilon()) * 10, "This value does is not a unit quaternion!");
    return val;
  }
};



}
}
}
#include "implementation/QuaternionExpression.hpp"

#endif /* QUATERNIONEXPRESSION_HPP_ */
