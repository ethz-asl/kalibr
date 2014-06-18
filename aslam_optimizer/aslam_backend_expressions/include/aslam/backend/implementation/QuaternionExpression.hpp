/*
 * QuaternionExpression.hpp
 *
 *  Created on: Apr 24, 201RIndex
 *      Author: hannes
 */

#include <sm/kinematics/quaternion_algebra.hpp>

namespace aslam {
namespace backend {
namespace quaternion {

#define _TEMPLATE template <typename TScalar, enum QuaternionMode EMode, typename TNode>
#define _CLASS QuaternionExpression<TScalar, EMode, TNode>

namespace internal {
inline constexpr int getRealIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 0 : 3;
}
inline constexpr int getIIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 1 : 0;
}
inline constexpr int getJIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 2 : 1;
}
inline constexpr int getKIndex(const QuaternionMode mode) {
  return isRealFirst(mode) ? 3 : 2;
}

template<typename TScalar, QuaternionMode EMode>
struct EigenQuaternionCalculator {
  typedef Eigen::Matrix<TScalar, 4, 1> vector_t;
  typedef Eigen::Matrix<TScalar, 3, 1> pure_imag_vector_t;
  typedef Eigen::Matrix<TScalar, 3, 1> lie_algebra_vector_t;

  enum {
    RIndex = getRealIndex(EMode),
    IIndex = getIIndex(EMode),
    JIndex = getJIndex(EMode),
    KIndex = getKIndex(EMode),
    IPureIndex = 0,
    JPureIndex = 1,
    KPureIndex = 2
  };

  inline static vector_t getIdentity() {
    return isRealFirst(EMode) ? vector_t(1, 0, 0, 0) : vector_t(0, 0, 0, 1);
  }

  inline static vector_t quatMultTraditional(const vector_t & a, const vector_t & b) {
    vector_t res;

    // aIIndex*bRIndex + aJIndex*bKIndex - aKIndex*bJIndex + aRIndex*bIIndex
    res[IIndex] = a[IIndex] * b[RIndex] + a[JIndex] * b[KIndex] - a[KIndex] * b[JIndex] + a[RIndex] * b[IIndex];
    // aKIndex*bIIndex - aIIndex*bKIndex + aJIndex*bRIndex + aRIndex*bJIndex
    res[JIndex] = a[KIndex] * b[IIndex] - a[IIndex] * b[KIndex] + a[JIndex] * b[RIndex] + a[RIndex] * b[JIndex];
    // aIIndex*bJIndex - aJIndex*bIIndex + aKIndex*bRIndex + aRIndex*bKIndex
    res[KIndex] = a[IIndex] * b[JIndex] - a[JIndex] * b[IIndex] + a[KIndex] * b[RIndex] + a[RIndex] * b[KIndex];
    // aRIndex*bRIndex - aJIndex*bJIndex - aKIndex*bKIndex - aIIndex*bIIndex
    res[RIndex] = a[RIndex] * b[RIndex] - a[JIndex] * b[JIndex] - a[KIndex] * b[KIndex] - a[IIndex] * b[IIndex];
    return res;
  }

  inline static vector_t quatMultTraditional(const pure_imag_vector_t & a, const vector_t & b) {
    vector_t res;
    res[IIndex] = a[IPureIndex] * b[RIndex] + a[JPureIndex] * b[KIndex] - a[KPureIndex] * b[JIndex];
    res[JIndex] = a[KPureIndex] * b[IIndex] - a[IPureIndex] * b[KIndex] + a[JPureIndex] * b[RIndex];
    res[KIndex] = a[IPureIndex] * b[JIndex] - a[JPureIndex] * b[IIndex] + a[KPureIndex] * b[RIndex];
    res[RIndex] = -a[JPureIndex] * b[JIndex] - a[KPureIndex] * b[KIndex] - a[IPureIndex] * b[IIndex];
    return res;
  }

  inline static vector_t quatMultTraditional(const vector_t & a, const pure_imag_vector_t & b) {
    vector_t res;
    res[IIndex] = +a[JIndex] * b[KPureIndex] - a[KIndex] * b[JPureIndex] + a[RIndex] * b[IPureIndex];
    res[JIndex] = a[KIndex] * b[IPureIndex] - a[IIndex] * b[KPureIndex] + +a[RIndex] * b[JPureIndex];
    res[KIndex] = a[IIndex] * b[JPureIndex] - a[JIndex] * b[IPureIndex] + +a[RIndex] * b[KPureIndex];
    res[RIndex] = -a[JIndex] * b[JPureIndex] - a[KIndex] * b[KPureIndex] - a[IIndex] * b[IPureIndex];
    return res;
  }

  template<int ISizeA, int ISizeB>
  inline static vector_t quatMult(const Eigen::Matrix<TScalar, ISizeA, 1> & a, const Eigen::Matrix<TScalar, ISizeB, 1> & b) {
    return isTraditionalMultOrder(EMode) ? quatMultTraditional(a, b) : quatMultTraditional(b, a);
  }

  inline static vector_t conjugate(const vector_t & v) {
    vector_t r(v);
    r.template block<3, 1>(IIndex, 0) *= -1;
    return r;
  }
  inline static vector_t invert(const vector_t & v) {
    vector_t r(conjugate(v));
    r /= r.dot(r);
    return r;
  }

  inline static auto getImagPart(const vector_t & v) -> decltype(v.template block<3, 1>(IIndex, 0)) {
    return v.template block<3, 1>(IIndex, 0);
  }

  inline static lie_algebra_vector_t log(const vector_t & v) {
    return sm::kinematics::quat2AxisAngle(convertToOtherMode<QuaternionMode::LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER>(v)) * TScalar(0.5);
  }
  
  inline static Eigen::Matrix<TScalar, 3, 4> dlog(const vector_t & v) {
    auto J = sm::kinematics::quatLogJacobian2(convertToOtherMode<QuaternionMode::LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER>(v)) * TScalar(0.5);
    if(isRealFirst(EMode)){
      return convertFromOtherMode<QuaternionMode::LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER, 3>(J.transpose()).transpose();
    }
    else{
      return J;
    }
  }

  inline static vector_t exp(const lie_algebra_vector_t & v) {
    return convertFromOtherMode<QuaternionMode::LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER>(sm::kinematics::axisAngle2quat(2.0 * v.template cast<double>()).template cast<TScalar>());
  }

  template <enum UnitQuaternionGeometry EGeometry>
  inline static vector_t update(const vector_t p, const lie_algebra_vector_t & v) {
    switch(EGeometry){
      case UnitQuaternionGeometry::LEFT_TRANSLATED:
        return quatMult(p, exp(v));
      case UnitQuaternionGeometry::RIGHT_TRANSLATED:
        return quatMult(exp(v), p);
    };
  }

  template <enum UnitQuaternionGeometry EGeometry = DefaultUnitQuaternionGeometry>
  static Eigen::Matrix<TScalar, 4, 3> dUpdate(const vector_t & p){
    return convertFromOtherMode<QuaternionMode::LAST_IS_REAL_AND_OPPOSITE_MULT_ORDER, 3>(
         ((isTraditionalMultOrder(EMode) != (EGeometry == UnitQuaternionGeometry::RIGHT_TRANSLATED)) ?
             sm::kinematics::quatOPlus(convertToOtherMode<QuaternionMode::LAST_IS_REAL_AND_OPPOSITE_MULT_ORDER>(p).template cast<double>()) :
             sm::kinematics::quatPlus(convertToOtherMode<QuaternionMode::LAST_IS_REAL_AND_OPPOSITE_MULT_ORDER>(p).template cast<double>())
         ).template block<4,3>(0, 0).template cast<TScalar>()
      );
  }

  inline static Eigen::Matrix<TScalar, 4, 3> dexp(const lie_algebra_vector_t & v) {
    auto J = sm::kinematics::quatExpJacobian((v * TScalar(2)).eval()) * TScalar(2);
    if(isRealFirst(EMode)){
      return convertFromOtherMode<QuaternionMode::LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER, 3>(J);
    }
    else{
      return J;
    }
  }

  template<enum QuaternionMode EOtherMode, int ICols = 1, typename DERIVED_MATRIX>
  inline static Eigen::Matrix<TScalar, 4, ICols> convertToOtherMode(const Eigen::MatrixBase<DERIVED_MATRIX > & v){
    if(isRealFirst(EMode) == isRealFirst(EOtherMode)){
      return v;
    } else {
      typedef EigenQuaternionCalculator<TScalar, EOtherMode> other_t;
      Eigen::Matrix<TScalar, 4, ICols> ret;
      ret.template block<1, ICols>(other_t::RIndex, 0) = v.template block<1, ICols>(RIndex, 0);
      ret.template block<3, ICols>(other_t::IIndex, 0) = v.template block<3, ICols>(IIndex, 0);
      return ret;
    }
  }

  template<enum QuaternionMode EOtherMode, int ICols = 1, typename DERIVED_MATRIX>
  inline static Eigen::Matrix<TScalar, 4, ICols> convertFromOtherMode(const Eigen::MatrixBase<DERIVED_MATRIX > & v){
    return EigenQuaternionCalculator<TScalar, EOtherMode>::template convertToOtherMode<EMode, ICols>(v);
  }

};
}

_TEMPLATE template <typename TOtherNode>
inline typename _CLASS::self_with_default_node_t _CLASS::operator * (const QuaternionExpression<TScalar, EMode, TOtherNode> & other) const {
  typedef _CLASS::self_with_default_node_t result_t;
  typedef QuaternionExpression<TScalar, EMode, TOtherNode> other_t;

  class ResultNode : public result_t::template BinaryOperationResult<ResultNode, self_t, other_t> {
  public:
    typedef typename result_t::template BinaryOperationResult<ResultNode, self_t, other_t> base_t;

    virtual ~ResultNode() {}
    inline typename base_t::apply_diff_return_t applyLhsDiff(const typename base_t::lhs_t::tangent_vector_t & tangent_vector) const {
      return internal::EigenQuaternionCalculator<TScalar, EMode>::quatMult(tangent_vector, this->getRhsNode().evaluate());  //TODO add matrix version?
    }
    inline typename base_t::apply_diff_return_t applyRhsDiff(const typename base_t::rhs_t::tangent_vector_t & tangent_vector) const {
      return internal::EigenQuaternionCalculator<TScalar, EMode>::quatMult(this->getLhsNode().evaluate(), tangent_vector);
    }
  private:
    virtual void evaluateImplementation() const {
      this->_currentValue = internal::EigenQuaternionCalculator<TScalar, EMode>::quatMult(this->getLhsNode().evaluate(), this->getRhsNode().evaluate());
    }
  };

  return ResultNode::create(*this, other);
}

_TEMPLATE
inline typename _CLASS::self_with_default_node_t _CLASS::inverse() const {
  typedef _CLASS::self_with_default_node_t result_t;
  typedef internal::EigenQuaternionCalculator<TScalar, EMode> calc_t;

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
  public:
    typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

    virtual ~ResultNode() {}
    inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangentVector) const {
      /*
       * d_q q^{-1} (v) = -(\bar q v \bar q) / (q\bar q)^2
       * while q\bar q = q.dot(q) = \bar q.dot(\bar q)
       */
      auto operandValConj = calc_t::conjugate(this->getOperandNode().evaluate());
      double valSquared = operandValConj.dot(operandValConj);
      return -calc_t::quatMult(operandValConj, calc_t::quatMult(tangentVector, operandValConj)) / (valSquared * valSquared);
    }
  private:
    virtual void evaluateImplementation() const {
      this->_currentValue = calc_t::invert(this->getOperandNode().evaluate());
    }
  };

  return ResultNode::create(*this);
}

_TEMPLATE
typename _CLASS::self_with_default_node_t _CLASS::conjugate() const {
  typedef _CLASS::self_with_default_node_t result_t;
  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
  public:
    typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

    virtual ~ResultNode() {}
    inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangentVector) const {
      return internal::EigenQuaternionCalculator<TScalar, EMode>::conjugate(tangentVector);
    }
  private:
    virtual void evaluateImplementation() const {
      this->_currentValue = internal::EigenQuaternionCalculator<TScalar, EMode>::conjugate(this->getOperandNode().evaluate());
    }
  };

  return ResultNode::create(*this);
}


_TEMPLATE
inline GenericMatrixExpression<3, 1, TScalar> _CLASS::imaginaryPart() const {
  typedef GenericMatrixExpression<3, 1, TScalar> result_t;
  typedef internal::EigenQuaternionCalculator<TScalar, EMode> calc_t;

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
  public:
    typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

    virtual ~ResultNode() {}
    inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangentVector) const {
      return calc_t::getImagPart(tangentVector);
    }
  private:
    virtual void evaluateImplementation() const {
      this->_currentValue = calc_t::getImagPart(this->getOperandNode().evaluate());
    }
  };

  return ResultNode::create(*this);
}


#undef _CLASS
#define _CLASS UnitQuaternionExpression<TScalar, EMode, TNode>

_TEMPLATE
template <typename TOtherNode>
GenericMatrixExpression<3, 1,TScalar> _CLASS::rotate3Vector(const GenericMatrixExpression<3, 1, TScalar, TOtherNode> & vector) const {
  typedef GenericMatrixExpression<3, 1, TScalar> result_t;
  typedef GenericMatrixExpression<3, 1, TScalar, TOtherNode> other_t;
  typedef internal::EigenQuaternionCalculator<TScalar, EMode> calc_t;

  class ResultNode : public result_t::template BinaryOperationResult<ResultNode, self_t, other_t> {
  public:
    typedef typename result_t::template BinaryOperationResult<ResultNode, self_t, other_t> base_t;

    virtual ~ResultNode() {}

    inline typename base_t::apply_diff_return_t applyLhsDiff(const typename base_t::lhs_t::tangent_vector_t & tangent_vector) const {
      auto lhsVal = this->getLhsNode().evaluate();
      auto rhsVal = this->getRhsNode().evaluate();
      return calc_t::getImagPart(calc_t::quatMult(calc_t::quatMult(lhsVal, rhsVal), calc_t::conjugate(tangent_vector)) + calc_t::quatMult(calc_t::quatMult(tangent_vector, rhsVal), calc_t::conjugate(lhsVal)));
    }
    inline typename base_t::apply_diff_return_t applyRhsDiff(const typename base_t::rhs_t::tangent_vector_t & tangent_vector) const {
      auto lhsVal = this->getLhsNode().evaluate();
      return calc_t::getImagPart(calc_t::quatMult(calc_t::quatMult(lhsVal, tangent_vector), calc_t::conjugate(lhsVal)));
    }
  private:
    virtual void evaluateImplementation() const {
      auto lhsVal = this->getLhsNode().evaluate();
      this->_currentValue = calc_t::getImagPart(calc_t::quatMult(calc_t::quatMult(lhsVal, this->getRhsNode().evaluate()), calc_t::conjugate(lhsVal)));
    }
  };

  return ResultNode::create(*this, vector);
}

_TEMPLATE
template <typename TOtherNode>
typename _CLASS::tangent_vector_expression_t _CLASS::log(const UnitQuaternionExpression<TScalar, EMode, TOtherNode> & other){
  typedef _CLASS::tangent_vector_expression_t result_t;
  typedef UnitQuaternionExpression<TScalar, EMode, TOtherNode> other_t;

  typedef internal::EigenQuaternionCalculator<TScalar, EMode> calc_t;

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, other_t> {
  public:
    virtual ~ResultNode() {}
    virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename result_t::differential_t & diff) const {
      Eigen::Matrix<TScalar, 3, 4> M = calc_t::dlog(this->getOperandNode().evaluate());
      this->getOperandNode().evaluateJacobians(outJacobians, ComposedMatrixDifferential<typename result_t::differential_t::domain_t, TScalar, decltype(M) &, 4>(M, diff));
    }
  private:
    virtual void evaluateImplementation() const {
      this->_currentValue = calc_t::log(this->getOperandNode().evaluate());
    }
  };

  return ResultNode::create(other);
}

_TEMPLATE
template <typename TOtherNode>
typename _CLASS::self_with_default_node_t _CLASS::exp(const GenericMatrixExpression<3, 1, TScalar, TOtherNode> & tangentVector){
  typedef _CLASS::self_with_default_node_t result_t;
  typedef GenericMatrixExpression<3, 1, TScalar, TOtherNode> other_t;
  typedef internal::EigenQuaternionCalculator<TScalar, EMode> calc_t;

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, other_t> {
  public:
    virtual ~ResultNode() {}
    virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename result_t::differential_t & diff) const {
      Eigen::Matrix<TScalar, 4, 3> M = calc_t::dexp(this->getOperandNode().evaluate());
      this->getOperandNode().evaluateJacobians(outJacobians, ComposedMatrixDifferential<typename result_t::differential_t::domain_t, TScalar, decltype(M) &, 3>(M, diff));
    }
  private:
    virtual void evaluateImplementation() const {
      this->_currentValue = calc_t::exp(this->getOperandNode().evaluate());
    }
  };

  return ResultNode::create(tangentVector);
}


_TEMPLATE
template <enum UnitQuaternionGeometry EGeometry, typename TOtherNode>
typename _CLASS::tangent_vector_expression_t _CLASS::geoLog(const UnitQuaternionExpression<TScalar, EMode, TOtherNode> & other) const{
  switch(EGeometry){
    case UnitQuaternionGeometry::LEFT_TRANSLATED:
      return log(this->inverse() * other);
    case UnitQuaternionGeometry::RIGHT_TRANSLATED:
      return log(other * this->inverse());
  };
}

_TEMPLATE
template <enum UnitQuaternionGeometry EGeometry, typename TOtherNode>
typename _CLASS::self_with_default_node_t _CLASS::geoExp(const GenericMatrixExpression<3, 1, TScalar, TOtherNode> & tangentVector) const{
  switch(EGeometry){
    case UnitQuaternionGeometry::LEFT_TRANSLATED:
      return *this * exp(tangentVector);
    case UnitQuaternionGeometry::RIGHT_TRANSLATED:
      return exp(tangentVector) * *this;
  };
}


#undef _TEMPLATE
#undef _CLASS

}  // namespace quaternion
}  // namespace backend
}  // namespace aslam

