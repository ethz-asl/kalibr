#ifndef ASLAM_BACKEND_GENERIC_MATRIX_EXPRESSION_NODE_HPP
#define ASLAM_BACKEND_GENERIC_MATRIX_EXPRESSION_NODE_HPP
#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/backend/Differential.hpp>

namespace aslam {
namespace backend {

template<int IRows, int ICols, typename TScalar> class ConstantGenericMatrixExpressionNode;

template<int IRows, int ICols, typename TScalar>
class GenericMatrixExpressionNode {
 public:
  typedef GenericMatrixExpressionNode<IRows, ICols, TScalar> self_t;
  typedef Eigen::Matrix<TScalar, IRows, ICols> matrix_t;
  typedef matrix_t value_t;
  typedef Eigen::Matrix<TScalar, IRows, ICols> tangent_vector_t;
  typedef Differential<tangent_vector_t, TScalar> differential_t;
  typedef boost::shared_ptr<self_t> ptr_t;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  mutable matrix_t _currentValue;
  mutable bool _valueDirty;

  inline GenericMatrixExpressionNode(const matrix_t & value)
      : _currentValue(value),
        _valueDirty(false) {
  }
 public:
  GenericMatrixExpressionNode(int rows = IRows, int cols = ICols, bool valueDirty = true)
      : _currentValue(rows, cols),
        _valueDirty(valueDirty) {
  }
  virtual ~GenericMatrixExpressionNode() {
  }

  inline matrix_t & getCurrentValue() {
    return _currentValue;
  }
  inline const matrix_t & getCurrentValue() const {
    return _currentValue;
  }
  inline const matrix_t & evaluate() const {
    evaluateImplementation();
    /* TODO activate value caching again.
    if (!isConstant() && _valueDirty) {
      evaluateImplementation();
      _valueDirty = false;
    };*/
    return _currentValue;
  }

  void evaluateJacobians(JacobianContainer & outJacobians, const differential_t & chainRuleDifferentail) const {
    evaluateJacobiansImplementation(outJacobians, chainRuleDifferentail);
  }

  const matrix_t toMatrix() const {
    return evaluate();
  }
  ;

  void getDesignVariables(DesignVariable::set_t & designVariables) const {
    return getDesignVariablesImplementation(designVariables);
  }
  bool isConstant() const {
    return isConstantImplementation();
  }

  void inline invalidate() {
    _valueDirty = true;
  }
  ;
 protected:
  virtual void evaluateImplementation() const = 0;
  virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;
  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const differential_t & chainRuleDifferentail) const = 0;
  virtual bool isConstantImplementation() const {
    return false;
  }

 public:
  typedef ConstantGenericMatrixExpressionNode<IRows, ICols, TScalar> constant_t;
};

template<int IRows, int ICols, typename TScalar>
class ConstantGenericMatrixExpressionNode : public GenericMatrixExpressionNode<IRows, ICols, TScalar> {
 public:
  typedef GenericMatrixExpressionNode<IRows, ICols, TScalar> base_t;
  template<typename DERIVED>
  ConstantGenericMatrixExpressionNode(const Eigen::MatrixBase<DERIVED> & value)
      : base_t(value) {
  }
  ConstantGenericMatrixExpressionNode(int rows = IRows, int cols = ICols)
      : base_t(rows, cols, false) {
  }
 protected:
  virtual bool isConstantImplementation() const {
    return true;
  }
  virtual void evaluateImplementation() const {
  }
  virtual void evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */, const typename base_t::differential_t & /* chainRuleDifferentail */) const {
  }
  virtual void getDesignVariablesImplementation(DesignVariable::set_t & /* designVariables */) const {
  }
};

}  // namespace backend
}  // namespace aslam

//#include "implementation/GenericMatrixExpressionNode.hpp"

#endif /* ASLAM_BACKEND_GENERIC_MATRIX_EXPRESSION_NODE_HPP */
