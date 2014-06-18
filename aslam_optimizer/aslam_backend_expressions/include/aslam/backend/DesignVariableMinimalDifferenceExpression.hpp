/*
 * DesignVariableMinimalDifferenceExpression.hpp
 *
 *  Created on: Sep 20, 2013
 *      Author: hannes
 */

#ifndef DESIGNVARIABLEMINIMALDIFFERENCEEXPRESSION_HPP_
#define DESIGNVARIABLEMINIMALDIFFERENCEEXPRESSION_HPP_

#include <aslam/backend/DesignVariable.hpp>
#include "VectorExpression.hpp"
#include "VectorExpressionNode.hpp"

namespace aslam {
namespace backend {
template<int D>
class DesignVariableMinimalDifferenceExpressionNode : public VectorExpressionNode<D> {
 public:
  typedef VectorExpressionNode<D> Base;
  typedef typename Base::vector_t vector_t;

  DesignVariableMinimalDifferenceExpressionNode(DesignVariable & dv, const Eigen::MatrixXd & xHat) : _xHat(xHat), _dv(dv) {}
  virtual ~DesignVariableMinimalDifferenceExpressionNode() {}

  virtual vector_t evaluateImplementation() const {
    Eigen::VectorXd v;
    _dv.minimalDifference(_xHat, v);
    return v;
  }

  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    Eigen::MatrixXd jac;
    Eigen::VectorXd v;
    _dv.minimalDifferenceAndJacobian(_xHat, v, jac);
    outJacobians.add(&_dv, jac);
  }
  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
    Eigen::MatrixXd jac;
    Eigen::VectorXd v;
    _dv.minimalDifferenceAndJacobian(_xHat, v, jac);
    outJacobians.add(&_dv, applyChainRule * jac);
  }
  virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    designVariables.insert(&_dv);
  }
 private:
  Eigen::MatrixXd _xHat;
  DesignVariable & _dv;
};

template<int D>
class DesignVariableMinimalDifferenceExpression : public VectorExpression<D> {
 public:
  DesignVariableMinimalDifferenceExpression(DesignVariable & dv, const Eigen::MatrixXd & xHat) :
    VectorExpression<D>(boost::shared_ptr<VectorExpressionNode<D> >(new DesignVariableMinimalDifferenceExpressionNode<D>(dv, xHat))) {}
};

}  // namespace backend
}  // namespace aslam

#endif /* DESIGNVARIABLEMINIMALDIFFERENCEEXPRESSION_HPP_ */
