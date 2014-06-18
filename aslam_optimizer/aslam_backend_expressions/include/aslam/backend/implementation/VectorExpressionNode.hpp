
namespace aslam {
namespace backend {
template<int D>
VectorExpressionNode<D>::VectorExpressionNode() {

}

template<int D>
VectorExpressionNode<D>::~VectorExpressionNode() {

}

template<int D>
typename VectorExpressionNode<D>::vector_t VectorExpressionNode<D>::evaluate() const {
  return evaluateImplementation();
}

template<int D>
typename VectorExpressionNode<D>::vector_t VectorExpressionNode<D>::toVector() const {
  return evaluateImplementation();
}

template<int D>
void VectorExpressionNode<D>::evaluateJacobians(JacobianContainer & outJacobians) const {
  evaluateJacobiansImplementation(outJacobians);
}

template<int D>
void VectorExpressionNode<D>::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
  evaluateJacobiansImplementation(outJacobians, applyChainRule);
}

template<int D>
void VectorExpressionNode<D>::evaluateJacobians(JacobianContainer & outJacobians, const differential_t & diff) const {
  evaluateJacobiansImplementationWithDifferential(outJacobians, diff);
}

template<int D>
void VectorExpressionNode<D>::evaluateJacobiansImplementationWithDifferential(JacobianContainer & outJacobians, const differential_t & chainRuleDifferentail) const {
  Eigen::MatrixXd chainRule(outJacobians.rows(), D);
  chainRuleDifferentail.convertIntoMatrix(outJacobians.rows(), D, chainRule);
  evaluateJacobiansImplementation(outJacobians, chainRule);
}

template<int D>
void VectorExpressionNode<D>::getDesignVariables(DesignVariable::set_t & designVariables) const {
  getDesignVariablesImplementation(designVariables);
}

}  // namespace backend
}  // namespace aslam
