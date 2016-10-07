
namespace aslam {
namespace backend {

template<int D>
void VectorExpressionNode<D>::evaluateJacobians(JacobianContainer & outJacobians) const {
  evaluateJacobiansImplementation(outJacobians);
}

template<int D>
void VectorExpressionNode<D>::evaluateJacobians(JacobianContainer & outJacobians, const differential_t & diff) const {
  evaluateJacobiansImplementationWithDifferential(outJacobians, diff);
}

template<int D>
void VectorExpressionNode<D>::evaluateJacobiansImplementationWithDifferential(JacobianContainer & outJacobians, const differential_t & chainRuleDifferentail) const {
  evaluateJacobiansImplementation(applyDifferentialToJacobianContainer(outJacobians, chainRuleDifferentail, getSize()));
}

template<int D>
void VectorExpressionNode<D>::getDesignVariables(DesignVariable::set_t & designVariables) const {
  getDesignVariablesImplementation(designVariables);
}

}  // namespace backend
}  // namespace aslam
