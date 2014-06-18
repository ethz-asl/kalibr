
namespace aslam {
  namespace backend {
    
    template<int D>
    VectorExpression<D>::VectorExpression(boost::shared_ptr< VectorExpressionNode<D> > root) :
      _root(root)
    {

    }

    template<int D>
    VectorExpression<D>::VectorExpression(VectorExpressionNode<D> * root) :
      _root(root, sm::null_deleter() )
    {
      
    }

    template<int D>
    VectorExpression<D>::~VectorExpression()
    {

    }

      
    template<int D>
    typename VectorExpression<D>::vector_t VectorExpression<D>::evaluate() const
    {
      return _root->evaluate();
    }

    template<int D>
    typename VectorExpression<D>::vector_t VectorExpression<D>::toValue() const
    {
      return _root->evaluate();
    }

      
    template<int D>
    void VectorExpression<D>::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      return _root->evaluateJacobians(outJacobians);
    }

    template<int D>
    void VectorExpression<D>::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      return _root->evaluateJacobians(outJacobians, applyChainRule);
    }

    template<int D>
    void VectorExpression<D>::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      return _root->getDesignVariables(designVariables);
    }

    template<int D>
    ScalarExpression VectorExpression<D>::toScalarExpression() const
    {
    	//static_assert(D == 1, "Incompatible vector size");
        boost::shared_ptr<ScalarExpressionNode> newRoot( new ScalarExpressionNodeFromVectorExpression(_root));
        return ScalarExpression(newRoot);
    }


  } // namespace backend
} // namespace aslam
