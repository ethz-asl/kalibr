
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
    VectorExpression<D>::VectorExpression(const vector_t & v) :
      _root(new ConstantVectorExpressionNode<D>(v))
    {
    }
      
    template<int D>
    int VectorExpression<D>::getSize() const {
      if(D != Eigen::Dynamic){
        return D;
      } else {
        assert(_root);
        return _root->getSize();
      }
    }

    template<int D>
    typename VectorExpression<D>::vector_t VectorExpression<D>::evaluate() const
    {
      if(isEmpty()){
        return VectorExpression<D>::vector_t::Zero(D);
      }
      return _root->evaluate();
    }

    template<int D>
    void VectorExpression<D>::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      if(!isEmpty())
        _root->evaluateJacobians(outJacobians);
    }

    template<int D>
    void VectorExpression<D>::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      if(!isEmpty())
        _root->getDesignVariables(designVariables);
    }

    template<int D>
    template<int ComponentIndex>
    ScalarExpression VectorExpression<D>::toScalarExpression() const
    {
      boost::shared_ptr<ScalarExpressionNode> newRoot( new ScalarExpressionNodeFromVectorExpression<D, ComponentIndex>(_root));
      return ScalarExpression(newRoot);
    }

    template<int D>
    ScalarExpression VectorExpression<D>::toScalarExpression() const{
      return toScalarExpression<0>();
    }

  } // namespace backend
} // namespace aslam
