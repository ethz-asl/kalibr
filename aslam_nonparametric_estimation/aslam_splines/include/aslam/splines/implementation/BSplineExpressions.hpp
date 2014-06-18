namespace aslam {
  namespace splines {
    
    
    template<int D>
    BSplineVectorExpressionNode<D>::BSplineVectorExpressionNode(bsplines::BSpline * spline, int derivativeOrder, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
      _spline(spline), _designVariables(designVariables), _time(time), _derivativeOrder(derivativeOrder)
    {
      SM_ASSERT_EQ(aslam::Exception, spline->coefficients().rows(), D, "The spline dimension should match the expression dimension");
    }

    template<int D>
    BSplineVectorExpressionNode<D>::~BSplineVectorExpressionNode()
    {

    }

    template<int D>
    typename BSplineVectorExpressionNode<D>::vector_t BSplineVectorExpressionNode<D>::evaluateImplementation() const
    {
      return _spline->evalD(_time, _derivativeOrder);
    }


    template<int D>
    void BSplineVectorExpressionNode<D>::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
    {
      Eigen::MatrixXd J;
      _spline->evalDAndJacobian(_time, _derivativeOrder, &J, NULL);

      for(size_t i = 0; i < _designVariables.size(); ++i)
	{
	  outJacobians.add(_designVariables[i], J.block<D,D>(0,i*D) );
	}
    }


    template<int D>
    void BSplineVectorExpressionNode<D>::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), D, "The chain rule matrix is the wrong size");

      Eigen::MatrixXd J;
      _spline->evalDAndJacobian(_time, _derivativeOrder, &J, NULL);
      
      for(size_t i = 0; i < _designVariables.size(); ++i)
	{
	  outJacobians.add(_designVariables[i], applyChainRule * J.block<D,D>(0,i*D) );
	}

    }

    template<int D>
    void BSplineVectorExpressionNode<D>::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
    {
      for(size_t i = 0; i < _designVariables.size(); ++i)
	{
	  designVariables.insert(_designVariables[i]);
	}

    }

    
  } // namespace splines
} // namespace aslam
