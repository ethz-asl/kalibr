
namespace aslam {
  namespace backend {

    template<int D>
    DesignVariableVector<D>::DesignVariableVector(vector_t v) : _v(v), _p_v(v)
    {

    }

    template<int D>
    DesignVariableVector<D>::~DesignVariableVector()
    {

    }

    template<int D>
    const typename DesignVariableVector<D>::vector_t & DesignVariableVector<D>::value() const
    {
      return _v;
    }

    template<int D>
    VectorExpression<D> DesignVariableVector<D>::toExpression()
    {
      return VectorExpression<D>(this);
    }


    /// \brief Revert the last state update.
    template<int D>
    void DesignVariableVector<D>::revertUpdateImplementation()
    {
      _v = _p_v;
    }

    /// \brief Update the design variable.
    template<int D>
    void DesignVariableVector<D>::updateImplementation(const double * dp, int size)
    {
      static_cast<void>(size); // unused in non debug build
      SM_ASSERT_EQ_DBG(aslam::InvalidArgumentException, size, D, "Update dimension doesn't match the state dimension");
      Eigen::Map< const vector_t > dv(dp);
      _p_v = _v;
      _v += dv;
    }

    /// \brief what is the number of dimensions of the perturbation variable.
    template<int D>
    int DesignVariableVector<D>::minimalDimensionsImplementation() const
    {
      return D;
    }


    template<int D>
    typename DesignVariableVector<D>::vector_t DesignVariableVector<D>::evaluateImplementation() const
    {
      return _v;
    }

    template<int D>
    void DesignVariableVector<D>::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      SM_ASSERT_EQ_DBG(aslam::InvalidArgumentException, outJacobians.rows(), D, "The Jacobian container dimension doesn't match the state. Are you missing a chain rule?");
      outJacobians.add(const_cast<DesignVariableVector<D>*>(this), Eigen::Matrix<double,D,D>::Identity());
    }

    template<int D>
    void DesignVariableVector<D>::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      SM_ASSERT_EQ_DBG(aslam::InvalidArgumentException, applyChainRule.cols(), D, "The chain rule matrix dimension doesn't match the state.");
      outJacobians.add(const_cast<DesignVariableVector<D>*>(this), applyChainRule);
    }

    template<int D>
    void DesignVariableVector<D>::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      designVariables.insert(const_cast<DesignVariableVector<D>*>(this));
    }

    template<int D>
    void DesignVariableVector<D>::getParametersImplementation(
        Eigen::MatrixXd& value) const {
      value = _v;
    }

    template<int D>
    void DesignVariableVector<D>::setParametersImplementation(
        const Eigen::MatrixXd& value) {
      _p_v = _v;
      _v = value;
    }

    template<int D>
    void DesignVariableVector<D>::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
    	SM_ASSERT_TRUE(aslam::Exception, (xHat.rows() == D)&&(xHat.cols() == 1), "Incompatible dimension of xHat.");
    	outDifference = _v - xHat;
    }

    template<int D>
    void DesignVariableVector<D>::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference,	Eigen::MatrixXd& outJacobian) const
    {
    	minimalDifferenceImplementation(xHat, outDifference);
    	outJacobian.setIdentity(D,D);
    }


  } // namespace backend
} // namespace aslam

