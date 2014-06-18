#include <sm/eigen/assert_macros.hpp>

namespace aslam {
  namespace backend {

    template<typename ITERATOR_T>
    void ErrorTerm::setDesignVariablesIterator(ITERATOR_T start, ITERATOR_T end)
    {
      /// \todo Set the back link to the error term in the design variable.
      SM_ASSERT_EQ(aslam::UnsupportedOperationException, _designVariables.size(), 0, "The design variable container already has objects. The design variables may only be set once");
      /// \todo: set the back-link in the design variable.
      int ii = 0;
      for (ITERATOR_T i = start; i != end; ++i, ++ii) {
        SM_ASSERT_TRUE(aslam::InvalidArgumentException, *i != NULL, "Design variable " << ii << " is null");
      }
      _designVariables.insert(_designVariables.begin(), start, end);
    }



    template<int C>
    ErrorTermFs<C>::ErrorTermFs() : 
      _evalJacobianTimer("ErrorTerm: Evaluate Jacobian", true),
      _buildHessianTimer("ErrorTerm: Build Hessian", true)
    {
      _sqrtInvR = inverse_covariance_t::Identity();
    }

    template<int C>
    ErrorTermFs<C>::~ErrorTermFs()
    {
    }



    template<int C>
    const typename ErrorTermFs<C>::error_t& ErrorTermFs<C>::error() const
    {
      return _error;
    }

    template<int C>
    Eigen::VectorXd ErrorTermFs<C>::vsErrorImplementation() const
    {
      return _error;
    }

    template<int C>
    typename ErrorTermFs<C>::inverse_covariance_t ErrorTermFs<C>::invR() const
    {
      return _sqrtInvR * _sqrtInvR.transpose();
    }


    namespace detail {
      template<int C>
      struct ErrorTermFsFunctor {
        typedef Eigen::Matrix<double, C, 1> value_t;
        typedef double scalar_t;
        typedef Eigen::VectorXd input_t;
        typedef Eigen::MatrixXd jacobian_t;

        ErrorTermFs<C>& _et;
        ErrorTermFsFunctor(ErrorTermFs<C>& et) :
          _et(et) {}

        input_t update(const input_t& x, int c, scalar_t delta) {
          input_t xnew = x;
          xnew[c] += delta;
          return xnew;
        }

        Eigen::VectorXd operator()(const Eigen::VectorXd& dr) {
          int offset = 0;
          for (size_t i = 0; i < _et.numDesignVariables(); i++) {
            DesignVariable* d = _et.designVariable(i);
            SM_ASSERT_LE_DBG(aslam::Exception, offset + d->minimalDimensions(), dr.size(), "The offset is out of bounds.");
            d->update(&dr[offset], d->minimalDimensions());
            offset += d->minimalDimensions();
          }
          SM_ASSERT_EQ_DBG(aslam::Exception, offset, dr.size(), "The input vector is too large. It wasn't covered by the design variables.");
          _et.evaluateError();
          value_t e = _et.error();
          for (size_t i = 0; i < _et.numDesignVariables(); i++) {
            DesignVariable* d = _et.designVariable(i);
            d->revertUpdate();
          }
          return e;
        }
      };

    } // namespace detail



    template<int C>
    void ErrorTermFs<C>::buildHessianImplementation(SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs, bool useMEstimator)
    {
      _evalJacobianTimer.start();
      JacobianContainer J(C);
      evaluateJacobians(J);
      _evalJacobianTimer.stop();
      _buildHessianTimer.start();
      double sqrtWeight = 1.0;
      if (useMEstimator)
        sqrtWeight = sqrt(_mEstimatorPolicy->getWeight(getRawSquaredError()));
      J.evaluateHessian(_error, sqrtWeight * _sqrtInvR, outHessian, outRhs);
      _buildHessianTimer.stop();
    }

    template<int C>
    template<typename DERIVED>
    void ErrorTermFs<C>::setError(const Eigen::MatrixBase<DERIVED>& e)
    {
      _error = e;
    }

    template<int C>
    template<typename DERIVED>
    void ErrorTermFs<C>::setInvR(const Eigen::MatrixBase<DERIVED>& invR)
    {
      SM_ASSERT_EQ(Exception, invR.rows(), invR.cols(), "The covariance matrix must be square");
      SM_ASSERT_EQ(Exception, invR.rows(), (int)dimension(), "The covariance matrix does not match the size of the error");
      // http://eigen.tuxfamily.org/dox-devel/classEigen_1_1LDLT.html#details
      // LDLT seems to work on positive semidefinite matrices.
      sm::eigen::computeMatrixSqrt(invR, _sqrtInvR);
    }

    template<int C>
    void ErrorTermFs<C>::getInvR(Eigen::MatrixXd& invR) const
    {
      invR = _sqrtInvR * _sqrtInvR.transpose();
    }

    template<int C>
    Eigen::MatrixXd ErrorTermFs<C>::vsInvR() const
    {
      return _sqrtInvR * _sqrtInvR.transpose();
    }

    template<int C>
    void ErrorTermFs<C>::vsSetInvR(const Eigen::MatrixXd& invR)
    {
      setInvR(invR);
    }

    template<int C>
    template<typename DERIVED>
    void ErrorTermFs<C>::setSqrtInvR(const Eigen::MatrixBase<DERIVED>& sqrtInvR)
    {
      _sqrtInvR = sqrtInvR;
    }

    template<int C>
    const typename ErrorTermFs<C>::inverse_covariance_t& ErrorTermFs<C>::sqrtInvR() const
    {
      return _sqrtInvR;
    }

    /// \brief evaluate the squared error from the error vector and
    ///        square root covariance matrix.
    template<int C>
    double ErrorTermFs<C>::evaluateChiSquaredError() const
    {
      error_t e = _sqrtInvR.transpose() * _error;
      return e.dot(e);
    }

    template<int C>
    void ErrorTermFs<C>::getWeightedJacobians(JacobianContainer& outJc, bool useMEstimator) 
    {
      // take a copy. \todo Don't take a copy.
      evaluateJacobians(outJc);
      outJc.applyChainRule(_sqrtInvR.transpose());
      JacobianContainer::map_t::iterator it = outJc.begin();
      double sqrtWeight = 1.0;
      if (useMEstimator)
        sqrtWeight = sqrt(_mEstimatorPolicy->getWeight(getRawSquaredError()));
      for (; it != outJc.end(); ++it) {
        it->second *=  sqrtWeight * it->first->scaling();
      }
    }


    template<int C>
    void ErrorTermFs<C>::getWeightedError(Eigen::VectorXd& e, bool useMEstimator) const
    {
      double sqrtWeight = 1.0;
      if (useMEstimator)
        sqrtWeight = sqrt(_mEstimatorPolicy->getWeight(getRawSquaredError()));
      e = _sqrtInvR.transpose() * _error * sqrtWeight;
    }

    template<int C>
    void ErrorTermFs<C>::checkJacobiansFinite() const {
      JacobianContainer J(C);
      evaluateJacobians(J);
      for (JacobianContainer::map_t::iterator it = J.begin(); it != J.end(); ++it) {      
        SM_ASSERT_MAT_IS_FINITE(Exception, it->second,
                                "Jacobian is not finite!");
      }
    }

    template<int C>
    void ErrorTermFs<C>::checkJacobiansNumerical(double tolerance) {

      JacobianContainer J(C);
      evaluateJacobians(J);
      
      
      detail::ErrorTermFsFunctor<C> functor(*this);
      sm::eigen::NumericalDiff<detail::ErrorTermFsFunctor<C> >
        numdiff(functor, tolerance);
      int inputSize = 0;
      for (size_t i = 0; i < numDesignVariables(); i++) {
        inputSize += designVariable(i)->minimalDimensions();
      }
      const Eigen::MatrixXd JNumComp =
        numdiff.estimateJacobian(Eigen::VectorXd::Zero(inputSize));
      int offset = 0;
      for (size_t i = 0; i < numDesignVariables(); i++) {
        DesignVariable* d = designVariable(i);
        const Eigen::MatrixXd JAna = J.Jacobian(d);
        const Eigen::MatrixXd JNum =
          JNumComp.block(0, offset, C, d->minimalDimensions());
        for (int r = 0; r < JAna.rows(); ++r)
          for (int c = 0; c < JAna.cols(); ++c)
            SM_ASSERT_NEAR(Exception, JAna(r, c), JNum(r, c), tolerance,
            "Analytical and numerical Jacobians differ!");
        offset += d->minimalDimensions();
      }
    }

  } // namespace backend
} // namespace aslam
