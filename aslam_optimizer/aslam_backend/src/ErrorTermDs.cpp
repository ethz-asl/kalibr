#include <sm/eigen/assert_macros.hpp>

#include <aslam/backend/ErrorTermDs.hpp>

namespace aslam {
  namespace backend {


    ErrorTermDs::ErrorTermDs(int dimensionErrorTerm) : 
        _jacobians(dimensionErrorTerm),
        _dimensionErrorTerm(dimensionErrorTerm),
        _evalJacobianTimer("ErrorTerm: Evaluate Jacobian", true),
        _buildHessianTimer("ErrorTerm: Build Hessian", true)
    {
      _sqrtInvR = inverse_covariance_t::Identity(dimensionErrorTerm, dimensionErrorTerm);
    }

    ErrorTermDs::~ErrorTermDs()
    {
    }

    const typename ErrorTermDs::error_t& ErrorTermDs::error() const
    {
      return _error;
    }

    Eigen::VectorXd ErrorTermDs::vsErrorImplementation() const
    {
      return _error;
    }

    typename ErrorTermDs::inverse_covariance_t ErrorTermDs::invR() const
    {
      return _sqrtInvR * _sqrtInvR.transpose();
    }


    namespace detail {
      struct ErrorTermDsFunctor {
        typedef Eigen::MatrixXd value_t;
        typedef double scalar_t;
        typedef Eigen::VectorXd input_t;
        typedef Eigen::MatrixXd jacobian_t;

        ErrorTermDs& _et;
        ErrorTermDsFunctor(ErrorTermDs& et) :
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


    void ErrorTermDs::buildHessianImplementation(SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs, bool useMEstimator)
    {
      JacobianContainer J(dimension());
      _evalJacobianTimer.start();
      evaluateJacobians(J);
      _evalJacobianTimer.stop();
      _buildHessianTimer.start();
      double sqrtWeight = 1.0;
      if (useMEstimator)
        sqrtWeight = sqrt(_mEstimatorPolicy->getWeight(getRawSquaredError()));
      J.evaluateHessian(_error, sqrtWeight * _sqrtInvR, outHessian, outRhs);
      _buildHessianTimer.stop();
    }

    void ErrorTermDs::clearJacobians()
    {
      _jacobians.clear();
    }

    //void ErrorTermDs::setError(const Eigen::MatrixBase<double>& e)
    void ErrorTermDs::setError(const Eigen::VectorXd& e)
    {
      _error = e;
    }

    void ErrorTermDs::setInvR(const Eigen::MatrixXd& invR)
    {
      SM_ASSERT_EQ(Exception, invR.rows(), invR.cols(), "The covariance matrix must be square");
      SM_ASSERT_EQ(Exception, invR.rows(), (int)dimension(), "The covariance matrix does not match the size of the error");
      // http://eigen.tuxfamily.org/dox-devel/classEigen_1_1LDLT.html#details
      // LDLT seems to work on positive semidefinite matrices.
      sm::eigen::computeMatrixSqrt(invR, _sqrtInvR);
    }

    void ErrorTermDs::getInvR(Eigen::MatrixXd& invR) const
    {
      invR = _sqrtInvR * _sqrtInvR.transpose();
    }

    Eigen::MatrixXd ErrorTermDs::vsInvR() const
    {
      return _sqrtInvR * _sqrtInvR.transpose();
    }

    void ErrorTermDs::vsSetInvR(const Eigen::MatrixXd& invR)
    {
      setInvR(invR);
    }

    void ErrorTermDs::setSqrtInvR(const Eigen::MatrixXd& sqrtInvR)
    {
      _sqrtInvR = sqrtInvR;
    }

    const typename ErrorTermDs::inverse_covariance_t& ErrorTermDs::sqrtInvR() const
    {
      return _sqrtInvR;
    }

    /// \brief evaluate the squared error from the error vector and
    ///        square root covariance matrix.
    double ErrorTermDs::evaluateChiSquaredError() const
    {
      error_t e = _sqrtInvR.transpose() * _error;
      return e.dot(e);
    }

    void ErrorTermDs::getWeightedJacobians(JacobianContainer& outJc, bool useMEstimator)
    {
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

    void ErrorTermDs::getWeightedError(Eigen::VectorXd& e, bool useMEstimator) const
    {
      double sqrtWeight = 1.0;
      if (useMEstimator)
        sqrtWeight = sqrt(_mEstimatorPolicy->getWeight(getRawSquaredError()));
//      std::cout << "_sqrtInvR is" << std::endl;
//      std::cout << _sqrtInvR << std::endl;
//      std::cout << "_error is" << std::endl;
//      std::cout << _error << std::endl;
      e = _sqrtInvR.transpose() * _error * sqrtWeight;
    }

    void ErrorTermDs::checkJacobiansFinite() const {
#ifdef BOOST_NO_AUTO_DECLARATIONS
        for (JacobianContainer::map_t::iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {      
#else
        for (auto it = _jacobians.begin(); it != _jacobians.end(); ++it) {
#endif
        SM_ASSERT_MAT_IS_FINITE(Exception, it->second, "Jacobian is not finite!");
      }
    }

    void ErrorTermDs::checkJacobiansNumerical(double tolerance) {
      detail::ErrorTermDsFunctor functor(*this);
      sm::eigen::NumericalDiff<detail::ErrorTermDsFunctor >
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
        const Eigen::MatrixXd JAna = _jacobians.Jacobian(d);
        const Eigen::MatrixXd JNum =
          JNumComp.block(0, offset, _dimensionErrorTerm, d->minimalDimensions());
        for (int r = 0; r < JAna.rows(); ++r)
          for (int c = 0; c < JAna.cols(); ++c)
            SM_ASSERT_NEAR(Exception, JAna(r, c), JNum(r, c), tolerance, "Analytical and numerical Jacobians differ!");
        offset += d->minimalDimensions();
      }
    }

    void ErrorTermDs::resizeJacobianContainer(int nrows)
    {
    	_jacobians = JacobianContainer(nrows);
    }

  } // namespace backend
} // namespace aslam
