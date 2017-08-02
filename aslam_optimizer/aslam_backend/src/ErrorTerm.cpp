#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <boost/make_shared.hpp>
#include <sm/logging.hpp>

namespace aslam {
  namespace backend {
    ErrorTerm::ErrorTerm() :
      _squaredError(0.0), _timestamp(0)
    {
      _mEstimatorPolicy = boost::make_shared<NoMEstimator>();
    }

    ErrorTerm::~ErrorTerm()
    {
    }


    /// \brief evaluate the error term.
    double ErrorTerm::evaluateError()
    {
      _squaredError = evaluateErrorImplementation();
      return _mEstimatorPolicy->getWeight(_squaredError) * _squaredError;
    }


    /// \brief evaluate the Jacobians.
    void ErrorTerm::evaluateJacobians(JacobianContainer & outJ) const
    {
      outJ.clear();
      evaluateJacobiansImplementation(outJ);
    }

    /// \brief build this error term's part of the Hessian matrix.
    ///
    /// the i/o variables outHessian and outRhs are the full Hessian and rhs in the Gauss-Newton
    /// problem. The correct blocks for each design varible are available from the design
    /// variable as dv.blockIndex()
    void ErrorTerm::buildHessian(SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs, bool useMEstimator)
    {
      buildHessianImplementation(outHessian, outRhs, useMEstimator);
    }

    /// \brief set the M-Estimator policy. This function takes a squared error
    ///        and returns a weight to apply to that error term.
    void ErrorTerm::setMEstimatorPolicy(const boost::shared_ptr<MEstimator>& mEstimator)
    {
      _mEstimatorPolicy = mEstimator;
    }


    /// \brief clear the m-estimator policy.
    void ErrorTerm::clearMEstimatorPolicy()
    {
      _mEstimatorPolicy = boost::make_shared<NoMEstimator>();
    }


    /// \brief compute the M-estimator weight from a squared error.
    double ErrorTerm::getMEstimatorWeight(double squaredError) const
    {
      return _mEstimatorPolicy->getWeight(squaredError);
    }

    /// \brief compute the M-estimator weight from a squared error.
    double ErrorTerm::getCurrentMEstimatorWeight() const
    {
      return _mEstimatorPolicy->getWeight(_squaredError);
    }

  

    /// \brief How many design varibles is this error term connected to?
    size_t ErrorTerm::numDesignVariables() const
    {
      return _designVariables.size();
    }

    /// \brief Get design variable i.
    DesignVariable* ErrorTerm::designVariable(size_t i)
    {
      SM_ASSERT_LT_DBG(aslam::IndexOutOfBoundsException, i, _designVariables.size(), "index out of bounds");
      return _designVariables[i];
    }

    /// \brief Get design variable i.
    const DesignVariable* ErrorTerm::designVariable(size_t i) const
    {
      SM_ASSERT_LT_DBG(aslam::IndexOutOfBoundsException, i, _designVariables.size(), "index out of bounds");
      return _designVariables[i];
    }

    void ErrorTerm::setDesignVariables(const std::vector<DesignVariable*>& designVariables)
    {
      /// \todo Set the back link to the error term in the design variable.
      SM_ASSERT_EQ(aslam::UnsupportedOperationException, _designVariables.size(), 0, "The design variable container already has objects. The design variables may only be set once");
      /// \todo: set the back-link in the design variable.
      for (unsigned i = 0; i < designVariables.size(); ++i) {
        SM_ASSERT_TRUE(aslam::InvalidArgumentException, designVariables[i] != NULL, "Design variable " << i << " is null");
      }
      _designVariables = designVariables;
    }

    void ErrorTerm::setDesignVariables(DesignVariable* dv1)
    {
      std::vector<DesignVariable*> v;
      v.push_back(dv1);
      setDesignVariables(v);
    }

    void ErrorTerm::setDesignVariables(DesignVariable* dv1, DesignVariable* dv2)
    {
      std::vector<DesignVariable*> v;
      v.push_back(dv1);
      v.push_back(dv2);
      setDesignVariables(v);
    }
    void ErrorTerm::setDesignVariables(DesignVariable* dv1, DesignVariable* dv2, DesignVariable* dv3)
    {
      std::vector<DesignVariable*> v;
      v.push_back(dv1);
      v.push_back(dv2);
      v.push_back(dv3);
      setDesignVariables(v);
    }
    void ErrorTerm::setDesignVariables(DesignVariable* dv1, DesignVariable* dv2, DesignVariable* dv3, DesignVariable* dv4)
    {
      std::vector<DesignVariable*> v;
      v.push_back(dv1);
      v.push_back(dv2);
      v.push_back(dv3);
      v.push_back(dv4);
      setDesignVariables(v);
    }


    Eigen::VectorXd ErrorTerm::vsError() const
    {
      return vsErrorImplementation();
    }

    std::string ErrorTerm::getMEstimatorName()
    {
      return _mEstimatorPolicy->name();
    }

    /// \brief Get the squared error (weighted by the M-estimator policy)
    double ErrorTerm::getWeightedSquaredError() const
    {
      return _mEstimatorPolicy->getWeight(_squaredError) * _squaredError;;
    }

    /// \brief Get the squared error (before weighting by the M-estimator policy)
    double ErrorTerm::getRawSquaredError() const
    {
      return _squaredError;
    }


    void ErrorTerm::getDesignVariables(DesignVariable::set_t& dvs)
    {
      dvs.insert(_designVariables.begin(), _designVariables.end());
    }

    size_t ErrorTerm::dimension() const
    {
      return getDimensionImplementation();
    }


    const std::vector<DesignVariable*>& ErrorTerm::designVariables() const
    {
      return _designVariables;
    }

    /// \brief Get the column base of this error term in the Jacobian matrix.
    size_t ErrorTerm::rowBase() const
    {
      return _rowBase;
    }

    /// \brief Set the column base of this error term in the Jacobian matrix.
    void ErrorTerm::setRowBase(size_t b)
    {
      _rowBase = b;
    }

    void ErrorTerm::setTime(const sm::timing::NsecTime& t)
    {
    	_timestamp = t;
    }



    namespace detail {
    struct ErrorTermFunctor {
      typedef Eigen::VectorXd value_t;
      typedef double scalar_t;
      typedef Eigen::VectorXd input_t;
      typedef Eigen::MatrixXd jacobian_t;

        ErrorTerm & _et;
        ErrorTermFunctor(ErrorTerm & et) :
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
          value_t e = _et.vsError();
          for (size_t i = 0; i < _et.numDesignVariables(); i++) {
            DesignVariable* d = _et.designVariable(i);
            d->revertUpdate();
          }
          return e;
        }
      };

    } // namespace detail

  
    // This is sub-optimal in terms of efficiency but it is mostly used for
    // unit testing and prototyping in any case.
    void ErrorTerm::evaluateJacobiansFiniteDifference(JacobianContainer & outJacobians)
    {
      outJacobians.clear();
      detail::ErrorTermFunctor functor(*this);
      sm::eigen::NumericalDiff< detail::ErrorTermFunctor > numdiff(functor, 1e-6);
      int inputSize = 0;
      for (size_t i = 0; i < numDesignVariables(); i++) {
        inputSize += designVariable(i)->minimalDimensions();
      }
      Eigen::MatrixXd J = numdiff.estimateJacobian(Eigen::VectorXd::Zero(inputSize));
      // Now pack the jacobian container.
      outJacobians.clear();
      int offset = 0;
      for (size_t i = 0; i < numDesignVariables(); i++) {
        DesignVariable* d = designVariable(i);
        outJacobians.add(d, J.block(0, offset, J.rows(), d->minimalDimensions()));
        offset += d->minimalDimensions();
      }
      // Done.
    }

  
  } // namespace backend
} // namespace aslam

