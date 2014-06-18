#include <aslam/backend/LinearSystemSolver.hpp>
#include <boost/thread.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <boost/ref.hpp>

namespace aslam {
  namespace backend {

    LinearSystemSolver::LinearSystemSolver() {}
    LinearSystemSolver::~LinearSystemSolver() {}

    void LinearSystemSolver::evaluateErrors(size_t threadId, size_t startIdx, size_t endIdx, bool useMEstimator)
    {
      SM_ASSERT_LT_DBG(Exception, threadId, _threadLocalErrors.size(), "Index out of bounds in thread " << threadId);
      SM_ASSERT_LE_DBG(Exception, endIdx, _errorTerms.size(), "Index out of bounds in thread " << threadId);
      Eigen::VectorXd e;
      for (size_t i = startIdx; i < endIdx; ++i) {
        SM_ASSERT_TRUE_DBG(Exception, _errorTerms[i] != NULL, "Null error term " << i);
        _threadLocalErrors[threadId] += _errorTerms[i]->evaluateError();
        _errorTerms[i]->getWeightedError(e, useMEstimator);
        _e.segment(_errorTerms[i]->rowBase(), _errorTerms[i]->dimension()) = -e;
      }
    }

    struct SafeJobReturnValue {
      SafeJobReturnValue(const std::exception& e) : _e(e) {}

      std::exception _e;
    };

    struct SafeJob {
      boost::function<void()> _fn;
      SafeJobReturnValue* _rval;
      SafeJob() : _rval(NULL) {}
      SafeJob(boost::function<void()> fn) : _fn(fn), _rval(NULL) {}
      ~SafeJob() {
        if (_rval) delete _rval;
      }

      void operator()() {
        try {
          _fn();
        } catch (const std::exception& e) {
          _rval = new SafeJobReturnValue(e);
          std::cout << "Exception in thread block: " << e.what() << std::endl;
        }
      }
    };

    void LinearSystemSolver::setupThreadedJob(boost::function<void(size_t, size_t, size_t, bool)> job, size_t nThreads, bool useMEstimator)
    {
      if (nThreads <= 1) {
        job(0, 0, _errorTerms.size(), useMEstimator);
      } else {
        nThreads = std::min(nThreads, _errorTerms.size());
        // Give some error terms to each thread.
        std::vector<int> indices(nThreads + 1, 0);
        int nJPerThread = std::max(1, (int)(_errorTerms.size() / nThreads));
        for (unsigned i = 0; i < nThreads; ++i)
          indices[i + 1] = indices[i] + nJPerThread;
        // deal with the remainder.
        indices.back() = _errorTerms.size();
        // Build a thread pool and evaluate the jacobians.
        boost::thread_group threads;
        std::vector<SafeJob> jobs(nThreads);
        for (size_t i = 0; i < nThreads; ++i) {
          jobs[i] = SafeJob(boost::bind(job, i, indices[i], indices[i + 1], useMEstimator));
          threads.create_thread(boost::ref(jobs[i]));
        }
        threads.join_all();
        // Now go through and look for exceptions.
        for (size_t i = 0; i < nThreads; ++i) {
          if (jobs[i]._rval) {
            throw jobs[i]._rval->_e;
          }
        }
      }
    }


    double LinearSystemSolver::evaluateError(size_t nThreads, bool useMEstimator)
    {
      nThreads = std::max((size_t)1, nThreads);
      _threadLocalErrors.clear();
      _threadLocalErrors.resize(nThreads, 0.0);
      setupThreadedJob(boost::bind(&LinearSystemSolver::evaluateErrors, this, _1, _2, _3, _4), nThreads, useMEstimator);
      // Gather the squared error results from the multiple threads.
      double error = 0.0;
      for (unsigned i = 0; i < _threadLocalErrors.size(); ++i)
        error += _threadLocalErrors[i];
      return error;
    }

    const Eigen::VectorXd& LinearSystemSolver::e() const
    {
      return _e;
    }

    const Eigen::VectorXd& LinearSystemSolver::rhs() const
    {
      return _rhs;
    }


    void LinearSystemSolver::setConditioner(const Eigen::VectorXd& diag)
    {
      SM_ASSERT_EQ(Exception, (size_t)diag.size(), _JCols, "The diagonal conditioner must have the same number of rows as the Hessian matrix");
      _diagonalConditioner = diag;
    }

    void LinearSystemSolver::setConstantConditioner(double diag)
    {
      _diagonalConditioner = Eigen::VectorXd::Constant(_JCols, diag);
    }


    void LinearSystemSolver::initMatrixStructure(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner)
    {
      setOrdering(dvs, errors);
      _errorTerms = errors;
      // Figure out the size of the Jacobian matrix.
      _JRows = 0;
      std::vector<ErrorTerm*>::const_iterator eit = errors.begin();
      for (; eit != errors.end(); ++eit) {
        _JRows += (*eit)->dimension();
      }
      _JCols = 0;
      std::vector<DesignVariable*>::const_iterator dit = dvs.begin();
      for (; dit != dvs.end(); ++dit) {
        _JCols += (*dit)->minimalDimensions();
      }
      // \todo Verify that this is similar to the "reserve()" feature in a standard vector.
      _e.resize(_JRows + _JCols);
      _e.conservativeResize(_JRows);
      _rhs.resize(_JCols);
      _diagonalConditioner = Eigen::VectorXd::Zero(_JCols);
      initMatrixStructureImplementation(dvs, errors, useDiagonalConditioner);
    }

    /// \brief the number of rows in the Jacobian matrix
    size_t LinearSystemSolver::JRows() const
    {
      return _JRows;
    }

    /// \brief the number of columns in the Jacobian matrix
    size_t LinearSystemSolver::JCols() const
    {
      return _JCols;
    }


  } // namespace backend
} // namespace aslam
