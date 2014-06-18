#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <sm/PropertyTree.hpp>

namespace aslam {
  namespace backend {
    SparseQrLinearSystemSolver::SparseQrLinearSystemSolver(const SparseQRLinearSolverOptions& options) :
        _factor(NULL),
        _options(options) {
    }

    SparseQrLinearSystemSolver::SparseQrLinearSystemSolver(const sm::PropertyTree& config) :
        _factor(NULL) {
      SparseQRLinearSolverOptions options;
      options.colNorm = config.getBool("colNorm", options.colNorm);
      options.qrTol = config.getDouble("qrTol", options.qrTol);
      options.normTol = config.getDouble("normTol", options.normTol);
      options.verbose = config.getBool("verbose", options.verbose);
      _options = options;
      // USING C++11 would allow to do constructor delegation and more elegant code
    }

    SparseQrLinearSystemSolver::~SparseQrLinearSystemSolver() {
      if (_factor) {
        _cholmod.free(_factor);
        _factor = NULL;
      }
    }


  void SparseQrLinearSystemSolver::initMatrixStructureImplementation(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool /* useDiagonalConditioner */)
    {
      _errorTerms = errors;
      if (_factor) {
        _cholmod.free(_factor);
        _factor = NULL;
      }
      // should not be available or am i wrong?
      _useDiagonalConditioner = false; // useDiagonalConditioner;
      _jacobianBuilder.initMatrixStructure(dvs, errors);
      // spqr is only available with LONG indices
      CompressedColumnMatrix<SuiteSparse_long>& J_transpose = _jacobianBuilder.J_transpose();
      if (_useDiagonalConditioner) {
        J_transpose.pushConstantDiagonalBlock(1.0);
      }
      // View this matrix as a sparse matrix.
      // These views should remain valid for the lifetime of the object.
      J_transpose.getView(&_cholmodLhs);
      _cholmod.view(_e, &_cholmodRhs);
      if (_useDiagonalConditioner) {
        J_transpose.popDiagonalBlock();
      }
    }

    void SparseQrLinearSystemSolver::buildSystem(size_t nThreads, bool useMEstimator)
    {
      //std::cout << "build system\n";
      _jacobianBuilder.buildSystem(nThreads, useMEstimator);
      CompressedColumnMatrix<SuiteSparse_long>& J_transpose = _jacobianBuilder.J_transpose();
      J_transpose.rightMultiply(_e, _rhs);
      //std::cout << "build system complete\n";
      _R.clear();
    }

    bool SparseQrLinearSystemSolver::solveSystem(Eigen::VectorXd& outDx)
    {
      CompressedColumnMatrix<SuiteSparse_long>& J_transpose = _jacobianBuilder.J_transpose();
      if (_useDiagonalConditioner) {
        J_transpose.pushDiagonalBlock(_diagonalConditioner);
      }
      J_transpose.getView(&_cholmodLhs);
      _cholmod.view(_e, &_cholmodRhs);
      //std::cout << "solve system\n";
      if (!_factor) {
        //std::cout << "\tAnalyze system\n";
        // Now do the symbolic analysis with cholmod.
        _factor = _cholmod.analyzeQR(&_cholmodLhs);
        //std::cout << "\tanalyze system complete\n";
      }
      // Now we can solve the system.
      outDx.resize(J_transpose.rows());
      cholmod_dense* sol = _cholmod.solve(&_cholmodLhs, _factor, &_cholmodRhs,
        _options.qrTol, _options.colNorm, _options.normTol);
      if (_useDiagonalConditioner) {
        J_transpose.popDiagonalBlock();
      }
      if (!sol) {
        std::cout << "Solution failed\n";
        return false;
      }
      try {
        SM_ASSERT_EQ_DBG(Exception, (int)sol->nrow, (int)outDx.size(), "Unexpected solution size");
        SM_ASSERT_EQ_DBG(Exception, sol->ncol, 1, "Unexpected solution size");
        SM_ASSERT_EQ_DBG(Exception, sol->xtype, (int)CholmodValueTraits<double>::XType, "Unexpected solution type");
        SM_ASSERT_EQ_DBG(Exception, sol->dtype, (int)CholmodValueTraits<double>::DType, "Unexpected solution type");
        memcpy((void*)&outDx[0], sol->x, sizeof(double)*sol->nrow);
      } catch (const Exception& e) {
        std::cout << e.what() << std::endl;
        // avoid leaking memory but still do error checking.
        // look at me! I done good.
        _cholmod.free(sol);
        throw;
      }
      _cholmod.free(sol);
      if (_options.verbose)
        std::cout << "numerical rank: " << _factor->rank << std::endl;
      // std::cout << "solve system complete\n";
      return true;
    }

    const SparseQRLinearSolverOptions&
    SparseQrLinearSystemSolver::getOptions() const {
      return _options;
    }

    SparseQRLinearSolverOptions&
    SparseQrLinearSystemSolver::getOptions() {
      return _options;
    }

    void SparseQrLinearSystemSolver::setOptions(
        const SparseQRLinearSolverOptions& options) {
      _options = options;
    }

    const CompressedColumnMatrix<SuiteSparse_long>&
        SparseQrLinearSystemSolver::getJacobianTranspose() const {
      return _jacobianBuilder.J_transpose();
    }

    SuiteSparse_long SparseQrLinearSystemSolver::getRank() const {
      SM_ASSERT_FALSE(Exception, _factor == NULL,
        "QR decomposition has not run yet");
      return _factor->rank;
    }

    double SparseQrLinearSystemSolver::getTol() const {
      SM_ASSERT_FALSE(Exception, _factor == NULL,
        "QR decomposition has not run yet");
      return _factor->tol;
    }

    std::vector<SuiteSparse_long>
        SparseQrLinearSystemSolver::getPermutationVector() const {
      SM_ASSERT_FALSE(Exception, _factor == NULL,
        "QR decomposition has not run yet");
      return std::vector<SuiteSparse_long>(_factor->Q1fill,
        _factor->Q1fill + _cholmodLhs.nrow);
    }

      Eigen::Matrix<SparseQrLinearSystemSolver::index_t, Eigen::Dynamic, 1> SparseQrLinearSystemSolver::getPermutationVectorEigen() const
      {
          SM_ASSERT_FALSE(Exception, _factor == NULL,
                          "QR decomposition has not run yet");
          Eigen::Map< Eigen::Matrix<SparseQrLinearSystemSolver::index_t, Eigen::Dynamic, 1> > pv( _factor->Q1fill, _cholmodLhs.nrow );
          return pv;
      }

    const CompressedColumnMatrix<SuiteSparse_long>&
        SparseQrLinearSystemSolver::getR() {
      if (_R.nnz() == 0) {
        CompressedColumnMatrix<SuiteSparse_long>& J_transpose =
          _jacobianBuilder.J_transpose();
        J_transpose.getView(&_cholmodLhs);
        cholmod_sparse* R;
        _cholmod.getR(&_cholmodLhs, &R);
        _R.fromCholmodSparse(R);
        _cholmod.free(R);
      }
      return _R;
    }

    size_t SparseQrLinearSystemSolver::getMemoryUsage() const {
      return _cholmod.getMemoryUsage();
    }

    void SparseQrLinearSystemSolver::analyzeSystem() {
      CompressedColumnMatrix<SuiteSparse_long>& J_transpose =
        _jacobianBuilder.J_transpose();
      J_transpose.getView(&_cholmodLhs);
      if (_factor == NULL)
        _factor = _cholmod.analyzeQR(&_cholmodLhs);
      SM_ASSERT_TRUE(Exception, _cholmod.factorize(&_cholmodLhs, _factor,
        _options.qrTol, true), "QR decomposition failed");
    }

    double SparseQrLinearSystemSolver::rhsJtJrhs() {
        CompressedColumnMatrix<SuiteSparse_long>& J_transpose = _jacobianBuilder.J_transpose();
        Eigen::VectorXd Jrhs;
        J_transpose.leftMultiply(_rhs, Jrhs);
        return Jrhs.squaredNorm();
    }
      
      
      
  } // namespace backend
} // namespace aslam
