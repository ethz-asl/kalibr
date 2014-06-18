#include <aslam/backend/SparseCholeskyLinearSystemSolver.hpp>
#include <sm/PropertyTree.hpp>

namespace aslam {
  namespace backend {
    SparseCholeskyLinearSystemSolver::SparseCholeskyLinearSystemSolver(const SparseCholeskyLinearSolverOptions& options) : _factor(NULL), _options(options) {}
  SparseCholeskyLinearSystemSolver::SparseCholeskyLinearSystemSolver(const sm::PropertyTree& /* config */) :
        _factor(NULL) {
      // NO OPTIONS CURRENTLY IMPLEMENTED
      // USING C++11 would allow to do constructor delegation and more elegant code
    }
    SparseCholeskyLinearSystemSolver::~SparseCholeskyLinearSystemSolver() {}

    void SparseCholeskyLinearSystemSolver::initMatrixStructureImplementation(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner)
    {
      _errorTerms = errors;
      if (_factor) {
        _cholmod.free(_factor);
        _factor = NULL;
      }
      // std::cout << "init structure\n";
      _useDiagonalConditioner = useDiagonalConditioner;
      _jacobianBuilder.initMatrixStructure(dvs, errors);
      CompressedColumnMatrix<int>& J_transpose = _jacobianBuilder.J_transpose();
      if (_useDiagonalConditioner) {
        J_transpose.pushConstantDiagonalBlock(1.0);
      }
      // View this matrix as a sparse matrix.
      // These views should remain valid for the lifetime of the object.
      J_transpose.getView(&_cholmodLhs);
      _cholmod.view(_rhs, &_cholmodRhs);
      if (_useDiagonalConditioner) {
        J_transpose.popDiagonalBlock();
      }
      // We can't to the factorization as the function requires numerical values.
    }


    void SparseCholeskyLinearSystemSolver::buildSystem(size_t nThreads, bool useMEstimator)
    {
      //std::cout << "build system\n";
      _jacobianBuilder.buildSystem(nThreads, useMEstimator);
      CompressedColumnMatrix<int>& J_transpose = _jacobianBuilder.J_transpose();
      J_transpose.rightMultiply(_e, _rhs);
      // std::cout << "build system complete\n";
    }

    bool SparseCholeskyLinearSystemSolver::solveSystem(Eigen::VectorXd& outDx)
    {
      CompressedColumnMatrix<int>& J_transpose = _jacobianBuilder.J_transpose();
      if (_useDiagonalConditioner) {
        J_transpose.pushDiagonalBlock(_diagonalConditioner);
      }
      J_transpose.getView(&_cholmodLhs);
      _cholmod.view(_rhs, &_cholmodRhs);
      // std::cout << "solve system\n";
      if (!_factor) {
        // std::cout << "\tAnalyze system\n";
        // Now do the symbolic analysis with cholmod.
        _factor = _cholmod.analyze(&_cholmodLhs);
        //  std::cout << "\tanalyze system complete\n";
      }
      // Now we can solve the system.
      outDx.resize(J_transpose.rows());
      cholmod_dense* sol = _cholmod.solve(&_cholmodLhs, _factor, &_cholmodRhs);
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
      //std::cout << "solve system complete\n";
      return true;
    }

    const SparseCholeskyLinearSolverOptions&
    SparseCholeskyLinearSystemSolver::getOptions() const {
      return _options;
    }

    SparseCholeskyLinearSolverOptions&
    SparseCholeskyLinearSystemSolver::getOptions() {
      return _options;
    }

    void SparseCholeskyLinearSystemSolver::setOptions(
        const SparseCholeskyLinearSolverOptions& options) {
      _options = options;
    }
      
    double SparseCholeskyLinearSystemSolver::rhsJtJrhs() {
        CompressedColumnMatrix<int>& J_transpose = _jacobianBuilder.J_transpose();
        Eigen::VectorXd Jrhs;
        J_transpose.leftMultiply(_rhs, Jrhs);
        return Jrhs.squaredNorm();
    }
      
      

  } // namespace backend
} // namespace aslam
