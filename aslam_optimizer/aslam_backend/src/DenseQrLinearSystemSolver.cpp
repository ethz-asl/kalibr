#include <aslam/backend/DenseQrLinearSystemSolver.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <Eigen/Dense> // householderQr.solve
#include <sm/PropertyTree.hpp>

namespace aslam {
  namespace backend {

    DenseQrLinearSystemSolver::DenseQrLinearSystemSolver(const DenseQRLinearSolverOptions& options) :
        _options(options) {
    }

  DenseQrLinearSystemSolver::DenseQrLinearSystemSolver(const sm::PropertyTree& /* config */) {
      // NO OPTIONS CURRENTLY IMPLEMENTED
      // USING C++11 would allow to do constructor delegation and more elegant code
    }

    DenseQrLinearSystemSolver::~DenseQrLinearSystemSolver()
    {
    }

    const Matrix* DenseQrLinearSystemSolver::Jacobian() const
    {
      return &_J;
    }

  void DenseQrLinearSystemSolver::initMatrixStructureImplementation(const std::vector<DesignVariable*>& /* dvs */, const std::vector<ErrorTerm*>& /* errors */, bool /* useDiagonalConditioner */)
    {
      // \todo Verify that this is similar to the "reserve()" feature in a standard vector.
      _J._M.resize(_JRows, _JCols);
    }

    void DenseQrLinearSystemSolver::buildSystem(size_t nThreads, bool useMEstimator)
    {
      _J._M.setZero();
      setupThreadedJob(boost::bind(&DenseQrLinearSystemSolver::evaluateJacobians, this, _1, _2, _3, _4), nThreads, useMEstimator);
      _rhs = _J._M.transpose() * _e;
    }


    bool DenseQrLinearSystemSolver::solveSystem(Eigen::VectorXd& outDx)
    {
      if (_useDiagonalConditioner) {
        // Append the diagonal. Thanks to the ceres developers for this trick.
        _J._M.conservativeResize(_JRows + _JCols, Eigen::NoChange);
        _J._M.bottomRows(_JCols) = _diagonalConditioner.asDiagonal();
        // Append zeros to make the sizes match.
        _e.conservativeResize(_JRows + _JCols);
        _e.tail(_JCols) = Eigen::VectorXd::Zero(_JCols);
      }
      outDx = _J._M.colPivHouseholderQr().solve(_e);
      if (_useDiagonalConditioner) {
        // Remove the diagonal
        _J._M.conservativeResize(_JRows, Eigen::NoChange);
        _e.conservativeResize(_JRows);
      }
      return true;
    }


  void DenseQrLinearSystemSolver::evaluateJacobians(size_t /* threadId */, size_t startIdx, size_t endIdx, bool useMEstimator)
    {
      for (size_t i = startIdx; i < endIdx; ++i) {
        JacobianContainer jc(_errorTerms[i]->dimension());
        ErrorTerm* e = _errorTerms[i];
        e->getWeightedJacobians(jc, useMEstimator);
        JacobianContainer::map_t::iterator it = jc.begin();
        for (; it != jc.end(); ++it) {
          _J._M.block(e->rowBase(), it->first->columnBase(), it->second.rows(), it->second.cols()) = it->second;
        }
      }
    }

    const DenseQRLinearSolverOptions&
    DenseQrLinearSystemSolver::getOptions() const {
      return _options;
    }

    DenseQRLinearSolverOptions&
    DenseQrLinearSystemSolver::getOptions() {
      return _options;
    }

    void DenseQrLinearSystemSolver::setOptions(
        const DenseQRLinearSolverOptions& options) {
      _options = options;
    }
      
      
    double DenseQrLinearSystemSolver::rhsJtJrhs() {
        Eigen::VectorXd Jrhs;
        _J.rightMultiply(_rhs, Jrhs);
        return Jrhs.squaredNorm();
    }
      
      

    const Eigen::MatrixXd& DenseQrLinearSystemSolver::getJacobian() const
    {
     return _J._M;
    }

  } // namespace backend
} // namespace aslam
