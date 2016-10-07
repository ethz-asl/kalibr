#ifndef ASLAM_BACKEND_SPARSE_QR_LINEAR_SYSTEM_SOLVER_HPP
#define ASLAM_BACKEND_SPARSE_QR_LINEAR_SYSTEM_SOLVER_HPP

#include "LinearSystemSolver.hpp"
#include "CompressedColumnJacobianTransposeBuilder.hpp"

#include "aslam/backend/SparseQRLinearSolverOptions.h"

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class SparseQrLinearSystemSolver : public LinearSystemSolver {
    public:
      typedef SuiteSparse_long index_t;

      SparseQrLinearSystemSolver(const SparseQRLinearSolverOptions& options = SparseQRLinearSolverOptions());
      SparseQrLinearSystemSolver(const sm::PropertyTree& config);
      virtual ~SparseQrLinearSystemSolver();

      // virtual void evaluateError(size_t nThreads, bool useMEstimator);
      virtual void buildSystem(size_t nThreads, bool useMEstimator);
      virtual bool solveSystem(Eigen::VectorXd& outDx);
      // virtual void solveConstantAugmentedSystem(double diagonalConditioner, Eigen::VectorXd & outDx);
      // virtual void solveAugmentedSystem(const Eigen::VectorXd & diagonalConditioner, Eigen::VectorXd & outDx);

      virtual std::string name() const { return "sparse_qr"; }

      /// Returns the current Jacobian transpose
      const CompressedColumnMatrix<index_t>& getJacobianTranspose() const;
      /// Returns the current estimated numerical rank
      index_t getRank() const;
      /// Returns the current tolerance
      double getTol() const;
      /// Returns the current permutation vector
      std::vector<index_t> getPermutationVector() const;
      /// Returns the current permutation vector
      Eigen::Matrix<index_t, Eigen::Dynamic, 1> getPermutationVectorEigen() const;
      /// Performs QR decomposition and returns the R matrix
      const CompressedColumnMatrix<index_t>& getR();
      /// Returns the current memory usage in bytes
      size_t getMemoryUsage() const;
      /// Performs symbolic and numeric analysis
      void analyzeSystem();

      /// Returns the options
      const SparseQRLinearSolverOptions& getOptions() const;
      /// Returns the options
      SparseQRLinearSolverOptions& getOptions();
      /// Sets the options
      void setOptions(const SparseQRLinearSolverOptions& options);
        
      /// Helper Function for DogLeg implementation; returns parts required for the steepest descent solution
      double rhsJtJrhs();

    private:
      virtual void initMatrixStructureImplementation(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner);

      CompressedColumnJacobianTransposeBuilder<index_t> _jacobianBuilder;

      Cholmod<index_t> _cholmod;
      cholmod_sparse _cholmodLhs;
      cholmod_dense  _cholmodRhs;
#ifndef QRSOLVER_DISABLED
      SuiteSparseQR_factorization<double>* _factor;
      CompressedColumnMatrix<index_t> _R;
#endif
      SparseQRLinearSolverOptions _options;
    };

  } // namespace backend
} // namespace aslam
#endif /* ASLAM_BACKEND_SPARSE_QR_LINEAR_SYSTEM_SOLVER_HPP */
