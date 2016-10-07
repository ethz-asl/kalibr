#ifndef ASLAM_BACKEND_SPARSE_CHOLESKY_LINEAR_SYSTEM_SOLVER_HPP
#define ASLAM_BACKEND_SPARSE_CHOLESKY_LINEAR_SYSTEM_SOLVER_HPP

#include "LinearSystemSolver.hpp"
#include "CompressedColumnJacobianTransposeBuilder.hpp"

#include "aslam/backend/SparseCholeskyLinearSolverOptions.h"

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class SparseCholeskyLinearSystemSolver : public LinearSystemSolver {
    public:
      SparseCholeskyLinearSystemSolver(const SparseCholeskyLinearSolverOptions& options = SparseCholeskyLinearSolverOptions());
      SparseCholeskyLinearSystemSolver(const sm::PropertyTree& config);
      virtual ~SparseCholeskyLinearSystemSolver();


      virtual void buildSystem(size_t nThreads, bool useMEstimator);
      virtual bool solveSystem(Eigen::VectorXd& outDx);

      /// Returns the options
      const SparseCholeskyLinearSolverOptions& getOptions() const;
      /// Returns the options
      SparseCholeskyLinearSolverOptions& getOptions();
      /// Sets the options
      void setOptions(const SparseCholeskyLinearSolverOptions& options);

      virtual std::string name() const {  return "sparse_cholesky"; };        
      /// Helper Function for DogLeg implementation; returns parts required for the steepest descent solution
      double rhsJtJrhs();
   
    
    private:
      virtual void initMatrixStructureImplementation(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner);

      CompressedColumnJacobianTransposeBuilder<int> _jacobianBuilder;

      Cholmod<> _cholmod;
      cholmod_sparse _cholmodLhs;
      cholmod_dense  _cholmodRhs;
      cholmod_factor* _factor;

      /// Options
      SparseCholeskyLinearSolverOptions _options;

    };

  } // namespace backend
} // namespace aslam
#endif /* ASLAM_BACKEND_SPARSE_CHOLESKY_LINEAR_SYSTEM_SOLVER_HPP */
