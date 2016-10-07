#ifndef ASLAM_DENSE_QR_LINEAR_SYSTEM_SOLVER_HPP
#define ASLAM_DENSE_QR_LINEAR_SYSTEM_SOLVER_HPP

#include "LinearSystemSolver.hpp"
#include "DenseMatrix.hpp"

#include "aslam/backend/DenseQRLinearSolverOptions.h"

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class DenseQrLinearSystemSolver : public LinearSystemSolver {
    public:
      DenseQrLinearSystemSolver(const DenseQRLinearSolverOptions& options = DenseQRLinearSolverOptions());
      DenseQrLinearSystemSolver(const sm::PropertyTree& config);
      virtual ~DenseQrLinearSystemSolver();

      /// \brief build the system of equations.
      virtual void buildSystem(size_t nThreads, bool useMEstimator);

      /// \brief solve the system storing the solution in outDx and returning true on success.
      virtual bool solveSystem(Eigen::VectorXd& outDx);

      /// \brief return the Jacobian matrix if available. Null if not available.
      virtual const Matrix* Jacobian() const;
      const Eigen::MatrixXd& getJacobian() const;

      virtual std::string name() const { return "dense_qr";};
      
      /// Returns the options
      const DenseQRLinearSolverOptions& getOptions() const;
      /// Returns the options
      DenseQRLinearSolverOptions& getOptions();
      /// Sets the options
      void setOptions(const DenseQRLinearSolverOptions& options);



      /// Helper Function for DogLeg implementation; returns parts required for the steepest descent solution
      double rhsJtJrhs();
    
    private:
      /// \brief a method for a thread to evaluate Jacobians
      void evaluateJacobians(size_t threadId, size_t startIdx, size_t endIdx, bool useMEstimator);

      /// \brief initialized the matrix structure for the problem with these error terms and errors.
      virtual void initMatrixStructureImplementation(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner);

      /// \brief the dense Jacobian matrix
      DenseMatrix _J;

      Eigen::VectorXd _truncated_e;

      /// Options
      DenseQRLinearSolverOptions _options;

    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_DENSE_QR_LINEAR_SYSTEM_SOLVER_HPP */
