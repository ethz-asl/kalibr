#ifndef ASLAM_BACKEND_LINEAR_SYSTEM_SOLVER_HPP
#define ASLAM_BACKEND_LINEAR_SYSTEM_SOLVER_HPP

#include <vector>
#include <Eigen/Core>
#include <boost/function.hpp>
#include <sm/assert_macros.hpp>

namespace aslam {
  namespace backend {

    class DesignVariable;
    class ErrorTerm;
    class Matrix;

    class LinearSystemSolver {
    public:
      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      LinearSystemSolver();

      virtual ~LinearSystemSolver();

      /// \brief Evaluate the error using nThreads.
      double evaluateError(size_t nThreads, bool useMEstimator);

      /// \brief initialized the matrix structure for the problem with these error terms and errors.
      void initMatrixStructure(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner);

      /// \brief build the system of equations.
      virtual void buildSystem(size_t nThreads, bool useMEstimator) = 0;

      /// \brief Set the diagonal matrix conditioner.
      ///        NOTE: The square of these values will be added to the diagonal of the Hessian matrix
      virtual void setConditioner(const Eigen::VectorXd& diag);

      /// \brief Set a constant diagonal conditioner.
      ///        NOTE: The square of this value will be added to the diagonal of the Hessian matrix
      virtual void setConstantConditioner(double diag);

      /// \brief solve the system storing the solution in outDx and returning true on success.
      virtual bool solveSystem(Eigen::VectorXd& outDx) = 0;

      virtual std::string name() const = 0;

      /// \brief return the right-hand side of the equation system.
      virtual const Eigen::VectorXd& rhs() const;

      /// \brief return the Jacobian matrix if available. Null if not available.
      virtual const Matrix* Jacobian() const {
        return NULL;
      }

      /// \brief return the Hessian matrix if avaliable. Null if not available.
      virtual const Matrix* Hessian() const {
        return NULL;
      }

      /// \brief return the full error vector
      virtual const Eigen::VectorXd& e() const;

      /// \brief the number of rows in the Jacobian matrix
      size_t JRows() const;

      /// \brief the number of columns in the Jacobian matrix
      size_t JCols() const;
      
      // helper function for dog leg implementation / steepest descent solution
      virtual double rhsJtJrhs() = 0;

    protected:
      /// \brief initialized the matrix structure for the problem with these error terms and errors.
      virtual void initMatrixStructureImplementation(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors, bool useDiagonalConditioner) = 0;

      /// \brief Set the row base and column base of the design variables (to tweak the ordering)
      ///        The default implementation doesn't do anything.
      virtual void setOrdering(const std::vector<DesignVariable*>& /* dvs */, const std::vector<ErrorTerm*>& /* errors */ ) { }

      /// \brief a function for one thread to evaluate a set of error terms.
      void evaluateErrors(size_t threadId, size_t startIdx, size_t endIdx, bool useMEstimator);

      /// \brief a function to split a multi-threaded job across all error term indices.
      void setupThreadedJob(boost::function<void(size_t, size_t, size_t, bool)> job, size_t nThreads, bool useMEstimator);

      /// \brief the vector of error terms.
      std::vector<ErrorTerm*> _errorTerms;

      /// \brief The squared error values calculated locally for a single thread.
      std::vector<double> _threadLocalErrors;

      /// \brief the error vector;
      Eigen::VectorXd _e;

      /// \brief the linear system rhs.
      Eigen::VectorXd _rhs;

      /// \brief Should we use a diagonal conditioner
      bool _useDiagonalConditioner;

      /// \brief The diagonal conditioner
      Eigen::VectorXd _diagonalConditioner;

      /// \brief The number of rows in the Jacobian matrix
      size_t _JRows;

      /// \brief The number of columns in the Jacobian matrix
      size_t _JCols;

    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_QR_SOLUTION_HPP */
