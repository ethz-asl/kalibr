#ifndef ASLAM_BACKEND_OPTIMIZER_HPP
#define ASLAM_BACKEND_OPTIMIZER_HPP


#include <sparse_block_matrix/linear_solver.h>
#include <boost/shared_ptr.hpp>
//#include <boost/function.hpp>
#include <sm/assert_macros.hpp>
#include <Eigen/Core>
#include "OptimizerOptions.hpp"
#include "backend.hpp"
#include "OptimizationProblemBase.hpp"
#include <aslam/Exceptions.hpp>
#include <sm/timing/Timer.hpp>
#include <sm/boost/null_deleter.hpp>
#include <boost/thread.hpp>

namespace aslam {
  namespace backend {
    /**
     * \class Optimizer
     *
     * A sparse Gauss-Newton/LM optimizer.
     *
     * The notation in this file follows Harley and Zisserman, Appendix 6.
     *
     * Some Additions to the standard algorithm:
     * Choice of Lambda:
     * H.B. Nielsen. Damping Parameter in Marquardt�s Method. Technical Report
     * IMM-REP-1999-05, Technical University of Denmark, 1999.
     *
     * Jacobian Normalisation:
     * Jean-Claude Trigeassou, Thierry Poinot & Sandrine Moreau (2003):
     * A Methodology for Estimation of Physical Parameters, Systems Analysis Modelling Simulation, 43:7, 925-943
     *
     * Erik Etien, Damien Halbert, and Thierry Poinot
     * Improved Jiles�Atherton Model for Least Square Identification Using Sensitivity Function Normalization
     * IEEE TRANSACTIONS ON MAGNETICS, VOL. 44, NO. 7, JULY 2008
     *
     */
    class Optimizer {
    public:
        //typedef sm::timing::Timer Timer;
      /// Swapping this to the dummy timer will disable timing
      typedef sm::timing::DummyTimer Timer;

      SM_DEFINE_EXCEPTION(Exception, aslam::Exception);

      typedef sparse_block_matrix::LinearSolver<Eigen::MatrixXd> LinearSolver;

      Optimizer(const OptimizerOptions& options = OptimizerOptions());
      virtual ~Optimizer();

      /// \brief Set up to work on the optimization problem.
      void setProblem(boost::shared_ptr<OptimizationProblemBase> problem);

      /// \brief Set up to work on the optimization problem.
      void setProblem(OptimizationProblemBase * problem, bool optimizerOwnsProblem) { setProblem(optimizerOwnsProblem ? boost::shared_ptr<OptimizationProblemBase>(problem) : boost::shared_ptr<OptimizationProblemBase>(problem, sm::null_deleter())); }

      /// \brief initialize the optimizer to run on an optimization problem.
      void initialize();

      /// \brief initialize the linear solver specified in the optimizer options.
      void initializeLinearSolver();

      /// \brief Run the optimization
      SolutionReturnValue optimize();
#ifndef QRSOLVER_DISABLED
      SolutionReturnValue optimizeDogLeg();
#endif

      /// \brief Get the optimizer options.
      OptimizerOptions& options();

      /// \brief Build the GaussNewton matrices from the optimization problem.
      void buildMatrices();

      /// \brief return the full Hessian
      const SparseBlockMatrix& H() const;

      /// \brief return the full rhs
      const Eigen::VectorXd& rhs() const;
      inline const Eigen::VectorXd& epsilon() const {
        return rhs();
      }

      /// \brief return the reduced system lhs
      const SparseBlockMatrix& A() const;

      /// \brief return the reduced system rhs
      const Eigen::VectorXd& b() const;

      /// \brief return the reduced system dx
      const Eigen::VectorXd& dx() const;


      /// The value of the objective function.
      double J() const;

      /// \brief compute the full covariance matrix. This is expensive.
      void computeCovariances();

      /// \brief compute only the diagonal covariance blocks.
      void computeDiagonalCovariances();

      /// \brief compute only the covariance blocks associated with the block indices passed as an argument
      void computeCovarianceBlocks(const std::vector<std::pair<int, int> >& blockIndices);

      /// \brief get a particular covariance block. If the block has not been computed, this will return NULL.
      const Eigen::MatrixXd* getCovarianceBlock(int blockRow, int blockCol) const;


      /// \brief get the covariance matrix as a sparse matrix. This matrix will only be filled in by what was computed above.
      const SparseBlockMatrix& P() const;

      /// \brief get the covariance matrix as a sparse matrix. This matrix will only be filled in by what was computed above.
      const SparseBlockMatrix& getCovariance() const;

      /// \brief Evaluate the error at the current state.
      double evaluateError();

      /// \brief Get dense design variable i.
      DesignVariable* denseVariable(size_t i);
      /// \brief Get sparse design variable i
      DesignVariable* sparseVariable(size_t i);

      /// \brief how many dense design variables are involved in the problem
      size_t numDenseDesignVariables() const;

      /// \brief how many sparse design variables are involved in the problem
      size_t numSparseDesignVariables() const;

      /// \brief print the internal timing information.
      void printTiming() const;

      /// \brief Do a bunch of checks to see if the problem is well-defined. This includes checking that every error term is
      ///        hooked up to design variables and running finite differences on error terms where this is possible.
      void checkProblemSetup();

      /// \brief Build the Gauss-Newton matrices.
      void buildGnMatrices();

      double applyNormalizedStateUpdate();
      void normalizeGnMatrices();
      void initialiseDesignVariableScales();
      SolutionReturnValue optimizeNormalized();
      const Eigen::VectorXd& getDvScales() const;

    private:

      /// \brief Zero the Gauss-Newton matrices.
      void zeroMatrices();

      /// \brief Revert the last state update.
      void revertLastStateUpdate();

      /// \brief Apply a state update.
      double applyStateUpdate();

      /// \brief Returns Rho for the LM lambda update
      double getLmRho();

      /// \brief Set the initial lambda by looking at the entries of the Hessian matrix.
      void setInitialLambda();

      /// \brief The current value of LM lambda.
      double _lambda;

      /// \brief The full Hessian matrix.
      SparseBlockMatrix _H;

      /// \brief The inverse of the Hessian matrix (the covariance).
      SparseBlockMatrix _invH;

      /// \brief The rhs of Gauss-Newton.
      Eigen::VectorXd _rhs;

      /// \brief The dense update vector.
      Eigen::VectorXd _dx;

      Eigen::VectorXd designVariableScales;

      // These next three items are not necessary as part of the
      // state but we want to use them for debugging/visualization/plotting.
      /// \brief The marginalized dense matrix.
      SparseBlockMatrix _A;

      /// \brief The rhs of the marginalized system of equations.
      Eigen::VectorXd _b;

      /// \brief the inverted blocks of the sparse lhs.
      std::vector<Eigen::MatrixXd> _invVi;

      /// \brief The current value of the cost function.
      double _J;
      /// \brief The previous value of the cost function.
      double _p_J;

      /// \brief The sparse linear solver.
      boost::shared_ptr<LinearSolver> _solver;
      boost::shared_ptr<LinearSolver> _fallbackSolver;

      /// \brief The current optimization problem.
      boost::shared_ptr<OptimizationProblemBase> _problem;

      /// \brief all design variables...first the non-marginalized ones (the dense ones), then the marginalized ones.
      std::vector<DesignVariable*> _designVariables;

      /// \brief all of the error terms involved in this problem
      std::set<ErrorTerm*> _errorTerms;

      /// \brief an index into the _designVariables member that tells where the first marginalized design variable lives.
      int _marginalizedStartingBlock;

      /// \brief the current set of options
      OptimizerOptions _options;
    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_OPTIMIZER_HPP */
