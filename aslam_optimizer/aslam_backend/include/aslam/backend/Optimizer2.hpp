#ifndef ASLAM_BACKEND_OPTIMIZER_2_HPP
#define ASLAM_BACKEND_OPTIMIZER_2_HPP


#include <boost/shared_ptr.hpp>
//#include <boost/function.hpp>
#include <sm/assert_macros.hpp>
#include <Eigen/Core>
#include "Optimizer2Options.hpp"
#include "backend.hpp"
#include "OptimizationProblemBase.hpp"
#include <aslam/Exceptions.hpp>
#include <sm/timing/Timer.hpp>
#include <boost/thread.hpp>
#include <sparse_block_matrix/linear_solver.h>
#include <aslam/backend/TrustRegionPolicy.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>
#include <aslam/backend/DogLegTrustRegionPolicy.hpp>

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {
    class LinearSystemSolver;

    /**
     * \class Optimizer2
     *
     * A sparse Gauss-Newton/LM optimizer.
     *
     * The notation in this file follows Harley and Zisserman, Appendix 6.
     *
     * Some Additions to the standard algorithm:
     * Choice of Lambda:
     * H.B. Nielsen. Damping Parameter in Marquardts Method. Technical Report
     * IMM-REP-1999-05, Technical University of Denmark, 1999.
     *
     * Jacobian Normalisation:
     * Jean-Claude Trigeassou, Thierry Poinot & Sandrine Moreau (2003):
     * A Methodology for Estimation of Physical Parameters, Systems Analysis Modelling Simulation, 43:7, 925-943
     *
     * Erik Etien, Damien Halbert, and Thierry Poinot
     * Improved JilesAtherton Model for Least Square Identification Using Sensitivity Function Normalization
     * IEEE TRANSACTIONS ON MAGNETICS, VOL. 44, NO. 7, JULY 2008
     *
     */
    class Optimizer2 {
    public:
        //  typedef sm::timing::Timer Timer;
      /// Swapping this to the dummy timer will disable timing
       typedef sm::timing::DummyTimer Timer;
      typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> SparseBlockMatrix;

      SM_DEFINE_EXCEPTION(Exception, aslam::Exception);

      Optimizer2(const Optimizer2Options& options = Optimizer2Options());
      Optimizer2(const sm::PropertyTree& config, boost::shared_ptr<LinearSystemSolver> linearSystemSolver, boost::shared_ptr<TrustRegionPolicy> trustRegionPolicy);
      virtual ~Optimizer2();

      /// \brief Set up to work on the optimization problem.
      void setProblem(boost::shared_ptr<OptimizationProblemBase> problem);

      /// \brief initialize the optimizer to run on an optimization problem.
      void initialize();

      /// \brief initialize the linear solver specified in the optimizer options.
      void initializeLinearSolver();
      
      void initializeTrustRegionPolicy();

      /// \brief Run the optimization
      SolutionReturnValue optimize();

      /// \brief Get the optimizer options.
      Optimizer2Options& options();

      /// \brief return the reduced system dx
      const Eigen::VectorXd& dx() const;

      /// The value of the objective function.
      double J() const;

      /// \brief compute the full covariance matrix. This is expensive.
      void computeCovariances(SparseBlockMatrix& outP, double lambda);

      /// \brief compute only the diagonal covariance blocks.
      void computeDiagonalCovariances(SparseBlockMatrix& outP, double lambda);

      /// \brief compute only the covariance blocks associated with the block indices passed as an argument
      void computeCovarianceBlocks(const std::vector<std::pair<int, int> >& blockIndices, SparseBlockMatrix& outP, double lambda);

      void computeHessian(SparseBlockMatrix& outH, double lambda);

      /// \brief Evaluate the error at the current state.
      double evaluateError(bool useMEstimator);

      /// \brief Get dense design variable i.
      DesignVariable* designVariable(size_t i);

      /// \brief how many dense design variables are involved in the problem
      size_t numDesignVariables() const;

      /// \brief print the internal timing information.
      void printTiming() const;

      /// \brief Do a bunch of checks to see if the problem is well-defined. This includes checking that every error term is
      ///        hooked up to design variables and running finite differences on error terms where this is possible.
      void checkProblemSetup();

      /// \brief Build the Gauss-Newton matrices.
      void buildGnMatrices(bool useMEstimator);

      /// Returns the linear solver
      template <class L>
      L* getSolver();


        const Matrix * getJacobian() const;
      
        const LinearSystemSolver * getBaseSolver() const;

    private:

      /// \brief Zero the Gauss-Newton matrices.
      void zeroMatrices();

      /// \brief Revert the last state update.
      void revertLastStateUpdate();

      /// \brief Apply a state update.
      double applyStateUpdate();

      /// \brief The dense update vector.
      Eigen::VectorXd _dx;

      /// \brief The current value of the cost function.
      double _J;

      /// \brief The previous value of the cost function.
      double _p_J;

      boost::shared_ptr<LinearSystemSolver> _solver;

      boost::shared_ptr<TrustRegionPolicy> _trustRegionPolicy;

      /// \brief The current optimization problem.
      boost::shared_ptr<OptimizationProblemBase> _problem;

      /// \brief all design variables...first the non-marginalized ones (the dense ones), then the marginalized ones.
      std::vector<DesignVariable*> _designVariables;

      /// \brief all of the error terms involved in this problem
      std::vector<ErrorTerm*> _errorTerms;

      /// \brief the current set of options
      Optimizer2Options _options;

    };

  } // namespace backend
} // namespace aslam

#include "aslam/backend/Optimizer2.tpp"

#endif /* ASLAM_BACKEND_OPTIMIZER_HPP */
