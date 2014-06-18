#ifndef ASLAM_BACKEND_OPTIMIZER_OPTIONS_HPP
#define ASLAM_BACKEND_OPTIMIZER_OPTIONS_HPP

namespace aslam {
  namespace backend {

    struct OptimizerOptions {
      OptimizerOptions() :
        convergenceDeltaJ(1e-3),
        convergenceDeltaX(1e-3),
        maxIterations(20),
        levenbergMarquardtLambdaInit(1e-3),
        levenbergMarquardtLambdaGamma(3),
        levenbergMarquardtLambdaBeta(2),
        levenbergMarquardtLambdaP(3),
        levenbergMarquardtLambdaMuInit(2),
        levenbergMarquardtEstimateLambdaScale(-1),
        doLevenbergMarquardt(true),
        doSchurComplement(false),
        verbose(false),
        resetSolverEveryIteration(false),
        linearSolverMaximumFails(0),
        linearSolver("cholmod")


      {};

      /// \brief stop when steps cause changes in the objective function below this threshold.
      double convergenceDeltaJ;

      /// \brief stop when the maximum change in and component of a design variable drops below this threshold
      double convergenceDeltaX;

      /// \brief stop if we reach this number of iterations without hitting any of the above stopping criteria.
      int maxIterations;

      /// \brief what value of lambda (for Levenberg-Marquardt) should be used when initializing the optimization
      double levenbergMarquardtLambdaInit;

      /// \brief the parameters required for levenberg marquard Lambda updates:
      double levenbergMarquardtLambdaGamma;
      int levenbergMarquardtLambdaBeta;     // odd!
      int levenbergMarquardtLambdaP;        // odd!
      double levenbergMarquardtLambdaMuInit;

      /// \brief negative values indicate that the LamdaInit inital value should be used
      double levenbergMarquardtEstimateLambdaScale;

      /// \brief should we use Levenberg-Marquardt? Otherwise use plain Gauss-Newton.
      bool doLevenbergMarquardt;

      /// \brief should we use the Schur complement trick? Currently not supported.
      bool doSchurComplement;

      /// \brief should we print out some information each iteration?
      bool verbose;

      /// \brief Should we reset the linear solver every iteration? This is currently necessary for any problem where error terms have the possibility of changing design variables.
      bool resetSolverEveryIteration;

      /// \brief The number of times the linear solver may fail before the optimisation is aborted. (>0 only if a fallback is available!)
      int linearSolverMaximumFails;

      /// \brief which linear solver should we use. Options are currently "block_cholesky", "sparse_cholesky", "sparse_qr".
      std::string linearSolver;
    };


  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_OPTIMIZER_OPTIONS_HPP */
