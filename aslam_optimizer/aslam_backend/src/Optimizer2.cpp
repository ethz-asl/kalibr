#include <aslam/backend/Optimizer2.hpp>
// std::partial_sum
#include <numeric>
#include <aslam/backend/ErrorTerm.hpp>
// M.inverse()
#include <Eigen/Dense>
#include <sm/eigen/assert_macros.hpp>
#include <sparse_block_matrix/linear_solver_dense.h>
#include <sparse_block_matrix/linear_solver_cholmod.h>
#ifndef QRSOLVER_DISABLED
#include <sparse_block_matrix/linear_solver_spqr.h>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#endif
#include <aslam/backend/sparse_matrix_functions.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/SparseCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/DenseQrLinearSystemSolver.hpp>
#include <sm/PropertyTree.hpp>


namespace aslam {
    namespace backend {


        Optimizer2::Optimizer2(const Optimizer2Options& options) :
            _options(options)
        {
            initializeLinearSolver();
            initializeTrustRegionPolicy();
        }

        Optimizer2::Optimizer2(const sm::PropertyTree& config, boost::shared_ptr<LinearSystemSolver> linearSystemSolver, boost::shared_ptr<TrustRegionPolicy> trustRegionPolicy) {
          Optimizer2Options options;
          options.convergenceDeltaJ = config.getDouble("convergenceDeltaJ", options.convergenceDeltaJ);
          options.convergenceDeltaX = config.getDouble("convergenceDeltaX", options.convergenceDeltaX);
          options.maxIterations = config.getInt("maxIterations", options.maxIterations);
          options.doSchurComplement = config.getBool("doSchurComplement", options.doSchurComplement);
          options.verbose = config.getBool("verbose", options.verbose);
          options.linearSolverMaximumFails = config.getInt("linearSolverMaximumFails", options.linearSolverMaximumFails);
          options.nThreads = config.getInt("nThreads", options.nThreads);
          options.linearSystemSolver = linearSystemSolver;
          options.trustRegionPolicy = trustRegionPolicy;
          _options = options;
          initializeLinearSolver();
          initializeTrustRegionPolicy();
          // USING C++11 would allow to do constructor delegation and more elegant code, i.e., directly call the upper constructor
        }

        Optimizer2::~Optimizer2()
        {
        }


        /// \brief Set up to work on the optimization problem.
        void Optimizer2::setProblem(boost::shared_ptr<OptimizationProblemBase> problem)
        {
            _problem = problem;
        }

        void Optimizer2::initializeTrustRegionPolicy()
        {
          if( !_options.trustRegionPolicy ) {
            _options.verbose && std::cout << "No trust region policy set in the options. Defaulting to levenberg_marquardt\n";
            _trustRegionPolicy.reset( new LevenbergMarquardtTrustRegionPolicy() );
          } else {
            _trustRegionPolicy = _options.trustRegionPolicy;
          }


          // \todo remove this check when the sparse qr solver supports an augmented diagonal
          if(_solver->name() == "sparse_qr" && _trustRegionPolicy->name() == "levenberg_marquardt") {
            _options.verbose && std::cout << "The sparse_qr solver is not compatible with levenberg_marquardt. Changing to the dog_leg trust region policy\n";
            _trustRegionPolicy.reset( new DogLegTrustRegionPolicy() );
          }

          _options.verbose && std::cout << "Using the " << _trustRegionPolicy->name() << " trust region policy\n";

        }


        void Optimizer2::initializeLinearSolver()
        {
          if( ! _options.linearSystemSolver ) {
            _options.verbose && std::cout << "No linear system solver set in the options. Defaulting to the sparse_cholesky solver\n";
            _solver.reset(new SparseCholeskyLinearSystemSolver());
          } else {
            _solver = _options.linearSystemSolver;
          }

          _options.verbose && std::cout << "Using the " << _solver->name() << " linear system solver\n";
        }

        /// \brief initialize the optimizer to run on an optimization problem.
        ///        This should be called before calling optimize()
        void Optimizer2::initialize()
        {
          initializeLinearSolver();
          initializeTrustRegionPolicy();

          SM_ASSERT_FALSE(Exception, _problem.get() == NULL, "No optimization problem has been set");
            _options.verbose && std::cout << "Initializing\n";
            Timer init("Optimizer2: Initialize Total");
            _designVariables.clear();
            _designVariables.reserve(_problem->numDesignVariables());
            _errorTerms.clear();
            _errorTerms.reserve(_problem->numErrorTerms());
            Timer initDv("Optimizer2: Initialize---Design Variables");
            // Run through all design variables adding active ones to an active list.
            // std::cout << "dvloop 1\n";
            for (size_t i = 0; i < _problem->numDesignVariables(); ++i) {
                DesignVariable* dv = _problem->designVariable(i);
                if (dv->isActive())
                    _designVariables.push_back(dv);
            }
            SM_ASSERT_FALSE(Exception, _designVariables.empty(), "It is illegal to run the optimizer with all marginalized design variables.");
            // Assign block indices to the design variables.
            // "blocks" will hold the structure of the left-hand-side of Gauss-Newton
            int columnBase = 0;
            // std::cout << "dvloop 2\n";
            for (size_t i = 0; i < _designVariables.size(); ++i) {
                _designVariables[i]->setBlockIndex(i);
                _designVariables[i]->setColumnBase(columnBase);
                columnBase += _designVariables[i]->minimalDimensions();
            }
            initDv.stop();
            Timer initEt("Optimizer2: Initialize---Error Terms");
            // Get all of the error terms that work on these design variables.
            int dim = 0;
            // std::cout << "eloop 1\n";
            for (unsigned i = 0; i < _problem->numErrorTerms(); ++i) {
                ErrorTerm* e = _problem->errorTerm(i);
                _errorTerms.push_back(e);
                e->setRowBase(dim);
                dim += e->dimension();
            }
            initEt.stop();
            SM_ASSERT_FALSE(Exception, _errorTerms.empty(), "It is illegal to run the optimizer with no error terms.");
            Timer initMx("Optimizer2: Initialize---Matrices");
            // Set up the block matrix structure.
            // std::cout << "init structure\n";
            //      initializeLinearSolver();
            _solver->initMatrixStructure(_designVariables, _errorTerms, _trustRegionPolicy->requiresAugmentedDiagonal());
            initMx.stop();
            _options.verbose && std::cout << "Optimization problem initialized with " << _designVariables.size() << " design variables and " << _errorTerms.size() << " error terms\n";
            // \todo Say how big the problem is.
            _options.verbose && std::cout << "The Jacobian matrix is " << dim << " x " << columnBase << std::endl;


            // \todo initialize the trust region stuff.

        }


        /*
        // returns true of stop!
        bool Optimizer2::evaluateStoppingCriterion(int iterations)
        {

        // as we have analytic Jacobians we can assume the precision to be:
        double epsilon = std::numeric_limits<double>::epsilon();

        double x_norm = ...;

        // the gradient: is simply the right hand side of GN:
        double grad_norm = _rhs.norm();
        double abs_J = fabs(_J);

        // the first condition:
        bool crit1 = grad_norm < sqrt(epsilon) * (1 + abs_J);

        bool crit2 = _dx.norm() < sqrt(epsilon) * (1 + x_norm);

        bool crit3 = fabs(_J - _p_J) < epsilon * (1 + abs_J);

        bool crit4 = iterations < _options.maxIterations;

        return (crit1 && crit2 && crit3) || crit4;

        }*/



        SolutionReturnValue Optimizer2::optimize()
        {
            Timer timeGn("Optimizer2: build Hessian", true);
            Timer timeErr("Optimizer2: evaluate error", true);
            Timer timeSchur("Optimizer2: Schur complement", true);
            Timer timeBackSub("Optimizer2: Back substitution", true);
            Timer timeSolve("Optimizer2: Solve linear system", true);
            // Select the design variables and (eventually) the error terms involved in the optimization.
            initialize();
            SolutionReturnValue srv;
            _p_J = 0.0;

            //std::cout << "Evaluate error for the first time\n";
            // This sets _J
            timeErr.start();
            evaluateError(true);
            timeErr.stop();
            _p_J = _J;
            srv.JStart = _p_J;
            // *** while not done
            _options.verbose && std::cout << "[" << srv.iterations << ".0]: J: " << _J << std::endl;
            // Set up the estimation problem.
            double deltaX = _options.convergenceDeltaX + 1.0;
            double deltaJ = _options.convergenceDeltaJ + 1.0;
            bool previousIterationFailed = false;
            bool linearSolverFailure = false;

            SM_ASSERT_TRUE(Exception, _solver.get() != NULL, "The solver is null");
            _trustRegionPolicy->setSolver(_solver);
            _trustRegionPolicy->optimizationStarting(_J);

            // Loop until convergence
            while (srv.iterations <  _options.maxIterations &&
                   srv.failedIterations < _options.maxIterations &&
                   ((deltaX > _options.convergenceDeltaX &&
                     fabs(deltaJ) > _options.convergenceDeltaJ) ||
                    linearSolverFailure)) {

                timeSolve.start();
                bool solutionSuccess = _trustRegionPolicy->solveSystem(_J, previousIterationFailed, _options.nThreads, _dx);
                timeSolve.stop();

                if (!solutionSuccess) {
                    _options.verbose && std::cout << "[WARNING] System solution failed\n";
                    previousIterationFailed = true;
                    linearSolverFailure = true;
                    srv.failedIterations++;
                } else {
                    /// Apply the state update. _A, _b, _dx, and _H are passed in implicitly.
                    timeBackSub.start();
                    deltaX = applyStateUpdate();
                    timeBackSub.stop();
                    // This sets _J
                    timeErr.start();
                    evaluateError(true);
                    timeErr.stop();
                    deltaJ = _p_J - _J;
                    // This was a regression.
                    if( _trustRegionPolicy->revertOnFailure() )
                    {
                        if(deltaJ < 0.0)
                        {
                            _options.verbose && std::cout << "Last step was a regression. Reverting\n";
                            revertLastStateUpdate();
                            srv.failedIterations++;
                            previousIterationFailed = true;
                        }
                        else
                        {
                            _p_J = _J;
                            previousIterationFailed = false;
                        }
                    }
                    else
                    {
                        _p_J = _J;
                    }
                    srv.iterations++;

                    _options.verbose && std::cout << "[" << srv.iterations << "]: J: " << _J << ", dJ: " << deltaJ << ", deltaX: " << deltaX << ", ";
                    _options.verbose && _trustRegionPolicy->printState(std::cout);
                    _options.verbose && std::cout << std::endl;
                } // if the linear solver failed / else
            }

            srv.JFinal = _p_J;
            srv.dXFinal = deltaX;
            srv.dJFinal = deltaJ;
            srv.linearSolverFailure = linearSolverFailure;
            return srv;
        }


            DesignVariable* Optimizer2::designVariable(size_t i)
            {
                SM_ASSERT_LT_DBG(Exception, i, _designVariables.size(), "index out of bounds");
                return _designVariables[i];
            }



            size_t Optimizer2::numDesignVariables() const
            {
                return _designVariables.size();
            }


            double Optimizer2::applyStateUpdate()
            {
                // Apply the update to the dense state.
                int startIdx = 0;
                for (size_t i = 0; i < numDesignVariables(); i++) {
                    DesignVariable* d = _designVariables[i];
                    const int dbd = d->minimalDimensions();
                    Eigen::VectorXd dxS = _dx.segment(startIdx, dbd);
                    dxS *= d->scaling();
                    if(dbd > 0)
                        d->update(&dxS[0], dbd);
                    startIdx += dbd;
                }
                // Track the maximum delta
                // \todo: should this be some other metric?
                double deltaX = _dx.array().abs().maxCoeff();
                return deltaX;
            }





            void Optimizer2::revertLastStateUpdate()
            {
                for (size_t i = 0; i < _designVariables.size(); i++) {
                    _designVariables[i]->revertUpdate();
                }
            }


            Optimizer2Options& Optimizer2::options()
            {
                return _options;
            }


            double Optimizer2::evaluateError(bool useMEstimator)
            {
                SM_ASSERT_TRUE(Exception, _solver.get() != NULL, "The solver is null");
                _J = _solver->evaluateError(_options.nThreads, useMEstimator);
                return _J;
            }


            /// \brief return the reduced system dx
            const Eigen::VectorXd& Optimizer2::dx() const
            {
                return _dx;
            }

            /// The value of the objective function.
            double Optimizer2::J() const
            {
                return _J;
            }

            void Optimizer2::printTiming() const
            {
                sm::timing::Timing::print(std::cout);
            }







            void Optimizer2::checkProblemSetup()
            {
                // Check that all error terms are hooked up to design variables.
            }



            void Optimizer2::computeDiagonalCovariances(SparseBlockMatrix& outP, double lambda)
            {
                SM_THROW(Exception, "Broken");

                std::vector<std::pair<int, int> > blockIndices;
                for (size_t i = 0; i < _designVariables.size(); ++i) {
                    blockIndices.push_back(std::make_pair(i, i));
                }
                computeCovarianceBlocks(blockIndices, outP, lambda);
            }

    void Optimizer2::computeCovarianceBlocks(const std::vector<std::pair<int, int> > & /* blockIndices */, SparseBlockMatrix& /* outP */, double /* lambda */)
            {
                SM_THROW(Exception, "Broken");

            }


    void Optimizer2::computeCovariances(SparseBlockMatrix& /* outP */, double /* lambda */)
            {
                SM_THROW(Exception, "Broken");

            }

        void Optimizer2::computeHessian(SparseBlockMatrix& outH, double lambda)
            {

              boost::shared_ptr<BlockCholeskyLinearSystemSolver> solver_sp;
              solver_sp.reset(new BlockCholeskyLinearSystemSolver());
              // True here for creating the diagonal conditioning.
              solver_sp->initMatrixStructure(_designVariables, _errorTerms, true);

              _options.verbose && std::cout << "Setting the diagonal conditioner to: " << lambda << ".\n";
              evaluateError(false);
              solver_sp->setConstantConditioner(lambda);
              solver_sp->buildSystem(_options.nThreads, false);
              solver_sp->copyHessian(outH);
            }

      const LinearSystemSolver * Optimizer2::getBaseSolver() const {
          return _solver.get();
      }



        const Matrix * Optimizer2::getJacobian() const {
            return _solver->Jacobian();
        }


        } // namespace backend
    } // namespace aslam
