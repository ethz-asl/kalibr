#include <aslam/backend/Optimizer.hpp>
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
#endif

#include <aslam/backend/sparse_matrix_functions.hpp>

namespace aslam {
  namespace backend {


    Optimizer::Optimizer(const OptimizerOptions& options) :
      _options(options)
    {
      initializeLinearSolver();
    }


    Optimizer::~Optimizer()
    {
    }


    /// \brief Set up to work on the optimization problem.
    void Optimizer::setProblem(boost::shared_ptr<OptimizationProblemBase> problem)
    {
      _problem = problem;
    }


    void Optimizer::initializeLinearSolver()
    {
      // \todo Add the remaining sparse_block_matrix solvers here.
      if (_options.linearSolver == "cholmod") {
        _options.verbose && std::cout << "Using the cholmod linear solver.\n";
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      } else if (_options.linearSolver == "dense") {
        _options.verbose && std::cout << "Using the dense linear solver.\n";
        _solver.reset(new sparse_block_matrix::LinearSolverDense<Eigen::MatrixXd>());
      }
#ifndef QRSOLVER_DISABLED
      else if (_options.linearSolver == "spqr") {
        _options.verbose && std::cout << "Using the SPQR linear solver.\n";
        _solver.reset(new sparse_block_matrix::LinearSolverQr<Eigen::MatrixXd>());
      }
#endif
      else {
        _options.verbose && std::cout << "Unknown linear solver specified: " << _options.linearSolver << ". Using cholmod.\n";
        _solver.reset(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      }
    }


    /// \brief initialize the optimizer to run on an optimization problem.
    ///        This should be called before calling optimize()
    void Optimizer::initialize()
    {
      Timer init("Optimizer: Initialize Total");
      Timer initDv("Optimizer: Initialize---Design Variables");
      _designVariables.clear();
      std::vector<DesignVariable*> marginalizedDesignVariables;
      // Run through all design variables adding active ones to an active list.
      for (size_t i = 0; i < _problem->numDesignVariables(); ++i) {
        DesignVariable* dv = _problem->designVariable(i);
        if (dv->isActive()) {
          if (dv->isMarginalized() && _options.doSchurComplement) {
            marginalizedDesignVariables.push_back(dv);
          } else {
            _designVariables.push_back(dv);
          }
        }
      }
      SM_ASSERT_FALSE(Exception, _designVariables.empty(), "It is illegal to run the optimizer with all marginalized design variables.");
      // Assign block indices to the design variables.
      // "blocks" will hold the structure of the left-hand-side of Gauss-Newton
      std::vector<int> blocks;
      for (size_t i = 0; i < _designVariables.size(); ++i) {
        _designVariables[i]->setBlockIndex(i);
        blocks.push_back(_designVariables[i]->minimalDimensions());
      }
      _marginalizedStartingBlock = _designVariables.size();
      for (size_t i = 0; i < marginalizedDesignVariables.size(); ++i) {
        _designVariables.push_back(marginalizedDesignVariables[i]);
        marginalizedDesignVariables[i]->setBlockIndex(i + _marginalizedStartingBlock);
        blocks.push_back(marginalizedDesignVariables[i]->minimalDimensions());
      }
      initDv.stop();
      Timer initEt("Optimizer: Initialize---Error Terms");
      // Get all of the error terms that work on these design variables.
      _errorTerms.clear();
      for (size_t i = 0; i < _designVariables.size(); ++i) {
        _problem->getErrors(_designVariables[i], _errorTerms);
      }
      initEt.stop();
      Timer initMx("Optimizer: Initialize---Matrices");
      // Set up the block matrix structure.
      // Surrently "blocks" just has the sizes of the minimal dimensions of each block.
      // the sparse matrix initialization requires the cumulative sum of this.
      std::partial_sum(blocks.begin(), blocks.end(), blocks.begin());
      // Now we can initialized the sparse Hessian matrix.
      _H = SparseBlockMatrix(blocks, blocks);
      //_invH = SparseBlockMatrix(blocks, blocks);
      _rhs.resize(_H.rows());
      // And the marginalized blocks. These are not strictly necessary
      // but they will help with debugging.
      blocks.resize(_marginalizedStartingBlock);
      _A = SparseBlockMatrix(blocks, blocks);
      _b.resize(_A.rows());
      _invVi.resize(numSparseDesignVariables());
      _dx.resize(_H.rowBaseOfBlock(_marginalizedStartingBlock));
      initMx.stop();
      _options.verbose && std::cout << "Optimization problem initialized with " << _designVariables.size() << " design variables and " << _errorTerms.size() << " error terms\n";
      _options.verbose && std::cout << "The dense part of the state is " << _H.rowBaseOfBlock(_marginalizedStartingBlock) << " parameters and the sparse part is " << _H.cols() - _H.rowBaseOfBlock(_marginalizedStartingBlock) << " parameters\n";
    }



    double Optimizer::getLmRho()
    {
      double d1 = _p_J - _J;    // update cost delta
      // L(0) - L(h):
      double d2 = _dx.transpose() * (_lambda * _dx + _b);
      return d1 / d2;
    }


    void Optimizer::setInitialLambda()
    {
      // check if _lambda already set:
      if (_lambda != 0) // if set return
        return;
      if (_options.levenbergMarquardtEstimateLambdaScale <= 0) {
        _lambda = _options.levenbergMarquardtLambdaInit;
      } else {
        double maximum = 0;
        // find the maximum diagonal element in H ans scale by LambdaScale:
        // loop the diagonal blocks:
        for (int i = 0; i < _H.bRows(); i++) {
          const Eigen::MatrixXd* H = _H.block(i, i, false); // do no allocate if missing!
          if (H) {
            for (int j = 0; j < H->rows(); j++) {
              if (fabs((*H)(j, j)) > maximum)
                maximum = fabs((*H)(i, j));
            }
          }
        }
        _lambda = maximum * _options.levenbergMarquardtEstimateLambdaScale;
        _options.verbose && std::cout << "Initialised Lambda: " << _lambda << std::endl;
      }
    }

    /*
    // returns true of stop!
    bool Optimizer::evaluateStoppingCriterion(int iterations)
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




    
          SolutionReturnValue Optimizer::optimizeDogLeg()
          {
            Timer timeGn("Optimizer: build Hessian", true);
            Timer timeErr("Optimizer: evaluate error", true);
            Timer timeSchur("Optimizer: Schur complement", true);
            Timer timeBackSub("Optimizer: Back substitution", true);
            Timer timeSolve("Optimizer: Solve linear system", true);
            Timer timeSD("Optimizer: Solve Steepest Descent", true);


              // Select the design variables and (eventually) the error terms involved in the optimization.
              initialize();

              SolutionReturnValue srv;
              _p_J = 0.0;

              // This sets _J
              evaluateError();

              _p_J = _J;
              srv.JStart = _p_J;

              // *** while not done
              _options.verbose && std::cout << "[" << srv.iterations << ".0]: J: " << _J << std::endl;

              // Set up the estimation problem.
              double deltaX = _options.convergenceDeltaX + 1.0;
              double deltaJ = _options.convergenceDeltaJ + 1.0;

              // choose initial delta to be something between GN and SD but biased towards GN
              // would be ideal but to avoid computing the GN solution in the first step before
              // looping we take the SD norm.
              double _delta = 0;
              double _p_delta = 0;
              // build Gauss newton matrices:
              timeGn.start();
              buildGnMatrices();
              timeGn.stop();

              Eigen::VectorXd _dx_sd(_H.rowBaseOfBlock(_marginalizedStartingBlock));
              Eigen::VectorXd _dx_gn(_H.rowBaseOfBlock(_marginalizedStartingBlock));

              double _dx_sd_norm = 0;
              double _dx_gn_norm = 0;
          double rho = 0;
          double L0 = 0;
          double _sd_scale = 0;
          double beta = 0;

          std::string stepType;

            // Loop until convergence
            while(   srv.iterations   < _options.maxIterations &&
                 deltaX       > _options.convergenceDeltaX &&
                     fabs(deltaJ)   > _options.convergenceDeltaJ)
         // while( !evaluateStoppingCriterion() )
          {
              // calculate steepest descent step:
              timeSD.start();

              Eigen::VectorXd Hrhs(_H.rows(),1);
              _H.multiply(&Hrhs, _rhs);

              _sd_scale = _rhs.squaredNorm() / Hrhs.squaredNorm();
             // std::cout << "  alpha:" << _sd_scale << std::endl;
              _dx_sd = _sd_scale * _rhs;

              timeSD.stop();

              // we need the norm for comparison:
              _dx_sd_norm = _dx_sd.norm();

              int stepIterations = 0;
              int linearSolverFailCounter = 0;

              bool gnComputed = false;
              do
              {
                // Trust Region smaller than SD:
                if (_dx_sd_norm >= _delta && _delta != 0)
                {
                  _dx = _delta / _dx_sd_norm * _dx_sd;  // scale SD step to fit into trust region
                  L0 = _delta * ( 2*_dx_sd_norm - _delta ) / ( 2*_sd_scale );
                  stepType = "SD";
                }
                // otherwise check the GN step
                else
                {
                  // calculate the GN step.
                  if(!gnComputed)
                  {
                    // set lambda to 0 in here...
                    _lambda = 0;

                    timeSchur.start();
                    applySchurComplement(_H, _rhs, _lambda,
                        _marginalizedStartingBlock,
                        _options.doLevenbergMarquardt,
                        _A, _invVi, _b);
                    timeSchur.stop();

                    // Solve the dense system.
                              if(_options.resetSolverEveryIteration)
                              {
                                  _solver->init();
                              }

                              timeSolve.start();
                              bool solutionSuccess = _solver->solve(_A, &_dx_gn[0], &_b[0]);
                              timeSolve.stop();

                              if(!solutionSuccess)
                              {

                                // the default solver failed. try the QR solver as a robust sparse alternative:
                                if(!_fallbackSolver) {  // initialise a new solver
                                  // for now take QR
                                  /// \todo: add an intelligent fallbackSolver structure.
                                  /// maybe add it as a property of the linear solvers and directly name the solvers in their
                                  /// class instead of the optimizer init()
                                  _fallbackSolver.reset(new sparse_block_matrix::LinearSolverQr<Eigen::MatrixXd>());
                                  _fallbackSolver->init();
                                  _options.verbose && std::cout << "Optimizer: The linear solution failed. Fallback to QR. (" << linearSolverFailCounter << ")" << std::endl;
                                }

                                solutionSuccess = _fallbackSolver->solve(_A, &_dx_gn[0], &_b[0]);
                                if(!solutionSuccess) {
                                  _options.verbose && std::cout << "Optimizer: The linear solution failed. Again... (" << linearSolverFailCounter << ")" << std::endl;
                                  _dx_gn = _dx_sd;
                                }

                                // set the default solver to the robust one.
                                if(linearSolverFailCounter > _options.linearSolverMaximumFails)
                                {
                                  _solver.reset(new sparse_block_matrix::LinearSolverQr<Eigen::MatrixXd>());
                                  _solver->init();
                                }
                                linearSolverFailCounter++;  // increment
                                // SM_ASSERT_TRUE(Exception, linearSolverFailCounter <= _options.linearSolverMaximumFails, "The linear solution failed");

                              }
                              gnComputed = true;  // now we have it!
                              // and calculate the norm:
                              _dx_gn_norm = _dx_gn.norm();
                  }

                  // set delta in the first step to take a full GN step:
                  if(_delta == 0) {
                    _delta = (_dx_sd + 0.5 * ( _dx_gn - _dx_sd )).norm();
                  }
                  // now check the size of the gn step:
                  if(_dx_gn_norm <= _delta)
                  {
                    _dx = _dx_gn; // trust region larger than GN step. take it!
                    L0 = _J;
                    stepType = "GN";
                  }
                  else  // otherwise interpolate on the line between the cauchy point and gn step
                  {
                    // get beta:
                    Eigen::VectorXd dgnsd = _dx_gn - _dx_sd;
                    double gdnsd_norm_sqr = dgnsd.squaredNorm();

                    double c = _dx_sd.transpose() * ( dgnsd );
                    if(c <= 0)
                    {
                      beta = -c + sqrt( c*c + gdnsd_norm_sqr*(_delta*_delta - _dx_sd_norm*_dx_sd_norm) );
                      beta /= gdnsd_norm_sqr;
                    }
                    else
                    {
                      beta = (_delta*_delta - _dx_sd_norm*_dx_sd_norm);
                      beta /= c + sqrt( c*c + gdnsd_norm_sqr*(_delta*_delta - _dx_sd_norm*_dx_sd_norm) );
                    }

                    _dx = _dx_sd + beta * ( dgnsd );
                    L0 = 1/2 * _sd_scale * (1-beta)*(1-beta)* _rhs.squaredNorm() + beta*(2-beta)*_J;
                    stepType = "DL";
                  }
                }
                // update:
                timeBackSub.start();
                deltaX = applyStateUpdate();
                timeBackSub.stop();

                // reevaluate error
              timeErr.start();
                evaluateError();
                timeErr.stop();

                // same check as in GN lambda update:
                // rho = getLmRho();
                rho = (_p_J - _J) / L0;
                if(std::isnan((double)rho))
                    rho = -1;

                //std::cout << "RHO: " << rho << " L0: " << L0 << std::endl;
                if(rho > 0)
                {
                  // update GN matrices:
                  timeGn.start();
                  buildGnMatrices();
                  timeGn.stop();
                }
                else
                {
                  revertLastStateUpdate();
                  evaluateError();
                }
                // update trust region
                _p_delta = _delta;
                if( rho > 0.75 ) // step succeeded
                {
                  double _dx_norm3 = 3 * _dx.norm();
                  if ( _delta < _dx_norm3 ) {
                    _delta = _dx_norm3;
                  }
                }
                else if (rho > 0 && rho < 0.25) // step almost failed
                {
                  _delta /= 2.0;
                }
                else if (rho <= 0)  // step failed
                {
                  // if we took a GN step set the trust region to the GN Step / 2
                  if(stepType == "GN")
                    _delta = _dx_gn_norm / 2.0;
                  else
                    _delta /= 2.0;
                }


                deltaJ = _p_J - _J;
                //std::cout << "_p_J:" << _p_J << std::endl;
                //std::cout << "_J:" << _J << std::endl;
                if(_J < _p_J)
                  _p_J = _J;
                else
                  _J = _p_J;

                _options.verbose && std::cout << "   (" << stepIterations << "): J: " << _J << ", dJ: " << deltaJ << ", deltaX: " << deltaX << ", delta: " << _p_delta << ", StepType: " << stepType;

                if(stepType == "DL")
                  std::cout << ", beta: " << beta << std::endl;
                else
                  std::cout << std::endl;

                stepIterations++;

              } while( rho < 0 && _delta > _options.convergenceDeltaX );

              // count iterations
              srv.iterations++;

              _options.verbose && std::cout << "[" << srv.iterations << "]: J: " << _J << ", dJ: " << deltaJ << ", deltaX: " << deltaX << ", delta: " << _delta << std::endl << std::endl;

            }

            srv.JFinal = _J;
            srv.lmLambdaFinal = _lambda;
            srv.dXFinal = deltaX;
            srv.dJFinal = deltaJ;

            return srv;



          }






    SolutionReturnValue Optimizer::optimize()
    {
      Timer timeGn("Optimizer: build Hessian", true);
      Timer timeErr("Optimizer: evaluate error", true);
      Timer timeSchur("Optimizer: Schur complement", true);
      Timer timeBackSub("Optimizer: Back substitution", true);
      Timer timeSolve("Optimizer: Solve linear system", true);
      // Select the design variables and (eventually) the error terms involved in the optimization.
      initialize();
      SolutionReturnValue srv;
      _p_J = 0.0;
      _lambda = 0;
      double gamma = _options.levenbergMarquardtLambdaGamma;
      double beta = _options.levenbergMarquardtLambdaBeta;
      int p = _options.levenbergMarquardtLambdaP;
      double mu = _options.levenbergMarquardtLambdaMuInit;
      // This sets _J
      timeErr.start();
      evaluateError();
      timeErr.stop();
      _p_J = _J;
      srv.JStart = _p_J;
      // *** while not done
      _options.verbose && std::cout << "[" << srv.iterations << ".0]: J: " << _J << std::endl;
      // Set up the estimation problem.
      double deltaX = _options.convergenceDeltaX + 1.0;
      double deltaJ = _options.convergenceDeltaJ + 1.0;
      bool isLmRegression = false;
      // Loop until convergence
      while (srv.iterations <  _options.maxIterations &&
             deltaX > _options.convergenceDeltaX &&
             fabs(deltaJ) > _options.convergenceDeltaJ) {
        // *** build: J, U, Vi, Wi, ea, ebi
        if (! isLmRegression) {
          timeGn.start();
          buildGnMatrices();
          timeGn.stop();
          setInitialLambda();
        }
        // **** Compute the invVis
        // **** Replace A with (U* - sum(YiWit)), and b with (ea = sum(Yi eb))
        timeSchur.start();
        applySchurComplement(_H, _rhs, _lambda,
                             _marginalizedStartingBlock,
                             _options.doLevenbergMarquardt,
                             _A, _invVi, _b);
        timeSchur.stop();
        // Solve the dense system.
        if (_options.resetSolverEveryIteration) {
          _solver->init();
        }
        timeSolve.start();
        bool solutionSuccess = _solver->solve(_A, &_dx[0], &_b[0]);
        timeSolve.stop();
        if (!solutionSuccess) {
          // \todo do something better
          //SM_ASSERT_TRUE(Exception, solutionSuccess, "The linear solution failed");
          isLmRegression = true;
          // **** if J is a regression
          revertLastStateUpdate();
          // ***** Update lambda
          // _lambda *= 1000;
          _lambda *= mu;
          mu *= 2;
          srv.failedIterations++;
        } else {
          /// Apply the state update. _A, _b, _dx, and _H are passed in implicitly.
          timeBackSub.start();
          deltaX = applyStateUpdate();
          timeBackSub.stop();
          // This sets _J
          timeErr.start();
          evaluateError();
          timeErr.stop();
          deltaJ = _p_J - _J;
          if (_options.doLevenbergMarquardt) {
            // update rho:
            double rho = getLmRho();
            // if(_J > _p_J)
            if (rho <= 0) {
              isLmRegression = true;
              // **** if J is a regression
              revertLastStateUpdate();
              // ***** Update lambda
              // _lambda *= 1000;
              _lambda *= mu;
              mu *= 2;
              srv.failedIterations++;
            } else {
              isLmRegression = false;
              // **** otherwise
              // ***** Update lambda
              if (_lambda > 1e-16) {
                // _lambda *= 0.1;
                double u1 = 1 / gamma;
                double u2 = 1 - (beta - 1) * pow((2 * rho - 1), p);
                if (u1 > u2)
                  _lambda *= u1;
                else
                  _lambda *= u2;
                mu = _options.levenbergMarquardtLambdaBeta;
              } else
                _lambda = 1e-15;
            }
          }
        }
        srv.iterations++;
        if (_J < _p_J)
          _p_J = _J;
        _options.verbose && std::cout << "[" << srv.iterations << "]: J: " << _J << ", dJ: " << deltaJ << ", deltaX: " << deltaX << ", lambda: " << _lambda << std::endl;
      }
      srv.JFinal = _J;
      srv.lmLambdaFinal = _lambda;
      srv.dXFinal = deltaX;
      srv.dJFinal = deltaJ;
      return srv;
    }


    DesignVariable* Optimizer::denseVariable(size_t i)
    {
      SM_ASSERT_LT_DBG(Exception, (int)i, _marginalizedStartingBlock, "index out of bounds");
      return _designVariables[i];
    }

    DesignVariable* Optimizer::sparseVariable(size_t i)
    {
      SM_ASSERT_LT_DBG(Exception, i + _marginalizedStartingBlock, _designVariables.size(), "index out of bounds");
      return _designVariables[i + _marginalizedStartingBlock];
    }


    size_t Optimizer::numDenseDesignVariables() const
    {
      return _marginalizedStartingBlock;
    }

    size_t Optimizer::numSparseDesignVariables() const
    {
      return _designVariables.size() - _marginalizedStartingBlock;
    }


    double Optimizer::applyStateUpdate()
    {
      // Apply the update to the dense state.
      int startIdx = 0;
      for (size_t i = 0; i < numDenseDesignVariables(); i++) {
        const int dbd = denseVariable(i)->minimalDimensions();
        Eigen::VectorXd dxS = _dx.segment(startIdx, dbd);
        dxS *= denseVariable(i)->scaling();
        if(dbd > 0) {
          denseVariable(i)->update(&dxS[0], dbd);
        }
        startIdx += dbd;
      }
      // Track the maximum delta
      // \todo: should this be some other metric?
      double deltaX = _dx.array().abs().maxCoeff();
      Eigen::VectorXd dsi;
      // Apply the update to the sparse blocks.
      // This is the back-substitution step.
      for (unsigned i = 0; i < _invVi.size(); i++) {
        buildDsi(i, _H, _rhs, _marginalizedStartingBlock, _invVi[i], _dx, dsi);
        // invert scaling in the update step
        dsi *= sparseVariable(i)->scaling();
        sparseVariable(i)->update(&dsi[0], dsi.size());
        // Track the maximum delta
        // \todo Do something better.
        deltaX = std::max(deltaX, dsi.array().abs().maxCoeff());
      }
      return deltaX;
    }





    void Optimizer::revertLastStateUpdate()
    {
      for (size_t i = 0; i < _designVariables.size(); i++) {
        _designVariables[i]->revertUpdate();
      }
    }


    OptimizerOptions& Optimizer::options()
    {
      return _options;
    }


    double Optimizer::evaluateError()
    {
      // \todo Make multi-threaded
      _J = 0.0;
      std::set<ErrorTerm*>::iterator it, it_end;
      it = _errorTerms.begin();
      it_end = _errorTerms.end();
      for (; it != it_end; ++it) {
        _J += (*it)->evaluateError();
      }
      return _J;
    }

    void Optimizer::buildGnMatrices()
    {
      // Do some initialization
      zeroMatrices();
      std::set<ErrorTerm*>::iterator it, it_end;
      it = _errorTerms.begin();
      it_end = _errorTerms.end();
      const bool useMEstimator = true;
      for (; it != it_end; ++it) {
        (*it)->buildHessian(_H, _rhs, useMEstimator);
      }
    }


    void Optimizer::zeroMatrices()
    {
      // clear(false) zeros all of the blocks but does not de-allocate them.
      _H.clear(false);
      _rhs.setZero();
      //_J = 0.0;
    }


    /// \brief return the full Hessian
    const SparseBlockMatrix& Optimizer::H() const
    {
      return _H;
    }

    /// \brief return the full rhs
    const Eigen::VectorXd& Optimizer::rhs() const
    {
      return _rhs;
    }

    /// \brief return the reduced system lhs
    const SparseBlockMatrix& Optimizer::A() const
    {
      return _A;
    }


    /// \brief return the reduced system rhs
    const Eigen::VectorXd& Optimizer::b() const
    {
      return _b;
    }


    /// \brief return the reduced system dx
    const Eigen::VectorXd& Optimizer::dx() const
    {
      return _dx;
    }

    /// The value of the objective function.
    double Optimizer::J() const
    {
      return _J;
    }

    void Optimizer::printTiming() const
    {
      sm::timing::Timing::print(std::cout);
    }



    void Optimizer::computeDiagonalCovariances()
    {
      std::vector<std::pair<int, int> > blockIndices;
      for (size_t i = 0; i < _designVariables.size(); ++i) {
        blockIndices.push_back(std::make_pair(i, i));
      }
      computeCovarianceBlocks(blockIndices);
    }

    void Optimizer::computeCovarianceBlocks(const std::vector<std::pair<int, int> > & blockIndices)
    {
      /// \todo Figure out properly why I have to create a new linear solver here.
      boost::shared_ptr<sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd> > solver(new sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd>());
      bool success = solver->solvePattern(_invH, blockIndices, _H);
      SM_ASSERT_TRUE(Exception, success, "Unable to retrieve covariance");
    }


    void Optimizer::computeCovariances()
    {
      std::vector<std::pair<int, int> > blockIndices;
      for (size_t i = 0; i < _designVariables.size(); ++i) {
        for (size_t j = i; j < _designVariables.size(); ++j) {
          blockIndices.push_back(std::make_pair(i, j));
        }
      }
      computeCovarianceBlocks(blockIndices);
    }

    const Eigen::MatrixXd* Optimizer::getCovarianceBlock(int blockRow, int blockCol) const
    {
      return _invH.block(blockRow, blockCol);
    }

    const SparseBlockMatrix& Optimizer::P() const
    {
      return _invH;
    }

    const SparseBlockMatrix& Optimizer::getCovariance() const
    {
      return _invH;
    }


    void Optimizer::checkProblemSetup()
    {
      // Check that all error terms are hooked up to design variables.
    }

  } // namespace backend
} // namespace aslam

