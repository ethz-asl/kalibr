#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <sm/PropertyTree.hpp>

namespace aslam {
    namespace backend {

    LevenbergMarquardtTrustRegionPolicy::LevenbergMarquardtTrustRegionPolicy() :
        _lambdaInit(1e-3),
        _gammaInit(3),
        _betaInit(2),
        _pInit(3),
        _muInit(2)
    {

    }
    LevenbergMarquardtTrustRegionPolicy::LevenbergMarquardtTrustRegionPolicy(double lambdaInit) :
        _lambdaInit(lambdaInit),
        _gammaInit(3),
        _betaInit(2),
        _pInit(3),
        _muInit(2)
    {

    }

    LevenbergMarquardtTrustRegionPolicy::LevenbergMarquardtTrustRegionPolicy(const sm::PropertyTree & config) {
      _lambdaInit = config.getDouble("lambdaInit", 1e-3);
      _gammaInit  = config.getDouble("gammaInit", 3.0);
      _betaInit   = config.getDouble("betaInit", 2.0); 
      _pInit      = config.getInt("pInit", 3);
      _muInit     = config.getDouble("muInit", 2.0);
    }
    
        LevenbergMarquardtTrustRegionPolicy::~LevenbergMarquardtTrustRegionPolicy() {}
        
        
        /// \brief called by the optimizer when an optimization is starting
        void LevenbergMarquardtTrustRegionPolicy::optimizationStartingImplementation(double /* J */)
        {
            // initialise lambda:
          _lambda = _lambdaInit;
          _gamma = _gammaInit;
          _beta = _betaInit;
          _p = _pInit;
          _mu = _muInit;
            
        }
        
        // Returns true if the solution was successful
    bool LevenbergMarquardtTrustRegionPolicy::solveSystemImplementation(double /* J */, bool previousIterationFailed, int nThreads, Eigen::VectorXd& outDx)
        {
            SM_ASSERT_TRUE(Exception, _solver.get() != NULL, "The solver is null");
            
            if (isFirstIteration()) {
                // This is the first step.
                _solver->buildSystem(nThreads, true);
            } else {
                ///get Rho and update Lambda:
                double rho = getLmRho();
              
                if (previousIterationFailed ) {
                  // The last step was a regression.
                  _mu *= 2;
                  _lambda *= _mu;
                } else if (rho <= 0 ) {
                  // No need to rebuild the system. Just reset the conditioner
                  _mu *= 10;
                  _lambda *= _mu;
                } else {
                    // The last iteration was successful
                    // Here we need to rebuild the system
                    _solver->buildSystem(nThreads, true);
                    if (_lambda > 1e-16) {
                        double u1 = 1 / _gamma;
                        double u2 = 1 - (_beta - 1) * pow((2 * rho - 1), _p);
                        if (u1 > u2)
                            _lambda *= u1;
                        else
                            _lambda *= u2;
                        _mu = _beta;
                    } else {
                        _lambda = 1e-15;
                    }
                }
            }
            
            _solver->setConstantConditioner(_lambda);
            bool success = _solver->solveSystem(_dx);
            outDx = _dx;
            return success;
        }
        
        /// \brief print the current state to a stream (no newlines).
        std::ostream & LevenbergMarquardtTrustRegionPolicy::printState(std::ostream & out) const
        {
            out << "LM - lambda:" << _lambda << " mu:" << _mu;
            return out;
        }

        
        bool LevenbergMarquardtTrustRegionPolicy::revertOnFailure()
        {
            return true;
        }
        
        
        double LevenbergMarquardtTrustRegionPolicy::getLmRho()
        {
            double d1 = get_dJ();    // update cost delta
            // L(0) - L(h):
            double d2 = _dx.transpose() * (_lambda * _dx + _solver->rhs());
            return d1 / d2;
        }
        
    bool LevenbergMarquardtTrustRegionPolicy::requiresAugmentedDiagonal() const {
      return true;
    }
    } // namespace backend
} // namespace aslam
