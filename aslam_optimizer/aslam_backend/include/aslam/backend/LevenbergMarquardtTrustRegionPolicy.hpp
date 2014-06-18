#ifndef ASLAM_BACKEND_LEVENBERG_MARQUARDT_TRUST_REGION_POLICY_HPP
#define ASLAM_BACKEND_LEVENBERG_MARQUARDT_TRUST_REGION_POLICY_HPP

#include <aslam/backend/TrustRegionPolicy.hpp>
#include <aslam/backend/LinearSystemSolver.hpp>
#include <boost/shared_ptr.hpp>

namespace sm {
class PropertyTree;
} // namespace sm

namespace aslam {
    namespace backend {
        
        class LevenbergMarquardtTrustRegionPolicy : public TrustRegionPolicy
        {
        public:
            LevenbergMarquardtTrustRegionPolicy();
          LevenbergMarquardtTrustRegionPolicy(const sm::PropertyTree & config);
            LevenbergMarquardtTrustRegionPolicy(double lambdaInit);
            virtual ~LevenbergMarquardtTrustRegionPolicy();
            
            /// \brief called by the optimizer when an optimization is starting
            virtual void optimizationStartingImplementation(double J);
            
            // Returns true if the solution was successful
          virtual bool solveSystemImplementation(double J, bool previousIterationFailed, int nThreads, Eigen::VectorXd& outDx);
            
            /// \brief should the optimizer revert on failure? You should probably return true
            virtual bool revertOnFailure();
            
            /// \brief print the current state to a stream (no newlines).
            virtual std::ostream & printState(std::ostream & out) const;
          virtual bool requiresAugmentedDiagonal() const;
          virtual std::string name() const { return "levenberg_marquardt"; }
        private:
          double getLmRho();
          double _lambdaInit;
          double _gammaInit;
          double _betaInit;
          int _pInit;
          double _muInit;
          
          double _lambda;
          double _gamma;
          double _beta;
          int _p;
          double _mu;
          
          Eigen::VectorXd _dx;
            
        };
        
    } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_LEVENBERG_MARQUARDT_TRUST_REGION_POLICY_HPP */
