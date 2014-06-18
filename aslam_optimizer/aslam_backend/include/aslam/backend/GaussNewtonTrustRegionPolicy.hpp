#ifndef ASLAM_BACKEND_GAUSS_NEWTON_TRUST_REGION_POLICY_HPP
#define ASLAM_BACKEND_GAUSS_NEWTON_TRUST_REGION_POLICY_HPP

#include <aslam/backend/TrustRegionPolicy.hpp>
#include <aslam/backend/LinearSystemSolver.hpp>
#include <boost/shared_ptr.hpp>

namespace aslam {
    namespace backend {
        
        class GaussNewtonTrustRegionPolicy : public TrustRegionPolicy
        {
        public:
            GaussNewtonTrustRegionPolicy();
            virtual ~GaussNewtonTrustRegionPolicy();
            
            /// \brief called by the optimizer when an optimization is starting
            virtual void optimizationStartingImplementation(double J);
            
            // Returns true if the solution was successful
          virtual bool solveSystemImplementation(double J, bool previousIterationFailed, int nThreads, Eigen::VectorXd& outDx);
            
            /// \brief should the optimizer revert on failure? You should probably return true
            bool revertOnFailure();
            
            /// \brief print the current state to a stream (no newlines).
            virtual std::ostream & printState(std::ostream & out) const;
          virtual std::string name() const { return "gauss_newton"; }
          virtual bool requiresAugmentedDiagonal() const;

        
            
        };
        
    } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_GAUSS_NEWTON_TRUST_REGION_POLICY_HPP */
