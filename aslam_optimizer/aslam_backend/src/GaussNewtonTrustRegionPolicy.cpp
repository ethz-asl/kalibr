#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>

namespace aslam {
    namespace backend {
        
        
        GaussNewtonTrustRegionPolicy::GaussNewtonTrustRegionPolicy()  {}
        GaussNewtonTrustRegionPolicy::~GaussNewtonTrustRegionPolicy() {}
        
        
        /// \brief called by the optimizer when an optimization is starting
    void GaussNewtonTrustRegionPolicy::optimizationStartingImplementation(double /* J */)
        {
            
        }
        
        // Returns true if the solution was successful
    bool GaussNewtonTrustRegionPolicy::solveSystemImplementation(double /* J */, bool /* previousIterationFailed */, int nThreads, Eigen::VectorXd& outDx)
        {
            _solver->buildSystem(nThreads, true);
            return _solver->solveSystem(outDx);
        }
        
        /// \brief print the current state to a stream (no newlines).
        std::ostream & GaussNewtonTrustRegionPolicy::printState(std::ostream & out) const
        {
            out << "GN" << std::endl;
            return out;
        }

        
        bool GaussNewtonTrustRegionPolicy::revertOnFailure()
        {
            return false;
        }

    bool GaussNewtonTrustRegionPolicy::requiresAugmentedDiagonal() const {
      return false;
    }
        
    } // namespace backend
} // namespace aslam
