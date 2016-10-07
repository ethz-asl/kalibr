#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>
#include <aslam/backend/util/CommonDefinitions.hpp>

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
            Timer timeBuild("GnTrustRegionPolicy: Build linear system", false);
            _solver->buildSystem(nThreads, true);
            timeBuild.stop();
            Timer timeSolve("GnTrustRegionPolicy: Solve linear system", false);// will stop on return
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
