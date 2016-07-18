#include <aslam/backend/TrustRegionPolicy.hpp>

namespace aslam {
    namespace backend {

        TrustRegionPolicy::TrustRegionPolicy(){}
        TrustRegionPolicy::~TrustRegionPolicy(){}


        /// \brief get the linear system solver
        boost::shared_ptr<LinearSystemSolver> TrustRegionPolicy::getSolver()
        {
            return _solver;
        }

        /// \brief set the linear system solver
        void TrustRegionPolicy::setSolver(boost::shared_ptr<LinearSystemSolver> solver)
        {
            _solver = solver;
        }

        bool TrustRegionPolicy::revertOnFailure()
        {
            return true;
        }


        /// \brief called by the optimizer when an optimization is starting
        void TrustRegionPolicy::optimizationStarting(double J)
        {
            _J = J;
            _p_J = J;
            _last_successful_J = J;
            _isFirstIteration = true;
            optimizationStartingImplementation(J);
        }

        // Returns true if the solution was successful
        bool TrustRegionPolicy::solveSystem(double J, bool previousIterationFailed, int nThreads, Eigen::VectorXd& outDx)
        {
            if(previousIterationFailed) {
                _J = J;
            } else {
                _p_J = _last_successful_J;
                _last_successful_J = J;
                _J = J;
            }

            bool success = solveSystemImplementation(J, previousIterationFailed, nThreads, outDx);
            _isFirstIteration = false;
            return success;
        }

        double TrustRegionPolicy::get_dJ()
        {
            return _p_J - _J;
        }



    } // namespace backend
} // namespace aslam
