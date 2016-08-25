#include <aslam/backend/DogLegTrustRegionPolicy.hpp>

namespace aslam {
    namespace backend {

        DogLegTrustRegionPolicy::DogLegTrustRegionPolicy()   {}
        DogLegTrustRegionPolicy::~DogLegTrustRegionPolicy() {}


        /// \brief called by the optimizer when an optimization is starting
        void DogLegTrustRegionPolicy::optimizationStartingImplementation(double /* J */)
        {
            _dx_sd_norm = 0;
            _dx_gn_norm = 0;
            _L0 = 0;
            _sd_scale = 0;
            _beta = 0;
            std::string _stepType;
            _delta  = 0;
            _p_delta = 0;

        }

        // Returns true if the solution was successful
    bool DogLegTrustRegionPolicy::solveSystemImplementation(double J, bool previousIterationFailed, int nThreads, Eigen::VectorXd& outDx)
        {
            SM_ASSERT_TRUE(Exception, _solver.get() != NULL, "The solver is null");
            bool solutionSuccess = true;

            ///////////////
            ///Update Delta
            // same check as in GN lambda update:
            double rho = get_dJ() / _L0;

            if( ! isFirstIteration() )
            {
                // update trust region
                _p_delta = _delta;
                if( rho > 0.75 ) // step succeeded
                {
                    double dx_norm3 = 3 * _dx.norm();
                    if ( _delta > dx_norm3 )
                        _delta = _delta;
                    else
                        _delta = dx_norm3;
                }
                else if (rho > 0 && rho < 0.25) // step almost failed
                {
                    _delta /= 2.0;
                }
                else if (rho <= 0)  // step failed
                {
                    // if we took a GN step set the trust region to the GN Step / 2
                    if(_stepType == "GN")
                        _delta = _dx_gn_norm / 2.0;
                    else
                        _delta /= 2.0;
                }
            }
            bool gnComputed = true;
            // successful step:
            // rebuild system and recalculate sd-solution
            if(!previousIterationFailed) {
                // update GN matrices:
                // std::cout << "Building system\n";
                _solver->buildSystem(nThreads, true);

                // calculate steepest descent step:

                _sd_scale = _solver->rhs().squaredNorm() / _solver->rhsJtJrhs();
                _dx_sd = _sd_scale * _solver->rhs();


                // we need the norm for comparison:
                _dx_sd_norm = _dx_sd.norm();

                // invalidate GN solution
                gnComputed = false;
            }

            // Trust Region smaller than SD:
            if (_dx_sd_norm >= _delta && _delta != 0)
            {
                _dx = _delta / _dx_sd_norm * _dx_sd;  // scale SD step to fit into trust region
                _L0 = _delta * ( 2*_dx_sd_norm - _delta ) / ( 2*_sd_scale );
                _stepType = "SD";
            }
            // otherwise check the GN step
            else
            {
                // calculate the GN step.
                if(!gnComputed)
                {
                    solutionSuccess = _solver->solveSystem(_dx_gn);

                    if(!solutionSuccess)
                        return solutionSuccess;

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
                    _L0 = J;
                    _stepType = "GN";
                }
                else  // otherwise interpolate on the line between the cauchy point and gn step
                {
                    // get beta:
                    Eigen::VectorXd dgnsd = _dx_gn - _dx_sd;
                    double gdnsd_norm_sqr = dgnsd.squaredNorm();

                    double c = _dx_sd.transpose() * ( dgnsd );
                    if(c <= 0)
                    {
                        _beta = -c + sqrt( c*c + gdnsd_norm_sqr*(_delta*_delta - _dx_sd_norm*_dx_sd_norm) );
                        _beta /= gdnsd_norm_sqr;
                    }
                    else
                    {
                        _beta = (_delta*_delta - _dx_sd_norm*_dx_sd_norm);
                        _beta /= c + sqrt( c*c + gdnsd_norm_sqr*(_delta*_delta - _dx_sd_norm*_dx_sd_norm) );
                    }

                    _dx = _dx_sd + _beta * ( dgnsd );
                    _L0 = 1/2 * _sd_scale * (1-_beta)*(1-_beta)* _solver->rhs().squaredNorm() + _beta*(2-_beta)*J;
                    _stepType = "DL";
                }
            }

            outDx = _dx;
            return solutionSuccess;

        }

        /// \brief print the current state to a stream (no newlines).
        std::ostream & DogLegTrustRegionPolicy::printState(std::ostream & out) const
        {
            out << "DL - delta:" << _delta << ", " << _stepType;
            if(_stepType == "DL")
                out << ", beta:" << _beta;
            return out;
        }


        bool DogLegTrustRegionPolicy::revertOnFailure()
        {
            return true;
        }

        bool DogLegTrustRegionPolicy::requiresAugmentedDiagonal() const {
            return false;
        }

    } // namespace backend
} // namespace aslam
