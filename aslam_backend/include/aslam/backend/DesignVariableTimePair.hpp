#ifndef ASLAM_DESIGN_VARIABLE_TIME_PAIR_HPP
#define ASLAM_DESIGN_VARIABLE_TIME_PAIR_HPP

#include <aslam/backend/DesignVariable.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {
  namespace backend {
    class DesignVariable;

    struct DesignVariableTimePair
    {
        DesignVariable* dv;
        sm::timing::NsecTime t;
    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_DESIGN_VARIABLE_TIME_PAIR_HPP */
