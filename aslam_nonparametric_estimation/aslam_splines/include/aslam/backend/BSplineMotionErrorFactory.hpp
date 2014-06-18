#ifndef _BSPLINEMOTIONERRORFACTORY_H_
#define _BSPLINEMOTIONERRORFACTORY_H_

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <bsplines/BSplinePose.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/backend/MarginalizationPriorErrorTerm.hpp>


namespace aslam {
namespace backend {

template<class BSplineDesignVariable>
void addMotionErrorTerms(OptimizationProblemBase& problem, BSplineDesignVariable& spline, const Eigen::MatrixXd& W, unsigned int errorTermOrder);

} // namespace backend
} // namespace aslam

#include "implementation/BSplineMotionErrorFactory.hpp"

#endif /* _BSPLINEMOTIONERRORFACTORY_H_ */
