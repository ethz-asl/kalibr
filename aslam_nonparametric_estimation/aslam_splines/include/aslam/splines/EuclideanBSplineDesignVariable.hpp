#ifndef ASLAM_SPLINES_EUCLIDEAN_SPLINE_HPP
#define ASLAM_SPLINES_EUCLIDEAN_SPLINE_HPP

#include "BSplineDesignVariable.hpp"
#include <aslam/backend/EuclideanExpression.hpp>

namespace aslam {
    namespace splines {

        class EuclideanBSplineDesignVariable : public BSplineDesignVariable<3>
        {
        public:
            EuclideanBSplineDesignVariable(const bsplines::BSpline & bspline);
            virtual ~EuclideanBSplineDesignVariable();

            aslam::backend::EuclideanExpression toEuclideanExpression(double time, int order);

            Eigen::Vector3d toEuclidean(double time, int order);
        };

    } // namespace splines
} // namespace aslam

#endif /* ASLAM_SPLINES_EUCLIDEAN_SPLINE_HPP */
