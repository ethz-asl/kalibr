#ifndef ASLAM_SPLINES_BSPLINE_DESIGN_VARIABLE_HPP
#define ASLAM_SPLINES_BSPLINE_DESIGN_VARIABLE_HPP

#include <aslam/backend/DesignVariableMappedVector.hpp>
#include <bsplines/BSpline.hpp>
#include <Eigen/StdVector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/VectorExpressionNode.hpp>
#include "BSplineExpressions.hpp"
namespace aslam {
    namespace splines {
        

        template<int DIM>
        class BSplineDesignVariable
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
            enum {
                Dimension = DIM
            };

            typedef aslam::backend::DesignVariableMappedVector<Dimension> dv_t;

            /// \brief this guy takes a copy.
            BSplineDesignVariable(const bsplines::BSpline & bspline);
      
            virtual ~BSplineDesignVariable();

            /// \brief get the spline.
            const bsplines::BSpline & spline();
      
            /// \brief get an expression
            aslam::backend::VectorExpression<DIM> toExpression(double time, int derivativeOrder);

            size_t numDesignVariables();
            aslam::backend::DesignVariableMappedVector<DIM> * designVariable(size_t i);
      
            std::vector<aslam::backend::DesignVariable *> getDesignVariables(double time) const;

            // Fabio:
            // add one Segment at the end of the PoseSpline
            void addSegment(double t, const Eigen::VectorXd & p);
            void addSegment2(double t, const Eigen::VectorXd & p, double lambda);
            void removeSegment();

        protected:
            /// \brief the internal spline.
            bsplines::BSpline _bspline;

            /// \brief the vector of design variables.
            boost::ptr_vector< dv_t > _designVariables;
      
        };
    
    } // namespace splines
} // namespace aslam

#include "implementation/BSplineDesignVariable.hpp"

#endif /* ASLAM_SPLINES_BSPLINE_DESIGN_VARIABLE_HPP */
