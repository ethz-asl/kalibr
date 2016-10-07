#ifndef ASLAM_SPLINES_BSPLINE_POSE_DESIGN_VARIABLE_HPP
#define ASLAM_SPLINES_BSPLINE_POSE_DESIGN_VARIABLE_HPP

#include <aslam/backend/DesignVariableMappedVector.hpp>
#include <bsplines/BSplinePose.hpp>
#include <Eigen/StdVector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>

namespace aslam {
    namespace splines {
        

        class BSplinePoseDesignVariable
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
            typedef aslam::backend::DesignVariableMappedVector<6> dv_t;

            /// \brief this guy takes a copy.
            BSplinePoseDesignVariable(const bsplines::BSplinePose & bsplinePose);
      
            virtual ~BSplinePoseDesignVariable();

            /// \brief get the spline.
            const bsplines::BSplinePose & spline();

            // \todo Return a Transformation expression, a Rotation expression, A Euclidean point expression, and lots of VectorExpressions.
            aslam::backend::TransformationExpression transformation(double tk);
            aslam::backend::RotationExpression orientation(double tk);
            aslam::backend::EuclideanExpression position(double tk);
      
            // Get a transformation expression where the time may change.
            aslam::backend::TransformationExpression transformationAtTime(const aslam::backend::ScalarExpression & time, double leftBuffer, double rightBuffer);
            aslam::backend::TransformationExpression transformationAtTime(const aslam::backend::ScalarExpression & time);

            // Fabio:
            aslam::backend::EuclideanExpression linearVelocity(double tk);
            aslam::backend::EuclideanExpression linearAcceleration(double tk);
            aslam::backend::EuclideanExpression angularVelocityBodyFrame(double tk);
            // Fabio:
            aslam::backend::EuclideanExpression angularAccelerationBodyFrame(double tk);

      aslam::backend::EuclideanExpression linearAccelerationBodyFrame(double tk);

            size_t numDesignVariables();
            aslam::backend::DesignVariableMappedVector<6> * designVariable(size_t i);


            // Luc:
            Eigen::VectorXi getActiveDesignVariableIndices(double tk);
            // Fabio:
            std::vector<aslam::backend::DesignVariable *> getDesignVariables(double tk);
      

            // Fabio:
            // add one Segment at the end of the PoseSpline
            void addSegment(double t, Eigen::Matrix4d T);
            void addSegment2(double t, Eigen::Matrix4d T, double lambda);
            void removeSegment();

        private:
            /// \brief the internal spline.
            bsplines::BSplinePose _bsplinePose;

            /// \brief the vector of design variables.
            boost::ptr_vector< dv_t > _designVariables;
      
        };
    
    } // namespace splines
} // namespace aslam

#endif /* ASLAM_SPLINES_BSPLINE_POSE_DESIGN_VARIABLE_HPP */
