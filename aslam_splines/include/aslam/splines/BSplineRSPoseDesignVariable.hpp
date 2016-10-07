#ifndef ASLAM_SPLINES_BSPLINE_RS_POSE_DESIGN_VARIABLE_HPP
#define ASLAM_SPLINES_BSPLINE_RS_POSE_DESIGN_VARIABLE_HPP

#include <aslam/backend/DesignVariableMappedVector.hpp>
#include <bsplines/BSplinePose.hpp>
#include <Eigen/StdVector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <aslam/backend/TransformationExpression.hpp>
//#include <aslam/backend/RotationExpression.hpp>
//#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/VectorExpression.hpp>

#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/backend/DesignVariableVector.hpp>




namespace aslam {
  namespace splines {
        

    class BSplineRSPoseDesignVariable
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef Eigen::Matrix<double, 1, 1> vector_t;
        
        
      /// \brief this guy takes a copy
      BSplineRSPoseDesignVariable(const bsplines::BSplinePose & bsplinePose, const double lineDelay);
      
      virtual ~BSplineRSPoseDesignVariable();

      /// \brief get the spline.
      const bsplines::BSplinePose & spline();
      double lineDelay();

      // \todo Return a Transformation expression, a Rotation expression, A Euclidean point expression, and lots of VectorExpressions.
      aslam::backend::TransformationExpression transformation(double t0, double line);
    //  aslam::backend::VectorExpression<1> lineDelay();
      
        
      size_t numDesignVariables();
      /// \brief Design variables involving splines must have the spline design variables in the FIRST positions! (important for BSplineMotion Error)
      aslam::backend::DesignVariable * designVariable(size_t i);

      // 
      Eigen::VectorXi getActiveDesignVariableIndices(double tk);
      
    private:
      /// \brief the internal spline.
      bsplines::BSplinePose _bsplinePose;
      vector_t _lineDelay;
        
      // design variables
      BSplinePoseDesignVariable* _bsplinePoseDV;
      aslam::backend::DesignVariableVector<1> * _lineDelayDV;
      
    };
    
  } // namespace splines
} // namespace aslam

#endif /* ASLAM_SPLINES_BSPLINE_RS_POSE_DESIGN_VARIABLE_HPP */
