#ifndef ASLAM_SPLINES_B_SPLINE_RS_EXPRESSIONS_HPP
#define ASLAM_SPLINES_B_SPLINE_RS_EXPRESSIONS_HPP


#include <bsplines/BSplinePose.hpp>
#include <aslam/backend/TransformationExpression.hpp>
//#include <aslam/backend/RotationExpression.hpp>
//#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/TransformationExpressionNode.hpp>
//#include <aslam/backend/RotationExpressionNode.hpp>
//#include <aslam/backend/EuclideanExpressionNode.hpp>
#include <aslam/backend/VectorExpressionNode.hpp>
#include <aslam/backend/DesignVariableVector.hpp>


namespace aslam {
  namespace splines {
    class BSplinePoseDesignVariable;
    class BSplineRSPoseDesignVariable;

   
    // aslam::backend::TransformationExpression transformation(double tk);
    class RSLineDelayTransformationExpressionNode : public aslam::backend::TransformationExpressionNode
    {
    public:    
        
      RSLineDelayTransformationExpressionNode(BSplinePoseDesignVariable * bspline, aslam::backend::DesignVariableVector<1> * lineDelay, double integrationStartTime, double line);
      virtual ~RSLineDelayTransformationExpressionNode();

    protected:
      virtual Eigen::Matrix4d toTransformationMatrixImplementation() override;
      virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const override;
      virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const override;


      BSplinePoseDesignVariable * _spline;
      aslam::backend::DesignVariableVector<1> * _lineDelay;
      double _integrationStartTime;
      double _line;

    };

  } // namespace splines
} // namespace aslam

//#include "implementation/BSplineExpressions.hpp"

#endif /* ASLAM_SPLINES_B_SPLINE_RS_EXPRESSIONS_HPP */
