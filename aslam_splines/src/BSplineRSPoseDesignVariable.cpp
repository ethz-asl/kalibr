#include <aslam/splines/BSplineRSPoseDesignVariable.hpp>
#include <aslam/backend/backend.hpp>
#include <aslam/splines/BSplineRSExpressions.hpp>

#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/VectorExpressionNode.hpp>



namespace aslam {
  namespace splines {

    /// \brief this guy takes a copy.
    BSplineRSPoseDesignVariable::BSplineRSPoseDesignVariable(const bsplines::BSplinePose & bsplinePose, const double lineDelay) :
      _bsplinePose(bsplinePose)
    {
      // here is where the magic happens.
      _bsplinePoseDV = new BSplinePoseDesignVariable(bsplinePose);
      _lineDelay(0,0) = lineDelay;
    //    _lineDelay = vector_t::Zero();
      _lineDelayDV = new aslam::backend::DesignVariableVector<1>(_lineDelay);
        
        
    }
    
    BSplineRSPoseDesignVariable::~BSplineRSPoseDesignVariable()
    {

    }
    
    /// \brief get the spline.
    const bsplines::BSplinePose & BSplineRSPoseDesignVariable::spline()
    {
      return _bsplinePoseDV->spline();
    }

    double BSplineRSPoseDesignVariable::lineDelay() {
        
        return (_lineDelayDV->value())(0,0);
      
    }
      
    size_t BSplineRSPoseDesignVariable::numDesignVariables()
    {
        return _bsplinePoseDV->numDesignVariables() + 1;
    }

    aslam::backend::DesignVariable * BSplineRSPoseDesignVariable::designVariable(size_t i)
    {
        SM_ASSERT_LT(aslam::Exception, i, numDesignVariables(), "Index out of bounds");
        
        if ( i == numDesignVariables()-1 ) 
            return _lineDelayDV;
        else
            return _bsplinePoseDV->designVariable(i);
    }

    Eigen::VectorXi BSplineRSPoseDesignVariable::getActiveDesignVariableIndices(double tk)
    {
      return _bsplinePose.localVvCoefficientVectorIndices(tk);
    }

    aslam::backend::TransformationExpression BSplineRSPoseDesignVariable::transformation(double t0, double line)
    {
        
    
        boost::shared_ptr<RSLineDelayTransformationExpressionNode> root( new RSLineDelayTransformationExpressionNode( _bsplinePoseDV, _lineDelayDV, t0, line) );
      
        return aslam::backend::TransformationExpression(root);

    }

 /*   aslam::backend::VectorExpression<1> BSplineRSPoseDesignVariable::lineDelay() {
        
        boost::shared_ptr<aslam::backend::VectorExpressionNode<1>> root( new aslam::backend::VectorExpressionNode<1>( _lineDelayDV ) );
        
        return aslam::backend::VectorExpression(root);
        
    }*/
      
      
  } // namespace splines
} // namespace aslam
