#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>

namespace aslam {
    namespace splines {
        

        
        
        EuclideanBSplineDesignVariable::EuclideanBSplineDesignVariable(const bsplines::BSpline & bspline) :
            BSplineDesignVariable<3>(bspline)
        {

        }

        EuclideanBSplineDesignVariable::~EuclideanBSplineDesignVariable()
        {

        }

        aslam::backend::EuclideanExpression EuclideanBSplineDesignVariable::toEuclideanExpression(double time, int order)
        {
            Eigen::VectorXi dvidxs = _bspline.localVvCoefficientVectorIndices(time);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }
            boost::shared_ptr<aslam::splines::BSplineEuclideanExpressionNode > root( new aslam::splines::BSplineEuclideanExpressionNode(&_bspline, dvs, time, order) );
            
            return aslam::backend::EuclideanExpression(root);
        }


        Eigen::Vector3d EuclideanBSplineDesignVariable::toEuclidean(double time, int order)
        {
            return _bspline.evalD(time,order);
        }
        
        aslam::backend::EuclideanExpression EuclideanBSplineDesignVariable::toEuclideanExpressionAtTime(const aslam::backend::ScalarExpression & time, 
                                                                                                        int order, double leftBuffer, double rightBuffer) {


            //Eigen::VectorXi dvidxs = _bspline.localVvCoefficientVectorIndices(time);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < _designVariables.size(); ++i)
            {
                dvs.push_back(&_designVariables[i]);
            }
            boost::shared_ptr<aslam::splines::BSplineEuclideanExpressionAtTimeNode > root( new aslam::splines::BSplineEuclideanExpressionAtTimeNode(&_bspline, dvs, time, order, leftBuffer, rightBuffer) );
            
            return aslam::backend::EuclideanExpression(root);
        }

    } // namespace splines
} // namespace aslam
