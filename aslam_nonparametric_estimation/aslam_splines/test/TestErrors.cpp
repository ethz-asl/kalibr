#include <sm/eigen/gtest.hpp>
#include <aslam/backend/SimpleSplineError.hpp>
#include <aslam/splines/BSplineDesignVariable.hpp>
#include <bsplines/BSpline.hpp>


using namespace bsplines;
using namespace aslam::splines;

BSplineDesignVariable<1> generateRandomBSpline()
{
    BSpline bspline(4);
    const int N = 10;
    Eigen::VectorXd times(N);
    for(int i = 0; i < N; ++i)
        times(i) = i;
    
    Eigen::Matrix<double, 1, Eigen::Dynamic> K(1,N);
    K.setRandom();
    
    
    bspline.initSpline3(times, K, 6, 1e-4);
    
    BSplineDesignVariable<1> bdv(bspline);
    for(size_t i = 0; i < bdv.numDesignVariables(); ++i)
    {
        bdv.designVariable(i)->setActive(true);
        bdv.designVariable(i)->setBlockIndex(i);
    }

    return bdv;
    
}



TEST(SplineErrorTestSuite, testSimpleSplineError)
{
    try
    {
        using namespace aslam::backend;
        BSplineDesignVariable<1> initSpline = generateRandomBSpline();
        Eigen::VectorXd values(1.0);

        VectorExpression<1> splineExpression =  initSpline.toExpression(5.0,0);
        SimpleSplineError<BSplineDesignVariable<1> > e(&initSpline, &splineExpression, values, 5.0);
        
        JacobianContainer estJ(e.dimension());
        e.evaluateJacobiansFiniteDifference(estJ);
        
        JacobianContainer J(e.dimension());
        e.evaluateJacobians(J);
        
        SCOPED_TRACE("");
        sm::eigen::assertNear(J.asDenseMatrix(), estJ.asDenseMatrix(), 1e-6, SM_SOURCE_FILE_POS, "Checking the jacobian vs. finite differences");
        
    }
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }
}


