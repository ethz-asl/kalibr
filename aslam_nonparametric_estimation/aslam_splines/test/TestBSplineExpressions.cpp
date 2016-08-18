#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
//#include <aslam/splines/BSplineRSPoseDesignVariable.hpp>
#include <sm/kinematics/EulerRodriguez.hpp>
#include <aslam/backend/Scalar.hpp>

using namespace aslam::backend;
using namespace sm::kinematics;
using namespace aslam::splines;
using namespace bsplines;

template<typename EXPRESSION_T>
struct ExpressionNodeFunctor
{
    typedef Eigen::VectorXd value_t;
    typedef value_t::Scalar scalar_t;
    typedef Eigen::VectorXd input_t;
    typedef Eigen::MatrixXd jacobian_t;
    typedef EXPRESSION_T expression_t;

    ExpressionNodeFunctor(expression_t dv) :
        _dv(dv) {}

    input_t update(const input_t & x, int c, scalar_t delta) { input_t xnew = x; xnew[c] += delta; return xnew; }

    expression_t _dv;

    Eigen::VectorXd operator()(const Eigen::VectorXd & dr)
        {
    
            Eigen::VectorXd p = _dv.toValue();
            JacobianContainer J(p.size());
            _dv.evaluateJacobians(J);

            int offset = 0;
            for(size_t i = 0; i < J.numDesignVariables(); i++)
            {
                DesignVariable * d = J.designVariable(i);
                d->update(&dr[offset],d->minimalDimensions());
                offset += d->minimalDimensions();
            }

            p = _dv.toValue();
    
 
            for(size_t i = 0; i < J.numDesignVariables(); i++)
            {
                DesignVariable * d = J.designVariable(i);
                d->revertUpdate();
            }

            return p;
   
        }



    void testJacobian()
        {
            sm::eigen::NumericalDiff<ExpressionNodeFunctor> numdiff(*this);
            /// Discern the size of the jacobian container
            Eigen::VectorXd p = _dv.toValue();
            JacobianContainer Jc(p.size());
            _dv.evaluateJacobians(Jc);

            Eigen::VectorXd dp(Jc.cols());
            dp.setZero();

            Eigen::MatrixXd Jest = numdiff.estimateJacobian(dp);

            sm::eigen::assertNear(Jc.asSparseMatrix(), Jest, 1e-6, SM_SOURCE_FILE_POS, "Testing the expression jacobian. A is analytical and B is from finite differences");
        }

};


BSplinePoseDesignVariable generateRandomSpline()
{
    boost::shared_ptr<EulerRodriguez> rk( new EulerRodriguez );
    BSplinePose bsplinePose(4, rk);
    const int N = 10;
    Eigen::VectorXd times(N);
    for(int i = 0; i < N; ++i)
        times(i) = i;

    Eigen::Matrix<double, 6, Eigen::Dynamic> K(6,N);
    K.setRandom();


    bsplinePose.initPoseSpline3(times, K, 6, 1e-4);
  
    BSplinePoseDesignVariable bdv(bsplinePose);
    for(size_t i = 0; i < bdv.numDesignVariables(); ++i)
    {
        bdv.designVariable(i)->setActive(true);
        bdv.designVariable(i)->setBlockIndex(i);
    }
    return bdv;

}


TEST(BSplineExpressionTestSuite, testPositionExpression)
{
    try {
        BSplinePoseDesignVariable bdv = generateRandomSpline();

        EuclideanExpression ep = bdv.position(5.0);

        ExpressionNodeFunctor<EuclideanExpression> functor(ep);

        functor.testJacobian();
    } 
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }
  
}


TEST(BSplineExpressionTestSuite, testRotationExpression)
{
    try {
        BSplinePoseDesignVariable bdv = generateRandomSpline();

        EuclideanExpression p = bdv.position(3.0);
        RotationExpression C = bdv.orientation(5.0);

        EuclideanExpression Cp = C * p;
        ExpressionNodeFunctor<EuclideanExpression> functor(Cp);

        functor.testJacobian();
    } 
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }
  
}



TEST(BSplineExpressionTestSuite, testTransformationExpression)
{
    try {
        HomogeneousPoint hp(Eigen::Vector4d::Random());
        HomogeneousExpression he = hp.toExpression();

        BSplinePoseDesignVariable bdv = generateRandomSpline();


        TransformationExpression T = bdv.transformation(5.0);

        HomogeneousExpression The = T * he;
        ExpressionNodeFunctor<HomogeneousExpression> functor(The);

        SCOPED_TRACE("");
        functor.testJacobian();
    } 
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }
  
}


TEST(BSplineExpressionTestSuite, testAccelerationExpression)
{
    try {
        BSplinePoseDesignVariable bdv = generateRandomSpline();

        EuclideanExpression ep = bdv.linearAcceleration(5.0);

        ExpressionNodeFunctor<EuclideanExpression> functor(ep);

        functor.testJacobian();
    } 
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }
  
}

// aslam::backend::EuclideanExpression linearAcceleration(double tk);

TEST(BSplineExpressionTestSuite, testAngularAccelerationBodyFrameExpression)
{
    try {
        BSplinePoseDesignVariable bdv = generateRandomSpline();

        EuclideanExpression ep = bdv.angularAccelerationBodyFrame(5.0);

        ExpressionNodeFunctor<EuclideanExpression> functor(ep);

        functor.testJacobian();
    }
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }

}
TEST(BSplineExpressionTestSuite, testAngularVelocityExpression)
{
    try {
        BSplinePoseDesignVariable bdv = generateRandomSpline();

        EuclideanExpression ep = bdv.angularVelocityBodyFrame(5.0);

        ExpressionNodeFunctor<EuclideanExpression> functor(ep);

        functor.testJacobian();
    } 
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }
  
}

//BSplineRSPoseDesignVariable generateRandomSplineRS()
//{
//    boost::shared_ptr<EulerRodriguez> rk( new EulerRodriguez );
//    BSplinePose bsplinePose(4, rk);
//    const int N = 10;
//    Eigen::VectorXd times(N);
//    for(int i = 0; i < N; ++i)
//        times(i) = i;
//
//    Eigen::Matrix<double, 6, Eigen::Dynamic> K(6,N);
//    K.setRandom();
//
//
//    bsplinePose.initPoseSpline3(times, K, 6, 1e-4);
//
//    // random linedelay:
//    double lineDelay = double(rand()) / RAND_MAX * 0.005;
//
//    BSplineRSPoseDesignVariable bdv(bsplinePose, lineDelay);
//    for(size_t i = 0; i < bdv.numDesignVariables(); ++i)
//    {
//        bdv.designVariable(i)->setActive(true);
//        bdv.designVariable(i)->setBlockIndex(i);
//    }
//    return bdv;
//
//}
//
//
//
//
//TEST(BSplineExpressionTestSuite, testRSTransformationExpression)
//{
//    try {
//        HomogeneousPoint hp(Eigen::Vector4d::Random());
//        HomogeneousExpression he = hp.toExpression();
//
//        BSplineRSPoseDesignVariable bdv = generateRandomSplineRS();
//
//
//        TransformationExpression T = bdv.transformation(5.0, 100);
//        HomogeneousExpression The = T * he;
//        ExpressionNodeFunctor<HomogeneousExpression> functor(The);
//
//        SCOPED_TRACE("");
//        functor.testJacobian();
//    }
//    catch(const std::exception & e)
//    {
//        FAIL() << e.what();
//    }
//
//}



TEST(BSplineExpressionTestSuite, testTransformationAtTimeExpression)
{
    try {
        HomogeneousPoint hp(Eigen::Vector4d::Random());
        HomogeneousExpression he = hp.toExpression();

        BSplinePoseDesignVariable bdv = generateRandomSpline();

        Scalar s(5.0);
        ScalarExpression se = s.toExpression();
        TransformationExpression T = bdv.transformationAtTime(se, 1.0,1.0);

        HomogeneousExpression The = T * he;
        ExpressionNodeFunctor<HomogeneousExpression> functor(The);

        SCOPED_TRACE("");
        functor.testJacobian();
    } 
    catch(const std::exception & e)
    {
        FAIL() << e.what();
    }
  
}

