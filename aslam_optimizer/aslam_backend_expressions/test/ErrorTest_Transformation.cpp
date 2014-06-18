#include <gtest/gtest.h>

#include <aslam/backend/ErrorTermTransformation.hpp>
#include <aslam/backend/test/RotationQuaternionAsVectorExpressionNode.hpp>
// This test harness makes it easy to test error terms.
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>


TEST(AslamVChargeBackendTestSuite, testTransformation)
{
  try {
      using namespace aslam::backend;
    sm::kinematics::Transformation T_random;
    T_random.setRandom(0.05, 0.01);
    sm::kinematics::Transformation T_prior;
    
    RotationQuaternion quat(T_prior.q());
    quat.setActive(true);
    quat.setBlockIndex(0);
    RotationExpression rexp(&quat);
    
    EuclideanPoint ep(T_prior.t());
    ep.setActive(true);
    ep.setBlockIndex(1);
    EuclideanExpression eexp(&ep);
    TransformationBasic Tb(rexp, eexp);
    TransformationExpression T(&Tb);

     
        
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(6,6);
    N(0,0) = 1e-3;
    N(1,1) = 1e-3;
    N(2,2) = 1e-3;
    N(3,3) = 1e0;
    N(4,4) = 1e0;
    N(5,5) = 1e0;

    // Create the ErrorTerm
    ErrorTermTransformation ett(T, T_random, N);
    // Create the test harness
    aslam::backend::ErrorTermTestHarness<6> harness(&ett);

    // Run the unit tests.
    harness.testAll(1e-5);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

TEST(AslamVChargeBackendTestSuite, testTransformationWithAdapter)
{
  try {
      using namespace aslam::backend;
    sm::kinematics::Transformation T_random;
    T_random.setRandom(0.05, 0.01);
    sm::kinematics::Transformation T_prior;

    RotationQuaternion quatVar(T_prior.q());
    quatVar.setActive(true);
    quatVar.setBlockIndex(0);

    RotationQuaternionAsVectorExpressionNode<> quat(quatVar);

    RotationExpression rexp(Vector2RotationQuaternionExpressionAdapter::adapt(VectorExpression<4>(&quat)));

    EuclideanPoint ep(T_prior.t());
    ep.setActive(true);
    ep.setBlockIndex(1);
    EuclideanExpression eexp(&ep);
    TransformationBasic Tb(rexp, eexp);
    TransformationExpression T(&Tb);

    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(6,6);
    N(0,0) = 1e-3;
    N(1,1) = 1e-3;
    N(2,2) = 1e-3;
    N(3,3) = 1e0;
    N(4,4) = 1e0;
    N(5,5) = 1e0;

    // Create the ErrorTerm
    ErrorTermTransformation ett(T, T_random, N);
    // Create the test harness
    aslam::backend::ErrorTermTestHarness<6> harness(&ett);

    // Run the unit tests.
    harness.testAll(1e-5);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}
