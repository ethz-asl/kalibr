#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <aslam/backend/test/RotationExpressionTests.hpp>
#include <aslam/backend/test/RotationQuaternionAsVectorExpressionNode.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/DesignVariableVector.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <aslam/backend/MapTransformation.hpp>


using namespace aslam::backend;
using namespace sm::kinematics;

// Test that the quaternion jacobian matches the finite difference jacobian
TEST(RotationExpressionNodeTestSuites, testQuat)
{
  try 
    {
      using namespace sm::kinematics;
      RotationQuaternion quat(quatRandom());
      quat.setActive(true);
      quat.setBlockIndex(0);
      RotationExpression qr(&quat);
      
      testJacobian(qr);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the quaternion jacobian matches the finite difference jacobian
TEST(RotationExpressionNodeTestSuites, testFromTransformation)
{
  try 
    {
      using namespace sm::kinematics;
      Transformation T;
      boost::shared_ptr<MappedRotationQuaternion> outQ;
      boost::shared_ptr<MappedEuclideanPoint> outT;
      T.setRandom();
      TransformationExpression A(transformationToExpression(T,outQ,outT));
      outQ->setActive(true);
      outQ->setBlockIndex(0);
      outT->setActive(true);
      outT->setBlockIndex(1);

      RotationExpression qr = A.toRotationExpression();
      
      testJacobian(qr);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}


// Test that the inverse quaternion matches the finite difference.
TEST(RotationExpressionNodeTestSuites, testQuatInverse1)
{
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat(quatRandom());
    quat.setActive(true);
    quat.setBlockIndex(0);
    RotationExpression qr(&quat);

    RotationExpression invqr = qr.inverse();
    
    testJacobian(invqr);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

// Test that the inverse computes the correct result.
TEST(RotationExpressionNodeTestSuites, testQuatInverse2)
{
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat(quatRandom());
    quat.setActive(true);
    quat.setBlockIndex(0);
    RotationExpression qr(&quat);

    RotationExpression dvInverse = qr.inverse();
    
    Eigen::Matrix3d invQ1 = dvInverse.toRotationMatrix();
    Eigen::Matrix3d invQ2 = qr.toRotationMatrix().transpose();

    sm::eigen::assertNear(invQ1, invQ2, 1e-14, SM_SOURCE_FILE_POS, "Test that the inverse computes the correct result");
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}


TEST(RotationExpressionNodeTestSuites, testQuatMultiply1)
{
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat(quatRandom());
    quat.setActive(true);
    quat.setBlockIndex(0);
    RotationExpression qr(&quat);

    RotationExpression dvMultiply = qr * qr;
    
    testJacobian(dvMultiply);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}


// Test that the binary multiplication produces the correct result.
TEST(RotationExpressionNodeTestSuites, testQuatMultiply2)
{
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat0(quatRandom());
    quat0.setActive(true);
    quat0.setBlockIndex(0);
    RotationExpression qr0(&quat0);

    RotationQuaternion quat1(quatRandom());
    quat1.setActive(true);
    quat1.setBlockIndex(1);
    RotationExpression qr1(&quat1);
    RotationExpression dvMultiply = qr0 * qr1;

    sm::eigen::assertEqual(qr0.toRotationMatrix() * qr1.toRotationMatrix(), dvMultiply.toRotationMatrix(), SM_SOURCE_FILE_POS, "Test multiplications");
    
    testJacobian(dvMultiply);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}


/// Test that 3x multiplication (q0,q1) q2 produces the correct result
TEST(RotationExpressionNodeTestSuites, testQuatMultiply3a)
{
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat0(quatRandom());
    quat0.setActive(true);
    quat0.setBlockIndex(0);
    RotationExpression qr0(&quat0);

    RotationQuaternion quat1(quatRandom());
    quat1.setActive(true);
    quat1.setBlockIndex(1);
    RotationExpression qr1(&quat1);

    RotationQuaternion quat2(quatRandom());
    quat2.setActive(true);
    quat2.setBlockIndex(2);
    RotationExpression qr2(&quat2);

    RotationExpression dvMultiply = qr2 * (qr0 * qr1);
    

    sm::eigen::assertNear(qr2.toRotationMatrix() * qr0.toRotationMatrix() * qr1.toRotationMatrix(), dvMultiply.toRotationMatrix(), 1e-14, SM_SOURCE_FILE_POS, "Test multiplications");
    
    testJacobian(dvMultiply);

  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}


/// Test that 3x multiplication (q0,q1) q2 and q0 (q1 q2) produce the same result
 TEST(RotationExpressionNodeTestSuites,stQuatMulty3b)
 {
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat0(quatRandom());
    quat0.setActive(true);
    quat0.setBlockIndex(0);
    RotationExpression qr0(&quat0);

    RotationQuaternion quat1(quatRandom());
    quat1.setActive(true);
    quat1.setBlockIndex(1);
    RotationExpression qr1(&quat1);

    RotationQuaternion quat2(quatRandom());
    quat2.setActive(true);
    quat2.setBlockIndex(2);
    RotationExpression qr2(&quat2);

    RotationExpression dvMultiply01 = qr0 * qr1;
   
    // Right multiplication:
    RotationExpression dvMultiply02Right = dvMultiply01 * qr2; 

    RotationExpression dvMultiply12 = qr1 * qr2;    
    RotationExpression dvMultiply02Left = qr0 * dvMultiply12;    

    
    sm::eigen::assertNear( dvMultiply02Left.toRotationMatrix(), dvMultiply02Right.toRotationMatrix(), 1e-14, SM_SOURCE_FILE_POS, "Test multiplications");
    
    JacobianContainer JcLeft(3);
    dvMultiply02Left.evaluateJacobians(JcLeft);

    JacobianContainer JcRight(3);
    dvMultiply02Right.evaluateJacobians(JcRight);
    
    sm::eigen::assertNear( JcLeft.asSparseMatrix(), JcRight.asSparseMatrix(), 1e-14, SM_SOURCE_FILE_POS, "Test multiplications");
    

  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

/// Test that 3x multiplication  q0 (q1 q2) produces the correct result
 TEST(RotationExpressionNodeTestSuites,stQuatMulty3c)
 {
  try {
     using namespace sm::kinematics;
    RotationQuaternion quat0(quatRandom());
    quat0.setActive(true);
    quat0.setBlockIndex(0);
    RotationExpression qr0(&quat0);

    RotationQuaternion quat1(quatRandom());
    quat1.setActive(true);
    quat1.setBlockIndex(1);
    RotationExpression qr1(&quat1);

    RotationQuaternion quat2(quatRandom());
    quat2.setActive(true);
    quat2.setBlockIndex(2);
    RotationExpression qr2(&quat2);

    RotationExpression dvMultiply = qr0 * qr1;
   
    // Right multiplication:
    RotationExpression dvMultiplyRight = dvMultiply * qr2; 

    sm::eigen::assertNear( qr0.toRotationMatrix() * qr1.toRotationMatrix() * qr2.toRotationMatrix(), dvMultiplyRight.toRotationMatrix(), 1e-14, SM_SOURCE_FILE_POS, "Test multiplications");

    testJacobian(dvMultiplyRight);

  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}


/// Test that 3x multiplication  q0 (q1 q2)^-1 produces the correct result
 TEST(RotationExpressionNodeTestSuites,stQuatMultInv1)
 {
  try {
     using namespace sm::kinematics;
    RotationQuaternion quat0(quatRandom());
    quat0.setActive(true);
    quat0.setBlockIndex(0);
    RotationExpression qr0(&quat0);

    RotationQuaternion quat1(quatRandom());
    quat1.setActive(true);
    quat1.setBlockIndex(1);
    RotationExpression qr1(&quat1);

    RotationQuaternion quat2(quatRandom());
    quat2.setActive(true);
    quat2.setBlockIndex(2);
    RotationExpression qr2(&quat2);

    RotationExpression dvMultiply01 = qr0 * qr1;
   
    RotationExpression dvInverse10 = dvMultiply01.inverse();


    // Right multiplication:
    RotationExpression dvMultiplyRight210 = qr2 * dvInverse10; 

    sm::eigen::assertNear( qr2.toRotationMatrix() * (qr0.toRotationMatrix() * qr1.toRotationMatrix()).transpose(), dvMultiplyRight210.toRotationMatrix(), 1e-14, SM_SOURCE_FILE_POS, "Test multiplications");

    testJacobian(dvMultiplyRight210);

  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

/// Test that 3x multiplication  q0^1 (q1 q2) produces the correct result
 TEST(RotationExpressionNodeTestSuites,stQuatMultInv2)
 {
  try {
     using namespace sm::kinematics;
    RotationQuaternion quat0(quatRandom());
    quat0.setActive(true);
    quat0.setBlockIndex(0);
    RotationExpression qr0(&quat0);

    RotationQuaternion quat1(quatRandom());
    quat1.setActive(true);
    quat1.setBlockIndex(1);
    RotationExpression qr1(&quat1);

    RotationQuaternion quat2(quatRandom());
    quat2.setActive(true);
    quat2.setBlockIndex(2);
    RotationExpression qr2(&quat2);

    RotationExpression dvMultiply01 = qr0 * qr1;
   
    RotationExpression dvInverse2 = qr2.inverse();


    // Right multiplication:
    RotationExpression dvMultiplyRight210 = dvInverse2 * dvMultiply01; 

    sm::eigen::assertNear( qr2.toRotationMatrix().transpose() * (qr0.toRotationMatrix() * qr1.toRotationMatrix()), dvMultiplyRight210.toRotationMatrix(), 1e-14, SM_SOURCE_FILE_POS, "Test multiplications");

    testJacobian(dvMultiplyRight210);

  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

//Test that the inverse quaternion matches the finite difference.
TEST(RotationExpressionNodeTestSuites, testQuatInverseTemplate1)
{
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat(quatRandom());
    quat.setActive(true);
    quat.setBlockIndex(0);
    RotationExpression qr(&quat);
    
    RotationExpression qi = qr.inverse();
    testJacobian(qi);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

TEST(RotationExpressionNodeTestSuites, testMinimalDifference)
{
  try {
    using namespace sm::kinematics;
    Eigen::Vector4d initialValue = quatRandom();
    RotationQuaternion quat(initialValue);
    quat.setActive(true);
    quat.setBlockIndex(0);

    Eigen::Vector4d epsQ = quatRandom();
    Eigen::Vector3d eps = sm::kinematics::qlog(epsQ);
    double updateValues[3] = {eps(0), eps(1), eps(2)};
    quat.update(updateValues, 3);

    Eigen::VectorXd diff;
    quat.minimalDifference(initialValue, diff);

    EXPECT_TRUE(eps.isApprox(diff));

  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

TEST(RotationExpressionNodeTestSuites, testMinimalDifferenceAndJacobian)
{
  try {
    using namespace sm::kinematics;
    Eigen::Vector4d quatInitial = quatRandom();
    RotationQuaternion quat(quatInitial);
    quat.setActive(true);
    quat.setBlockIndex(0);

    Eigen::Vector4d updatedQuat = quatRandom();
    Eigen::Vector3d updateAA = sm::kinematics::qlog(updatedQuat);
    double updateValues[3] = {updateAA(0), updateAA(1), updateAA(2)};
    quat.update(updateValues, 3);

    //std::cout << "initial value is " << std::endl << quatInitial << std::endl;

    Eigen::MatrixXd quatHat = Eigen::MatrixXd(4,1);
    quatHat(0,0) = quatInitial(0); quatHat(1,0) = quatInitial(1); quatHat(2,0) = quatInitial(2,0); quatHat(3,0) = quatInitial(3,0);


    Eigen::MatrixXd params;
    quat.getParameters(params);
    Eigen::Vector4d quatBar;
    quatBar(0) = params(0,0); quatBar(1) = params(1,0); quatBar(2) = params(2,0); quatBar(3) = params(3,0);

    Eigen::VectorXd minDist;
    Eigen::MatrixXd M;
    quat.minimalDifferenceAndJacobian(quatInitial, minDist, M);

    // choose small
    Eigen::Vector3d epsV; epsV.setRandom();epsV = epsV * 0.05;//epsV(0) = 0.001; epsV(1) = 0.001; epsV(2) = 0.001;
    double eps[3] = {epsV(0), epsV(1), epsV(2)};
    quat.update(eps, 3);

    Eigen::MatrixXd params2;
    quat.getParameters(params2);
    Eigen::Vector4d quatFinal;
    quatFinal(0) = params2(0,0); quatFinal(1) = params2(1,0); quatFinal(2) = params2(2,0); quatFinal(3) = params2(3,0);

    Eigen::Vector3d realMinDist = sm::kinematics::qlog(sm::kinematics::qplus(quatFinal, sm::kinematics::quatInv(quatInitial)));

    Eigen::Vector3d estMinDist = minDist + M*epsV;

//    std::cout << "realMinDist" << std::endl << realMinDist << std::endl;
//    std::cout << "estMinDist" << std::endl << estMinDist << std::endl;

    sm::eigen::assertNear(realMinDist, estMinDist, 1e-2, SM_SOURCE_FILE_POS, "Test min difference with jacobian");
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

// Test that the binary multiplication produces the correct result.
TEST(RotationExpressionNodeTestSuites, testVector2RotationQuaternionExpressionAdapter)
{
  try {
    using namespace sm::kinematics;
    RotationQuaternion quat0(quatRandom());
    quat0.setActive(true);
    quat0.setBlockIndex(0);
    RotationExpression qr0(&quat0);

    RotationQuaternion quat1Var(quatRandom());

    quat1Var.setActive(true);
    quat1Var.setBlockIndex(1);

    RotationQuaternionAsVectorExpressionNode<> quat1(quat1Var);

    RotationExpression qr1 = Vector2RotationQuaternionExpressionAdapter::adapt(VectorExpression<4>(&quat1));
    sm::eigen::assertEqual(qr1.toRotationMatrix(), quat1Var.toRotationMatrix(), SM_SOURCE_FILE_POS, "Test multiplications with adapter");
    {
      SCOPED_TRACE("");
      testJacobian(qr1);
    }

    RotationExpression dvMultiply = qr0 * qr1;
    sm::eigen::assertEqual(qr0.toRotationMatrix() * qr1.toRotationMatrix(), dvMultiply.toRotationMatrix(), SM_SOURCE_FILE_POS, "Test multiplications with adapter");
    {
      SCOPED_TRACE("");
      testJacobian(dvMultiply);
    }
  }
  catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}
