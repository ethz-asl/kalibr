#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <aslam/backend/test/ExpressionTests.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/MatrixExpression.hpp>
#include <aslam/backend/DesignVariableVector.hpp>
#include <aslam/backend/MapTransformation.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/MatrixBasic.hpp>

using namespace aslam::backend;
using namespace sm::kinematics;

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testPoint)
{
  try 
    {
      SCOPED_TRACE("");
      testExpression(EuclideanPoint(Eigen::Vector3d::Random()).toExpression(), 1);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testRotatedPoint)
{
  try 
    {
      using namespace sm::kinematics;
      RotationQuaternion quat(quatRandom());
      quat.setActive(true);
      quat.setBlockIndex(0);
      RotationExpression C(&quat);


      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(1);
      EuclideanExpression p(&point);
      
      EuclideanExpression Cp = C * p;
      
      SCOPED_TRACE("");
      testJacobian(Cp, 2);

      sm::eigen::assertNear(Cp.toEuclidean(), C.toRotationMatrix() * p.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}



// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testRotatedInverse)
{
  try 
    {
      using namespace sm::kinematics;
      RotationQuaternion quat(quatRandom());
      quat.setActive(true);
      quat.setBlockIndex(0);
      RotationExpression C(&quat);


      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(1);
      EuclideanExpression p(&point);
      
      EuclideanExpression Cp = C.inverse() * p;
      
      SCOPED_TRACE("");
      testJacobian(Cp, 2);

      sm::eigen::assertNear(Cp.toEuclidean(), C.toRotationMatrix().transpose() * p.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
} 


// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testRotationExpression1)
{
  try 
    {
      using namespace sm::kinematics;
      RotationQuaternion quat0(quatRandom());
      quat0.setActive(true);
      quat0.setBlockIndex(0);
      RotationExpression C0(&quat0);

      RotationQuaternion quat1(quatRandom());
      quat1.setActive(true);
      quat1.setBlockIndex(1);
      RotationExpression C1(&quat1);


      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(2);
      EuclideanExpression p(&point);
      
      RotationExpression C01 = C0 * C1;
      EuclideanExpression C01p = C01 * p;
      
      SCOPED_TRACE("");
      testJacobian(C01p, 3);

      sm::eigen::assertNear(C01p.toEuclidean(), C0.toRotationMatrix() * C1.toRotationMatrix() * p.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
} 

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testRotationExpression2)
{
  try 
    {
      using namespace sm::kinematics;
      RotationQuaternion quat0(quatRandom());
      quat0.setActive(true);
      quat0.setBlockIndex(0);
      RotationExpression C0(&quat0);

      RotationQuaternion quat1(quatRandom());
      quat1.setActive(true);
      quat1.setBlockIndex(1);
      RotationExpression C1(&quat1);

      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(2);
      EuclideanExpression p(&point);
      
      EuclideanExpression C1p = C1 * p;
      EuclideanExpression C01p = C0 * C1p;
      
      SCOPED_TRACE("");
      testJacobian(C01p, 3);

      sm::eigen::assertNear(C01p.toEuclidean(), C0.toRotationMatrix() * C1.toRotationMatrix() * p.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
} 

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testEuclideanCrossproduct)
{
  try
    {
      EuclideanPoint point1(Eigen::Vector3d::Random());
      EuclideanExpression p1(&point1);

      EuclideanPoint point2(Eigen::Vector3d::Random());
      EuclideanExpression p2(&point2);

      SCOPED_TRACE("");
      testExpression(p1.cross(p2), 2);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testEuclideanAddition)
{
  try
    {
      EuclideanPoint p1(Eigen::Vector3d::Random());
      EuclideanPoint p2(Eigen::Vector3d::Random());
      SCOPED_TRACE("");
      testExpression(p1.toExpression() + p2.toExpression(), 2);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testEuclideanSubtraction)
{
  try
    {
      EuclideanPoint point1(Eigen::Vector3d::Random());
      EuclideanExpression p1(&point1);

      EuclideanPoint point2(Eigen::Vector3d::Random());
      EuclideanExpression p2(&point2);

      SCOPED_TRACE("");
      testExpression(p1 - p2, 2);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testVectorSubtraction)
{
  try
    {
      EuclideanPoint point1(Eigen::Vector3d::Random());
      EuclideanExpression p1(&point1);

      Eigen::Vector3d p2(Eigen::Vector3d::Random());

      EuclideanExpression p_diff = p1 - p2;

      SCOPED_TRACE("");
      testExpression(p_diff, 1);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testNegation)
{
  try
    {
      EuclideanPoint point1(Eigen::Vector3d::Random());
      point1.setActive(true);
      point1.setBlockIndex(1);
      EuclideanExpression p1(&point1);

      EuclideanExpression pNegated = -p1;

      sm::eigen::assertNear(-point1.toEuclidean(), pNegated.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");

      SCOPED_TRACE("");
      testExpression(pNegated, 1);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testEuclideanDirection)
{
  try
    {
      Eigen::Vector3d p = Eigen::Vector3d::Random() * 10;
      EuclideanDirection point1(p);
      ASSERT_DOUBLE_MX_EQ(p, point1.toEuclidean(),1e-2, "Checking if the euclidean point is recovered correctly")

      SCOPED_TRACE("");
      testExpression(point1.toExpression(), 1);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testAdaptedVectorExpression)
{
  try
    {
      EuclideanPoint point1(Eigen::Vector3d::Random());
      EuclideanExpression p1(&point1);

      DesignVariableVector<3> point2(Eigen::Vector3d::Random());
      EuclideanExpression p2(&point2);

      EuclideanExpression p_add = p1 + p2;

      SCOPED_TRACE("");
      testExpression(p_add, 2);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testMatrixTransformedPoint)
{
  try
    {
      MatrixBasic a(Eigen::Matrix3d::Random());
      a.setActive(true);
      a.setBlockIndex(0);
      MatrixExpression A(&a);


      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(1);
      EuclideanExpression p(&point);

      EuclideanExpression Ap = A * p;

      SCOPED_TRACE("");
      testJacobian(Ap);

      sm::eigen::assertNear(Ap.toEuclidean(), A.toMatrix3x3() * p.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the matrix is updated correctly.
TEST(EuclideanExpressionNodeTestSuites, testMatrixUpdate)
{
  try
    {
      Eigen::Matrix3i updatePattern;
      updatePattern << 1, 1, 1,
                      0, 1, 1,
                      0, 0, 1;
      Eigen::Matrix3d dataMatrix = Eigen::Matrix3d::Random();
      MatrixBasic a(dataMatrix, updatePattern);

      double update[6] = {.1, .2, .3, .4, .5, .6};
      Eigen::Matrix3d updateMatrix;
      updateMatrix << .1, .2, .3,
                      0., .4, .5,
                      0., 0., .6;

      SCOPED_TRACE("");

      a.updateImplementation(update, 6);

      sm::eigen::assertNear(a.toMatrix3x3(), dataMatrix + updateMatrix, 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testMatrixdoubleTransformedPoint)
{
  try
    {
      MatrixBasic a(Eigen::Matrix3d::Random());
      a.setActive(true);
      a.setBlockIndex(0);
      MatrixExpression A(&a);

      MatrixBasic b(Eigen::Matrix3d::Random());
      b.setActive(true);
      b.setBlockIndex(1);
      MatrixExpression B(&b);


      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(2);
      EuclideanExpression p(&point);

      EuclideanExpression Ap = B * (A * p);

      SCOPED_TRACE("");
      testJacobian(Ap, 3);

      sm::eigen::assertNear(Ap.toEuclidean(), B.toMatrix3x3() * (A.toMatrix3x3() * p.toEuclidean()), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testDiagonalMatrixTransformedPoint)
{
  try
    {
      Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
      S(0,0)= 10*(float)drand48();
      S(1,1)= 10*(float)drand48();
      S(2,2)= 10*(float)drand48();
      MatrixBasic a(S,Eigen::Matrix3i::Identity());
      a.setActive(true);
      a.setBlockIndex(0);
      MatrixExpression A(&a);


      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(1);
      EuclideanExpression p(&point);

      EuclideanExpression Ap = A * p;

      SCOPED_TRACE("");
      testJacobian(Ap, 2);

      sm::eigen::assertNear(Ap.toEuclidean(), A.toMatrix3x3() * p.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testLowerTriangleMatrixTransformedPoint)
{
  try
    {
      Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
      Eigen::Matrix3i M_pattern = Eigen::Matrix3i::Zero();
      M(1,0)= 10*(float)drand48();
      M(2,0)= 10*(float)drand48();
      M(2,1)= 10*(float)drand48();
      // set the entries, that will be estimated by the calibration to 1
      M_pattern(1,0)= 1;
      M_pattern(2,0)= 1;
      M_pattern(2,1)= 1;
      MatrixBasic a(M,M_pattern);
      a.setActive(true);
      a.setBlockIndex(0);
      MatrixExpression A(&a);

      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(1);
      EuclideanExpression p(&point);

      EuclideanExpression Ap = A * p;

      SCOPED_TRACE("");
      testJacobian(Ap, 2);

      sm::eigen::assertNear(Ap.toEuclidean(), A.toMatrix3x3() * p.toEuclidean(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testLowerMixedMatrixTransformedPoint)
{
  try
    {
      Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
      Eigen::Matrix3i M_pattern = Eigen::Matrix3i::Zero();
      M(1,0)= 10*(float)drand48();
      M(2,0)= 10*(float)drand48();
      M(2,1)= 10*(float)drand48();
      // set the entries, that will be estimated by the calibration to 1
      M_pattern(1,0)= 1;
      M_pattern(2,0)= 1;
      M_pattern(2,1)= 1;
      MatrixBasic a(M,M_pattern);
      a.setActive(true);
      a.setBlockIndex(0);
      MatrixExpression A(&a);
      sm::eigen::assertNear(M, A.toMatrix3x3(), 1e-14, SM_SOURCE_FILE_POS, "Testing the initial matrix is good");

      Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
      S(0,0)= 10*(float)drand48();
      S(1,1)= 10*(float)drand48();
      S(2,2)= 10*(float)drand48();
      MatrixBasic b(S,Eigen::Matrix3i::Identity());
      b.setActive(true);
      b.setBlockIndex(1);
      MatrixExpression B(&b);

      sm::eigen::assertNear(S, B.toMatrix3x3(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");


      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(2);
      EuclideanExpression p(&point);

      EuclideanExpression Ap = B * (A * p);

      SCOPED_TRACE("");
      testJacobian(Ap, 3);

      sm::eigen::assertNear(Ap.toEuclidean(), B.toMatrix3x3() * A.toMatrix3x3() * p.toEuclidean(), 1e-13, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}





// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testTransformationTransformedPoint)
{
  try
    {
      Transformation T;
      boost::shared_ptr<MappedRotationQuaternion> outQ;
      boost::shared_ptr<MappedEuclideanPoint> outT;
      T.setRandom();
      TransformationExpression A(transformationToExpression(T,outQ,outT));
      outQ->setActive(true);
      outQ->setBlockIndex(0);
      outT->setActive(true);
      outT->setBlockIndex(1);

      
      EuclideanExpression p = A.toEuclideanExpression();

      SCOPED_TRACE("");
      testJacobian(p);
      SCOPED_TRACE("");
      sm::eigen::assertNear(p.toEuclidean(), A.toTransformationMatrix().topRightCorner<3,1>(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testRotationParameters)
{
  try
    {
      RotationQuaternion quat(quatRandom());
      quat.setActive(true);
      quat.setBlockIndex(0);
      RotationExpression qr(&quat);
      RotationalKinematics::Ptr ypr( new EulerAnglesYawPitchRoll());
      
      EuclideanExpression p = qr.toParameters(ypr);

      SCOPED_TRACE("");
      testJacobian(p);
      SCOPED_TRACE("");
      sm::eigen::assertNear(p.toEuclidean(), ypr->rotationMatrixToParameters(qr.toRotationMatrix()), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(EuclideanExpressionNodeTestSuites, testToHomogeneous)
{
  try 
    {
      aslam::backend::HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(true);
      point.setBlockIndex(0);
      HomogeneousExpression qr(&point);
      EuclideanExpression p = qr.toEuclideanExpression();
      
      SCOPED_TRACE("");
      testJacobian(p);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

