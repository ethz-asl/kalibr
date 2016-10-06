// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from libsm
#include <sm/eigen/gtest.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>



TEST(TransformationTestSuite, testConstructor)
{
  using namespace sm::kinematics;

  Transformation T_a_b;
  Eigen::Matrix4d T;
  T.setIdentity();
  
  sm::eigen::assertNear(T, T_a_b.T(), 1e-10, SM_SOURCE_FILE_POS, "Checking for the default constructor creating identity");

}


TEST(TransformationTestSuite, testTV4Multiplication)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      Transformation T_a_b(quatRandom(), Eigen::Vector3d::Random() * 100);
      Eigen::Vector4d v_b;
      v_b.setRandom();
      v_b *= 100.0;
      
      Eigen::Vector4d v_a = T_a_b * v_b;
      Eigen::Vector4d v_a_prime = T_a_b.T() * v_b;
      sm::eigen::assertNear(v_a, v_a_prime, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}

TEST(TransformationTestSuite, testTVhMultiplication)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      Transformation T_a_b(quatRandom(), Eigen::Vector3d::Random() * 100);
      Eigen::Vector3d v_b;
      v_b.setRandom();
      v_b *= 100.0;
      HomogeneousPoint V_b(v_b);
      
      Eigen::Vector3d v_a = T_a_b * v_b;
      HomogeneousPoint V_a = T_a_b * V_b;
      sm::eigen::assertNear(V_a.toEuclidean(), v_a, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}


TEST(TransformationTestSuite, testTVMultiplication)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      Transformation T_a_b(quatRandom(), Eigen::Vector3d::Random() * 100);
      Eigen::Vector3d v_b;
      v_b.setRandom();
      v_b *= 100.0;
      
      Eigen::Vector3d v_a = T_a_b * v_b;
      Eigen::Vector3d v_a_prime = T_a_b.C() * v_b + T_a_b.t();
      sm::eigen::assertNear(v_a, v_a_prime, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}



TEST(TransformationTestSuite, testTTMultiplication)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      Transformation T_a_b(quatRandom(), Eigen::Vector3d::Random() * 100);
      Transformation T_b_c(quatRandom(), Eigen::Vector3d::Random() * 100);
      
      Transformation T_a_c = T_a_b * T_b_c;
      Eigen::Matrix4d T_a_c_prime = T_a_b.T() * T_b_c.T();
      sm::eigen::assertNear(T_a_c.T(), T_a_c_prime, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}


TEST(TransformationTestSuite, testInvert)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      Transformation T_a_b(quatRandom(), Eigen::Vector3d::Random());

      Transformation T_b_a = T_a_b.inverse();
      Transformation T_a_b_prime = T_b_a.inverse();
      
      sm::eigen::assertNear(T_a_b_prime.T(), T_a_b.T(), 1e-10, SM_SOURCE_FILE_POS, "Checking for identity");
  
    }
}

TEST(TransformationTestSuite, testInvertProducesIdentity)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      Transformation T_a_b(quatRandom(), Eigen::Vector3d::Random());      
      Transformation T_b_a = T_a_b.inverse();
      Transformation Eye1 = T_a_b * T_b_a;
      Transformation Eye2 = T_b_a * T_a_b;
      
      sm::eigen::assertNear(Eye1.T(), Eigen::Matrix4d::Identity(), 1e-10, SM_SOURCE_FILE_POS, "Checking for identity");
      sm::eigen::assertNear(Eye2.T(), Eigen::Matrix4d::Identity(), 1e-10, SM_SOURCE_FILE_POS, "Checking for identity");
    }


}
