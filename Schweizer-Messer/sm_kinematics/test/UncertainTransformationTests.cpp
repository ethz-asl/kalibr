// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from libsm
#include <sm/eigen/gtest.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/UncertainTransformation.hpp>



TEST(UncertainTransformationTestSuite, testConstructor)
{
  using namespace sm::kinematics;

  UncertainTransformation T_a_b;
  Eigen::Matrix4d T;
  T.setIdentity();
  
  UncertainTransformation::covariance_t U;
  U.setZero();
  
  sm::eigen::assertNear(T, T_a_b.T(), 1e-10, SM_SOURCE_FILE_POS, "Checking for the default constructor creating identity");
  sm::eigen::assertNear(U, T_a_b.U(), 1e-10, SM_SOURCE_FILE_POS, "Checking for the default constructor creating zero uncertainty");


}


TEST(UncertainTransformationTestSuite, testTV4Multiplication)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      UncertainTransformation T_a_b;
      T_a_b.setRandom();
      Eigen::Vector4d v_b;
      v_b.setRandom();
      v_b *= 100.0;
      
      Eigen::Vector4d v_a = T_a_b * v_b;
      Eigen::Vector4d v_a_prime = T_a_b.T() * v_b;
      sm::eigen::assertNear(v_a, v_a_prime, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}

TEST(UncertainTransformationTestSuite, testTVhMultiplication)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      UncertainTransformation T_a_b;
      T_a_b.setRandom();
      Eigen::Vector3d v_b;
      v_b.setRandom();
      v_b *= 100.0;
      HomogeneousPoint V_b(v_b);
      
      Eigen::Vector3d v_a = T_a_b * v_b;
      HomogeneousPoint V_a = T_a_b * V_b;
      sm::eigen::assertNear(V_a.toEuclidean(), v_a, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}


TEST(UncertainTransformationTestSuite, testTVMultiplication)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      UncertainTransformation T_a_b;
      T_a_b.setRandom();
      Eigen::Vector3d v_b;
      v_b.setRandom();
      v_b *= 100.0;
      
      Eigen::Vector3d v_a = T_a_b * v_b;
      Eigen::Vector3d v_a_prime = T_a_b.C() * v_b + T_a_b.t();
      sm::eigen::assertNear(v_a, v_a_prime, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}



TEST(UncertainTransformationTestSuite, testTTMultiplication)
{
  try {
    using namespace sm::kinematics;

    for(int i = 0; i < 100; i++)
      {
	UncertainTransformation T_a_b;
	T_a_b.setRandom();

	UncertainTransformation T_b_c;
	T_b_c.setRandom();

	UncertainTransformation T_a_c = T_a_b * T_b_c;
	Eigen::Matrix4d T_a_c_prime = T_a_b.T() * T_b_c.T();
	sm::eigen::assertNear(T_a_c.T(), T_a_c_prime, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
      }

  } catch(const std::exception & e)
    {
      FAIL() << e.what();
    }


}


TEST(UncertainTransformationTestSuite, testInvert)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      UncertainTransformation T_a_b;
      T_a_b.setRandom();

      UncertainTransformation T_b_a = T_a_b.inverse();
      UncertainTransformation T_a_b_prime = T_b_a.inverse();
      
      sm::eigen::assertNear(T_a_b_prime.T(), T_a_b.T(), 1e-10, SM_SOURCE_FILE_POS, "Checking for identity");
  
    }
}

TEST(UncertainTransformationTestSuite, testInvertProducesIdentity)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 100; i++)
    {
      UncertainTransformation T_a_b;
      T_a_b.setRandom();
      UncertainTransformation T_b_a = T_a_b.inverse();
      UncertainTransformation Eye1 = T_a_b * T_b_a;
      UncertainTransformation Eye2 = T_b_a * T_a_b;
      
      sm::eigen::assertNear(Eye1.T(), Eigen::Matrix4d::Identity(), 1e-10, SM_SOURCE_FILE_POS, "Checking for identity");
      sm::eigen::assertNear(Eye2.T(), Eigen::Matrix4d::Identity(), 1e-10, SM_SOURCE_FILE_POS, "Checking for identity");
    }


}




TEST(UncertainTransformationTestSuite, testInverse)
{
 using namespace sm::kinematics;
  UncertainTransformation TT_01;
  TT_01.setRandom();
  UncertainTransformation TT_10 = TT_01.inverse();
  UncertainTransformation TTT_01 = TT_10.inverse();

  Eigen::Matrix4d r1_T_01 = TT_01.T();
  Eigen::Matrix4d r2_T_01 = TTT_01.T();

  sm::eigen::assertNear(r1_T_01, r2_T_01, 1e-10, SM_SOURCE_FILE_POS, "Testing that composition and inverse derive the same answer");

  UncertainTransformation::covariance_t r1_U_01 = TT_01.U();
  UncertainTransformation::covariance_t r2_U_01 = TTT_01.U();
  sm::eigen::assertNear(r1_U_01, r2_U_01, 1e-10, SM_SOURCE_FILE_POS, "Testing that composition and inverse derive the same answer");
  
}



TEST(UncertainTransformationTestSuite, testComposition2)
{

 using namespace sm::kinematics;
  UncertainTransformation TT_01;
  TT_01.setRandom();
  UncertainTransformation TT_12;
  TT_12.setRandom();
  UncertainTransformation TT_02 = TT_01 * TT_12;

  UncertainTransformation TT_10 = TT_01.inverse();
  UncertainTransformation TT_21 = TT_12.inverse();
  UncertainTransformation TT_20 = TT_21 * TT_10;

  UncertainTransformation TT_20direct = TT_02.inverse();

  Eigen::Matrix4d r1_T_20 = TT_20.T();
  Eigen::Matrix4d r2_T_20 = TT_20direct.T();

  sm::eigen::assertNear(r1_T_20, r2_T_20, 1e-10, SM_SOURCE_FILE_POS, "Testing that composition and inverse derive the same answer");

  UncertainTransformation::covariance_t r1_U_20 = TT_20.U();
  UncertainTransformation::covariance_t r2_U_20 = TT_20direct.U();
  sm::eigen::assertNear(r1_U_20, r2_U_20, 1e-8, SM_SOURCE_FILE_POS, "Testing that composition and inverse derive the same answer");
  
}



TEST(UncertainTransformationTestSuite, testTVMultiplication2)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 10; i++)
    {
      UncertainTransformation T_a_b;
      T_a_b.setRandom();

      UncertainHomogeneousPoint v_b;
      v_b.setRandom();
      
      UncertainHomogeneousPoint v_a = T_a_b * v_b;
      Eigen::Vector3d v_a_prime = T_a_b.C() * v_b.toEuclidean() + T_a_b.t();
      sm::eigen::assertNear(v_a.toEuclidean(), v_a_prime, 1e-10, SM_SOURCE_FILE_POS, "Checking for composition equal to matrix multiplication");
    }


}

// lestefan: added this
TEST(UncertainTransformationTestSuite, testUOplus)
{
  using namespace sm::kinematics;

  for(int i = 0; i < 10; i++)
    {
      UncertainTransformation T_a_b_1;
      T_a_b_1.setRandom();

      UncertainTransformation::covariance_t UOplus = T_a_b_1.UOplus();
      
      UncertainTransformation T_a_b_2=T_a_b_1;
      T_a_b_2.setUOplus(UOplus);
      sm::eigen::assertNear(T_a_b_1.U(), T_a_b_2.U(), 1e-10, SM_SOURCE_FILE_POS, "Checking for getting and setting OPlus-type uncertainties");
    }


}
