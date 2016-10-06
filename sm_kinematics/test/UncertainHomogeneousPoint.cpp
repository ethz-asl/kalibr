#include <sm/eigen/gtest.hpp>
#include <sm/kinematics/UncertainHomogeneousPoint.hpp>

#include "PointTestHarness.hpp"

TEST(UncertainHomogeneousPointTestSuite, testUHpoints)
{
  sm::PointTestHarness<sm::kinematics::UncertainHomogeneousPoint> harness(1e-7);
  harness.testAll();
}


TEST(UncertainHomogeneousPointTestSuite, testU3)
{
  using namespace sm::kinematics;
  Eigen::Vector3d p;
  p.setRandom();
  Eigen::Matrix3d U;
  U = sm::eigen::randomCovariance<3>();

  UncertainHomogeneousPoint uhp(p,U);
  
  sm::eigen::assertNear(U, uhp.U3(), 1e-10, SM_SOURCE_FILE_POS, "Checking that I can recover the Euclideam point uncertainty");
}

TEST(UncertainHomogeneousPointTestSuite, testUav)
{
  try {
    using namespace sm::kinematics;
    Eigen::Vector4d p;
    p.setRandom();
    p = p/p.norm();
    Eigen::Matrix3d U;
    U = sm::eigen::randomCovariance<3>();
    
    UncertainHomogeneousPoint uhp(p,U);
    
    sm::eigen::assertNear(U, uhp.U_av_form(), 1e-10, SM_SOURCE_FILE_POS, "Checking that I can recover the av/form point uncertainty");
  } 
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}

TEST(UncertainHomogeneousPointTestSuite, testU3normalized)
{
  using namespace sm::kinematics;
  Eigen::Vector3d p;
  p.setRandom();
  Eigen::Matrix3d U;
  U = sm::eigen::randomCovariance<3>();

  UncertainHomogeneousPoint uhp(p,U);

  uhp.normalize();
  
  ASSERT_NEAR(uhp.toHomogeneous().norm(),1.0,1e-14);
  sm::eigen::assertNear(p, uhp.toEuclidean(),1e-10, SM_SOURCE_FILE_POS, "Checking that I can recover the Euclidean point after normalization");
  sm::eigen::assertNear(U, uhp.U3(), 1e-10, SM_SOURCE_FILE_POS, "Checking that I can recover the Euclideam point uncertainty after normalization.");
}

TEST(UncertainHomogeneousPointTestSuite, testU3normalizedTwice)
{
  using namespace sm::kinematics;
  Eigen::Vector3d p;
  p.setRandom();
  Eigen::Matrix3d U;
  U = sm::eigen::randomCovariance<3>();

  UncertainHomogeneousPoint uhp(p,U);

  uhp.normalize();
  uhp.normalize();

  ASSERT_NEAR(uhp.toHomogeneous().norm(),1.0,1e-14);
  sm::eigen::assertNear(p, uhp.toEuclidean(),1e-8, SM_SOURCE_FILE_POS, "Checking that I can recover the Euclidean point after normalization");
  sm::eigen::assertNear(U, uhp.U3(), 1e-8, SM_SOURCE_FILE_POS, "Checking that I can recover the Euclideam point uncertainty after normalization.");
}
