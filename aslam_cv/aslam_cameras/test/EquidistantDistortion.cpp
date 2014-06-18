// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras/FiniteDifferences.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <sm/assert_macros.hpp>

TEST(AslamCamerasTestSuite, testEquiDistortion1)
{
  using namespace aslam::cameras;
  EquidistantDistortion d(-0.00185, 0.01901, -0.02422, 0.01429);
  Eigen::MatrixXd Jd, estJd;

  Eigen::Vector2d x0(1.0E-1,1.0E-1);
  Eigen::Vector2d x1(1.0E-1,1.0E-1);
  d.distort(x1);
  d.undistort(x1);
  ASSERT_DOUBLE_MX_EQ(x0,x1,0.1,"");

  Eigen::Vector2d x4(1.0E-1,1.0E-1);
  d.distortParameterJacobian(x4,Jd);
  ASLAM_CAMERAS_ESTIMATE_DISTORTION_JACOBIAN(distort, d, x4, 1e-3, estJd);
  ASSERT_DOUBLE_MX_EQ(Jd,estJd,1.0,"");
}
