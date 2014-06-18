// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras/FiniteDifferences.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <sm/assert_macros.hpp>

TEST(AslamCamerasTestSuite, testRTDistortion1)
{
  using namespace aslam::cameras;
  Eigen::Vector2d k(4e-3,4e-3);
  Eigen::Vector2d kd = k;
  Eigen::MatrixXd Jd, estJd;

  RadialTangentialDistortion d(-0.2, 0.13, 0.0005, 0.0005);

  d.distortParameterJacobian(kd,Jd);

  ASLAM_CAMERAS_ESTIMATE_DISTORTION_JACOBIAN(distort, d, k, 1e-3, estJd);

  d.distortParameterJacobian(kd,Jd);
  ASSERT_DOUBLE_MX_EQ(Jd,estJd,1.0,"");

}
