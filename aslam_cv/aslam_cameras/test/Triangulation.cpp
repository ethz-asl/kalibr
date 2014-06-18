// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras/Triangulation.hpp>
#include <sm/eigen/assert_macros.hpp>
#include <sm/eigen/gtest.hpp>

TEST(AslamCamerasTestSuite, testTriangulationIdeal)
{

  Eigen::Vector3d p0;
  p0.setRandom();
  p0 = p0 * 10.0;

  Eigen::Vector3d p1;
  p1.setRandom();
  p1 = p1 * 10.0;

  // The triangulated point
  Eigen::Vector3d tp;
  tp.setRandom();
  tp = tp * 10.0;

  // Make the ray not unit length just
  // to keep things interesting.
  Eigen::Vector3d r1;
  r1 = (tp - p1) * 0.1;

  Eigen::Vector3d r0;
  r0 = (tp - p0) * 0.1;

  Eigen::Vector3d outTriangulatedPoint;
  double outGap;
  double outS0;
  double outS1;

  aslam::cameras::triangulate( p0, r0, p1, r1,
      outTriangulatedPoint,
      outGap,
      outS0,
      outS1);

  ASSERT_DOUBLE_MX_EQ( outTriangulatedPoint, tp, 1e-6, "The triangulated point does not match");
  ASSERT_DOUBLE_MX_EQ( (p0 + (outS0 * r0)), tp, 1e-6, "The triangulated point does not match");
  ASSERT_DOUBLE_MX_EQ( (p1 + (outS1 * r1)), tp, 1e-6, "The triangulated point does not match");
  ASSERT_LT( fabs(outGap), 1e-6 );

}

TEST(AslamCamerasTestSuite, testTriangulationNoisy)
{

  Eigen::Vector3d p0;
  p0.setRandom();
  p0 = p0 * 10.0;

  Eigen::Vector3d p1;
  p1.setRandom();
  p1 = p1 * 10.0;

  // The triangulated point
  Eigen::Vector3d tp1;
  tp1.setRandom();
  tp1 = tp1 * 10.0;

  Eigen::Vector3d tp0 = tp1 + Eigen::Vector3d::Random() * 0.1;

  // Make the ray not unit length just
  // to keep things interesting.
  Eigen::Vector3d r1;
  r1 = (tp1 - p1) * 0.1;

  Eigen::Vector3d r0;
  r0 = (tp0 - p0) * 0.1;

  Eigen::Vector3d outTriangulatedPoint;
  double outGap;
  double outS0;
  double outS1;

  // Note: I could construct a better unit test here where I know the answer exactly
  // but just this miniute I can't think of how.
  aslam::cameras::triangulate( p0, r0, p1, r1,
      outTriangulatedPoint,
      outGap,
      outS0,
      outS1);

  double gap = ((p0 + (outS0 * r0)) - (p1 + (outS1 * r1))).norm();
  ASSERT_DOUBLE_MX_EQ( outTriangulatedPoint, (p0 + (outS0 * r0) + p1 + (outS1 * r1))*0.5, 1, "The triangulated point does not match");
  ASSERT_DOUBLE_MX_EQ( (p0 + (outS0 * r0)), tp0, 10, "The triangulated point does not match");
  ASSERT_DOUBLE_MX_EQ( (p1 + (outS1 * r1)), tp1, 10, "The triangulated point does not match");
  ASSERT_NEAR( outGap, gap, 1e-6 );

}
