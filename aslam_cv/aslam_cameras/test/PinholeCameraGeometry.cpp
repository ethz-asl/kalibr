// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

TEST(AslamCamerasTestSuite, testPinholeCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<PinholeCameraGeometry> harness(1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDefaultDistortedPinholeCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<DistortedPinholeCameraGeometry> harness(1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedPinholeCameraGeometry)
{
  using namespace aslam::cameras;

  RadialTangentialDistortion d(-0.2, 0.13, 0.0005, 0.0005);
  DistortedPinholeCameraGeometry geometry = DistortedPinholeCameraGeometry::getTestGeometry();
  geometry.projection().distortion() = d;

  CameraGeometryTestHarness<DistortedPinholeCameraGeometry> harness(geometry, 1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testPinholeRsCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<PinholeRsCameraGeometry> harness(1e-1);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDefaultDistortedRsPinholeCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<DistortedPinholeRsCameraGeometry> harness(2e-2);
  SCOPED_TRACE("");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedPinholeRsCameraGeometry)
{
  using namespace aslam::cameras;

  RadialTangentialDistortion d(-0.2, 0.13, 0.0005, 0.0005);
  DistortedPinholeRsCameraGeometry geometry = DistortedPinholeRsCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(d);

  CameraGeometryTestHarness<DistortedPinholeRsCameraGeometry> harness(geometry, 2e-2);
  SCOPED_TRACE("");
  harness.testAll();

}

// TEST(AslamCamerasTestSuite, testDistortedPinholeCameraGeometry) {

//     std::cout << "pinholetest";

//     // test distortion and distored jacobian:
//     aslam::cameras::PinholeCameraGeometry pcg( aslam::cameras::PinholeCameraGeometry::createDistortedTestGeometry() );
//     Eigen::Vector3d p = pcg.createRandomVisiblePoint();
//     Eigen::Vector2d k = pcg.createRandomKeypoint();

//     Eigen::MatrixXd J;
//     Eigen::MatrixXd estJ;

//     Eigen::Vector2d  k1 = pcg.euclideanToKeypoint(p);
//     Eigen::Vector2d  k2 = pcg.euclideanToKeypoint(p, J);
//     sm::eigen::assertEqual(k1,k2, SM_SOURCE_FILE_POS);
//     pcg.euclideanToKeypointFiniteDifference(p,estJ);
//     sm::eigen::assertNear(J,estJ, 1e-5, SM_SOURCE_FILE_POS);

//     // Project to the normalized plane.
//     double mx_u = p[0]/p[2];
//     double my_u = p[1]/p[2];

//     // Apply distortion.
//     double mx_d, my_d;
//     pcg.distortion(mx_u, my_u, &mx_d, &my_d);  
//     // Distortion only puts out deltas.
//     mx_d += mx_u;
//     my_d += my_u;
//     double hat_mx_u, hat_my_u;
//     pcg.undistortGN(mx_d, my_d, &hat_mx_u, &hat_my_u) ;

//     // Check if the error is small...
//     double hat_mx_d, hat_my_d;
//     pcg.distortion(hat_mx_u, hat_my_u, &hat_mx_d, &hat_my_d);  

//     EXPECT_NEAR(hat_mx_u + hat_mx_d, mx_d, 1e-10);
//     EXPECT_NEAR(hat_my_u + hat_my_d, my_d, 1e-10);

//     double ax,bx,cx,dx;
//     pcg.distortion(hat_mx_u, hat_my_u, &hat_mx_d, &hat_my_d, &ax, &bx, &cx, &dx);  

//     EXPECT_NEAR(hat_mx_u + hat_mx_d, mx_d, 1e-10);
//     EXPECT_NEAR(hat_my_u + hat_my_d, my_d, 1e-10);

//     EXPECT_NEAR(hat_mx_u, mx_u, 1e-10);
//     EXPECT_NEAR(hat_my_u, my_u, 1e-10);

// }
