// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

TEST(AslamCamerasTestSuite, testOmniCameraGeometry)
{
  using namespace aslam::cameras;
  // Use a forgiving tolerance because of the approximate
  // undistortion used in this model.
  CameraGeometryTestHarness<OmniCameraGeometry> harness(0.1);

  SCOPED_TRACE("omni camera");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedOmniCameraGeometryDefault)
{
  using namespace aslam::cameras;
  // Use a forgiving tolerance because of the approximate
  // undistortion used in this model.
  CameraGeometryTestHarness<DistortedOmniCameraGeometry> harness(0.5);

  SCOPED_TRACE("distorted omni default");
  harness.testAll();

}

TEST(AslamCamerasTestSuite, testDistortedOmniCameraGeometry)
{
  using namespace aslam::cameras;
  // Use a forgiving tolerance because of the approximate
  // undistortion used in this model.
  RadialTangentialDistortion d(-0.2, 0.13, 0.0005, 0.0005);
  DistortedOmniCameraGeometry geometry = DistortedOmniCameraGeometry::getTestGeometry();
  geometry.projection().setDistortion(d);
  CameraGeometryTestHarness<DistortedOmniCameraGeometry> harness(geometry,0.5);

  SCOPED_TRACE("distorted omni");
  harness.testAll();

}
