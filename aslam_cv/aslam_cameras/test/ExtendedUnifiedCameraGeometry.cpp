// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

TEST(AslamCamerasTestSuite, testExtendedUnifiedCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<ExtendedUnifiedCameraGeometry> harness(0.1);

  SCOPED_TRACE("extended unified camera");
  harness.testAll();

}
