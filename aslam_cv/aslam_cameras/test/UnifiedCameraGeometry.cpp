// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/test/CameraGeometryTestHarness.hpp>

TEST(AslamCamerasTestSuite, testUnifiedCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<UnifiedCameraGeometry> harness(0.1);

  SCOPED_TRACE("unified camera");
  harness.testAll();

}
