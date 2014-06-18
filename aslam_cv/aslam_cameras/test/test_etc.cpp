// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras/PinholeCameraGeometry.hpp>

TEST(SampleAslamTestSuite, testTest)
{
  // This test only looks if this object will compile at all!
  aslam::cameras::PinholeCameraGeometry pcg;

  //aslam::cameras::CameraGeometry<aslam::cameras::PinholeCameraGeometry> * cg = &pcg;

  Eigen::Vector4d ph;
  Eigen::Vector3d p = pcg.createRandomVisiblePoint();
  Eigen::Vector2d k = pcg.createRandomKeypoint();

  ph.head<3>() = p;
  ph[3] = 1.0;

  Eigen::MatrixXd J;
  Eigen::VectorXd k1;
  pcg.euclideanToKeypoint(p, k1);
  Eigen::Vector2d k2;
  pcg.euclideanToKeypoint(p, k2, J);

  Eigen::VectorXd k3;
  pcg.homogeneousToKeypoint(ph, k1);
  sm::eigen::assertEqual(k1,k2, SM_SOURCE_FILE_POS);

  // This computes the finite difference jacobian.
  Eigen::MatrixXd estJ;
  pcg.euclideanToKeypointFiniteDifference(p,k1,estJ);

  sm::eigen::assertNear(J,estJ, 1e-6, SM_SOURCE_FILE_POS);
}
