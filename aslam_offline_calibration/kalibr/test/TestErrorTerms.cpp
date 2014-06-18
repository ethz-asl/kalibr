#include <kalibr_errorterms/AccelerometerError.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

// GyroscopeError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR, const aslam::backend::EuclideanExpression & angularVelocity, const aslam::backend::EuclideanExpression & bias );

// AccelerometerError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d invR,
//                    aslam::backend::RotationExpression C_b_w, aslam::backend::EuclideanExpression acceleration_w,
//                    aslam::backend::EuclideanExpression bias, aslam::backend::EuclideanExpression g_w);

TEST(ImuCameraTests, testAccelerometer) {
// AccelerometerError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d invR,
  //                    aslam::backend::RotationExpression C_b_w, aslam::backend::EuclideanExpression acceleration_w,
  //                    aslam::backend::EuclideanExpression bias, aslam::backend::EuclideanExpression g_w);
  using namespace aslam::backend;
  using namespace kalibr_errorterms;

  RotationQuaternion q_b_w(sm::kinematics::quatRandom());
  EuclideanPoint a_w(Eigen::Vector3d::Random());
  EuclideanPoint bias(Eigen::Vector3d::Random());
  EuclideanPoint gravity(Eigen::Vector3d::Random());

  AccelerometerError aerr(Eigen::Vector3d::Random(),
                          Eigen::Matrix3d::Identity(), q_b_w.toExpression(),
                          a_w.toExpression(), bias.toExpression(),
                          gravity.toExpression());

  ErrorTermTestHarness<3> harness(&aerr);
  harness.testAll(1e-5);
}

TEST(ImuCameraTests, testGyroscope) {
  // GyroscopeError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR, const aslam::backend::EuclideanExpression & angularVelocity, const aslam::backend::EuclideanExpression & bias );

  using namespace aslam::backend;
  using namespace kalibr_errorterms;

  EuclideanPoint a_w(Eigen::Vector3d::Random());
  EuclideanPoint bias(Eigen::Vector3d::Random());

  GyroscopeError gerr(Eigen::Vector3d::Random(), Eigen::Matrix3d::Identity(),
                      a_w.toExpression(), bias.toExpression());

  ErrorTermTestHarness<3> harness(&gerr);

  harness.testAll(1e-5);
}
