// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

#include <limits>
#include <cmath>
#include <sstream>

// Helpful functions from libsm
#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>


TEST(QuaternionAlgebraTestSuite, testRotation)
{
  try {
  using namespace sm::kinematics;
  for(int i = 0; i < 1000; i++)
    {
      // Create a random quaternion
      Eigen::Vector4d q_a_b = quatRandom();
      
      // Create a random point in R3
      Eigen::Vector3d v_b;
      v_b.setRandom();
      
      Eigen::Vector3d v_a1 = quat2r(q_a_b) * v_b;
      Eigen::Vector3d v_a2 = quatRotate(q_a_b, v_b);
      
      sm::eigen::assertNear(v_a1, v_a2, 1e-10,SM_SOURCE_FILE_POS, "The rotation matrix And shortcut rotations are not equal");
      
    }
  } catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

TEST(QuaternionAlgebraTestSuite, testAxisAngle2QuatAndBackOnSpecialValues)
{
  const double eps = std::numeric_limits<double>::epsilon();
  const double sq1o2 = sqrt(1.0/2);
  try {
    std::pair<Eigen::Vector3d, Eigen::Vector4d> testPairs[] = {
        {{0, 0, 0}, {0, 0, 0, 1}},
        {{6.366197723675814e-10, 0, 0}, {1E-9, 0, 0, 1}},
        {{0.5, 0, 0}, {sq1o2, 0, 0, sq1o2}},
        {{1, 0, 0}, {1, 0, 0, 0}},
        {{1.5, 0, 0}, {sq1o2, 0, 0, -sq1o2}},
        {{2, 0, 0}, {0, 0, 0, -1}},
        {{-1, 0, 0}, {-1, 0, 0, 0}},
        {{-2, 0, 0}, {0, 0, 0, -1}},
        {{0, 0.5, 0}, {0, sq1o2, 0, sq1o2}},
        {{0, 1, 0}, {0, 1, 0, 0}},
        {{0, 1.5, 0}, {0, sq1o2, 0, -sq1o2}},
        {{0, 2, 0}, {0, 0, 0, -1}},
        {{0, -1, 0}, {0, -1, 0, 0}},
        {{0, -2, 0}, {0, 0, 0, -1}},
    };
    using namespace sm::kinematics;
    for(auto p : testPairs)
    {
      sm::eigen::assertNear(p.second, axisAngle2quat(p.first * M_PI), eps, SM_SOURCE_FILE_POS, "");
      if(p.second(3) != -1.0){
        sm::eigen::assertNear(p.first, quat2AxisAngle(p.second) / M_PI, eps, SM_SOURCE_FILE_POS, "");
        sm::eigen::assertNear(p.second, axisAngle2quat(quat2AxisAngle(p.second)), eps, SM_SOURCE_FILE_POS, "");
        sm::eigen::assertNear(p.first, quat2AxisAngle(axisAngle2quat(p.first)), 2 * eps, SM_SOURCE_FILE_POS, "");
      }
    }
  } catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

class Stringer {
 public:
  operator const char * () {
    return ss.str().c_str();
  }
  operator const std::string () {
    return ss.str();
  }

  template <typename T>
  Stringer & operator <<(const T & something){
    ss << something;
    return *this;
  }
 private:
  std::stringstream ss;
};

TEST(QuaternionAlgebraTestSuite, testAxisAngle2QuatAndBack)
{
  const double eps = std::numeric_limits<double>::epsilon();
  try {
    using namespace sm::kinematics;
    for(int i = 0; i < 1000; i++)
    {
      // Create a random quaternion
      Eigen::Vector4d q = quatRandom();
      sm::eigen::assertNear(q, axisAngle2quat(quat2AxisAngle(q)), q.block<3,1>(0, 0).norm() * eps * 20, SM_SOURCE_FILE_POS, "");

      // Create a random point in R3
      Eigen::Vector3d v_b; v_b.setRandom();
      double tolerance;
      if(i % 2 == 0){
        if(i % 4 == 0){
          // make use of the special code for quaternions close to identity more likely!
          v_b *= 1E-20;
        } else {
          // go very close to full rotation (but less than).
          v_b *= (M_PI * 2  - 8 * eps) / v_b.norm();
        }
      }
      tolerance = v_b.norm() * eps * 40;
      sm::eigen::assertNear(v_b, quat2AxisAngle(axisAngle2quat(v_b)), tolerance, SM_SOURCE_FILE_POS, Stringer() << "axisAngle2QuatAndBack is too little accurate : \naA2Q(v_b)="<< axisAngle2quat(v_b) << "\nv_b.norm() - 2Pi=" << (v_b.norm() - M_PI * 2) );
    }
  } catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

TEST(QuaternionAlgebraTestSuite, testExpJacobian)
{
  Eigen::Vector3d zero3 = Eigen::Vector3d::Zero();
  using namespace sm::kinematics;
  for(int i = 0; i < 100; i ++){
    const Eigen::Vector3d v = Eigen::Vector3d::Random();
    sm::eigen::assertNear(quatExpJacobian(v), sm::eigen::numericalDiff<Eigen::Vector4d, Eigen::Vector3d >( [&v](const Eigen::Vector3d & x) { return  axisAngle2quat(v + x); }, zero3), 1E-7, SM_SOURCE_FILE_POS,  "");
  }
}


TEST(QuaternionAlgebraTestSuite, testLogJacobian)
{
  Eigen::Vector4d zero4 = Eigen::Vector4d::Zero();
  using namespace sm::kinematics;
  for(int i = 0; i < 100; i ++){
    const Eigen::Vector4d q = quatRandom();
    sm::eigen::assertNear(quatLogJacobian2(q), sm::eigen::numericalDiff<Eigen::Vector3d, Eigen::Vector4d >( [&q](const Eigen::Vector4d & x) { return  quat2AxisAngle(qplus(axisAngle2quat(2 * qeps(qplus(x, quatInv(q)))), q)); }, zero4), 1E-7, SM_SOURCE_FILE_POS,  "");
//TODO figure out how quatLogJacobian was meant to work and write test (the following does not work)
// sm::eigen::assertNear(quatLogJacobian(q), sm::eigen::numericalDiff<Eigen::Vector3d, Eigen::Vector4d >( [&q](const Eigen::Vector4d & x) { return  quat2AxisAngle(qplus(axisAngle2quat(2 * qeps(qplus(x, quatInv(q)))), q)); }, zero4), 1E-7, SM_SOURCE_FILE_POS,  "");
  }
}

