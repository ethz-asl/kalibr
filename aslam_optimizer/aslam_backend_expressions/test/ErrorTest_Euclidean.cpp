#include <gtest/gtest.h>

#include <aslam/backend/ErrorTermEuclidean.hpp>
// This test harness makes it easy to test error terms.
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/TransformationBasic.hpp>


TEST(AslamVChargeBackendTestSuite, testEuclidean)
{
  try {
      using namespace aslam::backend;

    sm::kinematics::Transformation T_random;
    T_random.setRandom(0.05, 0.01);
    sm::kinematics::Transformation T_prior;
        
    EuclideanPoint ep(T_random.t());
    ep.setActive(true);

    Eigen::MatrixXd N = 2 * Eigen::MatrixXd::Identity(3,3);

    ErrorTermEuclidean ete(ep.toExpression(), Eigen::Vector3d::Random(), N);
    // Create the test harness
    aslam::backend::ErrorTermTestHarness<3> harness(&ete);

    // Run the unit tests.
    harness.testAll(1e-5);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}
