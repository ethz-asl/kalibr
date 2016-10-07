#include <gtest/gtest.h>

#include <aslam/backend/ScalarDesignVariable.hpp>
#include <aslam/backend/ErrorTermObservation.hpp>
#include <aslam/backend/ErrorTermMotion.hpp>
#include <aslam/backend/ErrorTermPrior.hpp>
// This test harness makes it easy to test error terms.
#include <aslam/backend/test/ErrorTermTestHarness.hpp>

TEST(AslamTutorialTestSuite, testObservation)
{
  try {
    using namespace aslam::backend;
    // A wall at 1.0
    ScalarDesignVariable w(1.0);

    // a robot at 2.0
    ScalarDesignVariable x(2.0); 

    // a measurement
    double y = 1.0 / (w.value() - x.value()) + 0.001;

    // Creat the observation
    ErrorTermObservation obs(&x, &w, y, 1.0);

    SCOPED_TRACE("");
    testErrorTerm(obs);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

TEST(AslamTutorialTestSuite, testMotion)
{
  try {
    using namespace aslam::backend;
    double u = 1.0;

    ScalarDesignVariable x_k(1.0);

    ScalarDesignVariable x_kp1(x_k.value() + u + 0.001); 

    // Creat the motion error term
    ErrorTermMotion motion(&x_k, &x_kp1, u, 1.0);

    SCOPED_TRACE("");
    testErrorTerm(motion);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

TEST(AslamTutorialTestSuite, testPrior)
{
  try {
    using namespace aslam::backend;
    double x_prior = 1.0;

    ScalarDesignVariable x(x_prior + 0.0001);

    // Create the prior
    ErrorTermPrior prior(&x, x_prior, 1.0);

    SCOPED_TRACE("");
    testErrorTerm(prior);
  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}
