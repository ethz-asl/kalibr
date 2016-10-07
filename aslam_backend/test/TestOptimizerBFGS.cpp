#include <sm/eigen/gtest.hpp>
#include <aslam/backend/OptimizerBFGS.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <sm/random.hpp>
#include <aslam/backend/test/ErrorTermTester.hpp>
#include "SampleDvAndError.hpp"

TEST(OptimizerBFGSTestSuite, testBFGS)
{
  try {
    using namespace aslam::backend;
    boost::shared_ptr<OptimizationProblem> problem_ptr(new OptimizationProblem);
    OptimizationProblem& problem = *problem_ptr;

    const int P = 2;
    const int E = 3;
    // Add some design variables.
    std::vector< boost::shared_ptr<Point2d> > p2d;
    p2d.reserve(P);
    for (int p = 0; p < P; ++p) {
      boost::shared_ptr<Point2d> point(new Point2d(Eigen::Vector2d::Random())); // random initialization of design variable
      p2d.push_back(point);
      problem.addDesignVariable(point);
      point->setBlockIndex(p);
      point->setActive(true);
    }

    // Add some error terms.
    std::vector< boost::shared_ptr<TestNonSquaredError> > e1;
    e1.reserve(P*E);
    for (int p = 0; p < P; ++p) {
      for (int e = 0; e < E; ++e) {
        TestNonSquaredError::grad_t g(p+1, e+1);
        boost::shared_ptr<TestNonSquaredError> err(new TestNonSquaredError(p2d[p].get(), g));
        err->_p = 1.0;
        e1.push_back(err);
        problem.addErrorTerm(err);
        SCOPED_TRACE("");
        testErrorTerm(err);
      }
    }
    // Now let's optimize.
    OptimizerBFGS::Options options;
    options.maxIterations = 500;
    options.numThreadsGradient = 8;
    options.convergenceGradientNorm = 0.0;
    options.convergenceDeltaX = 0.0;
    EXPECT_ANY_THROW(options.check());
    options.convergenceGradientNorm = 1e-15;
    EXPECT_NO_THROW(options.check());
    OptimizerBFGS optimizer(options);
    optimizer.setProblem(problem_ptr);

    // Test that linesearch options are correctly forwarded
    options.linesearch.initialStepLength = 1.1;
    optimizer.setOptions(options);
    EXPECT_DOUBLE_EQ(options.linesearch.initialStepLength, optimizer.getLineSearch().options().initialStepLength);

    EXPECT_NO_THROW(optimizer.checkProblemSetup());

    optimizer.initialize();
    SCOPED_TRACE("");
    optimizer.optimize();
    const auto& ret = optimizer.getStatus();

    EXPECT_GT(ret.convergence, ConvergenceStatus::FAILURE);
    EXPECT_LE(ret.gradientNorm, options.convergenceGradientNorm);
    EXPECT_GT(ret.numObjectiveEvaluations, 0);
    EXPECT_GT(ret.numDerivativeEvaluations, 0);
    EXPECT_GE(ret.error, 0.0);
    EXPECT_LT(ret.deltaError, 1e-12);
    EXPECT_LT(ret.maxDeltaX, 1e-3);
    EXPECT_LT(ret.error, std::numeric_limits<double>::max());
    EXPECT_GT(ret.numIterations, 0);

  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
