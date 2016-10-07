#include <sm/eigen/gtest.hpp>
#include <Eigen/Dense>
#include <aslam/backend/OptimizerBase.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/util/OptimizerProblemManagerBase.hpp>
#include "SampleDvAndError.hpp"

#include <sm/BoostPropertyTree.hpp>

using namespace aslam::backend;

struct TestOptimizerOptions : public OptimizerOptionsBase
{
  int testOption = -1;
};

struct TestOptimizerStatus : public OptimizerStatus
{
  using OptimizerStatus::reset;
  int testValue = -1;
};

class TestOptimizer : public OptimizerProblemManagerBase
{
 public:
  TestOptimizer(const TestOptimizerOptions& options) : _options(options) { }
  const OptimizerStatus& getStatus() const override { return _status; }
  const OptimizerOptionsBase& getOptions() const override { return _options; }
  void setOptions(const OptimizerOptionsBase& options) override { static_cast<OptimizerOptionsBase&>(_options) = options; }
  void setOptions(const TestOptimizerOptions& options) { _options = options; }
  using OptimizerProblemManagerBase::updateConvergenceStatus;
 private:
  void optimizeImplementation() override
  {
    _status.deltaError = _problemManager.evaluateError() - _status.error;
    _status.numObjectiveEvaluations++;
    _status.error += _status.deltaError;
    RowVectorType gradient;
    _problemManager.computeGradient(gradient, _options.numThreadsGradient, true, true, true);
    _status.numDerivativeEvaluations++;
    _status.gradientNorm = 0.0; // optimizer should stop due to this criterion
    _status.maxDeltaX = 1.0;
    _status.testValue += 1;
    _status.numIterations++;
    this->updateConvergenceStatus();
  }
 private:
  TestOptimizerOptions _options;
  TestOptimizerStatus _status;
  ProblemManager _problemManager;
};

void expectStatusInitialized(const OptimizerStatus& status)
{
  EXPECT_EQ(0, status.numIterations);
  EXPECT_EQ(0, status.numDerivativeEvaluations);
  EXPECT_EQ(0, status.numObjectiveEvaluations);
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::max(), status.error);
  EXPECT_TRUE(std::isnan(status.deltaError));
  EXPECT_TRUE(std::isnan(status.maxDeltaX));
  EXPECT_TRUE(std::isnan(status.gradientNorm));
  EXPECT_FALSE(status.success());
  EXPECT_FALSE(status.failure());
  EXPECT_EQ(ConvergenceStatus::IN_PROGRESS, status.convergence);
}

TEST(OptimizerTestSuite, testOptimizerOptions)
{
  {
    OptimizerOptionsBase options;
    EXPECT_NO_THROW(options.check()); // Default options should be valid

    // Test some invalid options
    options.convergenceDeltaObjective = -1.0;
    EXPECT_ANY_THROW(options.check());
    options = OptimizerOptionsBase(); // reset to default

    options.convergenceDeltaX = -1.0;
    EXPECT_ANY_THROW(options.check());
    options = OptimizerOptionsBase(); // reset to default

    options.convergenceGradientNorm = -1.0;
    EXPECT_ANY_THROW(options.check());
    options = OptimizerOptionsBase(); // reset to default

    options.maxIterations = -2;
    EXPECT_ANY_THROW(options.check());
    options = OptimizerOptionsBase(); // reset to default
  }

  { // Test construction from property tree
    sm::BoostPropertyTree pt;
    pt.setDouble("convergenceGradientNorm", -1.0); // invalid
    pt.setDouble("convergenceDeltaX", 1.0);
    pt.setDouble("convergenceDeltaObjective", 1.0);
    pt.setInt("maxIterations", 1);
    pt.setInt("numThreadsGradient", 1);
    pt.setInt("numThreadsError", 4);
    EXPECT_ANY_THROW(OptimizerOptionsBase options(pt)); // invalid option convergenceGradientNorm
    pt.setDouble("convergenceGradientNorm", 1.0);
    OptimizerOptionsBase options(pt);
    EXPECT_DOUBLE_EQ(pt.getDouble("convergenceGradientNorm"), options.convergenceGradientNorm);
    EXPECT_DOUBLE_EQ(pt.getDouble("convergenceDeltaX"), options.convergenceDeltaX);
    EXPECT_DOUBLE_EQ(pt.getDouble("convergenceDeltaObjective"), options.convergenceDeltaObjective);
    EXPECT_EQ(pt.getInt("maxIterations"), options.maxIterations);
    EXPECT_EQ(pt.getInt("numThreadsGradient"), options.numThreadsGradient);
    EXPECT_EQ(pt.getInt("numThreadsError"), options.numThreadsError);
  }
}

TEST(OptimizerTestSuite, testOptimizerStatus)
{
  TestOptimizerStatus status;
  expectStatusInitialized(status);

  status.numIterations++;
  status.numDerivativeEvaluations++;
  status.numObjectiveEvaluations++;
  status.convergence = ConvergenceStatus::FAILURE;

  status.reset();
  expectStatusInitialized(status);
}

TEST(OptimizerTestSuite, testOptimizerBase)
{
  try
  {
    TestOptimizerOptions options;
    options.convergenceDeltaObjective = 0.0;
    options.convergenceDeltaX = 0.0;
    options.convergenceGradientNorm = 1e-12;
    options.maxIterations = 1;
    TestOptimizer testOptimizer(options);
    OptimizerBase& optimizer = testOptimizer;
    EXPECT_DOUBLE_EQ(options.convergenceGradientNorm, optimizer.getOptions().convergenceGradientNorm);

    EXPECT_FALSE(optimizer.isInitialized());
    EXPECT_ANY_THROW(optimizer.initialize()); // no optimization problem set
    EXPECT_FALSE(optimizer.isInitialized());
    testOptimizer.updateConvergenceStatus(); // should not yield any change in convergence
    expectStatusInitialized(optimizer.getStatus());
    EXPECT_FALSE(optimizer.isConverged());
    EXPECT_FALSE(optimizer.isFailed());
    EXPECT_TRUE(optimizer.isInProgress());

    // Test setting base options
    options.maxIterations = 2;
    optimizer.setOptions(options);
    EXPECT_EQ(options.maxIterations, optimizer.getOptions().maxIterations);

    // Create a design variable
    Point2d dv(Eigen::Vector2d::Random());
    dv.setBlockIndex(0);
    dv.setColumnBase(0);
    dv.setActive(true);

    // Create an error term
    TestNonSquaredError err(&dv, TestNonSquaredError::grad_t::Random());

    boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem());
    problem->addDesignVariable(&dv, false);
    problem->addErrorTerm(&err, false);
    optimizer.setProblem(problem);

    {
      SCOPED_TRACE("Testing optimize() without calling initialize()");
      optimizer.optimize();
      EXPECT_TRUE(optimizer.isInitialized());
      EXPECT_TRUE(optimizer.isConverged());
      EXPECT_EQ(ConvergenceStatus::GRADIENT_NORM, optimizer.getStatus().convergence);
    }
    {
      SCOPED_TRACE("Testing reset()");
      optimizer.reset();
      expectStatusInitialized(optimizer.getStatus());
    }
  }
  catch (const std::exception& e)
  {
    FAIL() << e.what();
  }
}
