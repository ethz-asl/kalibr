#include <sm/eigen/gtest.hpp>
#include <sm/random.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/SamplerHybridMcmc.hpp>
#include <aslam/backend/SamplerMetropolisHastings.hpp>
#include <aslam/backend/test/ErrorTermTester.hpp>
#include "SampleDvAndError.hpp"

using namespace std;
using namespace boost;
using namespace aslam::backend;

/// \brief Encodes the error \f$ -\frac{\left(\mathbf x - \mathbf \mu\right)^2}{2.0 \sigma^2}\f$
class GaussianNegLogDensityError : public ScalarNonSquaredErrorTerm {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef ScalarNonSquaredErrorTerm parent_t;

  Scalar* _x; /// \brief The design variable
  Scalar::Vector1d _mu;  /// \brief The mean
  Scalar::Vector1d _varInv;  /// \brief The inverse variance

  GaussianNegLogDensityError(Scalar* x) : _x(x) {
    _x->setActive(true);
    parent_t::setDesignVariables(_x);
    setWeight(1.0);
    _mu << 0.0;
    _varInv << 1.0;
  }
  virtual ~GaussianNegLogDensityError() {}

  void setMean(const double mu) { _mu << mu; }
  void setVariance(const double var) { _varInv << 1./var; }

  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    const Scalar::Vector1d dv = (_x->_v - _mu);
    return 0.5*(_varInv*dv*dv)[0];
  }

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJ) {
    outJ.add( _x, (_x->_v - _mu)*_varInv );
  }

};

boost::shared_ptr<OptimizationProblem> setupProblem(const double meanTrue, const double sigmaTrue) {

  sm::random::seed(std::time(nullptr));

  boost::shared_ptr<OptimizationProblem> gaussian1dLogDensityPtr(new OptimizationProblem);
  OptimizationProblem& gaussian1dLogDensity = *gaussian1dLogDensityPtr;

  Scalar::Vector1d x;
  x << meanTrue + 10.0*sm::random::randn();

  // Add some design variables.
  boost::shared_ptr<Scalar> sdv(new Scalar(x));
  gaussian1dLogDensity.addDesignVariable(sdv);
  sdv->setBlockIndex(0);
  sdv->setActive(true);
  // Add error term
  boost::shared_ptr<GaussianNegLogDensityError> err(new GaussianNegLogDensityError(sdv.get()));
  err->setMean(meanTrue);
  err->setVariance(sigmaTrue*sigmaTrue);
  gaussian1dLogDensity.addErrorTerm(err);
  SCOPED_TRACE("");
  testErrorTerm(err);

  return gaussian1dLogDensityPtr;
}


TEST(OptimizerSamplerMcmcTestSuite, testSamplerMetropolisHastings)
{
  try {

    sm::random::seed(std::time(nullptr));

    const double meanTrue = 10.0;
    const double sigmaTrue = 2.0;
    boost::shared_ptr<OptimizationProblem> gaussian1dLogDensityPtr = setupProblem(meanTrue, sigmaTrue);
    OptimizationProblem& gaussian1dLogDensity = *gaussian1dLogDensityPtr;

    // Initialize and test options
    sm::BoostPropertyTree pt;
    pt.setDouble("transitionKernelSigma", 3.0);
    pt.setInt("nThreadsEvaluateLogDensity", 1);
    SamplerMetropolisHastingsOptions options(pt);
    EXPECT_DOUBLE_EQ(pt.getDouble("transitionKernelSigma"), options.transitionKernelSigma);
    EXPECT_DOUBLE_EQ(pt.getDouble("nThreadsEvaluateLogDensity"), options.nThreadsEvaluateLogDensity);

    // Set and test log density
    SamplerMetropolisHastings sampler(options);
    sampler.setNegativeLogDensity(gaussian1dLogDensityPtr);
    EXPECT_NO_THROW(sampler.checkNegativeLogDensitySetup());
    EXPECT_DOUBLE_EQ(0.0, sampler.statistics().getAcceptanceRate());
    EXPECT_EQ(0, sampler.statistics().getNumIterations());

    // Parameters
    const int nSamples = 1000;
    const int nStepsBurnIn = 100;
    const int nStepsSkip = 50;

    // Burn-in
    EXPECT_TRUE(sampler.isRecomputationNegLogDensityNecessary());
    sampler.run(nStepsBurnIn);
    EXPECT_GE(sampler.statistics().getAcceptanceRate(), 1e-3);
    EXPECT_LE(sampler.statistics().getAcceptanceRate(), 1.0);
    EXPECT_EQ(nStepsBurnIn, sampler.statistics().getNumIterations());

    // Now let's retrieve samples
    Eigen::VectorXd dvValues(nSamples);
    for (size_t i=0; i<nSamples; i++) {
      sampler.run(nStepsSkip);
      if (sampler.isLastSampledAccepted())
        EXPECT_FALSE(sampler.isRecomputationNegLogDensityNecessary());
      EXPECT_GE(sampler.statistics().getAcceptanceRate(), 0.0);
      EXPECT_LE(sampler.statistics().getAcceptanceRate(), 1.0);
      EXPECT_EQ((i+1)*nStepsSkip + nStepsBurnIn, sampler.statistics().getNumIterations());
      ASSERT_EQ(1, gaussian1dLogDensity.numDesignVariables());
      auto dv = gaussian1dLogDensity.designVariable(0);
      Eigen::MatrixXd p;
      dv->getParameters(p);
      ASSERT_EQ(1, p.size());
      dvValues[i] = p(0,0);
    }

    sampler.forceRecomputationNegLogDensity();
    EXPECT_TRUE(sampler.isRecomputationNegLogDensityNecessary());

    // check sample mean
    EXPECT_NEAR(dvValues.mean(), meanTrue, 4.*sigmaTrue) << "This failure does not necessarily have to be an error. It should just appear "
        " with a probability of 0.00633 %";

    // check sample variance
    EXPECT_NEAR((dvValues.array() - dvValues.mean()).matrix().squaredNorm()/(dvValues.rows() - 1.0), sigmaTrue*sigmaTrue, 1e0) << "This failure does "
        "not necessarily have to be an error. It should just appear very rarely";

    // Check that re-initializing resets values
    sampler.initialize();
    EXPECT_DOUBLE_EQ(0.0, sampler.statistics().getAcceptanceRate());
    EXPECT_EQ(0, sampler.statistics().getNumIterations());
    EXPECT_NO_THROW(sampler.run(1)); // Check that sampler runs ok

  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}


TEST(OptimizerSamplerMcmcTestSuite, testSamplerHybridMcmc)
{
  try {

    sm::random::seed(std::time(nullptr));

    const double meanTrue = 10.0;
    const double sigmaTrue = 2.0;
    boost::shared_ptr<OptimizationProblem> gaussian1dLogDensityPtr = setupProblem(meanTrue, sigmaTrue);
    OptimizationProblem& gaussian1dLogDensity = *gaussian1dLogDensityPtr;

    // Initialize and test options
    sm::BoostPropertyTree pt;
    pt.setDouble("initialLeapFrogStepSize", 0.3);
    pt.setDouble("minLeapFrogStepSize", 0.01);
    pt.setDouble("maxLeapFrogStepSize", 1.0);
    pt.setDouble("incFactorLeapFrogStepSize", 1.02);
    pt.setDouble("decFactorLeapFrogStepSize", 0.98);
    pt.setInt("nLeapFrogSteps", 5);
    pt.setInt("nThreads", 1);
    SamplerHybridMcmcOptions options(pt);
    EXPECT_DOUBLE_EQ(pt.getDouble("initialLeapFrogStepSize"), options.initialLeapFrogStepSize);
    EXPECT_DOUBLE_EQ(pt.getDouble("minLeapFrogStepSize"), options.minLeapFrogStepSize);
    EXPECT_DOUBLE_EQ(pt.getDouble("maxLeapFrogStepSize"), options.maxLeapFrogStepSize);
    EXPECT_DOUBLE_EQ(pt.getDouble("incFactorLeapFrogStepSize"), options.incFactorLeapFrogStepSize);
    EXPECT_DOUBLE_EQ(pt.getDouble("decFactorLeapFrogStepSize"), options.decFactorLeapFrogStepSize);
    EXPECT_DOUBLE_EQ(pt.getInt("nLeapFrogSteps"), options.nLeapFrogSteps);
    EXPECT_DOUBLE_EQ(pt.getInt("nThreads"), options.nThreads);

    // Set and test log density
    SamplerHybridMcmc sampler(options);
    sampler.setNegativeLogDensity(gaussian1dLogDensityPtr);
    EXPECT_NO_THROW(sampler.checkNegativeLogDensitySetup());
    EXPECT_DOUBLE_EQ(sampler.statistics().getAcceptanceRate(), 0.0);
    EXPECT_EQ(sampler.statistics().getNumIterations(), 0);

    // Parameters
    const int nSamples = 1000;
    const int nStepsBurnIn = 10;
    const int nStepsSkip = 5;

    // Burn-in
    sampler.run(nStepsBurnIn);
    EXPECT_GT(sampler.statistics().getAcceptanceRate(), 0.0);
    EXPECT_LE(sampler.statistics().getAcceptanceRate(), 1.0);
    EXPECT_EQ(nStepsBurnIn, sampler.statistics().getNumIterations());

    // Now let's retrieve samples
    Eigen::VectorXd dvValues(nSamples);
    for (size_t i=0; i<nSamples; i++) {
      sampler.run(nStepsSkip);
      EXPECT_GE(sampler.statistics().getAcceptanceRate(), 0.0);
      EXPECT_LE(sampler.statistics().getAcceptanceRate(), 1.0);
      EXPECT_EQ((i+1)*nStepsSkip + nStepsBurnIn, sampler.statistics().getNumIterations());
      ASSERT_EQ(gaussian1dLogDensity.numDesignVariables(), 1);
      auto dv = gaussian1dLogDensity.designVariable(0);
      Eigen::MatrixXd p;
      dv->getParameters(p);
      ASSERT_EQ(1, p.size());
      dvValues[i] = p(0,0);
    }

    EXPECT_DOUBLE_EQ((double)sampler.statistics().getNumAcceptedSamples(true)/(double)sampler.statistics().getNumIterations(),
                     sampler.statistics().getAcceptanceRate());
    EXPECT_GT(sampler.statistics().getWeightedMeanAcceptanceProbability(), 0.0);

    // check sample mean
    EXPECT_NEAR(dvValues.mean(), meanTrue, 4.*sigmaTrue) << "This failure does not necessarily have to be an error. It should just appear "
        " with a probability of 0.00633 %";

    // check sample variance
    EXPECT_NEAR((dvValues.array() - dvValues.mean()).matrix().squaredNorm()/(dvValues.rows() - 1.0), sigmaTrue*sigmaTrue, 1e0) << "This failure does "
        "not necessarily have to be an error. It should just appear very rarely";

    // Check that re-initializing resets values
    sampler.initialize();
    EXPECT_DOUBLE_EQ(0.0, sampler.statistics().getAcceptanceRate());
    EXPECT_EQ(0, sampler.statistics().getNumIterations());
    EXPECT_NO_THROW(sampler.run(1)); // Check that sampler runs ok

  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
