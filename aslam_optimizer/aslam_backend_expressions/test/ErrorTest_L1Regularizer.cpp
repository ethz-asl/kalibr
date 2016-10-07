#include <gtest/gtest.h>

#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/OptimizerRprop.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/test/ErrorTermTester.hpp>

#include <aslam/backend/L1Regularizer.hpp>

#include <sm/logging.hpp>

using namespace std;
using namespace aslam::backend;

class TestError : public ScalarNonSquaredErrorTerm {
 public:
  TestError(Scalar* dv, const double val) : _dv(dv), _val(val) {
    setDesignVariables(dv);
    setWeight(0.5);
  }
 private:
  virtual double evaluateErrorImplementation() override {
    double d = _val - _dv->getParameters()(0,0);
    return d*d;
  }

  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) override {
    Eigen::MatrixXd J(1,1);
    J(0,0) = -2.*(_val - _dv->getParameters()(0,0));
    outJacobians.add(_dv, J);
  }
  Scalar* _dv;
  double _val;
};

TEST(RegularizerTestSuite, testL1Regularizer)
{
  try {

    Scalar dv1(-2.0);
    Scalar dv2(3.0);
    Scalar dv3(0.0);
    vector<Scalar*> dvs;
    dvs.push_back(&dv1);
    dvs.push_back(&dv2);
    dvs.push_back(&dv3);
    for (size_t i=0; i<dvs.size(); i++) {
      dvs[i]->setActive(true);
      dvs[i]->setBlockIndex(i);
    }
    L1Regularizer reg(dvs, 2.0);

    SCOPED_TRACE("");
    double error = reg.evaluateError();
    EXPECT_DOUBLE_EQ(10.0, error);

    reg.setBeta(3.0);
    SCOPED_TRACE("");
    error = reg.evaluateError();
    EXPECT_DOUBLE_EQ(15.0, error);

    Eigen::MatrixXd Jexp(1,3);
    Jexp << -3.0, 3.0, 0.0;
    JacobianContainerSparse<1> jc(1);
    reg.evaluateJacobians(jc);
    Eigen::MatrixXd J = jc.asDenseMatrix();
    EXPECT_TRUE(J.isApprox(Jexp)) << "Computed: " << J << std::endl <<
        "Expected: " << Jexp;
  }
  catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

TEST(AslamVChargeBackendTestSuite, testL1RegularizerRprop)
{
  try {
    using namespace aslam::backend;

    boost::shared_ptr<OptimizationProblem> problem_ptr(new OptimizationProblem);
    OptimizationProblem& problem = *problem_ptr;
    vector<aslam::backend::Scalar*> dvs;

    // Add A design variable
    aslam::backend::Scalar dv1(1.0);
    dv1.setBlockIndex(0);
    dv1.setActive(true);
    problem.addDesignVariable(&dv1, false);
    dvs.push_back(&dv1);

    // Add another irrelevant design variable. This one should be driven to zero
    // through the regularization term
    aslam::backend::Scalar dv2(10.0);
    dv2.setBlockIndex(0);
    dv2.setActive(true);
    problem.addDesignVariable(&dv2, false);
    dvs.push_back(&dv2);

    // Add some error terms.
    const size_t numErrorTerms = 100;
    vector< boost::shared_ptr<TestError> > errorTerms;
    errorTerms.reserve(100);
    for (size_t i = 0; i < numErrorTerms; ++i) {
      errorTerms.emplace_back(new TestError(&dv1, (double)i/numErrorTerms));
      problem.addErrorTerm(errorTerms.back());
      SCOPED_TRACE("");
      testErrorTerm(errorTerms.back());
    }

    // Now let's optimize.
    OptimizerOptionsRprop options;
    options.maxIterations = 500;
    options.numThreadsGradient = 1;
    options.convergenceGradientNorm = 1e-6;
    options.convergenceDeltaX = 1e-6;
    options.regularizer.reset(new L1Regularizer(dvs, 1.0));
    OptimizerRprop optimizer(options);
    optimizer.setProblem(problem_ptr);

    EXPECT_NO_THROW(optimizer.checkProblemSetup());

    SCOPED_TRACE("");
    optimizer.optimize();

    EXPECT_NEAR(dv2.getParameters()(0,0), 0.0, 1e-3);

  } catch(const std::exception & e) {
    FAIL() << e.what();
  }
}
