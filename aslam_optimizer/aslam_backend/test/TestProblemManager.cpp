#include <sm/eigen/gtest.hpp>
#include <string>
#include <bitset>
#include <aslam/backend/util/ProblemManager.hpp>
#include <aslam/backend/JacobianContainerDense.hpp>
#include <aslam/backend/JacobianContainerSparse.hpp>
#include "SampleDvAndError.hpp"

using namespace std;
using namespace aslam::backend;

std::string to_string(const bool useMEstimator, const bool applyDvScaling, const bool useDenseJacobianContainer)
{
  std::ostringstream os;
  os << "Failure with options useMEstimator: "  << useMEstimator <<
      ", applyDvScaling: " + applyDvScaling << ", useDenseJacobianContainer: " << useDenseJacobianContainer;
  return os.str();
}

TEST(OptimizationProblemTestSuite, testProblemManager)
{
  // Create one squared error term with one design variable
  Point2d dv0(Eigen::Vector2d::Random());
  LinearErr err0(&dv0);

  // Create two non-squared error terms with with the same design variable
  Point2d dv1(Eigen::Vector2d::Random());
  TestNonSquaredError err1(&dv1, TestNonSquaredError::grad_t::Random());
  TestNonSquaredError err2(&dv1, TestNonSquaredError::grad_t::Random());

  boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem());
  problem->addDesignVariable(&dv0, false);
  problem->addDesignVariable(&dv1, false);
  problem->addErrorTerm(&err0, false);
  problem->addErrorTerm(&err1, false);
  problem->addErrorTerm(&err2, false);

  for (int mask = 0 ; mask != (1<<3) ; mask++)
  {
    bitset<3> bits(mask);
    const bool useMEstimator = bits[0];
    const bool applyDvScaling = bits[1];
    const bool useDenseJacobianContainer = bits[2];
    auto optStr = to_string(useMEstimator, applyDvScaling, useDenseJacobianContainer);

    SCOPED_TRACE(testing::Message() << optStr);

    dv0.setActive(false);
    dv1.setActive(true);

    ProblemManager pm;
    pm.setProblem(problem);

    ASSERT_FALSE(pm.isInitialized());
    pm.initialize();

    ASSERT_TRUE(pm.isInitialized());
    ASSERT_EQ(1, pm.numDesignVariables()); // only one active
    ASSERT_EQ(2, pm.numOptParameters()); // one active two-dimensional
    ASSERT_EQ(3, pm.numErrorTerms());

    JacobianContainerSparse<> jc1(err1.dimension());
    err1.evaluateWeightedJacobians(jc1);
    RowVectorType grad1 = jc1.asDenseMatrix();
    jc1.clear();
    err2.evaluateWeightedJacobians(jc1);
    RowVectorType grad2 = jc1.asDenseMatrix();

    RowVectorType grad = RowVectorType::Zero(pm.numOptParameters());
    JacobianContainerDense<RowVectorType&, 1> jcDense(grad);

    // Gradient should be zero, design variable is not activated
    pm.addGradientForErrorTerm(grad, &err0, useMEstimator, useDenseJacobianContainer);
    sm::eigen::assertEqual(RowVectorType::Zero(pm.numOptParameters()), grad, SM_SOURCE_FILE_POS);

    // Gradient should be equal to the one of the scalar error term
    grad.setZero();
    pm.addGradientForErrorTerm(jcDense, &err1, useMEstimator);
    sm::eigen::assertEqual(grad1, grad, SM_SOURCE_FILE_POS, optStr);
    pm.addGradientForErrorTerm(jcDense, &err1, useMEstimator); // test that nothing gets cleared
    sm::eigen::assertEqual(grad1 + grad1, grad, SM_SOURCE_FILE_POS, optStr);

    // Full gradient should be equal to the sum of the two scalar error terms
    grad.setZero();
    pm.computeGradient(grad, 1, useMEstimator, applyDvScaling, useDenseJacobianContainer);
    sm::eigen::assertEqual(grad1 + grad2, grad, SM_SOURCE_FILE_POS, optStr);

    // Now activate design variable 0
    dv0.setActive(true);
    pm.initialize();
    ASSERT_EQ(2, pm.numDesignVariables());
    ASSERT_EQ(4, pm.numOptParameters());
    ASSERT_EQ(3, pm.numErrorTerms());
    grad = RowVectorType::Zero(pm.numOptParameters());

    // Compute expected gradient for squared error
    JacobianContainerSparse<> jc0(err0.dimension());
    err0.getWeightedJacobians(jc0, useMEstimator);
    ColumnVectorType ev;
    err0.updateRawSquaredError();
    err0.getWeightedError(ev, true);
    RowVectorType grad0 = 2.0*ev.transpose()*jc0.asDenseMatrix();

    // check gradient for squared error term
    RowVectorType grad_expected = RowVectorType::Zero(pm.numOptParameters());
    grad_expected.segment(0, 2) = grad0;
    pm.addGradientForErrorTerm(grad, &err0, useMEstimator, useDenseJacobianContainer);
    sm::eigen::assertEqual(grad_expected, grad, SM_SOURCE_FILE_POS, optStr);

    // Full gradient should now contain all error terms
    grad.setZero();
    grad_expected.setZero();
    grad_expected.segment(0, 2) = grad0;
    grad_expected.segment(2, 2) = grad1 + grad2;
    pm.computeGradient(grad, 1, useMEstimator, applyDvScaling, useDenseJacobianContainer);
    sm::eigen::assertEqual(grad_expected, grad, SM_SOURCE_FILE_POS, optStr);
  }
}
