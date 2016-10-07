#include <vector>

#include <boost/make_shared.hpp>
#include <sm/eigen/gtest.hpp>

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/ProbDataAssocPolicy.hpp>

using aslam::backend::ErrorTermFs;
using aslam::backend::JacobianContainer;
using aslam::backend::ProbDataAssocPolicy;
using aslam::backend::FixedWeightMEstimator;

class DummyError : public ErrorTermFs<1> {
 private:
  double _error;

 public:
  explicit DummyError(double error) {
    _error = error;
    updateRawSquaredError();
  }

 protected:
  virtual double evaluateErrorImplementation() { return _error * _error; }

  virtual void evaluateJacobiansImplementation(JacobianContainer &) {}
};

TEST(ProbDataAssocPolicyTestSuite, callbackTest) {
  std::vector<ProbDataAssocPolicy::ErrorTermPtr> error_group_1;
  std::vector<ProbDataAssocPolicy::ErrorTermPtr> error_group_2;

  for (int i = 0; i < 3; i++) {
    boost::shared_ptr<FixedWeightMEstimator> m_estimator(
        new FixedWeightMEstimator(1));
    ProbDataAssocPolicy::ErrorTermPtr err(new DummyError(1));
    err->setMEstimatorPolicy(m_estimator);
    error_group_1.push_back(err);
  }
  for (int i = 1; i < 5; i++) {
    boost::shared_ptr<FixedWeightMEstimator> m_estimator(
        new FixedWeightMEstimator(1));
    ProbDataAssocPolicy::ErrorTermPtr err(new DummyError(i));
    err->setMEstimatorPolicy(m_estimator);
    error_group_2.push_back(err);
  }
  ProbDataAssocPolicy::ErrorTermGroups error_groups(
      new std::vector<ProbDataAssocPolicy::ErrorTermGroup>);
  error_groups->push_back(
      boost::make_shared<std::vector<ProbDataAssocPolicy::ErrorTermPtr>>(
          error_group_1));
  error_groups->push_back(
      boost::make_shared<std::vector<ProbDataAssocPolicy::ErrorTermPtr>>(
          error_group_2));
  ProbDataAssocPolicy policy(error_groups, 1);
  for (ProbDataAssocPolicy::ErrorTermGroup group : *error_groups) {
    for (ProbDataAssocPolicy::ErrorTermPtr error_term : *group) {
      // Initially all the weights should be 1
      EXPECT_EQ(error_term->getCurrentMEstimatorWeight(), 1);
    }
  }
  policy.callback();

  std::vector<double> group1_expected_w = {-1.09861228866811, -1.09861228866811,
                                           -1.09861228866811};
  std::vector<double> group2_expected_w = {
      -0.216722084482954, -1.716722084482954,
      -4.216722084482954, -7.716722084482954};
  std::vector<std::vector<double>> expected_weights;
  expected_weights.push_back(group1_expected_w);
  expected_weights.push_back(group2_expected_w);

  for (std::size_t i = 0; i < error_groups->size(); i++) {
    ProbDataAssocPolicy::ErrorTermGroup group = error_groups->at(i);
    for (std::size_t j = 0; j < group->size(); j++) {
      ProbDataAssocPolicy::ErrorTermPtr error_term = group->at(j);
      EXPECT_NEAR(expected_weights[i][j],
                  error_term->getCurrentMEstimatorWeight(), 1e-6);
    }
  }
}
