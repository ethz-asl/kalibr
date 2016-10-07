#include <boost/shared_ptr.hpp>
#include <sm/eigen/gtest.hpp>
#include <sm/random.hpp>

#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/SparseCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/DenseQrLinearSystemSolver.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <aslam/backend/test/ErrorTermTester.hpp>

#include "SampleDvAndError.hpp"

TEST(Optimizer2TestSuite, compareAllCombinationsOfSolversAndTrustRegionPolicies)
{
  using namespace aslam::backend;
  const int D = 4;
  const int E = 20;
  const int seed = 1;
  try {
    boost::shared_ptr<LinearSystemSolver> baseline_solver(new BlockCholeskyLinearSystemSolver());
    boost::shared_ptr<TrustRegionPolicy> baseline_policy(new LevenbergMarquardtTrustRegionPolicy());

    std::vector<boost::shared_ptr<LinearSystemSolver>> solvers;
    solvers.emplace_back(new SparseCholeskyLinearSystemSolver());
    solvers.emplace_back(new SparseQrLinearSystemSolver());
    solvers.emplace_back(new DenseQrLinearSystemSolver());

    std::vector<boost::shared_ptr<TrustRegionPolicy>> policies;
    policies.emplace_back(new DogLegTrustRegionPolicy());
    policies.emplace_back(new GaussNewtonTrustRegionPolicy());

    boost::shared_ptr<OptimizationProblem> pb = buildProblem(seed, D, E);
    std::vector< boost::shared_ptr<OptimizationProblem> > problems;
    for (size_t i = 0; i < solvers.size()*policies.size(); ++i) {
      boost::shared_ptr<OptimizationProblem> pi = buildProblem(seed, D, E);
      problems.push_back(pi);
      for (size_t j = 0; j < pb->numErrorTerms(); ++j) {
        double eb = pb->errorTerm(j)->evaluateError();
        double ei = pi->errorTerm(j)->evaluateError();
        ASSERT_EQ(ei, eb) << "The problems are not the same";
      }
    }
    Optimizer2Options options;
    options.linearSystemSolver = baseline_solver;
    options.maxIterations = 5;
    options.verbose = false;
    options.trustRegionPolicy = baseline_policy;
    Optimizer2 optimizer(options);
    optimizer.setProblem(pb);
    optimizer.optimize();
    int p = 0;
    for(size_t k = 0; k < policies.size(); ++k) {
      options.trustRegionPolicy = policies[k];
      for (size_t i = 0; i < solvers.size(); ++i) {
        options.linearSystemSolver = solvers[i];
        Optimizer2 optimizer_i(options);
        optimizer_i.setProblem(problems[p]);
        optimizer_i.optimize();
        for (size_t j = 0; j < pb->numErrorTerms(); ++j) {
          double eb = pb->errorTerm(j)->evaluateError();
          double ei = problems[p]->errorTerm(j)->evaluateError();
          ASSERT_NEAR(ei, eb, 1e-6) << "The errors did not reduce in the same way";
        }
        p++;
      }
    }
  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
