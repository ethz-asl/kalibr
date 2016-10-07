#include <sm/eigen/gtest.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include "SampleDvAndError.hpp"


TEST(CallbackTestSuite, testCallback)
{
  try {
    using namespace aslam::backend;
    boost::shared_ptr<OptimizationProblem> problem_ptr(new OptimizationProblem);
    OptimizationProblem& problem = *problem_ptr;
    const int P = 3;
    const int E = 2;
    // Add some design variables.
    std::vector< boost::shared_ptr<Point2d> > p2d;
    for (int p = 0; p < P; ++p) {
      boost::shared_ptr<Point2d> point(new Point2d(Eigen::Vector2d::Random()));
      p2d.push_back(point);
      problem.addDesignVariable(point);
      point->setBlockIndex(p);
      point->setActive(true);
    }
    // Add some error terms.
    std::vector< boost::shared_ptr<LinearErr> > errorTerms;
    for (int p = 0; p < P; ++p) {
      for (int e = 0; e < E; ++e) {
        boost::shared_ptr<LinearErr> err(new LinearErr(p2d[p].get()));
        errorTerms.push_back(err);
        problem.addErrorTerm(err);
      }
    }

    // Now let's optimize.
    OptimizerOptions options;
    options.verbose = false;
    options.linearSolver = "dense";
    options.levenbergMarquardtLambdaInit = 10;
    options.doSchurComplement = false;
    options.doLevenbergMarquardt = false;
    options.maxIterations = 3;
    Optimizer optimizer(options);
    optimizer.setProblem(problem_ptr);

    using namespace callback;
    int countInit = 0, countResUpdate = 0, countCostUpdate = 0, countDesignUpdate = 0;
    double startJ, lastUpdatedJ, expectedCost;
    optimizer.callback().add(Occasion::OPTIMIZATION_INITIALIZED,
        [&](const Argument & arg) {
          countInit ++;
          lastUpdatedJ = startJ = arg.currentCost;
          ASSERT_EQ(Occasion::OPTIMIZATION_INITIALIZED, arg.occasion);
        }
      );
    optimizer.callback().add(Occasion::DESIGN_VARIABLES_UPDATED,
        [&](const Argument & arg) {
          countDesignUpdate ++;
          ASSERT_EQ(Occasion::DESIGN_VARIABLES_UPDATED, arg.occasion);
        }
      );
    optimizer.callback().add(Occasion::RESIDUALS_UPDATED,
        [&](const Argument & arg) {
          countResUpdate ++;
          expectedCost = 0;
          for(auto && e : errorTerms){
            expectedCost += e->getSquaredError();
          }
          ASSERT_EQ(Occasion::RESIDUALS_UPDATED, arg.occasion);
        }
      );
    optimizer.callback().add(Occasion::COST_UPDATED,
        [&](const Argument & arg) { // void with argument
          countCostUpdate ++;
          ASSERT_DOUBLE_EQ(expectedCost, arg.currentCost);
          lastUpdatedJ = arg.currentCost;
          ASSERT_EQ(Occasion::COST_UPDATED, arg.occasion);
        }
      );
    optimizer.callback().add({Occasion::COST_UPDATED, Occasion::COST_UPDATED}, // register twice
        [&](const Argument &) { // void with argument
          countCostUpdate ++;
        }
      );
    optimizer.callback().add(Occasion::COST_UPDATED,
        [&](const Argument &) { // ProceedInstruction with argument
          countCostUpdate ++;
          return ProceedInstruction::CONTINUE;
        }
      );
    optimizer.callback().add(Occasion::COST_UPDATED,
        [&]() { // ProceedInstruction without argument
          countCostUpdate ++;
          return ProceedInstruction::CONTINUE;
        }
      );

    auto ret = optimizer.optimize();

    ASSERT_EQ(1, countInit);
    ASSERT_EQ(ret.iterations + 1, countResUpdate);
    ASSERT_EQ((ret.iterations + 1) * 5, countCostUpdate);
    ASSERT_EQ(ret.iterations, countDesignUpdate);
    ASSERT_EQ(ret.JStart, startJ);
    ASSERT_EQ(ret.JFinal, lastUpdatedJ);
  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}



