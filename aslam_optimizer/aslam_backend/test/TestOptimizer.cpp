#include <sm/eigen/gtest.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <sm/random.hpp>
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include "SampleDvAndError.hpp"


TEST(OptimizerTestSuite, testOptimizerMatrices)
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
    std::vector< boost::shared_ptr<LinearErr> > e1;
    for (int p = 0; p < P; ++p) {
      for (int e = 0; e < E; ++e) {
        boost::shared_ptr<LinearErr> err(new LinearErr(p2d[p].get()));
        e1.push_back(err);
        problem.addErrorTerm(err);
        SCOPED_TRACE("");
        ErrorTermTestHarness<2> eh(err.get());
        SCOPED_TRACE("");
        eh.testAll();
      }
    }
    // std::vector< boost::shared_ptr<LinearErr2> > e2;
    // Point2d * ps = p2d[P-1].get();
    // ps->setMarginalized(true);
    // for(int p = 0; p < P-1; ++p)
    //   {
    //  boost::shared_ptr<LinearErr2> err(new LinearErr2(p2d[p].get(), ps) )
    //  e2.push_back( err );
    //  problem.addErrorTerm(err);
    //  ErrorTermTestHarness<2> eh(err.get());
    //  eh.testAll();
    //   }
    // Now let's optimize.
    OptimizerOptions options;
    options.verbose = true;
    options.linearSolver = "dense";
    options.levenbergMarquardtLambdaInit = 10;
    options.doSchurComplement = false;
    options.doLevenbergMarquardt = false;
    options.maxIterations = 1;
    Optimizer optimizer(options);
    optimizer.setProblem(problem_ptr);
    optimizer.optimize();
    // for(int p = 0; p < P; ++p)
    //   {
    //  std::cout << "dv[" << p << "] index: " << problem.designVariable(p).blockIndex() << std::endl;
    //   }
    // for(unsigned i = 0; i < problem.numErrorTerms(); ++i)
    //   {
    //  std::cout << "e[" << i << "] has " << problem.errorTerm(i).numDesignVariables() << " design variables\n";
    //  for(unsigned j = 0; j < problem.errorTerm(i).numDesignVariables(); ++j)
    //    {
    //      std::cout << "\tdv[" << j << "] has block index " << problem.errorTerm(i).designVariable(j)->blockIndex() << std::endl;
    //      std::cout << "\tdv[" << j << "]: " << ( (void*)problem.errorTerm(i).designVariable(j)) << std::endl;
    //    }
    //   }
    for (int p = 0; p < P; ++p)
      problem.designVariable(p)->revertUpdate();
    // Now...build the Jacobian matrix by hand.
    Eigen::MatrixXd J(problem.numErrorTerms() * 2, P * 2);
    J.setZero();
    Eigen::VectorXd e(problem.numErrorTerms() * 2);
    e.setZero();
    Eigen::VectorXd rhs(P * 2);
    rhs.setZero();
    Eigen::MatrixXd invR(problem.numErrorTerms() * 2, problem.numErrorTerms() * 2);
    invR.setZero();
    boost::shared_ptr<SparseBlockMatrix> cloneH(optimizer.H().clone());
    cloneH->clear();

    Eigen::VectorXd ei;
    for (unsigned i = 0; i < problem.numErrorTerms(); ++i) {
      int ridx = 2 * i;
      JacobianContainer jc(problem.errorTerm(i)->dimension());
      problem.errorTerm(i)->evaluateError();
      problem.errorTerm(i)->getWeightedJacobians(jc, false);
      J.block<2, P * 2>(ridx, 0) = jc.asDenseMatrix(optimizer.H().colBlockIndices());
      problem.errorTerm(i)->getWeightedError(ei, false);
      e.segment<2>(ridx) = ei;
      problem.errorTerm(i)->buildHessian(*cloneH, rhs, false);
    }
    Eigen::MatrixXd estH = J.transpose() * J;
    Eigen::MatrixXd H = optimizer.H().toDense().selfadjointView<Eigen::Upper>();
    sm::eigen::assertNear(estH, H, 1e-14, SM_SOURCE_FILE_POS, "Did we build the Hessian correctly? A is from J^T * J and B is from the optimizer.");
    Eigen::VectorXd estRhs = -J.transpose() * e;
    sm::eigen::assertNear(estRhs, optimizer.rhs(), 1e-14, SM_SOURCE_FILE_POS, "Did we build the right hand side correctly?");
    for (unsigned i = 0; i < problem.numErrorTerms(); ++i) {
      cloneH->clear();
      J.setZero();
      int ridx = 2 * i;
      problem.errorTerm(i)->evaluateError();
      JacobianContainer Jc(problem.errorTerm(i)->dimension());
      problem.errorTerm(i)->evaluateJacobians(Jc);
      J = Jc.asDenseMatrix(optimizer.H().colBlockIndices());
      e.segment<2>(ridx) = dynamic_cast<ErrorTermFs<2>*>(problem.errorTerm(i))->error();
      Eigen::MatrixXd invR = problem.errorTerm(i)->vsInvR();
      //std::cout << "invR:\n" << invR << std::endl;
      //std::cout << "J:\n" << J << std::endl;
      problem.errorTerm(i)->buildHessian(*cloneH, rhs, false);
      Eigen::MatrixXd eH = J.transpose() * invR * J;
      sm::eigen::assertNear(*cloneH, eH, 1e-14, SM_SOURCE_FILE_POS, "Did we build the Hessian correctly? A is from J^T * J and B is from the optimizer.");
    }
  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}



TEST(OptimizerTestSuite, testOptimizerNormalized)
{
  try {
    using namespace aslam::backend;
    boost::shared_ptr<OptimizationProblem> problem_ptr(new OptimizationProblem);
    OptimizationProblem& problem = *problem_ptr;
    std::vector<Eigen::Vector2d> points;
    const int P = 3;
    const int E = 2;
    // Add some design variables.
    double scale = 0.001;
    std::vector< boost::shared_ptr<Point2d> > p2d;
    for (int p = 0; p < P; ++p) {
      Eigen::Vector2d pts = (Eigen::Vector2d::Random());
      points.push_back(pts);
      boost::shared_ptr<Point2d> point(new Point2d(pts));
      // set a random scaling:
      point->setScaling(scale);
      scale *= scale;
      std::cout << "scale [" << p << "] " << point->scaling() << std::endl;
      p2d.push_back(point);
      problem.addDesignVariable(point);
      point->setBlockIndex(p);
      point->setActive(true);
    }
    // Add some error terms.
    std::vector< boost::shared_ptr<LinearErr> > e1;
    for (int p = 0; p < P; ++p) {
      for (int e = 0; e < E; ++e) {
        boost::shared_ptr<LinearErr> err(new LinearErr(p2d[p].get()));
        e1.push_back(err);
        problem.addErrorTerm(err);
        // SCOPED_TRACE("");
        //  ErrorTermTestHarness<2> eh(err.get());
        // SCOPED_TRACE("");
        // eh.testAll();
      }
    }
    // Now let's optimize.
    OptimizerOptions options;
    options.verbose = true;
    options.linearSolver = "dense";
    options.levenbergMarquardtLambdaInit = 10;
    options.levenbergMarquardtEstimateLambdaScale = 1;
    options.doSchurComplement = false;
    options.doLevenbergMarquardt = false;
    options.maxIterations = 1;
    Optimizer optimizer(options);
    optimizer.setProblem(problem_ptr);
    optimizer.optimize();
//    std::cout << "A:" << std::endl << optimizer.H().toDense() << std::endl;
//    std::cout << "b:" << std::endl << optimizer.rhs() << std::endl;
//        boost::shared_ptr<OptimizationProblem> problem_ptr2( new OptimizationProblem );
//        OptimizationProblem & problem2 = *problem_ptr2;
//
//        // reset the points:
//
//
//        for(int p = 0; p < P; p++) {
//
//          p2d[p]->_v = points[p];
//          p2d[p]->_p_v = points[p];
//
//
//          // set a random scaling:
//          double scale = (double)rand() / RAND_MAX;
//          p2d[p]->setScaling(1);
//          std::cout << "scale [" << p << "] " << p2d[p]->scaling() << std::endl;
//          problem2.addDesignVariable(p2d[p]);
//
//
//
//
//        }
//        for(int p = 0; p < e1.size(); p++) {
//          problem2.addErrorTerm( e1[p] );
//        }
//        p2d.clear();
//        // Add some design variables.
//        for(int p = 0; p < P; ++p)
//          {
//          boost::shared_ptr<Point2d> point( new Point2d( points[p] ) );
//          p2d.push_back( point );
//          problem2.addDesignVariable(point);
//          point->setBlockIndex(p);
//          point->setActive(true);
//          }
//
//
//        e1.clear();
//        for(int p = 0; p < P; ++p)
//          {
//      for(int e = 0; e < E; ++e)
//        {
//          boost::shared_ptr<LinearErr> err( new LinearErr(p2d[p].get() ) );
//          e1.push_back( err );
//          problem2.addErrorTerm( err );
//          SCOPED_TRACE("");
//          ErrorTermTestHarness<2> eh(err.get());
//          SCOPED_TRACE("");
//          eh.testAll();
//        }
//          }
//
//        // Now let's optimize.
//        OptimizerOptions options2;
//        options2.verbose = true;
//        options2.linearSolver = "dense";
//        options2.levenbergMarquardtLambdaInit = 10;
//        options2.doSchurComplement = false;
//        options2.doLevenbergMarquardt = false;
//        options2.maxIterations = 10;
//
//        Optimizer optimizer2(options2);
//        optimizer2.setProblem( problem_ptr2 );
//        optimizer2.optimize();
    //   std::cout << "H2: " << std::endl << optimizer2.H().toDense() << std::endl;
    //   std::cout << "H1: " << std::endl << optimizer.H().toDense() << std::endl;
    //   std::cout << "rhs2" << std::endl << optimizer2.rhs() << std::endl;
    //   std::cout << "rhs1" << std::endl << optimizer.rhs() << std::endl;
    // verify
    // sm::eigen::assertNear(optimizer.J(),optimizer2.J(), 1e-3, SM_SOURCE_FILE_POS);
  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}


