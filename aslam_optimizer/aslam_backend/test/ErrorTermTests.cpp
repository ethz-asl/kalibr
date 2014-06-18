#include <sm/eigen/gtest.hpp>
#include "SampleDvAndError.hpp"


TEST(ErrorTermTestSuite, testInvR)
{
  using namespace aslam::backend;
  int D = 4;
  int E = 6;
  std::vector<DesignVariable*> dvs;
  std::vector<ErrorTerm*> errs;
  try {
    std::vector<int> blocks;
    int block = 0;
    for (size_t i = 0; i < errs.size(); ++i) {
      block += errs[i]->dimension();
      blocks.push_back(block);
      boost::shared_ptr<GemanMcClureMEstimator> me(new GemanMcClureMEstimator(errs[i]->getRawSquaredError()));
    }
    buildSystem(D, E, dvs, errs);
    for (size_t i = 0; i < errs.size(); ++i) {
      JacobianContainer jc(errs[i]->dimension());
      ErrorTerm* e = errs[i];
      Eigen::MatrixXd invR = sm::eigen::randomCovarianceXd(e->dimension());
      e->vsSetInvR(invR);
      e->evaluateError();
      Eigen::VectorXd ee = e->vsError();
      // Check that the raw squared error is e^T invR e
      double rse = e->getRawSquaredError();
      double trueRse = ee.dot(invR * ee);
      ASSERT_NEAR(rse, trueRse, 1e-6);
      // Check that the weighted squared error is w e^T invR e
      double w = e->getMEstimatorWeight(trueRse);
      double trueWRse = w * trueRse;
      double wrse = e->getWeightedSquaredError();
      ASSERT_NEAR(wrse, trueWRse, 1e-6);
      // Check that the weighted squared error, d,  produces  w e^T invR e == d^T d
      Eigen::VectorXd we;
      e->getWeightedError(we, false);
      // No M-estimator.
      rse = we.dot(we);
      ASSERT_NEAR(rse, trueRse, 1e-6);
      e->getWeightedError(we, true);
      // with M-estimator.
      wrse = we.dot(we);
      ASSERT_NEAR(wrse, trueWRse, 1e-6);
      JacobianContainer jcRaw(e->dimension());
      e->evaluateJacobians(jcRaw);
      {
        // No M-Estimator
        e->getWeightedJacobians(jc, false);
        e->getWeightedError(we, false);
        Eigen::MatrixXd J = jcRaw.asDenseMatrix();
        Eigen::MatrixXd wJ = jc.asDenseMatrix();
        Eigen::MatrixXd JtInvRJ = J.transpose() * invR * J;
        Eigen::MatrixXd wJtwJ = wJ.transpose() * wJ;
        // Check that wJ^T wJ == w J^T invR J
        ASSERT_DOUBLE_MX_EQ(JtInvRJ, wJtwJ, 1e-6, "Checking the weighted Jacobian");
        // Check that wJ^T we == w J^T invR e
        Eigen::VectorXd JtInvRe = J.transpose() * invR * ee;
        Eigen::VectorXd wJtwe = wJ.transpose() * we;
        ASSERT_DOUBLE_MX_EQ(JtInvRe, wJtwe, 1e-6, "Checking the weighted rhs");
      }
      {
        // with M-Estimator
        e->getWeightedJacobians(jc, true);
        e->getWeightedError(we, true);
        w = e->getMEstimatorWeight(trueRse);
        Eigen::MatrixXd J = jcRaw.asDenseMatrix();
        Eigen::MatrixXd wJ = jc.asDenseMatrix();
        Eigen::MatrixXd JtInvRJ =  J.transpose() * invR * J * w;
        Eigen::MatrixXd wJtwJ = wJ.transpose() * wJ;
        // Check that wJ^T wJ == w J^T invR J
        ASSERT_DOUBLE_MX_EQ(JtInvRJ, wJtwJ, 1e-6, "Checking the weighted Jacobian");
        // Check that wJ^T we == w J^T invR e
        Eigen::VectorXd JtInvRe = J.transpose() * invR * ee * w;
        Eigen::VectorXd wJtwe = wJ.transpose() * we;
        ASSERT_DOUBLE_MX_EQ(JtInvRe, wJtwe, 1e-6, "Checking the weighted rhs");
      }
    }
    deleteSystem(dvs, errs);
  } catch (const std::exception& e) {
    deleteSystem(dvs, errs);
    FAIL() << e.what();
  }
}



