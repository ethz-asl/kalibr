#include <boost/make_shared.hpp>

#include <sm/eigen/gtest.hpp>
#include "SampleDvAndError.hpp"

TEST(ErrorTermTestSuite, testMEstimatorGetter) {
  using aslam::backend::FixedWeightMEstimator;
  using aslam::backend::GemanMcClureMEstimator;

  Eigen::Vector2d v;
  Point2d p(v);
  LinearErr error_term(&p);
  boost::shared_ptr<FixedWeightMEstimator> mestimator(
      new FixedWeightMEstimator(4));
  error_term.setMEstimatorPolicy(mestimator);
  EXPECT_EQ(error_term.getCurrentMEstimatorWeight(), 4);
  error_term.getMEstimatorPolicy<FixedWeightMEstimator>()->setWeight(5);
  EXPECT_EQ(error_term.getCurrentMEstimatorWeight(), 5);
  boost::shared_ptr<GemanMcClureMEstimator> null_ptr =
      error_term.getMEstimatorPolicy<GemanMcClureMEstimator>();
  EXPECT_FALSE(null_ptr);
}

TEST(ErrorTermTestSuite, testInvR)
{
  using namespace aslam::backend;
  int D = 4;
  int E = 6;
  std::vector<DesignVariable*> dvs;
  std::vector<ErrorTerm*> errs;
  try {
    buildSystem(D, E, dvs, errs);
    for (size_t i = 0; i < errs.size(); ++i) {
      boost::shared_ptr<GemanMcClureMEstimator>
          me(new GemanMcClureMEstimator(0.321 * i)); // just some sigma2 value
      errs[i]->setMEstimatorPolicy(me);
    }
    for (size_t i = 0; i < errs.size(); ++i) {
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
      JacobianContainerSparse<> jcRaw(e->dimension());
      e->evaluateJacobians(jcRaw);
      {
        // No M-Estimator
        JacobianContainerSparse<> jc(errs[i]->dimension());
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
        JacobianContainerSparse<> jc(errs[i]->dimension());
        e->getWeightedJacobians(jc, true);
        e->getWeightedError(we, true);
        w = e->getMEstimatorWeight(trueRse);
        ASSERT_NE(1.0, w);
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

TEST(ErrorTermTestSuite, testNonSquaredErrorTerm) {
  using namespace aslam::backend;
  try {

    Eigen::Vector2d v(1.0, 2.0);
    Point2d p(v);
    p.setBlockIndex(0);
    TestNonSquaredError::grad_t g(1.0, 2.0);
    TestNonSquaredError e(&p, g);

    e._p = 3.0;
    for (double w : {0.1, 1.0, 10.}){
      e.setWeight(w);
      for (double mEstWeight : {0.1, 1.0, 10.}){
        if(mEstWeight != 1.0){
          e.setMEstimatorPolicy(boost::make_shared<FixedWeightMEstimator>(mEstWeight));
        }
        else {
          e.clearMEstimatorPolicy();
        }

        EXPECT_DOUBLE_EQ(w * 4.0, e.updateRawError());
        EXPECT_DOUBLE_EQ(w * mEstWeight * 4.0, e.getWeightedError());
        EXPECT_DOUBLE_EQ(e.getRawError(), e.getError(false));
        EXPECT_DOUBLE_EQ(e.getWeightedError(), e.getError(true));

        Eigen::MatrixXd grad;
        grad.resize(1,2); grad << 4., 8.;
        grad *= w;

        JacobianContainerSparse<> jc(1);

        // raw Jacobian
        jc.clear();
        e.evaluateJacobians(jc, false);
        Eigen::MatrixXd J = jc.asDenseMatrix();
        EXPECT_TRUE((J.array() == grad.array()).all()) << "J: " << J << std::endl <<
            "Grad: " << grad << std::endl;

        jc.clear();
        e.evaluateRawJacobians(jc);
        J = jc.asDenseMatrix();
        EXPECT_TRUE((J.array() == grad.array()).all()) << "J: " << J << std::endl <<
            "Grad: " << grad << std::endl;


        // weighted Jacobian
        jc.clear();
        e.evaluateJacobians(jc, true);
        J = jc.asDenseMatrix();
        grad *= mEstWeight;
        EXPECT_TRUE((J.array() == grad.array()).all()) << "J: " << J << std::endl <<
            "Grad: " << grad << std::endl;

        jc.clear();
        e.evaluateWeightedJacobians(jc);
        J = jc.asDenseMatrix();
        EXPECT_TRUE((J.array() == grad.array()).all()) << "J: " << J << std::endl <<
            "Grad: " << grad << std::endl;

      }
    }
  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}



