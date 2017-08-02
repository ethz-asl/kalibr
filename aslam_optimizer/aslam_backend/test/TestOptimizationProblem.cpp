#include <sm/eigen/gtest.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>

using namespace aslam::backend;

// Boring unit tests testing add/remove.

class Dv : public DesignVariable {
public:


protected:
  /// \brief Revert the last state update.
  virtual void revertUpdateImplementation() {
  }

  /// \brief Update the design variable.
  virtual void updateImplementation(const double* /* dp */, int /* size */) {
  }

  /// \brief what is the number of dimensions of the perturbation variable.
  virtual int minimalDimensionsImplementation() const {
    return 3;
  }

  /// Returns the content of the design variable
  virtual void getParametersImplementation(Eigen::MatrixXd& /* value */) const {
  }

  /// Sets the content of the design variable
  virtual void setParametersImplementation(const Eigen::MatrixXd& /* value */) {
  }

};


class Et1 : public ErrorTermFs<3> {
public:
  Et1(DesignVariable* dv) {
    setDesignVariables(dv);
  }

protected:
  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    return 0.0;
  }

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(JacobianContainer & /* J */) const {
  }

};


class Et2 : public ErrorTermFs<3> {
public:
  Et2(DesignVariable* dv1 , DesignVariable* dv2) {
    setDesignVariables(dv1, dv2);
  }

protected:
  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    return 0.0;
  }

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(JacobianContainer & /* J */) const {
  }

};


TEST(OptimizationProblemTestSuite, testAddRemoveDesignVariable1)
{
  OptimizationProblem op;
  Dv dv1;
  ASSERT_EQ(0, (int)op.numDesignVariables());
  op.addDesignVariable(&dv1, false);
  ASSERT_EQ(1, (int)op.numDesignVariables());
  op.removeDesignVariable(&dv1);
  ASSERT_EQ(0, (int)op.numDesignVariables());
}



TEST(OptimizationProblemTestSuite, testAddRemoveDesignVariable3)
{
  OptimizationProblem op;
  Dv dv1, dv2;
  ASSERT_EQ(0, (int)op.numDesignVariables());
  op.addDesignVariable(&dv1, false);
  ASSERT_EQ(1, (int)op.numDesignVariables());
  op.addDesignVariable(&dv2, false);
  ASSERT_EQ(2, (int)op.numDesignVariables());
  op.removeDesignVariable(&dv2);
  ASSERT_EQ(1, (int)op.numDesignVariables());
  ASSERT_TRUE(op.designVariable(0) == &dv1);
  op.removeDesignVariable(&dv1);
  ASSERT_EQ(0, (int)op.numDesignVariables());
}


TEST(OptimizationProblemTestSuite, testAddRemoveDesignVariable2)
{
  OptimizationProblem op;
  Dv dv1, dv2;
  ASSERT_EQ(0, (int)op.numDesignVariables());
  op.addDesignVariable(&dv1, false);
  ASSERT_EQ(1, (int)op.numDesignVariables());
  op.addDesignVariable(&dv2, false);
  ASSERT_EQ(2, (int)op.numDesignVariables());
  op.removeDesignVariable(&dv2);
  ASSERT_EQ(1, (int)op.numDesignVariables());
  ASSERT_TRUE(op.designVariable(0) == &dv1);
  op.removeDesignVariable(&dv1);
  ASSERT_EQ(0, (int)op.numDesignVariables());
}


TEST(OptimizationProblemTestSuite,  testAddRemoveErrorTerm1)
{
  OptimizationProblem op;
  Dv dv1, dv2;
  Et1 et11(&dv1), et12(&dv1), et21(&dv2), et22(&dv2);
  Et2 ett1(&dv1, &dv2), ett2(&dv1, &dv2);
  op.addDesignVariable(&dv1, false);
  op.addDesignVariable(&dv2, false);
  op.addErrorTerm(&et11, false);
  op.addErrorTerm(&et12, false);
  op.addErrorTerm(&et21, false);
  op.addErrorTerm(&et22, false);
  op.addErrorTerm(&ett1, false);
  op.addErrorTerm(&ett2, false);
  ASSERT_EQ(6, (int)op.numErrorTerms());
  std::set<ErrorTerm*> et1;
  op.getErrors(&dv1, et1);
  ASSERT_EQ(4, (int)et1.size());
  ASSERT_EQ(1, (int)et1.count(&et11));
  ASSERT_EQ(1, (int)et1.count(&et12));
  ASSERT_EQ(1, (int)et1.count(&ett1));
  ASSERT_EQ(1, (int)et1.count(&ett2));
  std::set<ErrorTerm*> et2;
  op.getErrors(&dv2, et2);
  ASSERT_EQ(4, (int)et2.size());
  ASSERT_EQ(1, (int)et2.count(&et21));
  ASSERT_EQ(1, (int)et2.count(&et22));
  ASSERT_EQ(1, (int)et2.count(&ett1));
  ASSERT_EQ(1, (int)et2.count(&ett2));
  // Remove one of the double errors
  op.removeErrorTerm(&ett2);
  ASSERT_EQ(5, (int)op.numErrorTerms());
  et1.clear();
  op.getErrors(&dv1, et1);
  ASSERT_EQ(3, (int)et1.size());
  ASSERT_EQ(1, (int)et1.count(&et11));
  ASSERT_EQ(1, (int)et1.count(&et12));
  ASSERT_EQ(1, (int)et1.count(&ett1));
  et2.clear();
  op.getErrors(&dv2, et2);
  ASSERT_EQ(3, (int)et2.size());
  ASSERT_EQ(1, (int)et2.count(&et21));
  ASSERT_EQ(1, (int)et2.count(&et22));
  ASSERT_EQ(1, (int)et2.count(&ett1));
  // Remove one of the single errors
  op.removeErrorTerm(&et11);
  ASSERT_EQ(4, (int)op.numErrorTerms());
  et1.clear();
  op.getErrors(&dv1, et1);
  ASSERT_EQ(2, (int)et1.size());
  ASSERT_EQ(1, (int)et1.count(&et12));
  ASSERT_EQ(1, (int)et1.count(&ett1));
  et2.clear();
  op.getErrors(&dv2, et2);
  ASSERT_EQ(3, (int)et2.size());
  ASSERT_EQ(1, (int)et2.count(&et21));
  ASSERT_EQ(1, (int)et2.count(&et22));
  ASSERT_EQ(1, (int)et2.count(&ett1));
  // Now remove a design variable.
  op.removeDesignVariable(&dv1);
  ASSERT_EQ(2, (int)op.numErrorTerms());
  et1.clear();
  op.getErrors(&dv1, et1);
  ASSERT_EQ(0, (int)et1.size());
  et2.clear();
  op.getErrors(&dv2, et2);
  /// This double variable should have been removed when the design variable was removed.
  ASSERT_EQ(0, (int)et2.count(&ett1));
  ASSERT_EQ(2, (int)et2.size());
  ASSERT_EQ(1, (int)et2.count(&et21));
  ASSERT_EQ(1, (int)et2.count(&et22));
}
