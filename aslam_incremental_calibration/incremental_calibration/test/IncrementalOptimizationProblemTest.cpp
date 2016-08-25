/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file IncrementalOptimizationProblemTest.cpp
    \brief This file tests the IncrementalOptimizationProblem class.
  */

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/JacobianContainer.hpp>

#include "aslam/calibration/core/IncrementalOptimizationProblem.h"
#include "aslam/calibration/core/OptimizationProblem.h"
#include "aslam/calibration/data-structures/VectorDesignVariable.h"
#include <aslam/Exceptions.hpp>
#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

class DummyErrorTerm :
  public aslam::backend::ErrorTermFs<3> {
public:
  DummyErrorTerm() = default;
  DummyErrorTerm(const DummyErrorTerm& other) = delete;
  DummyErrorTerm& operator = (const DummyErrorTerm& other) = delete;
  virtual ~DummyErrorTerm() {};
protected:
  virtual double evaluateErrorImplementation() {
    return 0;
  };
  virtual void evaluateJacobiansImplementation(
    aslam::backend::JacobianContainer& J) {};
};

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testIncrementalOptimizationProblem) {
  auto problem1 = boost::make_shared<OptimizationProblem>();
  auto dv1 = boost::make_shared<VectorDesignVariable<2> >();
  dv1->setActive(true);
  auto dv2 = boost::make_shared<VectorDesignVariable<3> >();
  dv2->setActive(true);
  auto dv3 = boost::make_shared<VectorDesignVariable<4> >();
  dv3->setActive(true);
  auto et1 = boost::make_shared<DummyErrorTerm>();
  auto et2 = boost::make_shared<DummyErrorTerm>();
  auto et3 = boost::make_shared<DummyErrorTerm>();
  auto et4 = boost::make_shared<DummyErrorTerm>();
  problem1->addDesignVariable(dv1, 0);
  problem1->addDesignVariable(dv2, 0);
  problem1->addDesignVariable(dv3, 1);
  problem1->addErrorTerm(et1);
  problem1->addErrorTerm(et2);
  problem1->addErrorTerm(et3);
  problem1->addErrorTerm(et4);
  auto problem2 = boost::make_shared<OptimizationProblem>();
  auto dv4 = boost::make_shared<VectorDesignVariable<2> >();
  dv4->setActive(true);
  auto dv5 = boost::make_shared<VectorDesignVariable<3> >();
  dv5->setActive(true);
  auto et5 = boost::make_shared<DummyErrorTerm>();
  auto et6 = boost::make_shared<DummyErrorTerm>();
  auto et7 = boost::make_shared<DummyErrorTerm>();
  problem2->addDesignVariable(dv4, 0);
  problem2->addDesignVariable(dv5, 0);
  problem2->addDesignVariable(dv3, 1);
  problem2->addErrorTerm(et5);
  problem2->addErrorTerm(et6);
  problem2->addErrorTerm(et7);
  auto problem3 = boost::make_shared<OptimizationProblem>();
  auto dv6 = boost::make_shared<VectorDesignVariable<6> >();
  dv6->setActive(true);
  auto dv7 = boost::make_shared<VectorDesignVariable<6> >();
  dv7->setActive(true);
  auto et8 = boost::make_shared<DummyErrorTerm>();
  auto et9 = boost::make_shared<DummyErrorTerm>();
  auto et10 = boost::make_shared<DummyErrorTerm>();
  problem3->addDesignVariable(dv4, 0);
  problem3->addDesignVariable(dv6, 0);
  problem3->addDesignVariable(dv3, 1);
  problem3->addErrorTerm(et8);
  problem3->addErrorTerm(et9);
  IncrementalOptimizationProblem incProblem;
  incProblem.add(problem1);
  incProblem.add(problem2);
  incProblem.add(problem3);
  ASSERT_EQ(incProblem.getNumOptimizationProblems(), 3);
  ASSERT_EQ(incProblem.getOptimizationProblem(0), problem1.get());
  ASSERT_EQ(incProblem.getOptimizationProblem(1), problem2.get());
  ASSERT_EQ(incProblem.getOptimizationProblem(2), problem3.get());
  ASSERT_THROW(incProblem.getOptimizationProblem(3),
    OutOfBoundException<size_t>);
  auto problems = incProblem.getOptimizationProblems();
  ASSERT_EQ(problems.size(), 3);
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv1.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv2.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv3.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv4.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv5.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv6.get()));
  ASSERT_FALSE(incProblem.isDesignVariableInProblem(dv7.get()));
  auto groups = incProblem.getDesignVariablesGroups();
  ASSERT_EQ(groups.size(), 2);
  auto dvs0 = incProblem.getDesignVariablesGroup(0);
  ASSERT_EQ(dvs0.size(), 5);
  ASSERT_EQ(dvs0, IncrementalOptimizationProblem::DesignVariablesP(
    {dv1.get(), dv2.get(), dv4.get(), dv5.get(), dv6.get()}));
  auto dvs1 = incProblem.getDesignVariablesGroup(1);
  ASSERT_EQ(dvs1.size(), 1);
  ASSERT_EQ(dvs1, IncrementalOptimizationProblem::DesignVariablesP(
    {dv3.get()}));
  ASSERT_THROW(incProblem.getDesignVariablesGroup(2),
    OutOfBoundException<size_t>);
  auto ets1 = incProblem.getErrorTerms(0);
  ASSERT_EQ(ets1, IncrementalOptimizationProblem::ErrorTermsSP(
    {et1, et2, et3, et4}));
  auto ets2 = incProblem.getErrorTerms(1);
  ASSERT_EQ(ets2, IncrementalOptimizationProblem::ErrorTermsSP(
    {et5, et6, et7}));
  auto ets3 = incProblem.getErrorTerms(2);
  ASSERT_EQ(ets3, IncrementalOptimizationProblem::ErrorTermsSP(
    {et8, et9}));
  ASSERT_THROW(incProblem.getErrorTerms(3), OutOfBoundException<size_t>);
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et1.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et2.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et3.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et4.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et5.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et6.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et7.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et8.get()));
  ASSERT_TRUE(incProblem.isErrorTermInProblem(et9.get()));
  ASSERT_FALSE(incProblem.isErrorTermInProblem(et10.get()));
  ASSERT_EQ(incProblem.numErrorTerms(), 9);
  ASSERT_EQ(incProblem.errorTerm(0), et1.get());
  ASSERT_EQ(incProblem.errorTerm(1), et2.get());
  ASSERT_EQ(incProblem.errorTerm(2), et3.get());
  ASSERT_EQ(incProblem.errorTerm(3), et4.get());
  ASSERT_EQ(incProblem.errorTerm(4), et5.get());
  ASSERT_EQ(incProblem.errorTerm(5), et6.get());
  ASSERT_EQ(incProblem.errorTerm(6), et7.get());
  ASSERT_EQ(incProblem.errorTerm(7), et8.get());
  ASSERT_EQ(incProblem.errorTerm(8), et9.get());
  ASSERT_THROW(incProblem.errorTerm(9), aslam::Exception);
  ASSERT_EQ(incProblem.getNumGroups(), 2);
  ASSERT_EQ(incProblem.getGroupId(dv1.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv2.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv3.get()), 1);
  ASSERT_EQ(incProblem.getGroupId(dv4.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv5.get()), 0);
  ASSERT_EQ(incProblem.getGroupId(dv6.get()), 0);
  ASSERT_THROW(incProblem.getGroupId(dv7.get()), InvalidOperationException);
  ASSERT_EQ(incProblem.getGroupDim(0), 16);
  ASSERT_EQ(incProblem.getGroupDim(1), 4);
  ASSERT_THROW(incProblem.getGroupDim(2), OutOfBoundException<size_t>);
  ASSERT_TRUE(incProblem.isGroupInProblem(0));
  ASSERT_TRUE(incProblem.isGroupInProblem(1));
  ASSERT_FALSE(incProblem.isGroupInProblem(2));
  ASSERT_EQ(incProblem.getGroupsOrdering(), std::vector<size_t>({0, 1}));
  ASSERT_EQ(incProblem.numDesignVariables(), 6);
  ASSERT_EQ(incProblem.designVariable(0), dv1.get());
  ASSERT_EQ(incProblem.designVariable(1), dv2.get());
  ASSERT_EQ(incProblem.designVariable(2), dv4.get());
  ASSERT_EQ(incProblem.designVariable(3), dv5.get());
  ASSERT_EQ(incProblem.designVariable(4), dv6.get());
  ASSERT_EQ(incProblem.designVariable(5), dv3.get());
  ASSERT_THROW(incProblem.designVariable(7), aslam::Exception);
  incProblem.setGroupsOrdering({1, 0});
  ASSERT_EQ(incProblem.getGroupsOrdering(), std::vector<size_t>({1, 0}));
  ASSERT_EQ(incProblem.designVariable(0), dv3.get());
  ASSERT_THROW(incProblem.setGroupsOrdering({1, 0, 2}),
    OutOfBoundException<size_t>);
  ASSERT_THROW(incProblem.setGroupsOrdering({1, 2}),
    OutOfBoundException<size_t>);
  ASSERT_THROW(incProblem.setGroupsOrdering({0, 0}),
    OutOfBoundException<size_t>);
  incProblem.permuteDesignVariables({1, 0, 2, 3, 4}, 0);
  ASSERT_EQ(incProblem.designVariable(1), dv2.get());
  incProblem.remove(2);
  ASSERT_EQ(incProblem.getNumOptimizationProblems(), 2);
  ASSERT_FALSE(incProblem.isDesignVariableInProblem(dv6.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv3.get()));
  ASSERT_TRUE(incProblem.isDesignVariableInProblem(dv4.get()));
  auto dvs0update = incProblem.getDesignVariablesGroup(0);
  ASSERT_EQ(dvs0update.size(), 4);
  ASSERT_EQ(dvs0update, IncrementalOptimizationProblem::DesignVariablesP(
    {dv2.get(), dv1.get(), dv4.get(), dv5.get()}));
  ASSERT_THROW(incProblem.remove(4), OutOfBoundException<size_t>);
  ASSERT_EQ(incProblem.numErrorTerms(), 7);
  ASSERT_FALSE(incProblem.isErrorTermInProblem(et8.get()));
  ASSERT_FALSE(incProblem.isErrorTermInProblem(et9.get()));
  ASSERT_EQ(incProblem.errorTerm(0), et1.get());
  ASSERT_EQ(incProblem.errorTerm(1), et2.get());
  ASSERT_EQ(incProblem.errorTerm(2), et3.get());
  ASSERT_EQ(incProblem.errorTerm(3), et4.get());
  ASSERT_EQ(incProblem.errorTerm(4), et5.get());
  ASSERT_EQ(incProblem.errorTerm(5), et6.get());
  ASSERT_EQ(incProblem.errorTerm(6), et7.get());
  ASSERT_THROW(incProblem.errorTerm(7), aslam::Exception);
  Eigen::MatrixXd dv1Param;
  Eigen::MatrixXd dv6Param;
  incProblem.saveDesignVariables();
  dv1->setParameters(Eigen::Vector2d::Ones());
  dv6->setParameters(Eigen::Matrix<double, 6, 1>::Ones());
  incProblem.restoreDesignVariables();
  dv1->getParameters(dv1Param);
  dv6->getParameters(dv6Param);
  ASSERT_EQ(dv1Param, Eigen::Vector2d::Zero());
  ASSERT_EQ(dv6Param, Eigen::MatrixXd::Ones(6, 1));
}
