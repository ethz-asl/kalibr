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

/** \file OptimizationProblemTest.cpp
    \brief This file tests the OptimizationProblem class.
  */

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/JacobianContainer.hpp>

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

TEST(AslamCalibrationTestSuite, testOptimizationProblem) {
  OptimizationProblem problem;
  auto dv1 = boost::make_shared<VectorDesignVariable<2> >();
  dv1->setActive(true);
  auto dv2 = boost::make_shared<VectorDesignVariable<3> >();
  dv2->setActive(true);
  auto dv3 = boost::make_shared<VectorDesignVariable<4> >();
  dv3->setActive(true);
  auto dv4 = boost::make_shared<VectorDesignVariable<4> >();
  dv4->setActive(true);
  auto et1 = boost::make_shared<DummyErrorTerm>();
  auto et2 = boost::make_shared<DummyErrorTerm>();
  auto et3 = boost::make_shared<DummyErrorTerm>();
  auto et4 = boost::make_shared<DummyErrorTerm>();
  problem.addDesignVariable(dv1, 0);
  problem.addDesignVariable(dv2, 1);
  problem.addDesignVariable(dv3, 1);
  problem.addErrorTerm(et1);
  problem.addErrorTerm(et2);
  problem.addErrorTerm(et3);
  ASSERT_TRUE(problem.isDesignVariableInProblem(dv1.get()));
  ASSERT_TRUE(problem.isDesignVariableInProblem(dv2.get()));
  ASSERT_TRUE(problem.isDesignVariableInProblem(dv3.get()));
  ASSERT_FALSE(problem.isDesignVariableInProblem(dv4.get()));
  ASSERT_THROW(problem.addDesignVariable(dv1), InvalidOperationException);
  ASSERT_EQ(problem.getNumGroups(), 2);
  ASSERT_EQ(problem.getDesignVariablesGroup(1),
    OptimizationProblem::DesignVariablesSP({dv2, dv3}));
  ASSERT_EQ(problem.getDesignVariablesGroup(0),
    OptimizationProblem::DesignVariablesSP({dv1}));
  ASSERT_THROW(problem.getDesignVariablesGroup(2), OutOfBoundException<size_t>);
  problem.setGroupsOrdering({1, 0});
  ASSERT_EQ(problem.getGroupsOrdering(), std::vector<size_t>({1, 0}));
  ASSERT_THROW(problem.setGroupsOrdering({2, 0}), OutOfBoundException<size_t>);
  ASSERT_THROW(problem.setGroupsOrdering({2, 0, 3}),
    OutOfBoundException<size_t>);
  ASSERT_THROW(problem.setGroupsOrdering({0, 0}), OutOfBoundException<size_t>);
  ASSERT_EQ(problem.getGroupId(dv1.get()), 0);
  ASSERT_EQ(problem.getGroupId(dv2.get()), 1);
  ASSERT_EQ(problem.getGroupId(dv3.get()), 1);
  ASSERT_THROW(problem.getGroupId(dv4.get()), InvalidOperationException);
  ASSERT_EQ(problem.numDesignVariables(), 3);
  ASSERT_EQ(problem.designVariable(0), dv2.get());
  ASSERT_EQ(problem.designVariable(1), dv3.get());
  ASSERT_EQ(problem.designVariable(2), dv1.get());
  ASSERT_THROW(problem.designVariable(3), aslam::Exception);
  problem.permuteDesignVariables({1, 0}, 1);
  ASSERT_EQ(problem.designVariable(0), dv3.get());
  ASSERT_THROW(problem.permuteDesignVariables({1, 0}, 2),
    OutOfBoundException<size_t>);
  ASSERT_EQ(problem.getGroupDim(0), 2);
  ASSERT_EQ(problem.getGroupDim(1), 7);
  ASSERT_TRUE(problem.isGroupInProblem(0));
  ASSERT_TRUE(problem.isGroupInProblem(1));
  ASSERT_FALSE(problem.isGroupInProblem(2));
  ASSERT_EQ(problem.numErrorTerms(), 3);
  ASSERT_TRUE(problem.isErrorTermInProblem(et1.get()));
  ASSERT_TRUE(problem.isErrorTermInProblem(et2.get()));
  ASSERT_TRUE(problem.isErrorTermInProblem(et3.get()));
  ASSERT_FALSE(problem.isErrorTermInProblem(et4.get()));
  auto ets = problem.getErrorTerms();
  ASSERT_EQ(ets.size(), 3);
  ASSERT_EQ(ets, OptimizationProblem::ErrorTermsSP({et1, et2, et3}));
  ASSERT_EQ(problem.errorTerm(0), et1.get());
  ASSERT_EQ(problem.errorTerm(1), et2.get());
  ASSERT_EQ(problem.errorTerm(2), et3.get());
  ASSERT_THROW(problem.errorTerm(3), aslam::Exception);
  ASSERT_THROW(problem.addErrorTerm(et1), InvalidOperationException);
  problem.permuteErrorTerms({2, 1, 0});
  auto etsp = problem.getErrorTerms();
  ASSERT_EQ(etsp, OptimizationProblem::ErrorTermsSP({et3, et2, et1}));
  ASSERT_EQ(problem.errorTerm(0), et3.get());
  ASSERT_EQ(problem.errorTerm(1), et2.get());
  ASSERT_EQ(problem.errorTerm(2), et1.get());
  Eigen::MatrixXd dv1Param;
  problem.saveDesignVariables();
  dv1->setParameters(Eigen::Vector2d::Ones());
  problem.restoreDesignVariables();
  dv1->getParameters(dv1Param);
  ASSERT_EQ(dv1Param, Eigen::Vector2d::Zero());
}
