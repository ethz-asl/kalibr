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

/** \file AlgorithmsTest.cpp
    \brief This file tests the algorithms.
  */

#include <cmath>
#include <cstddef>

#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <sm/eigen/gtest.hpp>

#include <aslam/backend/Cholmod.hpp>
#include <aslam/backend/CompressedColumnMatrix.hpp>

#include "aslam/calibration/algorithms/permute.h"
#include "aslam/calibration/algorithms/matrixOperations.h"
#include "aslam/calibration/algorithms/marginalize.h"
#include "aslam/calibration/statistics/UniformDistribution.h"
#include "aslam/calibration/exceptions/OutOfBoundException.h"

using namespace aslam::calibration;
using namespace aslam::backend;

TEST(AslamCalibrationTestSuite, testAlgorithms) {
  UniformDistribution<double> dist(0, 100);
  const std::vector<double> input = {dist.getSample(), dist.getSample(),
    dist.getSample(), dist.getSample(), dist.getSample(), dist.getSample()};
  const std::vector<size_t> p = {1, 3, 0, 2, 5, 4};
  auto pinput = input;
  permute(pinput, p);
  ASSERT_EQ(pinput, std::vector<double>({input[p[0]], input[p[1]], input[p[2]],
    input[p[3]], input[p[4]], input[p[5]]}));
  ASSERT_THROW(permute(pinput, std::vector<size_t>({1, 3, 0})),
    OutOfBoundException<size_t>);
  ASSERT_THROW(permute(pinput, std::vector<size_t>({10, 3, 0, 2, 5, 4})),
    OutOfBoundException<size_t>);
  Eigen::MatrixXd J = Eigen::MatrixXd::Random(100, 10);
  Eigen::MatrixXd JCov1 = (J.transpose() * J).inverse();
  Cholmod<std::ptrdiff_t> cholmod;
  CompressedColumnMatrix<std::ptrdiff_t> JTransposeSparse;
  JTransposeSparse.fromDense(J.transpose());
  cholmod_sparse JTransposeSparseCholmod;
  JTransposeSparse.getView(&JTransposeSparseCholmod);
  cholmod_sparse* RFactorSparseCholmod;
  cholmod.getR(&JTransposeSparseCholmod, &RFactorSparseCholmod);
  CompressedColumnMatrix<std::ptrdiff_t> RFactorSparse;
  RFactorSparse.fromCholmodSparse(RFactorSparseCholmod);
  cholmod.free(RFactorSparseCholmod);
  Eigen::MatrixXd RFactor;
  RFactorSparse.toDenseInto(RFactor);
  Eigen::MatrixXd JCov2 = (RFactor.transpose() * RFactor).inverse();
  sm::eigen::assertNear(JCov1, JCov2, 1e-6, SM_SOURCE_FILE_POS,
    "J'*J and R'*R does not match");
  Eigen::MatrixXd JCov3 = computeCovariance(RFactorSparse, 0,
    RFactorSparse.cols() - 1);
  sm::eigen::assertNear(JCov1, JCov3, 1e-6, SM_SOURCE_FILE_POS,
    "J'*J and covariance recovery does not match");
  Eigen::MatrixXd JCov4 = computeCovariance(RFactor, 0, RFactor.cols() - 1);
  sm::eigen::assertNear(JCov3, JCov4, 1e-6, SM_SOURCE_FILE_POS,
    "sparse and dense covariance recovery does not match");
  const double sumLogDiagR1 =
    computeSumLogDiagR(RFactorSparse, 0, RFactorSparse.cols() - 1);
  const double sumLogDiagR2 =
    computeSumLogDiagR(RFactor, 0, RFactor.cols() - 1);
  ASSERT_NEAR(sumLogDiagR1, sumLogDiagR2, 1e-9);
  const double RFactorDet = RFactor.determinant();
  ASSERT_NEAR(sumLogDiagR1, std::log2(std::fabs(RFactorDet)), 1e-9);
  const double JCov1Det = JCov1.determinant();
  ASSERT_NEAR(sumLogDiagR1, std::fabs(std::log2(std::fabs(JCov1Det))) / 2,
    1e-9);
  Eigen::MatrixXd NS, CS, Sigma, SigmaP, Omega;
  const double svLogSum = marginalize(JTransposeSparse, 0, NS, CS, Sigma,
    SigmaP, Omega);
  sm::eigen::assertNear(JCov1, Sigma, 1e-6, SM_SOURCE_FILE_POS,
    "SVD covariance recovery failed");
  ASSERT_NEAR(svLogSum,  std::fabs(std::log2(std::fabs(JCov1Det))), 1e-9);
}
