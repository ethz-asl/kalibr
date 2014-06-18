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

/** \file VectorDesignVariableTest.cpp
    \brief This file tests the VectorDesignVariable class.
  */

#include <cstddef>
#include <iostream>
#include <iomanip>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cholmod.h>
#include <SuiteSparseQR.hpp>

#include "aslam/calibration/statistics/NormalDistribution.h"
#include "aslam/calibration/core/LinearSolver.h"
#include "aslam/calibration/algorithms/linalg.h"
#include "aslam/calibration/base/Timestamp.h"

void evaluateSVDSPQRSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    const Eigen::VectorXd& x, double tol = 1e-9) {
  cholmod_common cholmod;
  cholmod_l_start(&cholmod);
  cholmod_sparse* A_CS = aslam::calibration::eigenDenseToCholmodSparseCopy(A,
    &cholmod);
  cholmod_dense b_CD;
  aslam::calibration::eigenDenseToCholmodDenseView(b, &b_CD);
  Eigen::VectorXd x_est;
  aslam::calibration::LinearSolver linearSolver;
  for (std::ptrdiff_t i = 1; i < A.cols(); ++i) {
//    double before = aslam::calibration::Timestamp::now();
    linearSolver.solve(A_CS, &b_CD, i, x_est);
//    double after = aslam::calibration::Timestamp::now();
    double error = (b - A * x_est).norm();
//    std::cout << std::fixed << std::setprecision(18) << "noscale: " << "error: "
//      << error << " est_diff: " << (x - x_est).norm() << " time: "
//      << after - before << std::endl;
    ASSERT_NEAR(error, 0, tol);
    linearSolver.getOptions().columnScaling = true;
//    before = aslam::calibration::Timestamp::now();
    linearSolver.solve(A_CS, &b_CD, i, x_est);
//    after = aslam::calibration::Timestamp::now();
    error = (b - A * x_est).norm();
//    std::cout << std::fixed << std::setprecision(18) << "onscale: " << "error: "
//      << error << " est_diff: " << (x - x_est).norm() << " time: "
//      << after - before << std::endl;
    linearSolver.getOptions().columnScaling = false;
    ASSERT_NEAR(error, 0, tol);
//    std::cout << "SVD rank: " << linearSolver.getSVDRank() << std::endl;
//    std::cout << "SVD rank deficiency: " << linearSolver.getSVDRankDeficiency()
//      << std::endl;
//    std::cout << "QR rank: " << linearSolver.getQRRank() << std::endl;
//    std::cout << "QR rank deficiency: "
//      << linearSolver.getQRRankDeficiency() << std::endl;
//    std::cout << "SV gap: " << linearSolver.getSvGap() << std::endl;
  }
  cholmod_l_free_sparse(&A_CS, &cholmod);
  cholmod_l_finish(&cholmod);
}

void evaluateSVDSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    const Eigen::VectorXd& x) {
//  const double before = aslam::calibration::Timestamp::now();
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,
    Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd x_est = svd.solve(b);
//  const double after = aslam::calibration::Timestamp::now();
//  const double error = (b - A * x_est).norm();
//  std::cout << std::fixed << std::setprecision(18) << "error: " << error
//    << " est_diff: " << (x - x_est).norm() << " time: " << after - before
//    << std::endl;
//  std::cout << "estimated rank: " << svd.nonzeroSingularValues() << std::endl;
//  std::cout << "estimated rank deficiency: "
//    << A.cols() - svd.nonzeroSingularValues() << std::endl;
}

void evaluateSPQRSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    const Eigen::VectorXd& x) {
  cholmod_common cholmod;
  cholmod_l_start(&cholmod);
  cholmod_sparse* A_CS = aslam::calibration::eigenDenseToCholmodSparseCopy(A,
    &cholmod);
  cholmod_dense b_CD;
  aslam::calibration::eigenDenseToCholmodDenseView(b, &b_CD);
  Eigen::VectorXd x_est;
//  const double before = aslam::calibration::Timestamp::now();
  SuiteSparseQR_factorization<double>* factor = SuiteSparseQR_factorize<double>(
    SPQR_ORDERING_BEST, SPQR_DEFAULT_TOL, A_CS, &cholmod);
  cholmod_dense* Qtb = SuiteSparseQR_qmult<double>(SPQR_QTX, factor, &b_CD,
    &cholmod);
  cholmod_dense* x_est_cd = SuiteSparseQR_solve<double>(SPQR_RETX_EQUALS_B,
    factor, Qtb, &cholmod);
  cholmod_l_free_dense(&Qtb, &cholmod);
  aslam::calibration::cholmodDenseToEigenDenseCopy(x_est_cd, x_est);
  cholmod_l_free_dense(&x_est_cd, &cholmod);
//  std::cout << "estimated rank: " << factor->rank << std::endl;
//  std::cout << "estimated rank deficiency: " << A.cols() - factor->rank
//    << std::endl;
  SuiteSparseQR_free(&factor, &cholmod);
//  const double after = aslam::calibration::Timestamp::now();
//  const double error = (b - A * x_est).norm();
//  std::cout << std::fixed << std::setprecision(18) << "error: " << error
//    << " est_diff: " << (x - x_est).norm() << " time: " << after - before
//    << std::endl;
  cholmod_l_free_sparse(&A_CS, &cholmod);
  cholmod_l_finish(&cholmod);
}

TEST(AslamCalibrationTestSuite, testLinearSolver) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(100, 30);
  const Eigen::VectorXd x = Eigen::VectorXd::Random(30);
  Eigen::VectorXd b = A * x;

//  std::cout << "-------------------------------------------------" << std::endl;
//  std::cout << "|                  Standard case                |" << std::endl;
//  std::cout << "-------------------------------------------------" << std::endl;
//  std::cout << "SVD-SPQR solver" << std::endl;
  evaluateSVDSPQRSolver(A, b, x);
//  std::cout << "SVD solver" << std::endl;
  evaluateSVDSolver(A, b, x);
//  std::cout << "SPQR solver" << std::endl;
  evaluateSPQRSolver(A, b, x);

//  std::cout << "-------------------------------------------------" << std::endl;
//  std::cout << "|                 Badly scaled case             |" << std::endl;
//  std::cout << "-------------------------------------------------" << std::endl;
//  A.col(2) = 1e6 * A.col(2);
//  A.col(28) = 1e6 * A.col(28);
//  b = A * x;
//  std::cout << "SVD-SPQR solver" << std::endl;
//  evaluateSVDSPQRSolver(A, b, x, 1e-3);
//  std::cout << "SVD solver" << std::endl;
//  evaluateSVDSolver(A, b, x);
//  std::cout << "SPQR solver" << std::endl;
//  evaluateSPQRSolver(A, b, x);

//  std::cout << "-------------------------------------------------" << std::endl;
//  std::cout << "|                 Rank-deficient case 1         |" << std::endl;
//  std::cout << "-------------------------------------------------" << std::endl;
//  A = Eigen::MatrixXd::Random(100, 30);
//  A.col(10) = Eigen::VectorXd::Zero(A.rows());
//  b = A * x;
//  std::cout << "SVD-SPQR solver" << std::endl;
//  evaluateSVDSPQRSolver(A, b, x, 1e-3);
//  std::cout << "SVD solver" << std::endl;
//  evaluateSVDSolver(A, b, x);
//  std::cout << "SPQR solver" << std::endl;
//  evaluateSPQRSolver(A, b, x);

//  std::cout << "-------------------------------------------------" << std::endl;
//  std::cout << "|                 Rank-deficient case 2         |" << std::endl;
//  std::cout << "-------------------------------------------------" << std::endl;
//  A = Eigen::MatrixXd::Random(100, 30);
//  A.col(10) = 2 * A.col(1) + 5 * A.col(20);
//  b = A * x;
//  std::cout << "SVD-SPQR solver" << std::endl;
//  evaluateSVDSPQRSolver(A, b, x, 1e-3);
//  std::cout << "SVD solver" << std::endl;
//  evaluateSVDSolver(A, b, x);
//  std::cout << "SPQR solver" << std::endl;
//  evaluateSPQRSolver(A, b, x);

//  std::cout << "-------------------------------------------------" << std::endl;
//  std::cout << "|                 Near rank-deficient case 1    |" << std::endl;
//  std::cout << "-------------------------------------------------" << std::endl;
//  A = Eigen::MatrixXd::Random(100, 30);
//  A.col(10) = Eigen::VectorXd::Zero(A.rows());
//  b = A * x;
//  A.col(10) = aslam::calibration::NormalDistribution<100>(
//    Eigen::VectorXd::Zero(A.rows()),
//    1e-6 * Eigen::MatrixXd::Identity(A.rows(), A.rows())).getSample();
//  std::cout << "SVD-SPQR solver" << std::endl;
//  evaluateSVDSPQRSolver(A, b, x, 1e-3);
//  std::cout << "SVD solver" << std::endl;
//  evaluateSVDSolver(A, b, x);
//  std::cout << "SPQR solver" << std::endl;
//  evaluateSPQRSolver(A, b, x);

//  std::cout << "-------------------------------------------------" << std::endl;
//  std::cout << "|                 Near rank-deficient case 2    |" << std::endl;
//  std::cout << "-------------------------------------------------" << std::endl;
//  A = Eigen::MatrixXd::Random(100, 30);
//  A.col(10) = 2 * A.col(1) + 5 * A.col(20);
//  b = A * x;
//  A.col(10) = aslam::calibration::NormalDistribution<100>(A.col(10),
//    1e-20 * Eigen::MatrixXd::Identity(A.rows(), A.rows())).getSample();
//  std::cout << "SVD-SPQR solver" << std::endl;
//  evaluateSVDSPQRSolver(A, b, x, 1e-3);
//  std::cout << "SVD solver" << std::endl;
//  evaluateSVDSolver(A, b, x);
//  std::cout << "SPQR solver" << std::endl;
//  evaluateSPQRSolver(A, b, x);
}
