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

#include "aslam/calibration/core/LinearSolver.h"

#include <cmath>

#include <algorithm>
#include <iostream>

#include <SuiteSparseQR.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/CompressedColumnMatrix.hpp>

#include "aslam/calibration/base/Timestamp.h"
#include "aslam/calibration/algorithms/linalg.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/exceptions/OutOfBoundException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    LinearSolver::LinearSolver(const Options& options) :
        _options(options),
        _factor(NULL),
        _svdRank(-1),
        _svGap(std::numeric_limits<double>::infinity()),
        _svdRankDeficiency(-1),
        _margStartIndex(-1),
        _svdTolerance(-1.0),
        _linearSolverTime(0.0),
        _marginalAnalysisTime(0.0) {
      cholmod_l_start(&_cholmod);
      _cholmod.SPQR_grain = 16; // maybe useless
    }

    LinearSolver::LinearSolver(const sm::PropertyTree& config) :
        _factor(NULL),
        _svdRank(-1),
        _svGap(std::numeric_limits<double>::infinity()),
        _svdRankDeficiency(-1),
        _margStartIndex(-1),
        _svdTolerance(-1.0),
        _linearSolverTime(0.0),
        _marginalAnalysisTime(0.0) {
      cholmod_l_start(&_cholmod);
      _cholmod.SPQR_grain = 16; // maybe useless
      _options.columnScaling = config.getBool("columnScaling",
        _options.columnScaling);
      _options.epsNorm = config.getDouble("epsNorm", _options.epsNorm);
      _options.epsSVD = config.getDouble("epsSVD", _options.epsSVD);
      _options.epsQR = config.getDouble("epsQR", _options.epsQR);
      _options.svdTol = config.getDouble("svdTol", _options.svdTol);
      _options.qrTol = config.getDouble("qrTol", _options.qrTol);
      _options.verbose = config.getBool("verbose", _options.verbose);
      // TODO: ideally, use constructor delegation, not available for now
    }

    LinearSolver::~LinearSolver() {
      clear();
      cholmod_l_finish(&_cholmod);
      if (_options.verbose && getMemoryUsage())
        std::cerr << __PRETTY_FUNCTION__ << ": cholmod memory leak"
          << std::endl;
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const LinearSolver::Options& LinearSolver::getOptions() const {
      return _options;
    }

    LinearSolver::Options& LinearSolver::getOptions() {
      return _options;
    }

    std::string LinearSolver::name() const {
      return std::string("marginal_spqr_svd");
    }

    std::ptrdiff_t LinearSolver::getSVDRank() const {
      return _svdRank;
    }

    std::ptrdiff_t LinearSolver::getSVDRankDeficiency() const {
      return _svdRankDeficiency;
    }

    double LinearSolver::getSvGap() const {
      return _svGap;
    }

    std::ptrdiff_t LinearSolver::getQRRank() const {
      if (_factor && _factor->QRnum)
        return _factor->rank;
      else
        return -1;
    }

    std::ptrdiff_t LinearSolver::getQRRankDeficiency() const {
      if (_factor && _factor->QRsym && _factor->QRnum)
        return _factor->QRsym->n - _factor->rank;
      else
        return -1;
    }

    std::ptrdiff_t LinearSolver::getMargStartIndex() const {
      return _margStartIndex;
    }

    void LinearSolver::setMargStartIndex(std::ptrdiff_t index) {
      _margStartIndex = index;
    }

    const aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&
        LinearSolver::getJacobianTranspose() const {
      return _jacobianBuilder.J_transpose();
    }

    double LinearSolver::getQRTolerance() const {
      if (_factor && _factor->QRsym && _factor->QRnum)
        return _factor->tol;
      else
        return SPQR_DEFAULT_TOL;
    }

    double LinearSolver::getSVDTolerance() const {
      return _svdTolerance;
    }

    const Eigen::VectorXd& LinearSolver::getSingularValues() const {
      return _singularValues;
    }

    const Eigen::MatrixXd& LinearSolver::getMatrixU() const {
      return _matrixU;
    }

    const Eigen::MatrixXd& LinearSolver::getMatrixV() const {
      return _matrixV;
    }

    Eigen::MatrixXd LinearSolver::getNullSpace() const {
      if (_svdRank != -1 && _svdRank <= _matrixV.cols())
        return _matrixV.rightCols(_matrixV.cols() - _svdRank);
      else
        return Eigen::MatrixXd(0, 0);
    }

    Eigen::MatrixXd LinearSolver::getRowSpace() const {
      if (_svdRank != -1 && _svdRank <= _matrixV.cols())
        return _matrixV.leftCols(_svdRank);
      else
        return Eigen::MatrixXd(0, 0);
    }

    Eigen::MatrixXd LinearSolver::getCovariance() const {
      if (_svdRank != -1 && _svdRank <= _matrixV.cols())
        return _matrixV.leftCols(_svdRank) *
          _singularValues.head(_svdRank).asDiagonal().inverse() *
          _matrixV.leftCols(_svdRank).adjoint();
      else
        return Eigen::MatrixXd(0, 0);
    }

    Eigen::MatrixXd LinearSolver::getRowSpaceCovariance() const {
      if (_svdRank != -1)
        return _singularValues.head(_svdRank).asDiagonal().inverse();
      else
        return Eigen::MatrixXd(0, 0);
    }

    double LinearSolver::getSingularValuesLog2Sum() const {
      if (_svdRank != -1)
        return _singularValues.head(_svdRank).array().log().sum() / std::log(2);
      else
        return 0.0;
    }

    size_t LinearSolver::getPeakMemoryUsage() const {
      return _cholmod.memory_usage;
    }

    size_t LinearSolver::getMemoryUsage() const {
      return _cholmod.memory_inuse;
    }

    double LinearSolver::getNumFlops() const {
      return _cholmod.SPQR_xstat[0];
    }

    double LinearSolver::getLinearSolverTime() const {
      return _linearSolverTime;
    }

    double LinearSolver::getMarginalAnalysisTime() const {
      return _marginalAnalysisTime;
    }

    double LinearSolver::getSymbolicFactorizationTime() const {
      if (_factor && _factor->QRsym)
        return _cholmod.other1[1];
      else
        return 0.0;
    }

    double LinearSolver::getNumericFactorizationTime() const {
      if (_factor && _factor->QRnum)
        return _cholmod.other1[2];
      else
        return 0.0;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void LinearSolver::buildSystem(size_t numThreads, bool useMEstimator) {
      _jacobianBuilder.buildSystem(numThreads, useMEstimator);
    }

    bool LinearSolver::solveSystem(Eigen::VectorXd& dx) {
      aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>& Jt =
        _jacobianBuilder.J_transpose();
      cholmod_sparse Jt_CS;
      Jt.getView(&Jt_CS);
      cholmod_sparse* J_CS = cholmod_l_transpose(&Jt_CS, 1, &_cholmod);
      if (J_CS == NULL)
        return false;
      cholmod_dense e_CD;
      eigenDenseToCholmodDenseView(_e, &e_CD);
      bool status = true;
      try {
        solve(J_CS, &e_CD, _margStartIndex, dx);
        if (_options.verbose) {
          std::cout << "SVD rank: " << getSVDRank() << std::endl;
          std::cout << "SVD rank deficiency: " << getSVDRankDeficiency()
            << std::endl;
          std::cout << "QR rank: " << getQRRank() << std::endl;
          std::cout << "QR rank deficiency: " << getQRRankDeficiency()
            << std::endl;
        }
      }
      catch (const OutOfBoundException<std::ptrdiff_t>& e) {
        if (_options.verbose)
          std::cerr << e.what() << std::endl;
        status = false;
      }
      catch (const InvalidOperationException& e) {
        if (_options.verbose)
          std::cerr << e.what() << std::endl;
        status = false;
      }
      catch (...) {
        if (_options.verbose)
          std::cerr << __PRETTY_FUNCTION__ << ": unknown exception"
            << std::endl;
        status = false;
      }
      cholmod_l_free_sparse(&J_CS, &_cholmod);
      return status;
    }

    double LinearSolver::rhsJtJrhs() {
      return 0.0;
    }

    void LinearSolver::initMatrixStructureImplementation(const
        std::vector<aslam::backend::DesignVariable*>& dvs, const
        std::vector<aslam::backend::ErrorTerm*>& errors, bool
        useDiagonalConditioner) {
      clear();
      _jacobianBuilder.initMatrixStructure(dvs, errors);
    }

    void LinearSolver::solve(cholmod_sparse* A, cholmod_dense* b,
        std::ptrdiff_t j, Eigen::VectorXd& x) {
      const double t0 = Timestamp::now();
      if (A->nrow != b->nrow)
        throw InvalidOperationException("inconsistent A and b", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      cholmod_sparse* A_l = columnSubmatrix(A, 0, j - 1, &_cholmod);
      cholmod_dense* G_l = NULL;
      if (_options.columnScaling) {
        try {
          G_l = columnScalingMatrix(A_l, &_cholmod, _options.epsNorm);
        }
        catch (...) {
          cholmod_l_free_sparse(&A_l, &_cholmod);
          throw;
        }
        if (!cholmod_l_scale(G_l, CHOLMOD_COL, A_l, &_cholmod)) {
          cholmod_l_free_dense(&G_l, &_cholmod);
          cholmod_l_free_sparse(&A_l, &_cholmod);
          throw InvalidOperationException("cholmod_l_scale failed", __FILE__,
            __LINE__, __PRETTY_FUNCTION__);
        }
      }
      if (_factor && _factor->QRsym &&
          (_factor->QRsym->m != static_cast<std::ptrdiff_t>(A_l->nrow) ||
          _factor->QRsym->n != static_cast<std::ptrdiff_t>(A_l->ncol) ||
          _factor->QRsym->anz != static_cast<std::ptrdiff_t>(A_l->nzmax)))
        clear();
      if (!_factor) {
        const double t2 = Timestamp::now();
        _factor = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
          SPQR_DEFAULT_TOL, A_l, &_cholmod);
        const double t3 = Timestamp::now();
        _cholmod.other1[1] = t3 - t2;
        if (_factor == NULL) {
          cholmod_l_free_sparse(&A_l, &_cholmod);
          if (G_l)
            cholmod_l_free_dense(&G_l, &_cholmod);
          throw InvalidOperationException("SuiteSparseQR_symbolic failed",
            __FILE__, __LINE__, __PRETTY_FUNCTION__);
        }
      }
      const double qrTolerance = (_options.qrTol != -1.0) ? _options.qrTol :
        qrTol(A_l, &_cholmod, _options.epsQR);
      const double t2 = Timestamp::now();
      const int status = SuiteSparseQR_numeric<double>(qrTolerance, A_l,
        _factor, &_cholmod);
      const double t3 = Timestamp::now();
      _cholmod.other1[2] = t3 - t2;
      cholmod_l_free_sparse(&A_l, &_cholmod);
      if (!status) {
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        throw InvalidOperationException("SuiteSparseQR_numeric failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      }
      cholmod_sparse* A_r;
      try {
        A_r = columnSubmatrix(A, j, A->ncol - 1, &_cholmod);
      }
      catch (...) {
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        throw;
      }
      cholmod_dense* G_r = NULL;
      if (_options.columnScaling) {
        try {
          G_r = columnScalingMatrix(A_r, &_cholmod, _options.epsNorm);
        }
        catch (...) {
          cholmod_l_free_sparse(&A_r, &_cholmod);
          cholmod_l_free_dense(&G_l, &_cholmod);
          throw;
        }
        if (!cholmod_l_scale(G_r, CHOLMOD_COL, A_r, &_cholmod)) {
          cholmod_l_free_sparse(&A_r, &_cholmod);
          cholmod_l_free_dense(&G_r, &_cholmod);
          cholmod_l_free_dense(&G_l, &_cholmod);
          throw InvalidOperationException("cholmod_l_scale failed", __FILE__,
            __LINE__, __PRETTY_FUNCTION__);
        }
      }
      cholmod_sparse* A_rt = cholmod_l_transpose(A_r, 1, &_cholmod);
      if (A_rt == NULL) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw InvalidOperationException("cholmod_l_transpose failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      cholmod_sparse* Omega = NULL;
      cholmod_sparse* A_rtQ = NULL;
      try {
        reduceLeftHandSide(_factor, A_rt, &Omega, &A_rtQ, &_cholmod);
      }
      catch (...) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        cholmod_l_free_sparse(&A_rt, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw;
      }
      analyzeSVD(Omega, _singularValues, _matrixU, _matrixV);
      cholmod_l_free_sparse(&Omega, &_cholmod);
      cholmod_dense* b_r;
      try {
        b_r = reduceRightHandSide(_factor, A_rt, A_rtQ, b, &_cholmod);
      }
      catch (...) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        cholmod_l_free_sparse(&A_rt, &_cholmod);
        cholmod_l_free_sparse(&A_rtQ, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw;
      }
      cholmod_l_free_sparse(&A_rt, &_cholmod);
      cholmod_l_free_sparse(&A_rtQ, &_cholmod);
      _svdTolerance = (_options.svdTol != -1.0) ? _options.svdTol :
        rankTol(_singularValues, _options.epsSVD);
      _svdRank = estimateNumericalRank(_singularValues, _svdTolerance);
      _svdRankDeficiency = _singularValues.size() - _svdRank;
      _svGap = svGap(_singularValues, _svdRank);
      Eigen::VectorXd x_r;
      solveSVD(b_r, _singularValues, _matrixU, _matrixV, _svdRank, x_r);
      cholmod_l_free_dense(&b_r, &_cholmod);
      cholmod_dense* x_l;
      try {
        x_l = solveQR(_factor, b, A_r, x_r, &_cholmod);
      }
      catch (...) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw;
      }
      if (_options.columnScaling) {
        const double* G_l_val = reinterpret_cast<const double*>(G_l->x);
        double* x_l_val = reinterpret_cast<double*>(x_l->x);
        for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(x_l->nrow);
            ++i)
          x_l_val[i] = G_l_val[i] * x_l_val[i];
        cholmod_l_free_dense(&G_l, &_cholmod);
        Eigen::Map<const Eigen::VectorXd> G_rEigen(
          reinterpret_cast<const double*>(G_r->x), G_r->nrow);
        x_r = G_rEigen.array() * x_r.array();
        cholmod_l_free_dense(&G_r, &_cholmod);
      }
      cholmod_l_free_sparse(&A_r, &_cholmod);
      x.resize(A->ncol);
      const double* x_l_val = reinterpret_cast<const double*>(x_l->x);
      std::copy(x_l_val, x_l_val + x_l->nzmax, x.data());
      cholmod_l_free_dense(&x_l, &_cholmod);
      x.tail(x_r.size()) = x_r;
      const double t1 = Timestamp::now();
      _linearSolverTime = t1 - t0;
    }

    void LinearSolver::analyzeMarginal(cholmod_sparse* A, std::ptrdiff_t j) {
      const double t0 = Timestamp::now();
      cholmod_sparse* A_l = columnSubmatrix(A, 0, j - 1, &_cholmod);
      if (_factor && _factor->QRsym &&
          (_factor->QRsym->m != static_cast<std::ptrdiff_t>(A_l->nrow) ||
          _factor->QRsym->n != static_cast<std::ptrdiff_t>(A_l->ncol) ||
          _factor->QRsym->anz != static_cast<std::ptrdiff_t>(A_l->nzmax)))
        clear();
      if (!_factor) {
        const double t2 = Timestamp::now();
        _factor = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
          SPQR_DEFAULT_TOL, A_l, &_cholmod);
        const double t3 = Timestamp::now();
        _cholmod.other1[1] = t3 - t2;
        if (_factor == NULL) {
          cholmod_l_free_sparse(&A_l, &_cholmod);
          throw InvalidOperationException("SuiteSparseQR_symbolic failed",
            __FILE__, __LINE__, __PRETTY_FUNCTION__);
        }
      }
      const double qrTolerance = (_options.qrTol != -1.0) ? _options.qrTol :
        qrTol(A_l, &_cholmod, _options.epsQR);
      const double t2 = Timestamp::now();
      const int status = SuiteSparseQR_numeric<double>(qrTolerance, A_l,
        _factor, &_cholmod);
      const double t3 = Timestamp::now();
      _cholmod.other1[2] = t3 - t2;
      cholmod_l_free_sparse(&A_l, &_cholmod);
      if (!status)
        throw InvalidOperationException("SuiteSparseQR_numeric failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      cholmod_sparse* A_r = columnSubmatrix(A, j, A->ncol - 1, &_cholmod);
      cholmod_sparse* A_rt = cholmod_l_transpose(A_r, 1, &_cholmod);
      if (A_rt == NULL) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        throw InvalidOperationException("cholmod_l_transpose failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      cholmod_sparse* Omega = NULL;
      cholmod_sparse* A_rtQ = NULL;
      try {
        reduceLeftHandSide(_factor, A_rt, &Omega, &A_rtQ, &_cholmod);
      }
      catch (...) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        cholmod_l_free_sparse(&A_rt, &_cholmod);
        throw;
      }
      cholmod_l_free_sparse(&A_r, &_cholmod);
      cholmod_l_free_sparse(&A_rt, &_cholmod);
      cholmod_l_free_sparse(&A_rtQ, &_cholmod);
      analyzeSVD(Omega, _singularValues, _matrixU, _matrixV);
      cholmod_l_free_sparse(&Omega, &_cholmod);
      if (_svdRank == -1) {
        _svdTolerance = (_options.svdTol != -1.0) ? _options.svdTol :
          rankTol(_singularValues, _options.epsSVD);
        _svdRank = estimateNumericalRank(_singularValues, _svdTolerance);
        _svdRankDeficiency = _singularValues.size() - _svdRank;
        _svGap = svGap(_singularValues, _svdRank);
      }
      const double t1 = Timestamp::now();
      _marginalAnalysisTime = t1 - t0;
    }

    bool LinearSolver::analyzeMarginal() {
      aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>& Jt =
        _jacobianBuilder.J_transpose();
      cholmod_sparse Jt_CS;
      Jt.getView(&Jt_CS);
      cholmod_sparse* J_CS = cholmod_l_transpose(&Jt_CS, 1, &_cholmod);
      if (J_CS == NULL)
        return false;
      bool status = true;
      try {
        analyzeMarginal(J_CS, _margStartIndex);
      }
      catch (const OutOfBoundException<std::ptrdiff_t>& e) {
        if (_options.verbose)
          std::cerr << e.what() << std::endl;
        status = false;
      }
      catch (const InvalidOperationException& e) {
        if (_options.verbose)
          std::cerr << e.what() << std::endl;
        status = false;
      }
      catch (...) {
        if (_options.verbose)
          std::cerr << __PRETTY_FUNCTION__ << ": unknown exception"
            << std::endl;
        status = false;
      }
      cholmod_l_free_sparse(&J_CS, &_cholmod);
      return status;
    }

    void LinearSolver::clear() {
      if (_factor) {
        SuiteSparseQR_free<double>(&_factor, &_cholmod);
        _factor = NULL;
      }
      _svdRank = -1;
      _svGap = std::numeric_limits<double>::infinity();
      _svdRankDeficiency = -1;
      _svdTolerance = -1.0;
      _singularValues.resize(0);
      _matrixU.resize(0, 0);
      _matrixV.resize(0, 0);
      _linearSolverTime = 0.0;
      _marginalAnalysisTime = 0.0;
    }

  }
}
