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

#include "aslam/calibration/algorithms/linalg.h"

#include <cmath>

#include <algorithm>

#include <Eigen/Dense>

#include <cholmod.h>
#include <SuiteSparseQR.hpp>
#include <spqr.hpp>

#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/exceptions/NullPointerException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    cholmod_sparse* columnSubmatrix(cholmod_sparse* A, std::ptrdiff_t
        colStartIdx, std::ptrdiff_t colEndIdx, cholmod_common* cholmod) {
      if (A == NULL)
        throw NullPointerException("A", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (colEndIdx < colStartIdx)
        throw OutOfBoundException<std::ptrdiff_t>(colEndIdx, colStartIdx,
          "colStartIdx must be lower than colEndIdx", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (colEndIdx >= static_cast<std::ptrdiff_t>(A->ncol))
        throw OutOfBoundException<std::ptrdiff_t>(colEndIdx, A->ncol,
          "index must be lower than the number of columns",  __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (colStartIdx < 0)
        throw OutOfBoundException<std::ptrdiff_t>(colStartIdx, 0,
          "columnSubmatrix(): index must be positive", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      const std::ptrdiff_t numIndices = colEndIdx - colStartIdx + 1;
      std::ptrdiff_t* colIndices = new std::ptrdiff_t[numIndices];
      for (std::ptrdiff_t j = colStartIdx; j <= colEndIdx; ++j)
        colIndices[j - colStartIdx] = j;
      cholmod_sparse* A_sub = cholmod_l_submatrix(A, NULL, -1, colIndices,
        numIndices, 1, 1, cholmod);
      delete [] colIndices;
      if (A_sub == NULL)
        throw InvalidOperationException("cholmod_l_submatrix failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      return A_sub;
    }

    cholmod_sparse* rowSubmatrix(cholmod_sparse* A, std::ptrdiff_t rowStartIdx,
        std::ptrdiff_t rowEndIdx, cholmod_common* cholmod) {
      if (A == NULL)
        throw NullPointerException("A", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (rowEndIdx < rowStartIdx)
        throw OutOfBoundException<std::ptrdiff_t>(rowEndIdx, rowStartIdx,
          "rowStartIdx must be lower than rowEndIdx", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (rowEndIdx >= static_cast<std::ptrdiff_t>(A->nrow))
        throw OutOfBoundException<std::ptrdiff_t>(rowEndIdx, A->nrow,
          "index must be lower than the number of rows", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (rowStartIdx < 0)
        throw OutOfBoundException<std::ptrdiff_t>(rowStartIdx, 0,
          "index must be positive", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      const std::ptrdiff_t numIndices = rowEndIdx - rowStartIdx + 1;
      std::ptrdiff_t* rowIndices = new std::ptrdiff_t[numIndices];
      for (std::ptrdiff_t i = rowStartIdx; i <= rowEndIdx; ++i)
       rowIndices[i - rowStartIdx] = i;
      cholmod_sparse* A_sub = cholmod_l_submatrix(A, rowIndices, numIndices,
        NULL, -1, 1, 1, cholmod);
      delete [] rowIndices;
      if (A_sub == NULL)
        throw InvalidOperationException("cholmod_l_submatrix failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      return A_sub;
    }

    double colNorm(const cholmod_sparse* A, std::ptrdiff_t j) {
      if (A == NULL)
        throw NullPointerException("A", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (j >= static_cast<std::ptrdiff_t>(A->ncol))
        throw OutOfBoundException<std::ptrdiff_t>(j, A->ncol,
          "index must be lower than the number of columns", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (j < 0)
        throw OutOfBoundException<std::ptrdiff_t>(j, 0,
          "index must be positive", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      const std::ptrdiff_t* col_ptr =
        reinterpret_cast<const std::ptrdiff_t*>(A->p);
      const double* values = reinterpret_cast<const double*>(A->x);
      const std::ptrdiff_t p = col_ptr[j];
      const std::ptrdiff_t numElements = col_ptr[j + 1] - p;
      double norm = 0.0;
      for (std::ptrdiff_t i = 0; i < numElements; ++i)
        norm += values[p + i] * values[p + i];
      return std::sqrt(norm);
    }

    cholmod_dense* columnScalingMatrix(cholmod_sparse* A, cholmod_common*
        cholmod, double eps) {
      if (A == NULL)
        throw NullPointerException("A", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      cholmod_dense* G = cholmod_l_allocate_dense(A->ncol, 1, A->ncol,
        CHOLMOD_REAL, cholmod);
      if (G == NULL)
        throw InvalidOperationException("cholmod_l_allocate_dense failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      const double normTol = std::sqrt(A->nrow * eps);
      double* values = reinterpret_cast<double*>(G->x);
      for (std::ptrdiff_t j = 0; j < static_cast<std::ptrdiff_t>(A->ncol);
          ++j) {
        const double norm = colNorm(A, j);
        if (norm < normTol)
          values[j] = 0.0;
        else
          values[j] = 1.0 / norm;
      }
      return G;
    }

    void cholmodSparseToEigenDenseCopy(const cholmod_sparse* in,
        Eigen::MatrixXd& out) {
      if (in == NULL)
        throw NullPointerException("in", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      out.setZero(in->nrow, in->ncol);
      const std::ptrdiff_t* row_ind =
        reinterpret_cast<const std::ptrdiff_t*>(in->i);
      const std::ptrdiff_t* col_ptr =
        reinterpret_cast<const std::ptrdiff_t*>(in->p);
      const double* values = reinterpret_cast<const double*>(in->x);
      for (std::ptrdiff_t c = 0; c < static_cast<std::ptrdiff_t>(in->ncol); ++c)
        for (std::ptrdiff_t v = col_ptr[c]; v < col_ptr[c + 1]; ++v) {
          out(row_ind[v], c) = values[v];
          if (in->stype && c != row_ind[v])
            out(c, row_ind[v]) = values[v];
        }
    }

    void eigenDenseToCholmodDenseView(const Eigen::VectorXd& in, cholmod_dense*
        out) {
      if (out == NULL)
        throw InvalidOperationException("out", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      out->nrow = in.size();
      out->ncol = 1;
      out->nzmax = in.size();
      out->d = in.size();
      out->x = reinterpret_cast<void*>(const_cast<double*>(in.data()));
      out->z = NULL;
      out->xtype = CHOLMOD_REAL;
      out->dtype = CHOLMOD_DOUBLE;
    }

    cholmod_dense* eigenDenseToCholmodDenseCopy(const Eigen::VectorXd& in,
        cholmod_common* cholmod) {
      cholmod_dense* out = cholmod_l_allocate_dense(in.size(), 1, in.size(),
        CHOLMOD_REAL, cholmod);
      if (out == NULL)
        throw InvalidOperationException("cholmod_l_allocate_dense failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      double* out_val = reinterpret_cast<double*>(out->x);
      const double* in_val = in.data();
      std::copy(in_val, in_val + in.size(), out_val);
      return out;
    }

    void cholmodDenseToEigenDenseCopy(const cholmod_dense* in, Eigen::VectorXd&
        out) {
      if (in == NULL)
        throw NullPointerException("in", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      out.resize(in->nrow);
      const double* in_val = reinterpret_cast<const double*>(in->x);
      std::copy(in_val, in_val + in->nrow, out.data());
    }

    cholmod_sparse* eigenDenseToCholmodSparseCopy(const Eigen::MatrixXd& in,
        cholmod_common* cholmod, double eps) {
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      size_t nzmax = 0;
      for (std::ptrdiff_t i = 0; i < in.rows(); ++i)
        for (std::ptrdiff_t j = 0; j < in.cols(); ++j)
          if (std::fabs(in(i, j)) > eps)
            nzmax++;
      cholmod_sparse* out = cholmod_l_allocate_sparse(in.rows(), in.cols(),
        nzmax, 1, 1, 0, CHOLMOD_REAL, cholmod);
      if (out == NULL)
        throw InvalidOperationException("cholmod_l_allocate_sparse failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      std::ptrdiff_t* row_ind = reinterpret_cast<std::ptrdiff_t*>(out->i);
      std::ptrdiff_t* col_ptr = reinterpret_cast<std::ptrdiff_t*>(out->p);
      double* values = reinterpret_cast<double*>(out->x);
      std::ptrdiff_t rowIt = 0;
      std::ptrdiff_t colIt = 1;
      for (std::ptrdiff_t c = 0; c < in.cols(); ++c) {
        for (std::ptrdiff_t r = 0; r < in.rows(); ++r)
          if (std::fabs(in(r, c)) > eps) {
            values[rowIt] = in(r, c);
            row_ind[rowIt] = r;
            rowIt++;
          }
        col_ptr[colIt] = rowIt;
        colIt++;
      }
      return out;
    }

    std::ptrdiff_t estimateNumericalRank(const Eigen::VectorXd& sv, double
        tol) {
      std::ptrdiff_t nrank = sv.size();
      for (std::ptrdiff_t i = sv.size() - 1; i > 0; --i) {
        if (sv(i) > tol)
          break;
        else
          nrank--;
      }
      return nrank;
    }

    double rankTol(const Eigen::VectorXd& sv, double eps) {
      if (!sv.size())
        throw InvalidOperationException("empty sv", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      return sv(0) * eps * sv.size();
    }

    double qrTol(cholmod_sparse* A, cholmod_common* cholmod, double eps) {
      if (A == NULL)
        throw NullPointerException("A", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      return 20.0 * static_cast<double>(A->nrow + A->ncol) * eps *
        spqr_maxcolnorm<double>(A, cholmod);
    }

    double svGap(const Eigen::VectorXd& sv, std::ptrdiff_t rank) {
      if (rank > sv.size() || rank <= 0)
        throw InvalidOperationException("inconsistent rank", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (rank < sv.size())
        return sv(rank - 1) / sv(rank);
      else
        return std::numeric_limits<double>::infinity();
    }

    void reduceLeftHandSide(SuiteSparseQR_factorization<double>* factor,
        cholmod_sparse* A_rt, cholmod_sparse** Omega, cholmod_sparse** A_rtQ,
        cholmod_common* cholmod) {
      if (factor == NULL || factor->QRsym == NULL || factor->QRnum == NULL)
        throw NullPointerException("factor", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (A_rt == NULL)
        throw NullPointerException("A_rt", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (Omega == NULL)
        throw NullPointerException("Omega", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (A_rtQ == NULL)
        throw NullPointerException("A_rtQ", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      cholmod_sparse* A_rtQFull = SuiteSparseQR_qmult<double>(SPQR_XQ, factor,
        A_rt, cholmod);
      if (A_rtQFull == NULL)
        throw InvalidOperationException("SuiteSparseQR_qmult failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      try {
        *A_rtQ = columnSubmatrix(A_rtQFull, 0, factor->QRsym->n, cholmod);
      }
      catch (const InvalidOperationException& e) {
        cholmod_l_free_sparse(&A_rtQFull, cholmod);
        throw;
      }
      cholmod_l_free_sparse(&A_rtQFull, cholmod);
      cholmod_sparse* A_rtQ2 = cholmod_l_aat(*A_rtQ, NULL, 0, 1, cholmod);
      if (A_rtQ2 == NULL)
        throw InvalidOperationException("cholmod_l_aat failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      A_rtQ2->stype = 1;
      cholmod_sparse* A_rt2 = cholmod_l_aat(A_rt, NULL, 0, 1, cholmod);
      if (A_rt2 == NULL) {
        cholmod_l_free_sparse(&A_rtQ2, cholmod);
        throw InvalidOperationException("cholmod_l_aat failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      A_rt2->stype = 1;
      double alpha[2];
      alpha[0] = 1.0;
      double beta[2];
      beta[0] = -1.0;
      *Omega = cholmod_l_add(A_rt2, A_rtQ2, alpha, beta, 1, 1, cholmod);
      cholmod_l_free_sparse(&A_rt2, cholmod);
      cholmod_l_free_sparse(&A_rtQ2, cholmod);
      if (*Omega == NULL)
        throw InvalidOperationException("cholmod_l_add failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
    }

    cholmod_dense* reduceRightHandSide(SuiteSparseQR_factorization<double>*
        factor, cholmod_sparse* A_rt, cholmod_sparse* A_rtQ, cholmod_dense* b,
        cholmod_common* cholmod) {
      if (factor == NULL || factor->QRsym == NULL || factor->QRnum == NULL)
        throw NullPointerException("factor", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (A_rt == NULL)
        throw NullPointerException("A_rt", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (b == NULL)
        throw NullPointerException("b", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (A_rtQ == NULL)
        throw NullPointerException("A_rtQ", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      cholmod_sparse* bSparse = cholmod_l_dense_to_sparse(b, 1, cholmod);
      if (bSparse == NULL)
        throw InvalidOperationException("cholmod_l_dense_to_sparse failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      cholmod_sparse* A_rtb = cholmod_l_ssmult(A_rt, bSparse, 0, 1, 1, cholmod);
      if (A_rtb == NULL) {
        cholmod_l_free_sparse(&bSparse, cholmod);
        throw InvalidOperationException("cholmod_l_ssmult failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      cholmod_sparse* QtbFull = SuiteSparseQR_qmult<double>(SPQR_QTX, factor,
        bSparse, cholmod);
      cholmod_l_free_sparse(&bSparse, cholmod);
      if (QtbFull == NULL) {
        cholmod_l_free_sparse(&A_rtb, cholmod);
        throw InvalidOperationException("SuiteSparseQR_qmult failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      cholmod_sparse* Qtb;
      try {
        Qtb = rowSubmatrix(QtbFull, 0, factor->QRsym->n, cholmod);
      }
      catch (const InvalidOperationException& e) {
        cholmod_l_free_sparse(&A_rtb, cholmod);
        cholmod_l_free_sparse(&QtbFull, cholmod);
        throw;
      }
      cholmod_l_free_sparse(&QtbFull, cholmod);
      cholmod_sparse* A_rtQQtb = cholmod_l_ssmult(A_rtQ, Qtb, 0, 1, 1, cholmod);
      cholmod_l_free_sparse(&Qtb, cholmod);
      if (A_rtQQtb == NULL) {
        cholmod_l_free_sparse(&A_rtb, cholmod);
        throw InvalidOperationException("cholmod_l_ssmult failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      double alpha[2];
      alpha[0] = 1.0;
      double beta[2];
      beta[0] = -1.0;
      cholmod_sparse* bReducedSparse = cholmod_l_add(A_rtb, A_rtQQtb, alpha,
        beta, 1, 1, cholmod);
      cholmod_l_free_sparse(&A_rtb, cholmod);
      cholmod_l_free_sparse(&A_rtQQtb, cholmod);
      if (bReducedSparse == NULL)
        throw InvalidOperationException("reduceRightHandSide(): "
          "cholmod_l_add failed", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      cholmod_dense* bReduced = cholmod_l_sparse_to_dense(bReducedSparse,
        cholmod);
      cholmod_l_free_sparse(&bReducedSparse, cholmod);
      if (bReduced == NULL)
        throw InvalidOperationException("cholmod_l_sparse_to_dense failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      return bReduced;
    }

    void analyzeSVD(const cholmod_sparse* Omega, Eigen::VectorXd& sv,
        Eigen::MatrixXd& U, Eigen::MatrixXd& V) {
      if (Omega == NULL)
        throw NullPointerException("Omega", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      Eigen::MatrixXd OmegaDense;
      cholmodSparseToEigenDenseCopy(Omega, OmegaDense);
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(OmegaDense,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
      U = svd.matrixU();
      V = svd.matrixV();
      sv = svd.singularValues();
    }

    void solveSVD(const cholmod_dense* b, const Eigen::VectorXd& sv, const
        Eigen::MatrixXd& U, const Eigen::MatrixXd& V, std::ptrdiff_t rank,
        Eigen::VectorXd& x) {
      if (b == NULL)
        throw NullPointerException("b", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (rank > V.cols())
        throw OutOfBoundException<std::ptrdiff_t>(rank, V.cols(),
          "inconsistent rank", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      if (V.cols() != sv.rows() ||
          U.rows() != static_cast<std::ptrdiff_t>(b->nrow))
        throw OutOfBoundException<std::ptrdiff_t>(U.rows(), b->nrow,
          "inconsistent matrices", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      Eigen::Map<const Eigen::VectorXd> bEigen(
        reinterpret_cast<const double*>(b->x), b->nrow);
      x = V.leftCols(rank) * sv.head(rank).asDiagonal().inverse() *
        U.leftCols(rank).adjoint() * bEigen;
    }

    cholmod_dense* solveQR(SuiteSparseQR_factorization<double>* factor,
        cholmod_dense* b, cholmod_sparse* A_r, const Eigen::VectorXd& x_r,
        cholmod_common* cholmod) {
      if (factor == NULL || factor->QRsym == NULL || factor->QRnum == NULL)
        throw NullPointerException("factor", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (b == NULL)
        throw NullPointerException("b", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (A_r == NULL)
        throw NullPointerException("A_r", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (x_r.size() != static_cast<int>(A_r->ncol))
        throw OutOfBoundException<int>(x_r.size(), A_r->ncol,
          "mismatch between x_r and A_r", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      if (cholmod == NULL)
        throw NullPointerException("cholmod", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      cholmod_dense x_rCholmod;
      eigenDenseToCholmodDenseView(x_r, &x_rCholmod);
      cholmod_dense* A_rx_r = cholmod_l_allocate_dense(A_r->nrow, 1, A_r->nrow,
        CHOLMOD_REAL, cholmod);
      if (A_rx_r == NULL)
        throw InvalidOperationException("cholmod_l_allocate_dense failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      double alpha[2];
      alpha[0] = 1.0;
      double beta[2];
      beta[0] = 0.0;
      if (!cholmod_l_sdmult(A_r, 0, alpha, beta, &x_rCholmod, A_rx_r,
          cholmod)) {
        cholmod_l_free_dense(&A_rx_r, cholmod);
        throw InvalidOperationException("cholmod_l_sdmult failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      Eigen::Map<const Eigen::VectorXd> bEigen(
        reinterpret_cast<const double*>(b->x), b->nrow);
      Eigen::Map<const Eigen::VectorXd> A_rx_rEigen(
        reinterpret_cast<const double*>(A_rx_r->x), A_rx_r->nrow);
      const Eigen::VectorXd bmA_rx_rEigen = bEigen - A_rx_rEigen;
      cholmod_l_free_dense(&A_rx_r, cholmod);
      cholmod_dense bmA_rx_r;
      eigenDenseToCholmodDenseView(bmA_rx_rEigen, &bmA_rx_r);
      cholmod_dense* QtbmA_rx_r = SuiteSparseQR_qmult<double>(SPQR_QTX, factor,
        &bmA_rx_r, cholmod);
      if (QtbmA_rx_r == NULL)
        throw InvalidOperationException("SuiteSparseQR_qmult failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      cholmod_dense* x_l = SuiteSparseQR_solve<double>(SPQR_RETX_EQUALS_B,
        factor, QtbmA_rx_r, cholmod);
      cholmod_l_free_dense(&QtbmA_rx_r, cholmod);
      if (x_l == NULL)
        throw InvalidOperationException("SuiteSparseQR_solve failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      return x_l;
    }

  }
}
