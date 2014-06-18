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

#include "aslam/calibration/algorithms/marginalize.h"

#include <cmath>

#include <Eigen/Dense>

#include <aslam/backend/CompressedColumnMatrix.hpp>

#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double colNorm(cholmod_sparse* A, size_t j) {
      if (j >= A->ncol)
        throw OutOfBoundException<size_t>(j,
          "colNorm(): index must be lower than the number of columns",
          __FILE__, __LINE__);
      const std::ptrdiff_t* col_ptr =
        reinterpret_cast<const std::ptrdiff_t*>(A->p);
      const double* values = reinterpret_cast<const double*>(A->x);
      const std::ptrdiff_t p = col_ptr[j];
      const std::ptrdiff_t numElements = col_ptr[j + 1] - p;
      double norm = 0;
      for (std::ptrdiff_t i = 0; i < numElements; ++i)
        norm += values[p + i] * values[p + i];
      return std::sqrt(norm);
    }

    Eigen::MatrixXd marginalJacobian(cholmod_sparse* J_x, cholmod_sparse*
        J_thetat, cholmod_common* cholmod) {
      // compute the QR factorization of J_x
      SuiteSparseQR_factorization<double>* QR = SuiteSparseQR_factorize<double>(
        SPQR_ORDERING_BEST, SPQR_DEFAULT_TOL, J_x, cholmod);
      if (QR == NULL)
        throw InvalidOperationException("marginalJacobian(): "
          "SuiteSparseQR_factorize failed");

      // compute the Jacobian of the reduced system
      cholmod_sparse* J_thetatQFull = SuiteSparseQR_qmult<double>(SPQR_XQ, QR,
        J_thetat, cholmod);
      if (J_thetatQFull == NULL) {
        SuiteSparseQR_free(&QR, cholmod);
        throw InvalidOperationException("marginalJacobian(): "
          "SuiteSparseQR_qmult failed");
      }
      std::ptrdiff_t* colIndices = new std::ptrdiff_t[J_x->ncol];
      for (size_t i = 0; i < J_x->ncol; ++i)
       colIndices[i] = i;
      cholmod_sparse* J_thetatQ = cholmod_l_submatrix(J_thetatQFull, NULL, -1,
        colIndices, J_x->ncol, 1, 0, cholmod);
      delete [] colIndices;
      if (J_thetatQ == NULL) {
        SuiteSparseQR_free(&QR, cholmod);
        cholmod_l_free_sparse(&J_thetatQFull, cholmod);
        throw InvalidOperationException("marginalJacobian(): "
          "cholmod_l_submatrix failed");
      }
      cholmod_sparse* J_thetat2 = cholmod_l_aat(J_thetat, NULL, 0, 1, cholmod);
      if (J_thetat2 == NULL) {
        SuiteSparseQR_free(&QR, cholmod);
        cholmod_l_free_sparse(&J_thetatQFull, cholmod);
        cholmod_l_free_sparse(&J_thetatQ, cholmod);
        throw InvalidOperationException("marginalJacobian(): "
          "cholmod_l_aat failed");
      }
      cholmod_sparse* J_thetatQ2 = cholmod_l_aat(J_thetatQ, NULL, 0, 1,
        cholmod);
      if (J_thetatQ2 == NULL) {
        SuiteSparseQR_free(&QR, cholmod);
        cholmod_l_free_sparse(&J_thetatQFull, cholmod);
        cholmod_l_free_sparse(&J_thetatQ, cholmod);
        cholmod_l_free_sparse(&J_thetat2, cholmod);
        throw InvalidOperationException("marginalJacobian(): "
          "cholmod_l_aat failed");
      }
      double alpha[2];
      alpha[0] = 1;
      double beta[2];
      beta[0] = -1;
      cholmod_sparse* Omega = cholmod_l_add(J_thetat2, J_thetatQ2, alpha, beta,
        1, 0, cholmod);
      if (Omega == NULL) {
        SuiteSparseQR_free(&QR, cholmod);
        cholmod_l_free_sparse(&J_thetatQFull, cholmod);
        cholmod_l_free_sparse(&J_thetatQ, cholmod);
        cholmod_l_free_sparse(&J_thetat2, cholmod);
        cholmod_l_free_sparse(&J_thetatQ2, cholmod);
        throw InvalidOperationException("marginalJacobian(): "
          "cholmod_l_add failed");
      }
      aslam::backend::CompressedColumnMatrix<std::ptrdiff_t> OmegaCCM;
      OmegaCCM.fromCholmodSparse(Omega);
      Eigen::MatrixXd OmegaDense(Omega->nrow, Omega->ncol);
      OmegaCCM.toDenseInto(OmegaDense);

      // clean allocated memory
      SuiteSparseQR_free(&QR, cholmod);
      cholmod_l_free_sparse(&J_thetatQFull, cholmod);
      cholmod_l_free_sparse(&J_thetatQ, cholmod);
      cholmod_l_free_sparse(&J_thetat2, cholmod);
      cholmod_l_free_sparse(&J_thetatQ2, cholmod);
      cholmod_l_free_sparse(&Omega, cholmod);

      return OmegaDense;
    }

    double marginalize(const
        aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>& Jt, size_t j,
        Eigen::MatrixXd& NS, Eigen::MatrixXd& CS, Eigen::MatrixXd& Sigma,
        Eigen::MatrixXd& SigmaP, Eigen::MatrixXd& Omega, double normTol, double
        epsTol) {
      // init cholmod
      cholmod_common cholmod;
      cholmod_l_start(&cholmod);

      // convert to cholmod_sparse
      cholmod_sparse JtCs;
      const_cast<aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&>(Jt).
        getView(&JtCs);
      cholmod_sparse* J = cholmod_l_transpose(&JtCs, 1, &cholmod);
      if (J == NULL) {
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_transpose failed");
      }

      // extract the part corresponding to the state/landmarks/...
      std::ptrdiff_t* colIndices = new std::ptrdiff_t[j];
      for (size_t i = 0; i < j; ++i)
       colIndices[i] = i;
      cholmod_sparse* J_x = cholmod_l_submatrix(J, NULL, -1, colIndices, j, 1,
        0, &cholmod);
      delete [] colIndices;
      if (J_x == NULL) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_submatrix failed");
      }

      // extract the part corresponding to the calibration parameters
      colIndices = new std::ptrdiff_t[J->ncol - j];
      for (size_t i = j; i < J->ncol; ++i)
       colIndices[i - j] = i;
      cholmod_sparse* J_theta = cholmod_l_submatrix(J, NULL, -1, colIndices,
        J->ncol - j, 1, 0, &cholmod);
      delete [] colIndices;
      if (J_theta == NULL) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_submatrix failed");
      }
      cholmod_sparse* J_thetat = cholmod_l_transpose(J_theta, 1, &cholmod);
      if (J_thetat == NULL) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_transpose failed");
      }

      // compute the marginal Jacobian
      try {
        Omega = marginalJacobian(J_x, J_thetat, &cholmod);
      }
      catch (const InvalidOperationException& e) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_finish(&cholmod);
        throw;
      }

      // scale J_x
      cholmod_dense* G_x = cholmod_l_allocate_dense(J_x->ncol, 1, J_x->ncol,
        CHOLMOD_REAL, &cholmod);
      if (G_x == NULL) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_allocate_dense failed");
      }
      try {
        double* values = reinterpret_cast<double*>(G_x->x);
        for (size_t j = 0; j < J_x->ncol; ++j) {
          const double normCol = colNorm(J_x, j);
          if (normCol < normTol)
            values[j] = 0.0;
          else
            values[j] = 1.0 / normCol;
        }
      }
      catch (const OutOfBoundException<size_t>& e) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_free_dense(&G_x, &cholmod) ;
        cholmod_l_finish(&cholmod);
        throw;
      }
      if (cholmod_l_scale(G_x, CHOLMOD_COL, J_x, &cholmod) == 0) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_free_dense(&G_x, &cholmod) ;
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_scale failed");
      }
      cholmod_l_free_dense(&G_x, &cholmod) ;

      // scale J_thetat
      cholmod_dense* G_theta = cholmod_l_allocate_dense(J_theta->ncol, 1,
        J_theta->ncol, CHOLMOD_REAL, &cholmod);
      if (G_theta == NULL) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_allocate_dense failed");
      }
      try {
        double* values = reinterpret_cast<double*>(G_theta->x);
        for (size_t j = 0; j < J_theta->ncol; ++j) {
          const double normCol = colNorm(J_theta, j);
          if (normCol < normTol)
            values[j] = 0.0;
          else
            values[j] = 1.0 / normCol;
        }
      }
      catch (const OutOfBoundException<size_t>& e) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_free_dense(&G_theta, &cholmod) ;
        cholmod_l_finish(&cholmod);
        throw;
      }
      if (cholmod_l_scale(G_theta, CHOLMOD_ROW, J_thetat, &cholmod) == 0) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_free_dense(&G_theta, &cholmod) ;
        cholmod_l_finish(&cholmod);
        throw InvalidOperationException("marginalize(): "
          "cholmod_l_scale failed");
      }
      cholmod_l_free_dense(&G_theta, &cholmod) ;

      // compute the scaled marginal Jacobian
      Eigen::MatrixXd OmegaScaled;
      try {
        OmegaScaled = marginalJacobian(J_x, J_thetat, &cholmod);
      }
      catch (const InvalidOperationException& e) {
        cholmod_l_free_sparse(&J, &cholmod);
        cholmod_l_free_sparse(&J_x, &cholmod);
        cholmod_l_free_sparse(&J_theta, &cholmod);
        cholmod_l_free_sparse(&J_thetat, &cholmod);
        cholmod_l_finish(&cholmod);
        throw;
      }

      // clean cholmod
      cholmod_l_free_sparse(&J, &cholmod);
      cholmod_l_free_sparse(&J_x, &cholmod);
      cholmod_l_free_sparse(&J_theta, &cholmod);
      cholmod_l_free_sparse(&J_thetat, &cholmod);
      cholmod_l_finish(&cholmod);

      // compute the thin SVD of OmegaScaled
      const Eigen::JacobiSVD<Eigen::MatrixXd> svdScaled(OmegaScaled,
        Eigen::ComputeThinU | Eigen::ComputeThinV);

      // compute the numerical rank
      size_t nrank = OmegaScaled.cols();
      const Eigen::VectorXd& SScaled = svdScaled.singularValues();
      const double tol = OmegaScaled.rows() * SScaled(0) * epsTol;
      for (std::ptrdiff_t i = OmegaScaled.cols() - 1; i > 0; --i) {
        if (SScaled(i) > tol)
          break;
        else
          nrank--;
       }

      // compute the thin SVD of Omega
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(Omega,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
      const Eigen::MatrixXd& V = svd.matrixV();

      // compute the numerical column space
      CS = V.block(0, 0, V.rows(), nrank);

      // compute the numerical null space
      NS = V.block(0, nrank, V.rows(), Omega.cols() - nrank);

      // compute the projected covariance matrix
      Eigen::MatrixXd invS(Eigen::MatrixXd::Zero(Omega.cols(), Omega.cols()));
      SigmaP = Eigen::MatrixXd::Zero(nrank, nrank);
      double svLogSum = 0;
      const Eigen::VectorXd& S = svd.singularValues();
      for (size_t i = 0; i < nrank; ++i) {
        SigmaP(i, i) = 1.0 / S(i);
        svLogSum = svLogSum + log2(S(i));
        invS(i, i) = SigmaP(i, i);
      }

      // compute the covariance matrix
      Sigma = V * invS * V.transpose();
      return svLogSum;
    }

  }
}
