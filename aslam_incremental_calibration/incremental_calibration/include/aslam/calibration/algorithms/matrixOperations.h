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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file matrixOperations.h
    \brief This file defines some useful matrix operations.
  */

#ifndef ASLAM_CALIBRATION_ALGORITHMS_MATRIX_OPERATIONS_H
#define ASLAM_CALIBRATION_ALGORITHMS_MATRIX_OPERATIONS_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /** 
     * This function computes a covariance block from the R matrix of a QR
     * factorization.
     * \brief Covariance block recovery
     * 
     * \param[in] R upper triangular R matrix
     * \param[in] colBegin column index where to start recovery
     * \param[in] colEnd column index where to stop recovery
     * \return covariance block
     */
    template <typename T>
    Eigen::MatrixXd computeCovariance(const T& R, size_t colBegin,
      size_t colEnd);

    /** 
     * This function computes the sum of the logarithm of the diagonal elements
     * of the input matrix.
     * \brief Sums the logarithm of the diagonal elements of the input matrix
     * 
     * \param[in] R input matrix
     * \param[in] colBegin column index where to start sum
     * \param[in] colEnd column index where to stop sum
     * \return sum of logarithm of diagonal elements
     */
    template <typename T>
    double computeSumLogDiagR(const T& R, size_t colBegin, size_t colEnd);

    /** 
     * This function checks the validity of intput column index.
     * \brief Index checking
     * 
     * \param[in] R input matrix
     * \param[in] colBegin column index where to start sum
     * \param[in] colEnd column index where to stop sum
     */
    template <typename T>
    void checkColumnIndices(const T& R, size_t colBegin, size_t colEnd);
    /** @}
      */

  }
}

#include "aslam/calibration/algorithms/matrixOperations.tpp"

#endif // ASLAM_CALIBRATION_ALGORITHMS_MATRIX_OPERATIONS_H
