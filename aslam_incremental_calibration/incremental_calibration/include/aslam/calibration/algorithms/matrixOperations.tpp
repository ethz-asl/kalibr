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

#include "aslam/calibration/exceptions/OutOfBoundException.h"

#include <cmath>
#include <cstddef>

#include <limits>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <typename T>
    void checkColumnIndices(const T& R, size_t colBegin, size_t colEnd) {
      if (colBegin >= colEnd)
        throw OutOfBoundException<size_t>(colBegin,
          "checkColumnIndices(): "
          "column index begin must be smaller that column end index",
          __FILE__, __LINE__);
      if (colEnd >= static_cast<size_t>(R.cols()))
        throw OutOfBoundException<size_t>(colEnd,
          "checkColumnIndices(): "
          "column end index must be smaller than the columns of R",
          __FILE__, __LINE__);
    }

    template <typename T>
    Eigen::MatrixXd computeCovariance(const T& R, size_t colBegin,
        size_t colEnd) {
      checkColumnIndices(R, colBegin, colEnd);
      // NOTE: What about checking the form of R? Upper triangular matrix
      const size_t numCols = R.cols();
      const size_t dim = numCols - colBegin;
      Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(dim, dim);
      for (std::ptrdiff_t l = numCols - 1, Sigma_l = dim - 1;
          l >= (std::ptrdiff_t)(numCols - dim); --l, --Sigma_l) {
        double temp1 = 0;
        for (std::ptrdiff_t j = l + 1, Sigma_j = Sigma_l + 1;
            j < (std::ptrdiff_t)numCols; ++j, ++Sigma_j)
          temp1 += R(l, j) * covariance(Sigma_j, Sigma_l);
        const double R_ll = R(l, l);
        covariance(Sigma_l, Sigma_l) = 1 / R_ll * (1 / R_ll - temp1);
        for (std::ptrdiff_t i = l - 1, Sigma_i = Sigma_l - 1;
            i >= std::ptrdiff_t(numCols - dim); --i, --Sigma_i) {
          temp1 = 0;
          for (std::ptrdiff_t j = i + 1, Sigma_j = Sigma_i + 1;
              j <= l; ++j, ++Sigma_j)
            temp1 += R(i, j) * covariance(Sigma_j, Sigma_l);
          double temp2 = 0;
          for (std::ptrdiff_t j = l + 1, Sigma_j = Sigma_l + 1;
              j < (std::ptrdiff_t)numCols; ++j, ++Sigma_j)
            temp2 += R(i, j) * covariance(Sigma_l, Sigma_j);
          covariance(Sigma_i, Sigma_l) = 1 / R(i, i) * (-temp1 - temp2);
          covariance(Sigma_l, Sigma_i) = covariance(Sigma_i, Sigma_l);
        }
      }
      const size_t blockDim = colEnd - colBegin + 1;
      return covariance.block(0, 0, blockDim, blockDim);
    }

    template <typename T>
    double computeSumLogDiagR(const T& R, size_t colBegin, size_t colEnd) {
      checkColumnIndices(R, colBegin, colEnd);
      // NOTE: What about checking the form of R? Upper triangular matrix
      double sumLogDiagR = 0;
      for (size_t i = colBegin; i <= colEnd; ++i) {
        const double value = R(i, i);
        if (std::fabs(value) > std::numeric_limits<double>::epsilon())
          sumLogDiagR += std::log2(std::fabs(value));
      }
      return sumLogDiagR;
    }

  }
}
