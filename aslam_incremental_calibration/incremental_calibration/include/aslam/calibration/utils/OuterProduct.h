/******************************************************************************
 * Copyright (C) 2011 by Jerome Maye                                          *
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

/** \file OuterProduct.h
    \brief This file defines the outer product functions
  */

#ifndef ASLAM_CALIBRATION_UTILS_OUTERPRODUCT_H
#define ASLAM_CALIBRATION_UTILS_OUTERPRODUCT_H

#include <cstdlib>

#include <Eigen/Core>

/** The OuterProduct namespace contains outer product functions.
    \brief Outer product functions
  */
namespace OuterProduct {
  /** \name Methods
    @{
    */
  /// The compute function generates the outer product of 2 vectors
  template <typename X, size_t M, size_t N>
  Eigen::Matrix<X, M, N> compute(const Eigen::Matrix<X, M, 1>& v1,
      const Eigen::Matrix<X, N, 1>& v2);
  /// The compute function generates the self outer product of a vector
  template <typename X, size_t M>
  Eigen::Matrix<X, M, M> compute(const Eigen::Matrix<X, M, 1>& v);
  /** @}
    */

}

#include "aslam/calibration/utils/OuterProduct.tpp"

#endif // ASLAM_CALIBRATION_UTILS_OUTERPRODUCT_H
