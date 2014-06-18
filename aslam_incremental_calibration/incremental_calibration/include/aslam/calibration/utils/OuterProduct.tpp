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

namespace OuterProduct {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  template <typename X, size_t M, size_t N>
  Eigen::Matrix<X, M, N> compute(const Eigen::Matrix<X, M, 1>& v1,
      const Eigen::Matrix<X, N, 1>& v2) {
    return v1 * v2.transpose();
  };

  template <typename X, size_t M>
  Eigen::Matrix<X, M, M> compute(const Eigen::Matrix<X, M, 1>& v) {
    Eigen::Matrix<X, M, M> result = Eigen::Matrix<X, M, M>::Zero(v.size(),
      v.size());
    for (size_t i = 0; i < (size_t)v.size(); ++i)
      for (size_t j = i; j < (size_t)v.size(); ++j) {
        result(i, j) = v(i) * v(j);
        result(j, i) = result(i, j);
      }
    return result;
  };

}
