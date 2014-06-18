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

/** \file permute.h
    \brief This file defines the permute function, which permutes containers.
  */

#ifndef ASLAM_CALIBRATION_ALGORITHMS_PERMUTE_H
#define ASLAM_CALIBRATION_ALGORITHMS_PERMUTE_H

#include <vector>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /** 
     * This function permutes an STL vector.
     * \brief STL vector permutation
     * 
     * \param[in, out] container vector to be permuted
     * \param[in] permutation permutation vector
     */
    template <typename T, typename I> void permute(std::vector<T>& container,
      const std::vector<I>& permutation);
    /** @}
      */

  }
}

#include "aslam/calibration/algorithms/permute.tpp"

#endif // ASLAM_CALIBRATION_ALGORITHMS_PERMUTE_H
