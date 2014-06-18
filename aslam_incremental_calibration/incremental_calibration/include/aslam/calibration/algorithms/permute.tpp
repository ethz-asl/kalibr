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

#include <utility>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <typename T, typename I>
    void permute(std::vector<T>& container, const std::vector<I>& permutation) {
      if (container.size() != permutation.size())
        throw OutOfBoundException<I>(permutation.size(), container.size(),
          "permutation vector must match container size",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      for (I i = 0; i < container.size(); ++i) {
        if (permutation[i] >= static_cast<I>(container.size()))
          throw OutOfBoundException<I>(permutation[i], static_cast<I>(
            container.size()), "permutation vector index out of bound",
            __FILE__, __LINE__, __PRETTY_FUNCTION__);
        I k = permutation[i];
        while (k < i)
          k = permutation[k];
        if (k > i)
          std::swap<T>(container[i], container[k]);
      }
    }

  }
}
