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

#include <cmath>

#include "aslam/calibration/exceptions/BadArgumentException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename X>
    LogGammaFunction<X>::LogGammaFunction(size_t dim) :
        mDim(dim) {
    }

    template <typename X>
    LogGammaFunction<X>::LogGammaFunction(const LogGammaFunction& other) :
        mDim(other.mDim) {
    }

    template <typename X>
    LogGammaFunction<X>& LogGammaFunction<X>::operator =
        (const LogGammaFunction& other) {
      if (this != &other) {
        mDim = other.mDim;
      }
      return *this;
    }

    template <typename X>
    LogGammaFunction<X>::~LogGammaFunction() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <typename X>
    void LogGammaFunction<X>::read(std::istream& stream) {
    }

    template <typename X>
    void LogGammaFunction<X>::write(std::ostream& stream) const {
      stream << "dimension: " << mDim;
    }

    template <typename X>
    void LogGammaFunction<X>::read(std::ifstream& stream) {
    }

    template <typename X>
    void LogGammaFunction<X>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X>
    double LogGammaFunction<X>::getValue(const VariableType& argument) const {
      double sum = 0.0;
      for (size_t i = 0; i < mDim; ++i) {
        sum += lgamma(argument - 0.5 * i);
      }
      return sum + mDim * (mDim - 1) * 0.25 * log(M_PI);
    }

    template <typename X>
    size_t LogGammaFunction<X>::getDim() const {
      return mDim;
    }

    template <typename X>
    void LogGammaFunction<X>::setDim(size_t dim) {
      mDim = dim;
    }

  }
}
