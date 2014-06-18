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

#include "aslam/calibration/functions/IncompleteGammaQFunction.h"

#include <boost/math/special_functions/gamma.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncompleteGammaQFunction::IncompleteGammaQFunction(double alpha) :
        mAlpha(alpha) {
    }

    IncompleteGammaQFunction::IncompleteGammaQFunction(const
        IncompleteGammaQFunction& other) :
        mAlpha(other.mAlpha) {
    }

    IncompleteGammaQFunction& IncompleteGammaQFunction::operator = (const
        IncompleteGammaQFunction& other) {
      if (this != &other) {
        mAlpha = other.mAlpha;
      }
      return *this;
    }

    IncompleteGammaQFunction::~IncompleteGammaQFunction() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    void IncompleteGammaQFunction::read(std::istream& stream) {
    }

    void IncompleteGammaQFunction::write(std::ostream& stream) const {
      stream << "alpha: " << mAlpha;
    }

    void IncompleteGammaQFunction::read(std::ifstream& stream) {
    }

    void IncompleteGammaQFunction::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double IncompleteGammaQFunction::getValue(const VariableType& argument)
        const {
      return boost::math::gamma_q(mAlpha, argument);
    }

    double IncompleteGammaQFunction::getAlpha() const {
      return mAlpha;
    }

    void IncompleteGammaQFunction::setAlpha(double alpha) {
      mAlpha = alpha;
    }

  }
}
