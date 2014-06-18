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

/** \file Equals.h
    \brief This file defines the equality testing template
  */

#ifndef ASLAM_CALIBRATION_TPL_EQUALS_H
#define ASLAM_CALIBRATION_TPL_EQUALS_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The Equals structure defines the equality testing template
        \brief Equality testing template
      */
    template <typename A, typename B> struct Equals {
    public:
      /// Defines the not equal
      typedef False Result;
    };

    /** The Equals structure defines the equality testing template
        \brief Equality testing template
      */
    template <typename A> struct Equals<A, A> {
    public:
      /// Defines the equal
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_EQUALS_H
