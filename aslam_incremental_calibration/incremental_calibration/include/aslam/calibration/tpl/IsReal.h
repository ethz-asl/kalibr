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

/** \file IsReal.h
    \brief This file determines if types are real-valued
  */

#ifndef ASLAM_CALIBRATION_TPL_ISREAL_H
#define ASLAM_CALIBRATION_TPL_ISREAL_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The IsReal structure determines if a type is real
        \brief Real types definitions
      */
    template <typename T> struct IsReal {
    public:
      /// Non-real types definition
      typedef False Result;
    };

    /** The IsReal structure determines if a type is real
        \brief Real types definitions
      */
    template <> struct IsReal<float> {
    public:
      /// Real type definition
      typedef True Result;
    };

    /** The IsReal structure determines if a type is real
        \brief Real types definitions
      */
    template <> struct IsReal<double> {
    public:
      /// Real type definition
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_ISREAL_H
