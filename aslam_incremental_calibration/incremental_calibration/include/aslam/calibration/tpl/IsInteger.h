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

/** \file IsInteger.h
    \brief This file determines if types are integer
  */

#ifndef ASLAM_CALIBRATION_TPL_ISINTEGER_H
#define ASLAM_CALIBRATION_TPL_ISINTEGER_H

#include "aslam/calibration/tpl/Not.h"
#include "aslam/calibration/tpl/IsReal.h"

namespace aslam {
  namespace calibration {

    /** The IsInteger structure determines if a type is integer
        \brief Integer types definitions
      */
    template <typename T> struct IsInteger :
      public Not<typename IsReal<T>::Result> {
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_ISINTEGER_H
