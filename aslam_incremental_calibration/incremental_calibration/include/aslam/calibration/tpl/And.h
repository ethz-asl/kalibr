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

/** \file And.h
    \brief This file defines the and template
  */

#ifndef ASLAM_CALIBRATION_TPL_AND_H
#define ASLAM_CALIBRATION_TPL_AND_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The And structure defines the and template
        \brief And template
      */
    template <typename A, typename B> struct And {
    public:
      /// Defines and resulting to false
      typedef False Result;
    };

    /** The And structure defines the and template
        \brief And template
      */
    template <> struct And<True, True> {
    public:
      /// Defines and resulting to true
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_AND_H
