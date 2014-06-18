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

/** \file Or.h
    \brief This file defines the or template
  */

#ifndef ASLAM_CALIBRATION_TPL_OR_H
#define ASLAM_CALIBRATION_TPL_OR_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The Or structure defines the Or template
        \brief Or template
      */
    template <typename A, typename B> struct Or {
    public:
      /// Defines true
      typedef True Result;
    };

    /** The Or structure defines the Or template
        \brief Or template
      */
    template <> struct Or<False, False> {
    public:
      /// Defines false
      typedef False Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_OR_H
