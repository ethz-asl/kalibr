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

/** \file IfThenElse.h
    \brief This file defines the If-Then-Else template
  */

#ifndef ASLAM_CALIBRATION_TPL_IFTHENELSE_H
#define ASLAM_CALIBRATION_TPL_IFTHENELSE_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The IfThenElse structure defines the If-Then-Else template
        \brief If-Then-Else template
      */
    template <typename C, typename A, typename B> struct IfThenElse;

    /** The IfThenElse structure defines the If-Then-Else template
        \brief If-Then-Else template
      */
    template <typename A, typename B> struct IfThenElse<True, A, B> {
    public:
      /// Definition for If selection
      typedef A Result;
    };

    /** The IfThenElse structure defines the If-Then-Else template
        \brief If-Then-Else template
      */
    template <typename A, typename B> class IfThenElse<False, A, B> {
    public:
      /// Definition for Else selection
      typedef B Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_IFTHENELSE_H
