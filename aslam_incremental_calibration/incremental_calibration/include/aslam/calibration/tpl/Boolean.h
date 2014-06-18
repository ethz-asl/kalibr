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

/** \file Boolean.h
    \brief This file defines the boolean template
  */

#ifndef ASLAM_CALIBRATION_TPL_BOOLEAN_H
#define ASLAM_CALIBRATION_TPL_BOOLEAN_H

namespace aslam {
  namespace calibration {

    /** The False structure defines the false template
        \brief False definition
      */
    struct False {
    public:
      /// False definition
      typedef False Result;
    };

    /** The True structure defines the true template
        \brief True definition
      */
    struct True {
    public:
      /// True definition
      typedef True Result;
      /// Numeric definition
      typedef bool Numeric;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_BOOLEAN_H
