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

/** \file SizeTSupport.h
    \brief This file defines the Eigen support for size_t type.
  */

#ifndef ASLAM_CALIBRATION_UTILS_SIZETSUPPORT_H
#define ASLAM_CALIBRATION_UTILS_SIZETSUPPORT_H

#include <cstdlib>

#include <Eigen/Core>

namespace Eigen {

  /** The NumTraits<size_t> structure defines support for size_t type in Eigen.
      \brief Eigen support for size_t
    */
  template<> struct NumTraits<size_t> {
    /// Real definition
    typedef size_t Real;
    /// Floating point definition
    typedef double FloatingPoint;
    /// Enum for Eigen
    enum {
      /// Is complex
      IsComplex = 0,
      /// Has floating point
      HasFloatingPoint = 0,
      /// Read cost
      ReadCost = 1,
      /// Add cost
      AddCost = 1,
      /// Multiplicative cost
      MulCost = 1,
    };
  };

}

#endif // ASLAM_CALIBRATION_UTILS_SIZETSUPPORT_H
