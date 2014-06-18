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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file DiscreteFunction.h
    \brief This file is an interface to the discrete functions
  */

#ifndef ASLAM_CALIBRATION_FUNCTIONS_DISCRETEFUNCTION_H
#define ASLAM_CALIBRATION_FUNCTIONS_DISCRETEFUNCTION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    template <typename Y, typename X, int M = 1, int N = 1>
      class DiscreteFunction;

  }
}

#include "aslam/calibration/functions/DiscreteFunction1v.h"
#include "aslam/calibration/functions/DiscreteFunctionMv.h"

#endif // ASLAM_CALIBRATION_FUNCTIONS_DISCRETEFUNCTION_H
