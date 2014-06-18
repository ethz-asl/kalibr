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

#include "aslam/calibration/functions/LogGammaFunction.h"

#include "aslam/calibration/exceptions/BadArgumentException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    LogGammaFunction<size_t>::LogGammaFunction() {
    }

    LogGammaFunction<size_t>::~LogGammaFunction() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double LogGammaFunction<size_t>::getValue(const VariableType& argument)
        const {
      if (argument)
        return LogFactorialFunction::getValue(argument - 1);
      else throw BadArgumentException<size_t>(argument,
        "LogGammaFunction<size_t>::getValue(): argument must be strictly "
        "positive",
        __FILE__, __LINE__);
    }

  }
}
