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

/** \file Distribution.h
    \brief This file contains an interface to any kind of distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_DISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_DISTRIBUTION_H

#include "aslam/calibration/functions/Function.h"

namespace aslam {
  namespace calibration {

    /** The Distribution class represents an interface to any kind of
        distributions.
        \brief Distribution
      */
    template <typename X> class Distribution :
      public virtual Function<double, X> {
    public:
      /** \name Types
        @{
        */
      /// Random variable type
      typedef typename Function<double, X>::Domain RandomVariable;
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~Distribution();
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/Distribution.tpp"

#endif // ASLAM_CALIBRATION_STATISTICS_DISTRIBUTION_H
