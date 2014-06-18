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

/** \file SampleDistribution.h
    \brief This file defines the class SampleDistribution, which represents an
           interface to sample distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_SAMPLEDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_SAMPLEDISTRIBUTION_H

#include <vector>

#include "aslam/calibration/statistics/Distribution.h"

namespace aslam {
  namespace calibration {

    /** The SampleDistribution class represents an interface to sample
        distributions, i.e. distributions which can directly be sampled from
        \brief Sample distribution
      */
    template <typename X> class SampleDistribution :
      public virtual Distribution<X> {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~SampleDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access a sample drawn from the distribution
      virtual X getSample() const = 0;
      /// Access samples drawn from the distribution
      void getSamples(std::vector<X>& samples, size_t numSamples) const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/SampleDistribution.tpp"

#endif // ASLAM_CALIBRATION_STATISTICS_SAMPLEDISTRIBUTION_H
