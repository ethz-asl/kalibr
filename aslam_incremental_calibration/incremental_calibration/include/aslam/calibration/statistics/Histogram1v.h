/******************************************************************************
 * Copyright (C) 2011 by Jerome Maye                                          *
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

/** \file Histogram1v.h
    \brief This file contains the definition of a univariate histogram.
  */

#include "aslam/calibration/data-structures/Grid.h"

namespace aslam {
  namespace calibration {

    /** The Histogram1v class defines univariate histograms.
        \brief Univariate histogram
      */
    template <typename T> class Histogram<T, 1> :
      public Grid<T, double, 1> {
    public:
      /** \name Types definitions
        @{
        */
      /// Coordinate type
      typedef typename Grid<T, double, 1>::Coordinate Coordinate;
      /// Index type
      typedef typename Grid<T, double, 1>::Index Index;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs histogram from parameters
      Histogram(const T& min = T(0), const T& max = T(1),
        const T& binSize = T(1));
      /// Copy constructor
      Histogram(const Histogram& other);
      /// Assignment operator
      Histogram& operator = (const Histogram& other);
      /// Destructor
      virtual ~Histogram();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the mean value of the histogram
      double getMean() const;
      /// Returns the median value of the histogram
      double getMedian() const;
      /// Returns the mode value of the histogram
      double getMode() const;
      /// Returns the variance of the histogram
      double getVariance() const;
      /// Returns the sum of the histogram
      double getSum() const;
      /// Add a sample to the histogram
      void addSample(const T& sample);
      /// Add samples to the histogram
      void addSamples(const std::vector<T>& samples);
      /// Returns a normalized copy of the histogram
      Histogram getNormalized() const;
      /** @}
        */

    protected:

    };

  }
}

#include "aslam/calibration/statistics/Histogram1v.tpp"
