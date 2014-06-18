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

/** \file HistogramMv.h
    \brief This file contains the definition of a multivariate histogram.
  */

#include "aslam/calibration/data-structures/Grid.h"

namespace aslam {
  namespace calibration {

    /** The HistogramMv class defines multivariate histograms.
        \brief Multivariate histogram
      */
    template <typename T, int M> class Histogram :
      public Grid<T, double, M> {
    public:
      /** \name Types definitions
        @{
        */
      /// Coordinate type
      typedef typename Grid<T, double, M>::Coordinate Coordinate;
      /// Index type
      typedef typename Grid<T, double, M>::Index Index;
      /// Mean type
      typedef Eigen::Matrix<double, M, 1> Mean;
      /// Mode type
      typedef Eigen::Matrix<double, M, 1> Mode;
      /// Covariance type
      typedef Eigen::Matrix<double, M, M> Covariance;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs histogram from parameters
      Histogram(const Coordinate& min = Coordinate::Zero(),
        const Coordinate& max = Coordinate::Ones(),
        const Coordinate& binSize = Coordinate::Ones());
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
      Mean getMean() const;
      /// Returns the mode value of the histogram
      Mode getMode() const;
      /// Returns the covariance of the histogram
      Covariance getCovariance() const;
      /// Returns the sum of the histogram
      double getSum() const;
      /// Add a sample to the histogram
      void addSample(const Coordinate& sample);
      /// Add samples to the histogram
      void addSamples(const std::vector<Coordinate>& samples);
      /// Returns a normalized copy of the histogram
      Histogram getNormalized() const;
      /** @}
        */

    protected:

    };

  }
}

#include "aslam/calibration/statistics/HistogramMv.tpp"
