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

/** \file Randomizer.h
    \brief This file defines the Randomizer class, which implements random
           sampling from several distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_RANDOMIZER_H
#define ASLAM_CALIBRATION_STATISTICS_RANDOMIZER_H

#include "aslam/calibration/base/Serializable.h"
#include "aslam/calibration/utils/SizeTSupport.h"
#include "aslam/calibration/tpl/IsReal.h"
#include "aslam/calibration/tpl/IsInteger.h"

namespace aslam {
  namespace calibration {

    /** The Randomizer class implements random sampling from several
        distributions
        \brief Random sampling from distributions
      */
    template <typename T = double, int M = 1> class Randomizer :
      public virtual Serializable {
    public:
      /** \name Traits
        @{
        */
      /// Specialization for integer or real types
      struct Traits {
      public:
        /// Round function for real types
        template <typename Z, typename IsReal<Z>::Result::Numeric>
          static Z round(const Z& value);
        /// Round function for integer types
        template <typename Z, typename IsInteger<Z>::Result::Numeric>
          static Z round(const double& value);
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs randomizer from seed
      Randomizer(const T& seed = getSeed());
      /// Copy constructor
      Randomizer(const Randomizer& other);
      /// Assignment operator
      Randomizer& operator = (const Randomizer& other);
      /// Destructor
      virtual ~Randomizer();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Sets the seed of the random sampler
      void setSeed(const T& seed);
      /// Returns the seed of the random sampler
      static T getSeed();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Returns a sample from a uniform distribution within specified support
      T sampleUniform(const T& minSupport = T(0), const T& maxSupport = T(1))
        const;
      /// Returns a sample from a normal distribution
      T sampleNormal(const T& mean = T(0), const T& variance = T(1)) const;
      /// Returns a sample from a categorical distribution
      size_t sampleCategorical(const Eigen::Matrix<double, M, 1>&
        probabilities = Eigen::Matrix<double, M, 1>::Constant(1.0 / M)) const;
      /// Returns a sample from a Poisson distribution
      size_t samplePoisson(double mean = 1.0) const;
      /// Returns a sample from a exponential distribution
      double sampleExponential(double rate = 1.0) const;
      /// Returns a sample from a geometric distribution
      size_t sampleGeometric(double successProbability = 0.5) const;
      /// Returns a sample from a gamma distribution
      double sampleGamma(double shape = 1.0, double invScale = 1.0) const;
      /** @}
        */

    protected:
      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Seed of the random sampling
      T mSeed;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/Randomizer.tpp"

#endif // ASLAM_CALIBRATION_STATISTICS_RANDOMIZER_H
