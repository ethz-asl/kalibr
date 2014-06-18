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

/** \file UniformDistribution1v.h
    \brief This file contains an interface to the univariate uniform
           distributions
  */

#include "aslam/calibration/statistics/ContinuousDistribution.h"
#include "aslam/calibration/statistics/DiscreteDistribution.h"
#include "aslam/calibration/statistics/SampleDistribution.h"
#include "aslam/calibration/base/Serializable.h"
#include "aslam/calibration/tpl/IfThenElse.h"
#include "aslam/calibration/tpl/IsReal.h"
#include "aslam/calibration/tpl/IsInteger.h"

namespace aslam {
  namespace calibration {

    /** The UniformDistribution1v class represents an interface to the
        univariate uniform distributions.
        \brief Univariate uniform distribution
      */
    template <typename X> class UniformDistribution<X> :
      public IfThenElse<typename IsReal<X>::Result,
        ContinuousDistribution<X>, DiscreteDistribution<X> >::Result,
      public SampleDistribution<X>,
      public virtual Serializable {
    public:
      /** \name Types
        @{
        */
      /// Distribution type
      typedef typename IfThenElse<typename IsReal<X>::Result,
        ContinuousDistribution<X>, DiscreteDistribution<X> >::Result
        DistributionType;
      /// Random variable type
      typedef typename DistributionType::RandomVariable RandomVariable;
      /// Mean type
      typedef typename DistributionType::Mean Mean;
      /// Variance type
      typedef typename DistributionType::Variance Variance;
      /// Mode type
      typedef typename DistributionType::Mode Mode;
      /// Median type
      typedef typename DistributionType::Median Median;
      /** @}
        */

      /** \name Traits
        @{
        */
      /// Specialization for integer or real types
      struct Traits {
      public:
        /// Get support area for real types
        template <typename Z, typename IsReal<Z>::Result::Numeric>
          static Z getSupportArea(const Z& minSupport, const Z& maxSupport);
        /// Get support area for integer types
        template <typename Z, typename IsInteger<Z>::Result::Numeric>
          static Z getSupportArea(const Z& minSupport, const Z& maxSupport);
        /// Get variance for real types
        template <typename Z, typename IsReal<Z>::Result::Numeric>
          static double getVariance(const Z& minSupport, const Z& maxSupport);
        /// Get variance for integer types
        template <typename Z, typename IsInteger<Z>::Result::Numeric>
          static double getVariance(const Z& minSupport, const Z& maxSupport);
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs distribution from parameters
      UniformDistribution(const RandomVariable& minSupport = X(0), const
        RandomVariable& maxSupport = X(1));
      /// Copy constructor
      UniformDistribution(const UniformDistribution& other);
      /// Assignment operator
      UniformDistribution& operator = (const UniformDistribution& other);
      /// Destructor
      virtual ~UniformDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Sets the support of the distribution
      void setSupport(const RandomVariable& minSupport, const RandomVariable&
        maxSupport);
      /// Sets the minimum support
      void setMinSupport(const RandomVariable& minSupport);
      /// Returns the minimum support
      const RandomVariable& getMinSupport() const;
      /// Sets the maximum support
      void setMaxSupport(const RandomVariable& maxSupport);
      /// Returns the maximum support
      const RandomVariable& getMaxSupport() const;
      /// Returns the support area
      const X& getSupportArea() const;
      /// Returns the mean of the distribution
      Mean getMean() const;
      /// Returns the median of the distribution
      Median getMedian() const;
      /// Returns the mode of the distribution
      Mode getMode() const;
      /// Returns the variance of the distribution
      Variance getVariance() const;
      /// Access the probablity density function at the given value
      virtual double pdf(const RandomVariable& value) const;
      /// Access the probablity mass function at the given value
      virtual double pmf(const RandomVariable& value) const;
      /// Access a sample drawn from the distribution
      virtual RandomVariable getSample() const;
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
      /// Minimum support of the distribution
      RandomVariable mMinSupport;
      /// Maximum support of the distribution
      RandomVariable mMaxSupport;
      /// Support area
      RandomVariable mSupportArea;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/UniformDistribution1v.tpp"
