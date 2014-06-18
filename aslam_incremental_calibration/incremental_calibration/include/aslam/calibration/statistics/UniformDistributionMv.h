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

/** \file UniformDistributionMv.h
    \brief This file contains an interface to the multivariate uniform
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

    /** The UniformDistributionMv class represents an interface to the
        multivariate uniform distributions.
        \brief Multivariate uniform distribution
      */
    template <typename X, int M> class UniformDistribution:
      public IfThenElse<typename IsReal<X>::Result,
        ContinuousDistribution<X, M>, DiscreteDistribution<X, M> >::Result,
      public SampleDistribution<Eigen::Matrix<X, M, 1> >,
      public virtual Serializable {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types
        @{
        */
      /// Distribution type
      typedef typename IfThenElse<typename IsReal<X>::Result,
        ContinuousDistribution<X, M>, DiscreteDistribution<X, M> >::Result
        DistributionType;
      /// Random variable type
      typedef typename DistributionType::RandomVariable RandomVariable;
      /// Mean type
      typedef typename DistributionType::Mean Mean;
      /// Mean type
      typedef typename DistributionType::Median Median;
      /// Mode type
      typedef typename DistributionType::Mode Mode;
      /// Covariance type
      typedef typename DistributionType::Covariance Covariance;
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
          static Z getSupportArea(const Eigen::Matrix<Z, M, 1>& minSupport,
            const Eigen::Matrix<Z, M, 1>& maxSupport);
        /// Get support area for integer types
        template <typename Z, typename IsInteger<Z>::Result::Numeric>
          static Z getSupportArea(const Eigen::Matrix<Z, M, 1>& minSupport,
            const Eigen::Matrix<Z, M, 1>& maxSupport);
        /// Get covariance for real types
        template <typename Z, typename IsReal<Z>::Result::Numeric>
          static Eigen::Matrix<double, M, M> getCovariance(
            const Eigen::Matrix<Z, M, 1>& minSupport,
            const Eigen::Matrix<Z, M, 1>& maxSupport);
        /// Get covariance for integer types
        template <typename Z, typename IsInteger<Z>::Result::Numeric>
          static Eigen::Matrix<double, M, M> getCovariance(
            const Eigen::Matrix<Z, M, 1>& minSupport,
            const Eigen::Matrix<Z, M, 1>& maxSupport);
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs distribution from parameters
      UniformDistribution(const RandomVariable& minSupport =
        RandomVariable::Zero(),
        const RandomVariable& maxSupport = RandomVariable::Ones());
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
      /// Access the multivariate uniform distribution's support area
      const X& getSupportArea() const;
      /// Returns the mean of the distribution
      Mean getMean() const;
      /// Returns the median of the distribution
      Median getMedian() const;
      /// Returns the mode of the distribution
      Mode getMode() const;
      /// Returns the variance of the distribution
      Covariance getCovariance() const;
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
      X mSupportArea;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/UniformDistributionMv.tpp"
