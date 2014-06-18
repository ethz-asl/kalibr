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

/** \file NormalDistributionMv.h
    \brief This file defines the NormalDistributionMv class, which represents a
           multivariate normal distribution
  */

#include <tuple>

#include <Eigen/Cholesky>

#include "aslam/calibration/statistics/ContinuousDistribution.h"
#include "aslam/calibration/statistics/SampleDistribution.h"
#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The NormalDistributionMv class represents a multivariate normal
        distribution.
        \brief Multivariate normal distribution
      */
    template <int M> class NormalDistribution :
      public ContinuousDistribution<double, M>,
      public SampleDistribution<Eigen::Matrix<double, M, 1> >,
      public virtual Serializable {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types
        @{
        */
      /// Distribution type
      typedef ContinuousDistribution<double, M> DistributionType;
      /// Random variable type
      typedef typename DistributionType::RandomVariable RandomVariable;
      /// Mean type
      typedef typename DistributionType::Mean Mean;
      /// Mode type
      typedef typename DistributionType::Mode Mode;
      /// Covariance type
      typedef typename DistributionType::Covariance Covariance;
      /// Precision type
      typedef Covariance Precision;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs the distribution from the parameters
      NormalDistribution(const Mean& mean = Mean::Zero(), const Covariance&
        covariance = Covariance::Identity());
      /// Constructs a normal distribution from the parameters
      NormalDistribution(const std::tuple<Mean, Covariance>& parameters);
      /// Copy constructor
      NormalDistribution(const NormalDistribution& other);
      /// Assignment operator
      NormalDistribution<M>& operator = (const NormalDistribution& other);
      /// Destructor
      virtual ~NormalDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Sets the mean of the distribution
      void setMean(const Mean& mean);
      /// Returns the mean of the distribution
      Mean getMean() const;
      /// Sets the covariance matrix of the distribution
      void setCovariance(const Covariance& covariance);
      /// Returns the covariance matrix of the distribution
      Covariance getCovariance() const;
      /// Returns the precision matrix of the distribution
      Precision getPrecision() const;
      /// Returns the determinant of the covariance matrix
      double getDeterminant() const;
      /// Returns the normalizer of the distribution
      double getNormalizer() const;
      /// Returns the mode of the distribution
      Mode getMode() const;
      /// Returns the cholesky decomposition of the covariance matrix
      const Eigen::LLT<Covariance>& getTransformation() const;
      /// Access the probability density function at the given value
      virtual double pdf(const RandomVariable& value) const;
      /// Access the log-probability density function at the given value
      double logpdf(const RandomVariable& value) const;
      /// Access a sample drawn from the distribution
      virtual RandomVariable getSample() const;
      /// Returns the KL-divergence with another distribution
      double KLDivergence(const NormalDistribution<M>& other) const;
      /// Returns the squared Mahalanobis distance from a point
      double mahalanobisDistance(const RandomVariable& value) const;
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
      /// Mean of the normal distribution
      Mean mMean;
      /// Covariance matrix of the normal distribution
      Covariance mCovariance;
      /// Precision matrix of the normal distribution
      Covariance mPrecision;
      /// Determinant of the covariance matrix
      double mDeterminant;
      /// Normalizer of the distribution
      double mNormalizer;
      /// Cholesky decomposition of the covariance matrix
      Eigen::LLT<Covariance> mTransformation;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/NormalDistributionMv.tpp"
