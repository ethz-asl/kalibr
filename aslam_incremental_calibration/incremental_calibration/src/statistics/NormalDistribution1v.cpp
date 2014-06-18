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

#include "aslam/calibration/statistics/NormalDistribution.h"

#include "aslam/calibration/statistics/Randomizer.h"
#include "aslam/calibration/exceptions/BadArgumentException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    NormalDistribution<1>::NormalDistribution(Mean mean, Variance variance) :
        mMean(mean) {
      setVariance(variance);
    }

    NormalDistribution<1>::NormalDistribution(const std::tuple<Mean, Variance>&
        parameters) :
        mMean(std::get<0>(parameters)) {
      setVariance(std::get<1>(parameters));
    }

    NormalDistribution<1>::NormalDistribution(const NormalDistribution& other) :
        mMean(other.mMean),
        mVariance(other.mVariance),
        mPrecision(other.mPrecision),
        mStandardDeviation(other.mStandardDeviation),
        mNormalizer(other.mNormalizer) {
    }

    NormalDistribution<1>& NormalDistribution<1>::operator =
        (const NormalDistribution& other) {
      if (this != &other) {
        mMean = other.mMean;
        mVariance = other.mVariance;
        mPrecision = other.mPrecision;
        mStandardDeviation = other.mStandardDeviation;
        mNormalizer = other.mNormalizer;
      }
      return *this;
    }

    NormalDistribution<1>::~NormalDistribution() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    void NormalDistribution<1>::read(std::istream& stream) {
    }

    void NormalDistribution<1>::write(std::ostream& stream) const {
      stream << "mean: " << mMean << std::endl
        << "variance: " << mVariance;
    }

    void NormalDistribution<1>::read(std::ifstream& stream) {
    }

    void NormalDistribution<1>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    void NormalDistribution<1>::setMean(Mean mean) {
      mMean = mean;
    }

    NormalDistribution<1>::Mean NormalDistribution<1>::getMean() const {
      return mMean;
    }

    void NormalDistribution<1>::setVariance(Variance variance) {
      if (variance <= 0.0)
        throw BadArgumentException<Variance>(variance,
          "NormalDistribution::setVariance(): variance must be strictly bigger "
          "than 0",
          __FILE__, __LINE__);
      mVariance = variance;
      mPrecision = 1.0 / variance;
      mStandardDeviation = sqrt(variance);
      mNormalizer = 0.5 * log(2.0 * M_PI * mVariance);
    }

    NormalDistribution<1>::Variance NormalDistribution<1>::getVariance() const {
      return mVariance;
    }

    NormalDistribution<1>::Precision NormalDistribution<1>::getPrecision()
        const {
      return mPrecision;
    }

    NormalDistribution<1>::Std NormalDistribution<1>::getStandardDeviation()
        const {
      return mStandardDeviation;
    }

    double NormalDistribution<1>::getNormalizer() const {
      return mNormalizer;
    }

    double NormalDistribution<1>::pdf(const RandomVariable& value) const {
      return exp(logpdf(value));
    }

    double NormalDistribution<1>::logpdf(const RandomVariable& value) const {
      return -0.5 * mahalanobisDistance(value) - mNormalizer;
    }

    double NormalDistribution<1>::cdf(const RandomVariable& value) const {
      return 0.5 * (1.0 + erf((value - mMean) / sqrt(2 * mVariance)));
    }

    NormalDistribution<1>::RandomVariable NormalDistribution<1>::getSample()
        const {
      const static Randomizer<double> randomizer;
      return randomizer.sampleNormal(mMean, mVariance);
    }

    double NormalDistribution<1>::KLDivergence(const NormalDistribution<1>&
        other) const {
      return 0.5 * (log(other.mVariance * mPrecision) +
        other.mPrecision * mVariance - 1.0 +
        (mMean - other.mMean) * other.mPrecision * (mMean - other.mMean));
    }

    double NormalDistribution<1>::mahalanobisDistance(const RandomVariable&
        value) const {
      return (value - mMean) * mPrecision * (value - mMean);
    }

    NormalDistribution<1>::Median NormalDistribution<1>::getMedian() const {
      return mMean;
    }

    NormalDistribution<1>::Mode NormalDistribution<1>::getMode() const {
      return mMean;
    }

  }
}
