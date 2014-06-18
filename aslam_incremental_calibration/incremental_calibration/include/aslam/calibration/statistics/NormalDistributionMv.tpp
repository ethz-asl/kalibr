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

#include <Eigen/LU>

#include "aslam/calibration/statistics/Randomizer.h"
#include "aslam/calibration/exceptions/BadArgumentException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <int M>
    NormalDistribution<M>::NormalDistribution(const Mean& mean,
        const Covariance& covariance):
        mMean(mean) {
      setCovariance(covariance);
    }

    template <int M>
    NormalDistribution<M>::NormalDistribution(const
        std::tuple<Mean, Covariance>& parameters):
        mMean(std::get<0>(parameters)) {
      setCovariance(std::get<1>(parameters));
    }

    template <int M>
    NormalDistribution<M>::NormalDistribution(const NormalDistribution& other) :
        mMean(other.mMean),
        mCovariance(other.mCovariance),
        mPrecision(other.mPrecision),
        mDeterminant(other.mDeterminant),
        mNormalizer(other.mNormalizer),
        mTransformation(other.mTransformation) {
    }

    template <int M>
    NormalDistribution<M>& NormalDistribution<M>::operator = (const
        NormalDistribution& other) {
      if (this != &other) {
        mMean = other.mMean;
        mCovariance = other.mCovariance;
        mPrecision = other.mPrecision;
        mDeterminant = other.mDeterminant;
        mNormalizer = other.mNormalizer;
        mTransformation = other.mTransformation;
      }
      return *this;
    }

    template <int M>
    NormalDistribution<M>::~NormalDistribution() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <int M>
    void NormalDistribution<M>::read(std::istream& stream) {
    }

    template <int M>
    void NormalDistribution<M>::write(std::ostream& stream) const {
      stream << "mean: " << std::endl << mMean << std::endl
        << "covariance: " << std::endl << mCovariance;
    }

    template <int M>
    void NormalDistribution<M>::read(std::ifstream& stream) {
    }

    template <int M>
    void NormalDistribution<M>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <int M>
    void NormalDistribution<M>::setMean(const Mean& mean) {
      mMean = mean;
    }

    template <int M>
    typename NormalDistribution<M>::Mean NormalDistribution<M>::getMean()
        const {
      return mMean;
    }

    template <int M>
    void NormalDistribution<M>::setCovariance(const Covariance& covariance) {
      if (covariance.transpose() != covariance)
        throw BadArgumentException<Covariance>(covariance,
          "NormalDistribution<M>::setCovariance(): "
          "covariance must be symmetric",
          __FILE__, __LINE__);
      mTransformation = Eigen::LLT<Covariance>(covariance);
      if (!Eigen::LDLT<Covariance>(covariance).isPositive())
        throw BadArgumentException<Covariance>(covariance,
          "NormalDistribution<M>::setCovariance(): covariance must be positive "
          "definite",
          __FILE__, __LINE__);
      if ((covariance.diagonal().array() < 0).any())
        throw BadArgumentException<Covariance>(covariance,
          "NormalDistribution<M>::setCovariance(): variances must be positive",
          __FILE__, __LINE__);
      mPrecision = covariance.inverse();
      mDeterminant = covariance.determinant();
      mNormalizer = 0.5 * mMean.size() * log(2.0 * M_PI) + 0.5 *
        log(mDeterminant);
      mCovariance = covariance;
    }

    template <int M>
    typename NormalDistribution<M>::Covariance
        NormalDistribution<M>::getCovariance() const {
      return mCovariance;
    }

    template <int M>
    typename NormalDistribution<M>::Precision
        NormalDistribution<M>::getPrecision() const {
      return mPrecision;
    }

    template <int M>
    double NormalDistribution<M>::getDeterminant() const {
      return mDeterminant;
    }

    template <int M>
    double NormalDistribution<M>::getNormalizer() const {
      return mNormalizer;
    }

    template <int M>
    const Eigen::LLT<typename NormalDistribution<M>::Covariance>&
        NormalDistribution<M>::getTransformation() const {
      return mTransformation;
    }

    template <int M>
    double NormalDistribution<M>::pdf(const RandomVariable& value) const {
      return exp(logpdf(value));
    }

    template <int M>
    double NormalDistribution<M>::logpdf(const RandomVariable& value)
        const {
      return -0.5 * mahalanobisDistance(value) - mNormalizer;
    }

    template <int M>
    typename NormalDistribution<M>::RandomVariable
        NormalDistribution<M>::getSample() const {
      RandomVariable sample(mMean.size());
      const static Randomizer<double> randomizer;
      for (size_t i = 0; i < (size_t)mMean.size(); ++i)
        sample(i) = randomizer.sampleNormal();
      return mMean + mTransformation.matrixL() * sample;
    }

    template <int M>
    double NormalDistribution<M>::KLDivergence(const NormalDistribution<M>&
        other) const {
      return 0.5 * (log(other.mDeterminant / mDeterminant) +
        (other.mPrecision * mCovariance).trace() - mMean.size() +
        ((mMean - other.mMean).transpose() * other.mPrecision *
        (mMean - other.mMean))(0, 0));
    }

    template <int M>
    double NormalDistribution<M>::mahalanobisDistance(const RandomVariable& 
        value) const {
      return ((value - mMean).transpose() * mPrecision *
        (value - mMean))(0, 0);
    }

    template <int M>
    typename NormalDistribution<M>::Mode NormalDistribution<M>::getMode()
        const {
      return mMean;
    }

  }
}
