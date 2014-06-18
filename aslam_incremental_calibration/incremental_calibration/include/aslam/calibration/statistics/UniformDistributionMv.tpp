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

#include "aslam/calibration/statistics/Randomizer.h"
#include "aslam/calibration/exceptions/BadArgumentException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename X, int M>
    UniformDistribution<X, M>::UniformDistribution(const RandomVariable&
        minSupport, const RandomVariable& maxSupport) :
        mSupportArea(0.0) {
      setSupport(minSupport, maxSupport);
    }

    template <typename X, int M>
    UniformDistribution<X, M>::UniformDistribution(const UniformDistribution&
        other) :
        mMinSupport(other.mMinSupport),
        mMaxSupport(other.mMaxSupport),
        mSupportArea(other.mSupportArea) {
    }

    template <typename X, int M>
    UniformDistribution<X, M>& UniformDistribution<X, M>::operator =
        (const UniformDistribution& other) {
      if (this != &other) {
        mMaxSupport = other.mMaxSupport;
        mMinSupport = other.mMinSupport;
        mSupportArea = other.mSupportArea;
      }
      return *this;
    }

    template <typename X, int M>
    UniformDistribution<X, M>::~UniformDistribution() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <typename X, int M>
    void UniformDistribution<X, M>::read(std::istream& stream) {
    }

    template <typename X, int M>
    void UniformDistribution<X, M>::write(std::ostream& stream) const {
      stream << "minimum support: " << std::endl << mMinSupport << std::endl
        << "maximum support: " << std::endl << mMaxSupport << std::endl
        << "support area: " << mSupportArea;
    }

    template <typename X, int M>
    void UniformDistribution<X, M>::read(std::ifstream& stream) {
    }

    template <typename X, int M>
    void UniformDistribution<X, M>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X, int M>
    void UniformDistribution<X, M>::setSupport(const RandomVariable& minSupport,
        const RandomVariable& maxSupport) {
      if ((minSupport.array() >= maxSupport.array()).any())
        throw BadArgumentException<RandomVariable>(minSupport,
          "UniformDistribution<X, M>::setSupport(): minimum support must be "
          "smaller than maximum support",
          __FILE__, __LINE__);
      mMinSupport = minSupport;
      mMaxSupport = maxSupport;
      mSupportArea = Traits::template getSupportArea<X, true>(minSupport,
        maxSupport);
    }

    template <typename X, int M>
    void UniformDistribution<X, M>::setMinSupport(const RandomVariable&
        minSupport) {
      setSupport(minSupport, mMaxSupport);
    }

    template <typename X, int M>
    const typename UniformDistribution<X, M>::RandomVariable&
        UniformDistribution<X, M>::getMinSupport() const {
      return mMinSupport;
    }

    template <typename X, int M>
    void UniformDistribution<X, M>::setMaxSupport(const RandomVariable&
        maxSupport) {
      setSupport(mMinSupport, maxSupport);
    }

    template <typename X, int M>
    const typename UniformDistribution<X, M>::RandomVariable&
        UniformDistribution<X, M>::getMaxSupport() const {
      return mMaxSupport;
    }

    template <typename X, int M>
    const X& UniformDistribution<X, M>::getSupportArea() const {
      return mSupportArea;
    }

    template <typename X, int M>
    double UniformDistribution<X, M>::pdf(const RandomVariable& value) const {
      if ((value.array() <= mMaxSupport.array()).all() &&
          (value.array() >= mMinSupport.array()).all())
        return 1.0 / mSupportArea;
      else
        return 0;
    }

    template <typename X, int M>
    double UniformDistribution<X, M>::pmf(const RandomVariable& value) const {
      return pdf(value);
    }

    template <typename X, int M>
    typename UniformDistribution<X, M>::RandomVariable
        UniformDistribution<X, M>::getSample() const {
      const static Randomizer<X> randomizer;
      RandomVariable sample(mMinSupport.size());
      for (size_t i = 0; i < (size_t)sample.size(); ++i)
        sample(i) = randomizer.sampleUniform(mMinSupport(i), mMaxSupport(i));
      return sample;
    }

    template <typename X, int M>
    typename UniformDistribution<X, M>::Mean
        UniformDistribution<X, M>::getMean() const {
      return 0.5 * (mMaxSupport - mMinSupport).template cast<double>();
    }

    template <typename X, int M>
    typename UniformDistribution<X, M>::Mode
        UniformDistribution<X, M>::getMode() const {
      throw InvalidOperationException("UniformDistribution<X, M>::getMode(): "
        "undefined mode");
    }

    template <typename X, int M>
    typename UniformDistribution<X, M>::Median
        UniformDistribution<X, M>::getMedian() const {
      return 0.5 * (mMaxSupport - mMinSupport).template cast<double>();
    }

    template <typename X, int M>
    typename UniformDistribution<X, M>::Covariance
        UniformDistribution<X, M>::getCovariance() const {
      return Traits::template getCovariance<X, true>(mMinSupport, mMaxSupport);
    }

    template <typename X, int M>
    template <typename Z, typename IsReal<Z>::Result::Numeric>
    Z UniformDistribution<X, M>::Traits::getSupportArea(
        const Eigen::Matrix<Z, M, 1>& minSupport,
        const Eigen::Matrix<Z, M, 1>& maxSupport) {
      double supportArea = 1;
      for (size_t i = 0; i < (size_t)minSupport.size(); ++i)
        supportArea *= maxSupport(i) - minSupport(i);
      return supportArea;
    }

    template <typename X, int M>
    template <typename Z, typename IsInteger<Z>::Result::Numeric>
    Z UniformDistribution<X, M>::Traits::getSupportArea(
        const Eigen::Matrix<Z, M, 1>& minSupport,
        const Eigen::Matrix<Z, M, 1>& maxSupport) {
      double supportArea = 1;
      for (size_t i = 0; i < (size_t)minSupport.size(); ++i)
        supportArea *= maxSupport(i) - minSupport(i) + 1;
      return supportArea;
    }

    template <typename X, int M>
    template <typename Z, typename IsReal<Z>::Result::Numeric>
    Eigen::Matrix<double, M, M>
    UniformDistribution<X, M>::Traits::getCovariance(
        const Eigen::Matrix<Z, M, 1>& minSupport,
        const Eigen::Matrix<Z, M, 1>& maxSupport) {
      const Eigen::Matrix<double, M, M> covariance =
        Eigen::Matrix<double, M, M>::Identity(minSupport.size(),
        minSupport.size());
      const double supportArea =
        Traits::template getSupportArea<Z, true>(minSupport, maxSupport);
      return (supportArea * supportArea) / 12.0 * covariance;
    }

    template <typename X, int M>
    template <typename Z, typename IsInteger<Z>::Result::Numeric>
    Eigen::Matrix<double, M, M>
    UniformDistribution<X, M>::Traits::getCovariance(
        const Eigen::Matrix<Z, M, 1>& minSupport,
        const Eigen::Matrix<Z, M, 1>& maxSupport) {
      const Eigen::Matrix<double, M, M> covariance =
        Eigen::Matrix<double, M, M>::Identity(minSupport.size(),
        minSupport.size());
      const double supportArea =
        Traits::template getSupportArea<Z, true>(minSupport, maxSupport);
      return (supportArea * supportArea - 1.0) / 12.0 * covariance;
    }

  }
}
