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

    template <typename X>
    UniformDistribution<X>::UniformDistribution(const RandomVariable&
        minSupport, const RandomVariable& maxSupport) {
      setSupport(minSupport, maxSupport);
    }

    template <typename X>
    UniformDistribution<X>::UniformDistribution(const UniformDistribution&
        other) :
        mMinSupport(other.mMinSupport),
        mMaxSupport(other.mMaxSupport),
        mSupportArea(other.mSupportArea) {
    }

    template <typename X>
    UniformDistribution<X>& UniformDistribution<X>::operator =
        (const UniformDistribution& other) {
      if (this != &other) {
        mMaxSupport = other.mMaxSupport;
        mMinSupport = other.mMinSupport;
        mSupportArea = other.mSupportArea;
      }
      return *this;
    }

    template <typename X>
    UniformDistribution<X>::~UniformDistribution() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <typename X>
    void UniformDistribution<X>::read(std::istream& stream) {
    }

    template <typename X>
    void UniformDistribution<X>::write(std::ostream& stream) const {
      stream << "minimum support: " << mMinSupport << std::endl
        << "maximum support: " << mMaxSupport;
    }

    template <typename X>
    void UniformDistribution<X>::read(std::ifstream& stream) {
    }

    template <typename X>
    void UniformDistribution<X>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename X>
    void UniformDistribution<X>::setSupport(const RandomVariable& minSupport,
        const RandomVariable& maxSupport) {
      if (minSupport >= maxSupport)
        throw BadArgumentException<RandomVariable>(minSupport,
          "UniformDistribution<X>::setSupport(): "
          "minimum support must be smaller "
          "than maximum support",
          __FILE__, __LINE__);
      mMinSupport = minSupport;
      mMaxSupport = maxSupport;
      mSupportArea = Traits::template getSupportArea<RandomVariable, true>(
        minSupport, maxSupport);
    }

    template <typename X>
    void UniformDistribution<X>::setMinSupport(const RandomVariable&
        minSupport) {
      setSupport(minSupport, mMaxSupport);
    }

    template <typename X>
    const typename UniformDistribution<X>::RandomVariable&
        UniformDistribution<X>::getMinSupport() const {
      return mMinSupport;
    }

    template <typename X>
    void UniformDistribution<X>::setMaxSupport(const RandomVariable&
        maxSupport) {
      setSupport(mMinSupport, maxSupport);
    }

    template <typename X>
    const typename UniformDistribution<X>::RandomVariable&
        UniformDistribution<X>::getMaxSupport() const {
      return mMaxSupport;
    }

    template <typename X>
    const X& UniformDistribution<X>::getSupportArea() const {
      return mSupportArea;
    }

    template <typename X>
    double UniformDistribution<X>::pdf(const RandomVariable& value) const {
      if (value >= mMinSupport && value <= mMaxSupport)
        return 1.0 / mSupportArea;
      else
        return 0;
    }

    template <typename X>
    double UniformDistribution<X>::pmf(const RandomVariable& value) const {
      return pdf(value);
    }

    template <typename X>
    typename UniformDistribution<X>::RandomVariable
        UniformDistribution<X>::getSample() const {
      const static Randomizer<X> randomizer;
      return randomizer.sampleUniform(mMinSupport, mMaxSupport);
    }

    template <typename X>
    typename UniformDistribution<X>::Mean UniformDistribution<X>::getMean()
        const {
      return 0.5 * (mMaxSupport - mMinSupport);
    }

    template <typename X>
    typename UniformDistribution<X>::Median UniformDistribution<X>::getMedian()
        const {
      return 0.5 * (mMaxSupport - mMinSupport);
    }

    template <typename X>
    typename UniformDistribution<X>::Mode UniformDistribution<X>::getMode()
        const {
      throw InvalidOperationException("UniformDistribution<X>::getMode(): "
        "undefined mode");
    }

    template <typename X>
    typename UniformDistribution<X>::Variance
        UniformDistribution<X>::getVariance() const {
      return Traits::template getVariance<X, true>(mMinSupport, mMaxSupport);
    }

    template <typename X>
    template <typename Z, typename IsReal<Z>::Result::Numeric>
    Z UniformDistribution<X>::Traits::getSupportArea(const Z& minSupport,
        const Z& maxSupport) {
      return maxSupport - minSupport;
    }

    template <typename X>
    template <typename Z, typename IsInteger<Z>::Result::Numeric>
    Z UniformDistribution<X>::Traits::getSupportArea(const Z& minSupport,
        const Z& maxSupport) {
      return maxSupport - minSupport + 1;
    }

    template <typename X>
    template <typename Z, typename IsReal<Z>::Result::Numeric>
    double UniformDistribution<X>::Traits::getVariance(const Z& minSupport,
        const Z& maxSupport) {
      const double supportArea =
        Traits::template getSupportArea<Z, true>(minSupport, maxSupport);
      return (supportArea * supportArea) / 12.0;
    }

    template <typename X>
    template <typename Z, typename IsInteger<Z>::Result::Numeric>
    double UniformDistribution<X>::Traits::getVariance(const Z& minSupport,
         const Z& maxSupport) {
      const double supportArea =
        Traits::template getSupportArea<Z, true>(minSupport, maxSupport);
      return (supportArea * supportArea - 1.0) / 12.0;
    }

  }
}
