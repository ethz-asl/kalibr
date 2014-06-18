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

#include <boost/math/distributions/gamma.hpp>

#include "aslam/calibration/statistics/Randomizer.h"
#include "aslam/calibration/functions/LogGammaFunction.h"
#include "aslam/calibration/functions/DigammaFunction.h"
#include "aslam/calibration/functions/IncompleteGammaPFunction.h"
#include "aslam/calibration/exceptions/BadArgumentException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename T>
    GammaDistribution<T>::GammaDistribution(const T& shape, double invScale) {
      setShape(shape);
      setInvScale(invScale);
    }

    template <typename T>
    GammaDistribution<T>::GammaDistribution(const GammaDistribution& other) :
        mShape(other.mShape),
        mInvScale(other.mInvScale),
        mNormalizer(other.mNormalizer) {
    }

    template <typename T>
    GammaDistribution<T>& GammaDistribution<T>::operator =
        (const GammaDistribution& other) {
      if (this != &other) {
        mShape = other.mShape;
        mInvScale = other.mInvScale;
        mNormalizer = other.mNormalizer;
      }
      return *this;
    }

    template <typename T>
    GammaDistribution<T>::~GammaDistribution() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <typename T>
    void GammaDistribution<T>::read(std::istream& stream) {
    }

    template <typename T>
    void GammaDistribution<T>::write(std::ostream& stream) const {
      stream << "shape: " << mShape << std::endl
        << "inverse scale: " << mInvScale;
    }

    template <typename T>
    void GammaDistribution<T>::read(std::ifstream& stream) {
    }

    template <typename T>
    void GammaDistribution<T>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename T>
    void GammaDistribution<T>::setShape(const T& shape) {
      if (shape <= 0)
        throw BadArgumentException<T>(shape,
          "GammaDistribution::setShape(): shape must be strictly positive",
          __FILE__, __LINE__);
      mShape = shape;
      computeNormalizer();
    }

    template <typename T>
    const T& GammaDistribution<T>::getShape() const {
      return mShape;
    }

    template <typename T>
    void GammaDistribution<T>::setInvScale(double invScale) {
      if (invScale <= 0)
        throw BadArgumentException<double>(invScale,
          "GammaDistribution::setScale(): inverse scale must be "
          "strictly positive",
          __FILE__, __LINE__);
      mInvScale = invScale;
      computeNormalizer();
    }

    template <typename T>
    double GammaDistribution<T>::getInvScale() const {
      return mInvScale;
    }

    template <typename T>
    void GammaDistribution<T>::computeNormalizer() {
      LogGammaFunction<T> logGammaFunction;
      mNormalizer = logGammaFunction(mShape) - mShape * log(mInvScale);
    }

    template <typename T>
    double GammaDistribution<T>::getNormalizer() const {
      return mNormalizer;
    }

    template <typename T>
    double GammaDistribution<T>::pdf(const RandomVariable& value) const {
      if (value < 0)
        return 0.0;
      else
        return exp(logpdf(value));
    }

    template <typename T>
    double GammaDistribution<T>::logpdf(const RandomVariable& value) const {
      if (value == 0 && mShape == T(1))
        return -mNormalizer;
      else
        return (mShape - 1) * log(value) - value * mInvScale - mNormalizer;
    }

    template <typename T>
    double GammaDistribution<T>::cdf(const RandomVariable& value) const {
      const IncompleteGammaPFunction incGammaPFunction(mShape);
      if (value <= 0)
        return 0.0;
      else
        return incGammaPFunction(value * mInvScale);
    }

    template <typename T>
    typename GammaDistribution<T>::RandomVariable
        GammaDistribution<T>::invcdf(double probability) const {
      if (probability < 0 || probability > 1)
        throw BadArgumentException<double>(probability,
          "GammaDistribution::invcdf(): probability must lie in [0, 1]",
          __FILE__, __LINE__);
        return boost::math::quantile(boost::math::gamma_distribution<>(
          mShape, 1.0 / mInvScale), probability);
    }

    template <typename T>
    typename GammaDistribution<T>::RandomVariable
        GammaDistribution<T>::getSample() const {
      const static Randomizer<double> randomizer;
      return randomizer.sampleGamma(mShape, mInvScale);
    }

    template <typename T>
    typename GammaDistribution<T>::Mean GammaDistribution<T>::getMean() const {
      return mShape / mInvScale;
    }

    template <typename T>
    typename GammaDistribution<T>::Mode GammaDistribution<T>::getMode() const {
      if (mShape >= 1)
        return (mShape - 1) / mInvScale;
      else
        throw InvalidOperationException("GammaDistribution<T>::getMode(): "
          "shape must be bigger or equal than 1");
    }

    template <typename T>
    typename GammaDistribution<T>::Variance GammaDistribution<T>::getVariance()
        const {
      return mShape / (mInvScale * mInvScale);
    }

    template <typename T>
    double GammaDistribution<T>::KLDivergence(const GammaDistribution<T>& other)
        const {
      LogGammaFunction<T> logGammaFunction;
      const DigammaFunction<T> digammaFunction;
      return (mShape - 1) * digammaFunction(mShape) -
        (other.mShape - 1) * digammaFunction(other.mShape) -
        logGammaFunction(mShape) +
        logGammaFunction(other.mShape) + other.mShape *
        (log(mInvScale) - log(other.mInvScale)) +
        mShape * (1.0 / mInvScale - 1.0 / other.mInvScale) * other.mInvScale;
    }

  }
}
