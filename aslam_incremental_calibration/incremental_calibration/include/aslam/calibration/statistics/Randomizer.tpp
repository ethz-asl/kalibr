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

#include "aslam/calibration/base/Timestamp.h"
#include "aslam/calibration/exceptions/BadArgumentException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename T, int M>
    Randomizer<T, M>::Randomizer(const T& seed) :
        mSeed(seed) {
    }

    template <typename T, int M>
    Randomizer<T, M>::Randomizer(const Randomizer& other) :
        mSeed(other.mSeed) {
    }

    template <typename T, int M>
    Randomizer<T, M>& Randomizer<T, M>::operator = (const Randomizer& other) {
      if (this != &other) {
        mSeed = other.mSeed;
      }
      return *this;
    }

    template <typename T, int M>
    Randomizer<T, M>::~Randomizer() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <typename T, int M>
    void Randomizer<T, M>::read(std::istream& stream) {
    }

    template <typename T, int M>
    void Randomizer<T, M>::write(std::ostream& stream) const {
      stream << "seed: " << mSeed;
    }

    template <typename T, int M>
    void Randomizer<T, M>::read(std::ifstream& stream) {
    }

    template <typename T, int M>
    void Randomizer<T, M>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename T, int M>
    void Randomizer<T, M>::setSeed(const T& seed) {
      mSeed = seed;
      srandom(seed);
    }

    template <typename T, int M>
    T Randomizer<T, M>::getSeed() {
      static bool seeded = false;
      if (!seeded) {
        const double time = Timestamp::now();
        const T seed = (time - floor(time)) * 1e6;
        srandom(seed);
        seeded = true;
      }
      return random();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <typename T, int M>
    T Randomizer<T, M>::sampleUniform(const T& minSupport, const T& maxSupport)
        const {
      if (minSupport >= maxSupport)
        throw BadArgumentException<T>(minSupport,
          "Randomizer<T, M>::sampleUniform(): minimum support must be smaller "
          "than maximum support",
          __FILE__, __LINE__);
      return minSupport + Traits::template round<T, true>(random() /
        (double)RAND_MAX * (maxSupport - minSupport));
    }

    template <typename T, int M>
    T Randomizer<T, M>::sampleNormal(const T& mean, const T& variance) const {
      if (variance <= 0)
        throw BadArgumentException<T>(variance,
          "Randomizer<T, M>::sampleNormal(): "
          "variance must be strictly positive",
          __FILE__, __LINE__);
      double u, v, s;
      do {
        u = 2.0 * sampleUniform() - 1.0;
        v = 2.0 * sampleUniform() - 1.0;
        s = u * u + v * v;
      }
      while (s >= 1.0 || s == 0.0);
      return Traits::template round<T, true>(mean + sqrt(variance) * u *
        sqrt(-2.0 * log(s) / s));
    }

    template <typename T, int M>
    size_t Randomizer<T, M>::sampleCategorical(const
        Eigen::Matrix<double, M, 1>& probabilities) const {
      if (fabs(probabilities.sum() - 1.0) > 1e-12 ||
          (probabilities.array() < 0).any())
        throw BadArgumentException<Eigen::Matrix<double, M, 1> >(
          probabilities,
          "Randomizer<T, M>::sampleCategorical: success probabilities must sum "
          "to 1 and probabilities bigger or equal to 0",
          __FILE__, __LINE__);
      double sum = probabilities(0);
      const double u = sampleUniform();
      for (size_t i = 1; i < (size_t)probabilities.size(); ++i)
        if (u > sum)
          sum += probabilities(i);
        else
          return i - 1;
      return probabilities.size() - 1;
    }

    template <typename T, int M>
    size_t Randomizer<T, M>::samplePoisson(double mean) const {
      if (mean <= 0)
        throw BadArgumentException<double>(mean,
          "Randomizer<T, M>::samplePoisson(): mean must be strictly positive",
          __FILE__, __LINE__);
      const double l = exp(-mean);
      size_t k = 0;
      double p = 1.0;
      do {
        k++;
        p *= sampleUniform();
      }
      while (p > l);
      return k - 1;
    }

    template <typename T, int M>
    double Randomizer<T, M>::sampleExponential(double rate) const {
      if (rate <= 0)
        throw BadArgumentException<double>(rate,
          "Randomizer<T, M>::sampleExponential(): "
          "rate must be strictly positive",
          __FILE__, __LINE__);
      double u;
      do {
        u = sampleUniform();
      }
      while (u == 0);
      return -log(u) / rate;
    }

    template <typename T, int M>
    size_t Randomizer<T, M>::sampleGeometric(double probability) const {
      if (probability <= 0.0 || probability > 1.0)
        throw BadArgumentException<double>(probability,
          "Randomizer<T, M>::sampleGeometric(): success probability must be "
          "between 0 and 1",
          __FILE__, __LINE__);
      double u;
      do {
        u = sampleUniform();
      }
      while (u == 0);
      return floor(log(u) / log(1 - probability));
    }

    template <typename T, int M>
    double Randomizer<T, M>::sampleGamma(double shape, double invScale) const {
      if (shape <= 0)
        throw BadArgumentException<double>(shape,
          "Randomizer<T, M>::sampleGamma(): shape must be strictly positive",
          __FILE__, __LINE__);
      if (invScale <= 0)
        throw BadArgumentException<double>(invScale,
          "Randomizer<T, M>::sampleGamma(): inverse scale must be strictly "
          "positive",
          __FILE__, __LINE__);
      const size_t integralPart = floor(shape);
      const double fractionalPart = shape - integralPart;
      double y = 0;
      for (size_t i = 0; i < integralPart; ++i)
        y += sampleExponential(1.0);
      double b = (M_E + fractionalPart) / M_E;
      double z = 0;
      if (fabs(fractionalPart) > std::numeric_limits<double>::epsilon())
        while (true) {
          const double p = b * sampleUniform();
          if (p > 1) {
            z = -log((b - p) / fractionalPart);
            if (sampleUniform() > pow(z, fractionalPart - 1))
              continue;
            else
              break;
          }
          else {
            z = pow(p, 1.0 / fractionalPart);
            if (sampleUniform() > exp(-z))
              continue;
            else
              break;
          }
        }
      return (y + z) / invScale;
    }

    template <typename T, int M>
    template <typename Z, typename IsReal<Z>::Result::Numeric>
    Z Randomizer<T, M>::Traits::round(const Z& value) {
      return value;
    }

    template <typename T, int M>
    template <typename Z, typename IsInteger<Z>::Result::Numeric>
    Z Randomizer<T, M>::Traits::round(const double& value) {
      return (Z)::round(value);
    }

  }
}
