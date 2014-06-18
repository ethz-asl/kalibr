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

#include <cmath>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename T>
    Transformation<T, 2>::Transformation() {
    }

    template <typename T>
    Transformation<T, 2>::Transformation(const Eigen::Matrix<double, 3, 3>&
        transformationMatrix) :
        mTransformationMatrix(transformationMatrix) {
    }

    template <typename T>
    Transformation<T, 2>::Transformation(T x, T y, T yaw) {
      setTransformation(x, y, yaw);
    }

    template <typename T>
    Transformation<T, 2>::Transformation(const Transformation& other) :
        mTransformationMatrix(other.mTransformationMatrix),
        mRotationMatrix(other.mRotationMatrix),
        mTranslationMatrix(other.mTranslationMatrix) {
    }

    template <typename T>
    Transformation<T, 2>& Transformation<T, 2>::operator =
        (const Transformation& other) {
      if (this != &other) {
        mTransformationMatrix = other.mTransformationMatrix;
        mRotationMatrix = other.mRotationMatrix;
        mTranslationMatrix = other.mTranslationMatrix;
      }
      return *this;
    }

    template <typename T>
    Transformation<T, 2>::~Transformation() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <typename T>
    void Transformation<T, 2>::read(std::istream& stream) {
    }

    template <typename T>
    void Transformation<T, 2>::write(std::ostream& stream) const {
      stream << "Transformation matrix: " << std::endl << mTransformationMatrix;
    }

    template <typename T>
    void Transformation<T, 2>::read(std::ifstream& stream) {
    }

    template <typename T>
    void Transformation<T, 2>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename T>
    void Transformation<T, 2>::setTransformationMatrix(const
        Eigen::Matrix<double, 3, 3>& transformationMatrix) {
      mTransformationMatrix = transformationMatrix;
    }

    template <typename T>
    const Eigen::Matrix<double, 3, 3>&
        Transformation<T, 2>::getTransformationMatrix() {
      return mTransformationMatrix;
    }

    template <typename T>
    void Transformation<T, 2>::setTransformation(T x, T y, T yaw) {
      const double cosYaw = cos(yaw);
      const double sinYaw = sin(yaw);
      mRotationMatrix(0, 0) = cosYaw;
      mRotationMatrix(0, 1) = -sinYaw;
      mRotationMatrix(0, 2) = 0;
      mRotationMatrix(1, 0) = sinYaw;
      mRotationMatrix(1, 1) = cosYaw;
      mRotationMatrix(1, 2) = 0;
      mRotationMatrix(2, 0) = 0;
      mRotationMatrix(2, 1) = 0;
      mRotationMatrix(2, 2) = 1;
      mTranslationMatrix(0, 0) = 1;
      mTranslationMatrix(0, 1) = 0;
      mTranslationMatrix(0, 2) = x;
      mTranslationMatrix(1, 0) = 0;
      mTranslationMatrix(1, 1) = 1;
      mTranslationMatrix(1, 2) = y;
      mTranslationMatrix(2, 0) = 0;
      mTranslationMatrix(2, 1) = 0;
      mTranslationMatrix(2, 2) = 1;
      mTransformationMatrix = mTranslationMatrix * mRotationMatrix;
    }

    template <typename T>
    Transformation<T, 2> Transformation<T, 2>::getInverse() const {
      return Transformation<T, 2>(*this).inverse();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <typename T>
    void Transformation<T, 2>::transform(const Eigen::Matrix<T, 2, 1>& src,
        Eigen::Matrix<T, 2, 1>& dest) const {
      Eigen::Matrix<T, 3, 1> point(src(0), src(1), T(1));
      dest = (mTransformationMatrix * point).template head<2>();
    }

    template <typename T>
    Eigen::Matrix<T, 2, 1> Transformation<T, 2>::operator () (const
        Eigen::Matrix<T, 2, 1>& src) const {
      Eigen::Matrix<T, 3, 1> point(src(0), src(1), T(1));
      return (mTransformationMatrix * point).template head<2>();
    }

    template <typename T>
    const Transformation<T, 2>& Transformation<T, 2>::inverse() {
      mRotationMatrix.transposeInPlace();
      mTranslationMatrix(0, 2) = -mTranslationMatrix(0, 2);
      mTranslationMatrix(1, 2) = -mTranslationMatrix(1, 2);
      mTransformationMatrix = mRotationMatrix * mTranslationMatrix;
      return *this;
    }

  }
}
