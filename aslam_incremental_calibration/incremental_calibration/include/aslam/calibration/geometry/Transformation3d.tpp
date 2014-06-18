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

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    template <typename T>
    Transformation<T, 3>::Transformation() {
    }

    template <typename T>
    Transformation<T, 3>::Transformation(const Eigen::Matrix<double, 4, 4>&
        transformationMatrix) :
        mTransformationMatrix(transformationMatrix) {
    }

    template <typename T>
    Transformation<T, 3>::Transformation(T x, T y, T z, T roll, T pitch,
        T yaw) {
      setTransformation(x, y, z, roll, pitch, yaw);
    }

    template <typename T>
    Transformation<T, 3>::Transformation(const Transformation& other) :
        mTransformationMatrix(other.mTransformationMatrix),
        mRotationMatrix(other.mRotationMatrix),
        mTranslationMatrix(other.mTranslationMatrix) {
    }

    template <typename T>
    Transformation<T, 3>& Transformation<T, 3>::operator =
        (const Transformation& other) {
      if (this != &other) {
        mTransformationMatrix = other.mTransformationMatrix;
        mRotationMatrix = other.mRotationMatrix;
        mTranslationMatrix = other.mTranslationMatrix;
      }
      return *this;
    }

    template <typename T>
    Transformation<T, 3>::~Transformation() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    template <typename T>
    void Transformation<T, 3>::read(std::istream& stream) {
    }

    template <typename T>
    void Transformation<T, 3>::write(std::ostream& stream) const {
      stream << "Transformation matrix: " << std::endl << mTransformationMatrix;
    }

    template <typename T>
    void Transformation<T, 3>::read(std::ifstream& stream) {
    }

    template <typename T>
    void Transformation<T, 3>::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    template <typename T>
    void Transformation<T, 3>::setTransformationMatrix(const
        Eigen::Matrix<double, 4, 4>& transformationMatrix) {
      mTransformationMatrix = transformationMatrix;
    }

    template <typename T>
    const Eigen::Matrix<double, 4, 4>&
        Transformation<T, 3>::getTransformationMatrix() {
      return mTransformationMatrix;
    }

    template <typename T>
    void Transformation<T, 3>::setTransformation(T x, T y, T z, T roll, T pitch,
        T yaw) {
      const double sinRoll = sin(roll);
      const double cosRoll = cos(roll);
      const double sinPitch = sin(pitch);
      const double cosPitch = cos(pitch);
      const double sinYaw = sin(yaw);
      const double cosYaw = cos(yaw);
      mRotationMatrix(0, 0) = cosYaw * cosPitch;
      mRotationMatrix(0, 1) = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
      mRotationMatrix(0, 2) = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
      mRotationMatrix(0, 3) = 0;
      mRotationMatrix(1, 0) = sinYaw * cosPitch;
      mRotationMatrix(1, 1) = sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
      mRotationMatrix(1, 2) = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
      mRotationMatrix(1, 3) = 0;
      mRotationMatrix(2, 0) = -sinPitch;
      mRotationMatrix(2, 1) = cosPitch * sinRoll;
      mRotationMatrix(2, 2) = cosPitch * cosRoll;
      mRotationMatrix(2, 3) = 0;
      mRotationMatrix(3, 0) = 0;
      mRotationMatrix(3, 1) = 0;
      mRotationMatrix(3, 2) = 0;
      mRotationMatrix(3, 3) = 1;
      mTranslationMatrix(0, 0) = 1;
      mTranslationMatrix(0, 1) = 0;
      mTranslationMatrix(0, 2) = 0;
      mTranslationMatrix(0, 3) = x;
      mTranslationMatrix(1, 0) = 0;
      mTranslationMatrix(1, 1) = 1;
      mTranslationMatrix(1, 2) = 0;
      mTranslationMatrix(1, 3) = y;
      mTranslationMatrix(2, 0) = 0;
      mTranslationMatrix(2, 1) = 0;
      mTranslationMatrix(2, 2) = 1;
      mTranslationMatrix(2, 3) = z;
      mTranslationMatrix(3, 0) = 0;
      mTranslationMatrix(3, 1) = 0;
      mTranslationMatrix(3, 2) = 0;
      mTranslationMatrix(3, 3) = 1;
      mTransformationMatrix = mTranslationMatrix * mRotationMatrix;
    }

    template <typename T>
    Transformation<T, 3> Transformation<T, 3>::getInverse() const {
      return Transformation<T, 3>(*this).inverse();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    template <typename T>
    const Transformation<T, 3>& Transformation<T, 3>::inverse() {
      mRotationMatrix.transposeInPlace();
      mTranslationMatrix(0, 3) = -mTranslationMatrix(0, 3);
      mTranslationMatrix(1, 3) = -mTranslationMatrix(1, 3);
      mTranslationMatrix(2, 3) = -mTranslationMatrix(2, 3);
      mTransformationMatrix = mRotationMatrix * mTranslationMatrix;
      return *this;
    }

    template <typename T>
    void Transformation<T, 3>::transform(const Eigen::Matrix<T, 3, 1>& src,
        Eigen::Matrix<T, 3, 1>& dest) const {
      Eigen::Matrix<T, 4, 1> point(src(0), src(1), src(2), T(1));
      dest = (mTransformationMatrix * point).template head<3>();
    }

    template <typename T>
    Eigen::Matrix<T, 3, 1> Transformation<T, 3>::operator () (const
        Eigen::Matrix<T, 3, 1>& src) const {
      Eigen::Matrix<T, 4, 1> point(src(0), src(1), src(2), T(1));
      return (mTransformationMatrix * point).template head<3>();
    }

  }
}
