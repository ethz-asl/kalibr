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

/** \file Transformation3d.h
    \brief This file defines a transformation in 3d.
  */

#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The Transformation3d class represents a 3d transformation.
        \brief 3d transformation
      */
    template <typename T> class Transformation<T, 3> :
      public virtual Serializable {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      Transformation();
      /// Constructs from a given transformation matrix
      Transformation(const Eigen::Matrix<double, 4, 4>& transformationMatrix);
      /// Constructs from rotation and translation
      Transformation(T x, T y, T z, T roll, T pitch, T yaw);
      /// Copy constructor
      Transformation(const Transformation& other);
      /// Assignment operator
      Transformation& operator = (const Transformation& other);
      /// Destructor
      virtual ~Transformation();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Sets the transformation matrix
      void setTransformationMatrix(const Eigen::Matrix<double, 4, 4>&
        transformationMatrix);
      /// Returns the transformation matrix
      const Eigen::Matrix<double, 4, 4>& getTransformationMatrix();
      /// Sets the transformation from translation and rotation
      void setTransformation(T x, T y, T z, T roll, T pitch, T yaw);
      /// Returns the inverse transformation
      Transformation getInverse() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Inverse the transformation
      const Transformation& inverse();
      /// Transform a point
      void transform(const Eigen::Matrix<T, 3, 1>& src, Eigen::Matrix<T, 3, 1>&
        dest) const;
      /// Transform a point using operator
      Eigen::Matrix<T, 3, 1> operator () (const Eigen::Matrix<T, 3, 1>& src)
        const;
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
      /// Transformation matrix
      Eigen::Matrix<double, 4, 4> mTransformationMatrix;
      /// Rotation matrix
      Eigen::Matrix<double, 4, 4> mRotationMatrix;
      /// Translation matrix
      Eigen::Matrix<double, 4, 4> mTranslationMatrix;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/geometry/Transformation3d.tpp"
