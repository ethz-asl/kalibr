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

#include "sm/kinematics/EulerAnglesZXY.h"

#include <sm/assert_macros.hpp>

namespace sm {
  namespace kinematics {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    EulerAnglesZXY::EulerAnglesZXY() {
    }

    EulerAnglesZXY::EulerAnglesZXY(const EulerAnglesZXY& other) :
        RotationalKinematics(other) {
    }

    EulerAnglesZXY& EulerAnglesZXY::operator = (const EulerAnglesZXY& other) {
      if (this != &other) {
        RotationalKinematics::operator=(other);
      }
      return *this;
    }

    EulerAnglesZXY::~EulerAnglesZXY() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    Eigen::Matrix3d EulerAnglesZXY::parametersToRotationMatrix(
        const Eigen::Vector3d& parameters, Eigen::Matrix3d * S ) const {
      if (S)
        *S = parametersToSMatrix(parameters);
      const double cx = cos(parameters(1));
      const double sx = sin(parameters(1));
      const double cy = cos(parameters(2));
      const double sy = sin(parameters(2));
      const double cz = cos(parameters(0));
      const double sz = sin(parameters(0));
      return (Eigen::Matrix3d() <<
        cz * cy - sz * sx * sy, -sz * cx, cz * sy + sz * sx * cy,
        sz * cy + cz * sx * sy, cz * cx, sz * sy - cz * sx * cy,
        -cx * sy, sx, cx * cy).finished();
    }

    Eigen::Vector3d EulerAnglesZXY::rotationMatrixToParameters(
        const Eigen::Matrix3d& rotationMatrix) const {
      const double sx = rotationMatrix(2, 1);
      const double r_x = asin(sx);
      const double r_y = atan2(-rotationMatrix(2, 0), rotationMatrix(2, 2));
      const double r_z = atan2(-rotationMatrix(0, 1), rotationMatrix(1, 1));
      return Eigen::Vector3d(r_z, r_x, r_y);
    }

    Eigen::Matrix3d EulerAnglesZXY::parametersToSMatrix(
        const Eigen::Vector3d& parameters) const {
      const double cx = cos(parameters(1));
      const double sx = sin(parameters(1));
      const double cz = cos(parameters(0));
      const double sz = sin(parameters(0));
      return (Eigen::Matrix3d() <<
        0, -cz, cx * sz,
        0, -sz, -cx * cz,
        -1, 0, -sx).finished();
    }

    Eigen::Vector3d EulerAnglesZXY::angularVelocityAndJacobian(
        const Eigen::Vector3d& p, const Eigen::Vector3d& pdot,
        Eigen::Matrix<double, 3, 6>* Jacobian) const {
      const double cx = cos(p(1));
      const double sx = sin(p(1));
      const double cz = cos(p(0));
      const double sz = sin(p(0));
      Eigen::Matrix3d S = parametersToSMatrix(p);
      if (Jacobian) {
        *Jacobian = Eigen::Matrix<double, 3, 6>::Zero();
        (*Jacobian)(0, 0) = sz * pdot[1] + cx * cz * pdot[2];
        (*Jacobian)(0, 1) = -sx * sz * pdot[2];
        (*Jacobian)(1, 0) = -cz * pdot[1] + cx * sz * pdot[2];
        (*Jacobian)(1, 1) = sx * cz * pdot[2];
        (*Jacobian)(2, 1) = -cx * pdot[2];
        Jacobian->topRightCorner<3, 3>() = S;
      }
      return S * pdot;
    }

  }
}
