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

/** \file EulerAnglesZXY.h
    \brief This file defines the EulerAnglesZXY class, which implements
           the rotational kinematics based on the ZXY convention.
  */

#ifndef SM_KINEMATICS_EULER_ANGLES_ZXY_H
#define SM_KINEMATICS_EULER_ANGLES_ZXY_H

#include <Eigen/Core>

#include "sm/kinematics/RotationalKinematics.hpp"

namespace sm {
  namespace kinematics {

    /** The class EulerAnglesZXY implements a rotational kinematics based
        on the ZXY convention.
        \brief ZXY rotational kinematics
      */
    class EulerAnglesZXY :
      public RotationalKinematics
    {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      EulerAnglesZXY();
      /// Copy constructor
      EulerAnglesZXY(const EulerAnglesZXY& other);
      /// Assignment operator
      EulerAnglesZXY& operator = (const EulerAnglesZXY& other);
      /// Destructor
      virtual ~EulerAnglesZXY();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Returns the rotation matrix associated with the input parameters
      virtual Eigen::Matrix3d parametersToRotationMatrix(const Eigen::Vector3d&
        parameters, Eigen::Matrix3d * S = NULL) const;
      /// Returns the parameters associated with the input rotation matrix
      virtual Eigen::Vector3d rotationMatrixToParameters(const Eigen::Matrix3d&
        rotationMatrix) const;
      /// Returns the "S"-matrix associated to the input parameters
      virtual Eigen::Matrix3d parametersToSMatrix(const Eigen::Vector3d&
        parameters) const;
      /// Returns the angular velocity for given input parameters and derivative
      virtual Eigen::Vector3d angularVelocityAndJacobian(const Eigen::Vector3d&
        p, const Eigen::Vector3d& pdot,
        Eigen::Matrix<double, 3, 6>* Jacobian = NULL) const;
      /** @}
        */

    };

  }
}

#endif // SM_KINEMATICS_EULER_ANGLES_ZXY_H
