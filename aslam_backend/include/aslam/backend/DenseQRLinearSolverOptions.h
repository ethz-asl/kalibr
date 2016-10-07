/******************************************************************************
 * Copyright (C) 2012 by Jerome Maye                                          *
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

/** \file DenseQRLinearSolverOptions.h
    \brief This file defines the DenseQRLinearSolverOptions class which
           contains specific options for the dense QR linear solver.
  */

#ifndef ASLAM_BACKEND_DENSE_QR_LINEAR_SOLVER_OPTIONS_H
#define ASLAM_BACKEND_DENSE_QR_LINEAR_SOLVER_OPTIONS_H

namespace aslam {
  namespace backend {

    /** The class DenseQRLinearSolverOptions contains specific options for the
        dense QR linear solver.
        \brief Dense QR linear solver options
      */
    class DenseQRLinearSolverOptions {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      DenseQRLinearSolverOptions();
      /// Copy constructor
      DenseQRLinearSolverOptions(const DenseQRLinearSolverOptions& other);
      /// Assignment operator
      DenseQRLinearSolverOptions& operator =
        (const DenseQRLinearSolverOptions& other);
      /// Destructor
      virtual ~DenseQRLinearSolverOptions();
      /** @}
        */

    };

  }
}

#endif // ASLAM_BACKEND_DENSE_QR_LINEAR_SOLVER_OPTIONS_H
