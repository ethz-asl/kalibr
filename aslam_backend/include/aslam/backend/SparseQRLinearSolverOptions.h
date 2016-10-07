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

/** \file SparseQRLinearSolverOptions.h
    \brief This file defines the SparseQRLinearSolverOptions class which
           contains specific options for the sparse QR linear solver.
  */

#ifndef ASLAM_BACKEND_SPARSE_QR_LINEAR_SOLVER_OPTIONS_H
#define ASLAM_BACKEND_SPARSE_QR_LINEAR_SOLVER_OPTIONS_H

namespace aslam {
  namespace backend {

    /** The class SparseQRLinearSolverOptions contains specific options for the
        sparse QR linear solver.
        \brief Sparse QR linear solver options
      */
    class SparseQRLinearSolverOptions {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      SparseQRLinearSolverOptions();
      /// Copy constructor
      SparseQRLinearSolverOptions(const SparseQRLinearSolverOptions& other);
      /// Assignment operator
      SparseQRLinearSolverOptions& operator =
        (const SparseQRLinearSolverOptions& other);
      /// Destructor
      virtual ~SparseQRLinearSolverOptions();
      /** @}
        */

      /** \name Members
        @{
        */
      /// Column normalization
      bool colNorm;
      /// Tolerance for QR
      double qrTol;
      /// Tolerance for a zero 2-norm column
      double normTol;
      /// Verbose mode
      bool verbose;
      /** @}
        */

    };

  }
}

#endif // ASLAM_BACKEND_SPARSE_QR_LINEAR_SOLVER_OPTIONS_H
