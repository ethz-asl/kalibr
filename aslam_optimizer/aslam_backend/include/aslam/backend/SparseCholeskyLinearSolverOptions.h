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

/** \file SparseCholeskyLinearSolverOptions.h
    \brief This file defines the SparseCholeskyLinearSolverOptions class which
           contains specific options for the sparse Cholesky linear solver.
  */

#ifndef ASLAM_BACKEND_SPARSE_CHOLESKY_LINEAR_SOLVER_OPTIONS_H
#define ASLAM_BACKEND_SPARSE_CHOLESKY_LINEAR_SOLVER_OPTIONS_H

namespace aslam {
  namespace backend {

    /** The class SparseCholeskyLinearSolverOptions contains specific options
        for the sparse Cholesky linear solver.
        \brief Sparse Cholesky linear solver options
      */
    class SparseCholeskyLinearSolverOptions {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      SparseCholeskyLinearSolverOptions();
      /// Copy constructor
      SparseCholeskyLinearSolverOptions(const SparseCholeskyLinearSolverOptions&
        other);
      /// Assignment operator
      SparseCholeskyLinearSolverOptions& operator =
        (const SparseCholeskyLinearSolverOptions& other);
      /// Destructor
      virtual ~SparseCholeskyLinearSolverOptions();
      /** @}
        */

    };

  }
}

#endif // ASLAM_BACKEND_SPARSE_CHOLESKY_LINEAR_SOLVER_OPTIONS_H
