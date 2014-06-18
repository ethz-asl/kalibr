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

/** \file BlockCholeskyLinearSolverOptions.h
    \brief This file defines the BlockCholeskyLinearSolverOptions class which
           contains specific options for the block Cholesky linear solver.
  */

#ifndef ASLAM_BACKEND_BLOCK_CHOLESKY_LINEAR_SOLVER_OPTIONS_H
#define ASLAM_BACKEND_BLOCK_CHOLESKY_LINEAR_SOLVER_OPTIONS_H

namespace aslam {
  namespace backend {

    /** The class BlockCholeskyLinearSolverOptions contains specific options for
        the block Cholesky linear solver.
        \brief Block Cholesky linear solver options
      */
    class BlockCholeskyLinearSolverOptions {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      BlockCholeskyLinearSolverOptions();
      /// Copy constructor
      BlockCholeskyLinearSolverOptions(const BlockCholeskyLinearSolverOptions&
        other);
      /// Assignment operator
      BlockCholeskyLinearSolverOptions& operator =
        (const BlockCholeskyLinearSolverOptions& other);
      /// Destructor
      virtual ~BlockCholeskyLinearSolverOptions();
      /** @}
        */

    };

  }
}

#endif // ASLAM_BACKEND_BLOCK_CHOLESKY_LINEAR_SOLVER_OPTIONS_H
