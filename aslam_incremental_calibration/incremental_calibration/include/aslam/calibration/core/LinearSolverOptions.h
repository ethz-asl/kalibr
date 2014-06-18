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

/** \file LinearSolverOptions.h
    \brief This file defines the LinearSolverOptions structure.
  */

#ifndef ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_OPTIONS_H
#define ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_OPTIONS_H

namespace aslam {
  namespace calibration {

    /** The structure LinearSolverOptions defines the options for the
        LinearSolver class.
        \brief Linear solver options
      */
    struct LinearSolverOptions {
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      LinearSolverOptions();
      /** @}
        */

      /** \name Members
        @{
        */
      /// Perform column scaling/normalization
      bool columnScaling;
      /// Epsilon for when to consider an element being zero in the norm
      double epsNorm;
      /// Epsilon for SVD numerical rank
      double epsSVD;
      /// Epsilon for QR tolerance computation
      double epsQR;
      /// Fixed tolerance for SVD numerical rank
      double svdTol;
      /// Fixed tolerance for QR
      double qrTol;
      /// Verbose mode
      bool verbose;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_OPTIONS_H
