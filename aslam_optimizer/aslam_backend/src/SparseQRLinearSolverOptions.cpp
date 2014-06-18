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

#include "aslam/backend/SparseQRLinearSolverOptions.h"

#include <SuiteSparseQR.hpp>

namespace aslam {
  namespace backend {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    SparseQRLinearSolverOptions::SparseQRLinearSolverOptions() :
        colNorm(false),
        qrTol(SPQR_DEFAULT_TOL),
        normTol(1e-8),
        verbose(false) {
    }

    SparseQRLinearSolverOptions::SparseQRLinearSolverOptions(
        const SparseQRLinearSolverOptions& other) :
        colNorm(other.colNorm),
        qrTol(other.qrTol),
        normTol(other.normTol),
        verbose(other.verbose) {
    }

    SparseQRLinearSolverOptions& SparseQRLinearSolverOptions::operator =
        (const SparseQRLinearSolverOptions& other) {
      if (this != &other) {
        colNorm = other.colNorm;
        qrTol = other.qrTol;
        normTol = other.normTol;
        verbose = other.verbose;
      }
      return *this;
    }

    SparseQRLinearSolverOptions::~SparseQRLinearSolverOptions() {
    }

  }
}
