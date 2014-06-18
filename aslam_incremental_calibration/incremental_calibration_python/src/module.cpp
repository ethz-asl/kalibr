/******************************************************************************
 * Copyright (C) 2013 by Paul Furgale and Jerome Maye                         *
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

/** \file module.cpp
    \brief This file defines the top-level python exports.
  */

#include <numpy_eigen/boost_python_headers.hpp>

//void exportVisionDataAssociation();
void exportOptimizationProblem();
void exportIncrementalEstimator();
void exportLinearSolver();

BOOST_PYTHON_MODULE(libincremental_calibration_python) {
  exportOptimizationProblem();
  exportIncrementalEstimator();
  exportLinearSolver();
}
