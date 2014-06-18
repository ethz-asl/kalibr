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

/** \file LinearSolver.cpp
    \brief This file defines the python exports for the LinearSolver class.
  */

#include <boost/shared_ptr.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/LinearSystemSolver.hpp>

#include <aslam/calibration/core/LinearSolverOptions.h>
#include <aslam/calibration/core/LinearSolver.h>

using namespace boost::python;
using namespace aslam::calibration;
using namespace aslam::backend;
using namespace sm;

void exportLinearSolver() {
  /// Export LinearSolverOptions structure
  class_<LinearSolverOptions>("LinearSolverOptions", init<>())
    .def_readwrite("columnScaling", &LinearSolverOptions::columnScaling)
    .def_readwrite("epsNorm", &LinearSolverOptions::epsNorm)
    .def_readwrite("epsSVD", &LinearSolverOptions::epsSVD)
    .def_readwrite("epsQR", &LinearSolverOptions::epsQR)
    .def_readwrite("svdTol", &LinearSolverOptions::svdTol)
    .def_readwrite("qrTol", &LinearSolverOptions::qrTol)
    .def_readwrite("verbose", &LinearSolverOptions::verbose)
    ;

  /// Function for querying the options
  LinearSolverOptions& (LinearSolver::*getOptions)() =
    &LinearSolver::getOptions;

  /// Export LinearSolver class
  class_<LinearSolver, boost::shared_ptr<LinearSolver>,
    bases<LinearSystemSolver>, boost::noncopyable>("LinearSolver",
    init<const LinearSolverOptions&>())
    .def(init<const PropertyTree&>())
    .def("getOptions", getOptions, return_internal_reference<>())
    ;
}
