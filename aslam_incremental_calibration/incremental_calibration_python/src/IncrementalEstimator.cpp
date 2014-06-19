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

/** \file IncrementalEstimator.cpp
    \brief This file defines the python exports for the IncrementalEstimator
           class.
  */

#include <boost/shared_ptr.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/backend/CompressedColumnMatrix.hpp>
#include <aslam/backend/Optimizer2Options.hpp>

#include <aslam/calibration/core/LinearSolverOptions.h>
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/IncrementalOptimizationProblem.h>

using namespace boost::python;
using namespace aslam::backend;
using namespace aslam::calibration;

/// This functions gets rid of the reference
Eigen::MatrixXd getNobsBasis(const IncrementalEstimator* ie) {
  return ie->getNobsBasis();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getNobsBasisScaled(const IncrementalEstimator* ie) {
  return ie->getNobsBasis(true);
}

/// This functions gets rid of the reference
Eigen::MatrixXd getObsBasis(const IncrementalEstimator* ie) {
  return ie->getObsBasis();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getObsBasisScaled(const IncrementalEstimator* ie) {
  return ie->getObsBasis(true);
}

/// This functions gets rid of the reference
Eigen::MatrixXd getSigma2Theta(const IncrementalEstimator* ie) {
  return ie->getSigma2Theta();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getSigma2ThetaScaled(const IncrementalEstimator* ie) {
  return ie->getSigma2Theta(true);
}

/// This functions gets rid of the reference
Eigen::MatrixXd getSigma2ThetaObs(const IncrementalEstimator* ie) {
  return ie->getSigma2ThetaObs();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getSigma2ThetaObsScaled(const IncrementalEstimator* ie) {
  return ie->getSigma2ThetaObs(true);
}

/// This functions gets rid of the reference
Eigen::MatrixXd getSingularValues(const IncrementalEstimator* ie) {
  return ie->getSingularValues();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getScaledSingularValues(const IncrementalEstimator* ie) {
  return ie->getSingularValues(true);
}

void exportIncrementalEstimator() {
  /// Export options for the IncrementalEstimator class
  class_<IncrementalEstimator::Options>("IncrementalEstimatorOptions", init<>())
    .def_readwrite("infoGainDelta",
      &IncrementalEstimator::Options::infoGainDelta)
    .def_readwrite("checkValidity",
      &IncrementalEstimator::Options::checkValidity)
    .def_readwrite("verbose", &IncrementalEstimator::Options::verbose)
    ;

  /// Export return value for the IncrementalEstimator class
  class_<IncrementalEstimator::ReturnValue>("IncrementalEstimatorReturnValue",
    init<>())
    .def_readwrite("batchAccepted",
      &IncrementalEstimator::ReturnValue::batchAccepted)
    .def_readwrite("informationGain",
      &IncrementalEstimator::ReturnValue::informationGain)
    .def_readwrite("rankPsi", &IncrementalEstimator::ReturnValue::rankPsi)
    .def_readwrite("rankPsiDeficiency",
      &IncrementalEstimator::ReturnValue::rankPsiDeficiency)
    .def_readwrite("rankTheta",
      &IncrementalEstimator::ReturnValue::rankTheta)
    .def_readwrite("rankThetaDeficiency",
      &IncrementalEstimator::ReturnValue::rankThetaDeficiency)
    .def_readwrite("svdTolerance",
      &IncrementalEstimator::ReturnValue::svdTolerance)
    .def_readwrite("qrTolerance",
      &IncrementalEstimator::ReturnValue::qrTolerance)
    .def_readwrite("nobsBasis", &IncrementalEstimator::ReturnValue::nobsBasis)
    .def_readwrite("nobsBasisScaled",
      &IncrementalEstimator::ReturnValue::nobsBasisScaled)
    .def_readwrite("obsBasis",
      &IncrementalEstimator::ReturnValue::obsBasis)
    .def_readwrite("obsBasisScaled",
      &IncrementalEstimator::ReturnValue::obsBasisScaled)
    .def_readwrite("sigma2Theta",
      &IncrementalEstimator::ReturnValue::sigma2Theta)
    .def_readwrite("sigma2ThetaScaled",
      &IncrementalEstimator::ReturnValue::sigma2ThetaScaled)
    .def_readwrite("sigma2ThetaObs",
      &IncrementalEstimator::ReturnValue::sigma2ThetaObs)
    .def_readwrite("sigma2ThetaObsScaled",
      &IncrementalEstimator::ReturnValue::sigma2ThetaObsScaled)
    .def_readwrite("singularValues",
      &IncrementalEstimator::ReturnValue::singularValues)
    .def_readwrite("singularValuesScaled",
      &IncrementalEstimator::ReturnValue::singularValuesScaled)
    .def_readwrite("numIterations",
      &IncrementalEstimator::ReturnValue::numIterations)
    .def_readwrite("JStart", &IncrementalEstimator::ReturnValue::JStart)
    .def_readwrite("JFinal", &IncrementalEstimator::ReturnValue::JFinal)
    .def_readwrite("elapsedTime",
      &IncrementalEstimator::ReturnValue::elapsedTime)
    ;

  /// Functions for querying the options
  IncrementalEstimator::Options& (IncrementalEstimator::*getOptions)() = 
    &IncrementalEstimator::getOptions;
  Optimizer2Options& (IncrementalEstimator::*getOptimizerOptions)() =
    &IncrementalEstimator::getOptimizerOptions;
  LinearSolverOptions& (IncrementalEstimator::*getLinearSolverOptions)()
    = &IncrementalEstimator::getLinearSolverOptions;

  /// Removes a measurement batch from the estimator
  void (IncrementalEstimator::*removeBatch1)(size_t) =
    &IncrementalEstimator::removeBatch;
  void (IncrementalEstimator::*removeBatch2)(
    const IncrementalEstimator::BatchSP&) = &IncrementalEstimator::removeBatch;

  /// Export IncrementalEstimator class
  class_<IncrementalEstimator, boost::shared_ptr<IncrementalEstimator>,
    boost::noncopyable>("IncrementalEstimator", init<size_t,
    const IncrementalEstimator::Options&, const LinearSolverOptions&,
    const Optimizer2Options&>("IncrementalEstimator(groupId, Options, "
    "LinearSolverOptions, OptimizerOptions) -- The group id should identify "
    "the calibration parameters"))
    .def(init<size_t>("IncrementalEstimator(groupId) -- The group id should "
      "identify the calibration parameters"))
    .def("getOptions", getOptions, return_internal_reference<>())
    .def("getOptimizerOptions", getOptimizerOptions,
      return_internal_reference<>())
    .def("getLinearSolverOptions", getLinearSolverOptions,
      return_internal_reference<>())
    .def("addBatch", &IncrementalEstimator::addBatch)
    .def("reoptimize", &IncrementalEstimator::reoptimize)
    .def("getNumBatches", &IncrementalEstimator::getNumBatches)
    .def("removeBatch", removeBatch1)
    .def("removeBatch", removeBatch2)
    .def("getMargGroupId", &IncrementalEstimator::getMargGroupId)
    .def("getInformationGain", &IncrementalEstimator::getInformationGain)
    .def("getJacobianTranspose", &IncrementalEstimator::getJacobianTranspose,
      return_internal_reference<>())
    .def("getRankPsi", &IncrementalEstimator::getRankPsi)
    .def("getRankPsiDeficiency", &IncrementalEstimator::getRankPsiDeficiency)
    .def("getRankTheta", &IncrementalEstimator::getRankTheta)
    .def("getRankThetaDeficiency",
      &IncrementalEstimator::getRankThetaDeficiency)
    .def("getSVDTolerance", &IncrementalEstimator::getSVDTolerance)
    .def("getQRTolerance", &IncrementalEstimator::getQRTolerance)
    .def("getPeakMemoryUsage", &IncrementalEstimator::getPeakMemoryUsage)
    .def("getMemoryUsage", &IncrementalEstimator::getMemoryUsage)
    .def("getNumFlops", &IncrementalEstimator::getNumFlops)
    .def("getNobsBasis", &getNobsBasis)
    .def("getNobsBasisScaled", &getNobsBasisScaled)
    .def("getObsBasis", &getObsBasis)
    .def("getObsBasisScaled", &getObsBasisScaled)
    .def("getSigma2Theta", &getSigma2Theta)
    .def("getSigma2ThetaScaled", &getSigma2ThetaScaled)
    .def("getSigma2ThetaObs", &getSigma2ThetaObs)
    .def("getSigma2ThetaObsScaled", &getSigma2ThetaObsScaled)
    .def("getSingularValues", &getSingularValues)
    .def("getScaledSingularValues", &getScaledSingularValues)
    .def("getProblem", &IncrementalEstimator::getProblem,
      boost::python::return_internal_reference<>())
    ;
}
