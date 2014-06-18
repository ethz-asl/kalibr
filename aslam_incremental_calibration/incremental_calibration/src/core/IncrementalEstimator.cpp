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

#include "aslam/calibration/core/IncrementalEstimator.h"

#include <algorithm>
#include <utility>
#include <vector>
#include <ostream>

#include <boost/make_shared.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>

#include "aslam/calibration/core/LinearSolver.h"
#include "aslam/calibration/core/IncrementalOptimizationProblem.h"
#include "aslam/calibration/base/Timestamp.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalEstimator::IncrementalEstimator(size_t groupId,
        const Options& options, const LinearSolverOptions&
        linearSolverOptions, const OptimizerOptions& optimizerOptions) :
        _options(options),
        _margGroupId(groupId),
        _optimizer(boost::make_shared<Optimizer>(optimizerOptions)),
        _problem(boost::make_shared<IncrementalOptimizationProblem>()),
        _informationGain(0.0),
        _svLog2Sum(0.0),
        _svdTolerance(0.0),
        _qrTolerance(-1.0),
        _rankTheta(-1),
        _rankThetaDeficiency(-1),
        _rankPsi(-1),
        _rankPsiDeficiency(-1),
        _peakMemoryUsage(0),
        _memoryUsage(0),
        _numFlops(0.0),
        _initialCost(0.0),
        _finalCost(0.0) {
      // create linear solver and trust region policy for the optimizer
      OptimizerOptions& optOptions = _optimizer->options();
      optOptions.linearSystemSolver =
        boost::make_shared<LinearSolver>(linearSolverOptions);
      optOptions.trustRegionPolicy = boost::make_shared<TrustRegionPolicy>();
      _optimizer->initializeLinearSolver();
      _optimizer->initializeTrustRegionPolicy();

      // attach the problem to the optimizer
      _optimizer->setProblem(_problem);
    }

    IncrementalEstimator::IncrementalEstimator(const sm::PropertyTree& config) :
        _informationGain(0.0),
        _svLog2Sum(0.0),
        _svdTolerance(0.0),
        _qrTolerance(-1.0),
        _rankTheta(-1),
        _rankThetaDeficiency(-1),
        _rankPsi(-1),
        _rankPsiDeficiency(-1),
        _peakMemoryUsage(0),
        _memoryUsage(0),
        _numFlops(0.0),
        _initialCost(0.0),
        _finalCost(0.0) {
      // create the optimizer, linear solver, and trust region policy
      _optimizer = boost::make_shared<Optimizer>(
        sm::PropertyTree(config, "optimizer"),
        boost::make_shared<LinearSolver>(
        sm::PropertyTree(config, "optimizer/linearSolver")),
        boost::make_shared<TrustRegionPolicy>());

      // create the problem and attach it to the optimizer
      _problem = boost::make_shared<IncrementalOptimizationProblem>();
      _optimizer->setProblem(_problem);

      // parse the options and set them
      _options.infoGainDelta = config.getDouble("infoGainDelta",
        _options.infoGainDelta);
      _options.checkValidity = config.getBool("checkValidity",
        _options.checkValidity);
      _options.verbose = config.getBool("verbose", _options.verbose);
      _margGroupId = config.getInt("groupId");
    }

    IncrementalEstimator::~IncrementalEstimator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const IncrementalOptimizationProblem*
        IncrementalEstimator::getProblem() const {
      return _problem.get();
    }

    const IncrementalEstimator::Options&
        IncrementalEstimator::getOptions() const {
      return _options;
    }

    IncrementalEstimator::Options& IncrementalEstimator::getOptions() {
      return _options;
    }

    const LinearSolverOptions& IncrementalEstimator::getLinearSolverOptions()
        const {
      return _optimizer->getSolver<LinearSolver>()->getOptions();
    }

    LinearSolverOptions& IncrementalEstimator::getLinearSolverOptions() {
      return _optimizer->getSolver<LinearSolver>()->getOptions();
    }

    const IncrementalEstimator::OptimizerOptions&
        IncrementalEstimator::getOptimizerOptions() const {
      return _optimizer->options();
    }

    IncrementalEstimator::OptimizerOptions&
        IncrementalEstimator::getOptimizerOptions() {
      return _optimizer->options();
    }

    size_t IncrementalEstimator::getMargGroupId() const {
      return _margGroupId;
    }

    double IncrementalEstimator::getInformationGain() const {
      return _informationGain;
    }

    const aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&
        IncrementalEstimator::getJacobianTranspose() const {
      return _optimizer->getSolver<LinearSolver>()->getJacobianTranspose();
    }

    std::ptrdiff_t IncrementalEstimator::getRankPsi() const {
      return _rankPsi;
    }

    std::ptrdiff_t IncrementalEstimator::getRankPsiDeficiency() const {
      return _rankPsiDeficiency;
    }

    std::ptrdiff_t IncrementalEstimator::getRankTheta() const {
      return _rankTheta;
    }

    std::ptrdiff_t IncrementalEstimator::getRankThetaDeficiency() const {
      return _rankThetaDeficiency;
    }

    double IncrementalEstimator::getSVDTolerance() const {
      return _svdTolerance;
    }

    double IncrementalEstimator::getQRTolerance() const {
      return _qrTolerance;
    }

    size_t IncrementalEstimator::getPeakMemoryUsage() const {
      return _peakMemoryUsage;
    }

    size_t IncrementalEstimator::getMemoryUsage() const {
      return _memoryUsage;
    }

    double IncrementalEstimator::getNumFlops() const {
      return _numFlops;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getNobsBasis(bool scaled)
        const {
      if (scaled)
        return _nobsBasisScaled;
      else
        return _nobsBasis;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getObsBasis(bool scaled)
        const {
      if (scaled)
        return _obsBasisScaled;
      else
        return _obsBasis;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getSigma2Theta(bool scaled)
        const {
      if (scaled)
        return _sigma2ThetaScaled;
      else
        return _sigma2Theta;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getSigma2ThetaObs(bool scaled)
        const {
      if (scaled)
        return _sigma2ThetaObsScaled;
      else
        return _sigma2ThetaObs;
    }

    const Eigen::VectorXd& IncrementalEstimator::getSingularValues(bool scaled)
        const {
      if (scaled)
        return _singularValuesScaled;
      else
        return _singularValues;
    }

    double IncrementalEstimator::getInitialCost() const {
      return _initialCost;
    }

    double IncrementalEstimator::getFinalCost() const {
      return _finalCost;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    IncrementalEstimator::ReturnValue IncrementalEstimator::reoptimize() {
      // query the time
      const double timeStart = Timestamp::now();

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // set the marginalization index of the linear solver
      size_t JCols = 0;
      for (auto it = _problem->getGroupsOrdering().cbegin();
          it != _problem->getGroupsOrdering().cend(); ++it)
        JCols += _problem->getGroupDim(*it);
      const size_t dim = _problem->getGroupDim(_margGroupId);
      auto linearSolver = _optimizer->getSolver<LinearSolver>();
      linearSolver->setMargStartIndex(static_cast<std::ptrdiff_t>(JCols - dim));

      // optimize
      aslam::backend::SolutionReturnValue srv = _optimizer->optimize();

      // grep the scaled linear system informations
      if (linearSolver->getOptions().columnScaling) {
        _singularValuesScaled = linearSolver->getSingularValues();
        _nobsBasisScaled = linearSolver->getNullSpace();
        _obsBasisScaled = linearSolver->getRowSpace();
        _sigma2ThetaScaled = linearSolver->getCovariance();
        _sigma2ThetaObsScaled = linearSolver->getRowSpaceCovariance();
      }
      else {
        _singularValuesScaled.resize(0);
        _nobsBasisScaled.resize(0, 0);
        _obsBasisScaled.resize(0, 0);
        _sigma2ThetaScaled.resize(0, 0);
        _sigma2ThetaObsScaled.resize(0, 0);
      }

      // analyze the unscaled marginal system
      linearSolver->analyzeMarginal();

      // retrieve informations from the linear solver
      _informationGain = 0.0;
      _svLog2Sum = linearSolver->getSingularValuesLog2Sum();
      _nobsBasis = linearSolver->getNullSpace();
      _obsBasis = linearSolver->getRowSpace();
      _sigma2Theta = linearSolver->getCovariance();
      _sigma2ThetaObs = linearSolver->getRowSpaceCovariance();
      _singularValues = linearSolver->getSingularValues();
      _svdTolerance = linearSolver->getSVDTolerance();
      _qrTolerance = linearSolver->getQRTolerance();
      _rankTheta = linearSolver->getSVDRank();
      _rankThetaDeficiency = linearSolver->getSVDRankDeficiency();
      _rankPsi = linearSolver->getQRRank();
      _rankPsiDeficiency = linearSolver->getQRRankDeficiency();
      _peakMemoryUsage = linearSolver->getPeakMemoryUsage();
      _memoryUsage = linearSolver->getMemoryUsage();
      _numFlops = linearSolver->getNumFlops();
      _initialCost = srv.JStart;
      _finalCost = srv.JFinal;

      // update output structure
      ReturnValue ret;
      ret.batchAccepted = true;
      ret.informationGain = 0.0;
      ret.rankPsi = _rankPsi;
      ret.rankPsiDeficiency = _rankPsiDeficiency;
      ret.rankTheta = _rankTheta;
      ret.rankThetaDeficiency = _rankThetaDeficiency;
      ret.svdTolerance = _svdTolerance;
      ret.qrTolerance = _qrTolerance;
      ret.nobsBasis = _nobsBasis;
      ret.nobsBasisScaled = _nobsBasisScaled;
      ret.obsBasis = _obsBasis;
      ret.obsBasisScaled = _obsBasisScaled;
      ret.sigma2Theta = _sigma2Theta;
      ret.sigma2ThetaScaled = _sigma2ThetaScaled;
      ret.sigma2ThetaObs = _sigma2ThetaObs;
      ret.sigma2ThetaObsScaled = _sigma2ThetaObsScaled;
      ret.singularValues = _singularValues;
      ret.singularValuesScaled = _singularValuesScaled;
      ret.numIterations = srv.iterations;
      ret.JStart = _initialCost;
      ret.JFinal = _finalCost;
      ret.elapsedTime = Timestamp::now() - timeStart;
      return ret;
    }

    IncrementalEstimator::ReturnValue
        IncrementalEstimator::addBatch(const BatchSP& problem, bool force) {
      // query the time
      const double timeStart = Timestamp::now();

      // insert new batch in the problem
      _problem->add(problem);

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // save design variables in case the batch is rejected
      if (!force)
        _problem->saveDesignVariables();

      // set the marginalization index of the linear solver
      size_t JCols = 0;
      for (auto it = _problem->getGroupsOrdering().cbegin();
          it != _problem->getGroupsOrdering().cend(); ++it)
        JCols += _problem->getGroupDim(*it);
      const size_t dim = _problem->getGroupDim(_margGroupId);
      auto linearSolver = _optimizer->getSolver<LinearSolver>();
      linearSolver->setMargStartIndex(static_cast<std::ptrdiff_t>(JCols - dim));

      // optimize
      aslam::backend::SolutionReturnValue srv = _optimizer->optimize();

      // return value
      ReturnValue ret;

      // fill statistics from optimizer
      ret.numIterations = srv.iterations;
      ret.JStart = srv.JStart;
      ret.JFinal = srv.JFinal;

      // grep the scaled singular values if scaling enabled
      if (linearSolver->getOptions().columnScaling) {
        ret.singularValuesScaled = linearSolver->getSingularValues();
        ret.nobsBasisScaled = linearSolver->getNullSpace();
        ret.obsBasisScaled = linearSolver->getRowSpace();
        ret.sigma2ThetaScaled = linearSolver->getCovariance();
        ret.sigma2ThetaObsScaled = linearSolver->getRowSpaceCovariance();
      }
      else {
        ret.singularValuesScaled.resize(0);
        ret.nobsBasisScaled.resize(0, 0);
        ret.obsBasisScaled.resize(0, 0);
        ret.sigma2ThetaScaled.resize(0, 0);
        ret.sigma2ThetaObsScaled.resize(0, 0);
      }

      // analyze marginal system (unscaled system)
      linearSolver->analyzeMarginal();

      // fill statistics from the linear solver
      ret.rankPsi = linearSolver->getQRRank();
      ret.rankPsiDeficiency = linearSolver->getQRRankDeficiency();
      ret.rankTheta = linearSolver->getSVDRank();
      ret.rankThetaDeficiency = linearSolver->getSVDRankDeficiency();
      ret.svdTolerance = linearSolver->getSVDTolerance();
      ret.qrTolerance = linearSolver->getQRTolerance();
      ret.nobsBasis = linearSolver->getNullSpace();
      ret.obsBasis = linearSolver->getRowSpace();
      ret.sigma2Theta = linearSolver->getCovariance();
      ret.sigma2ThetaObs = linearSolver->getRowSpaceCovariance();
      ret.singularValues = linearSolver->getSingularValues();

      // check if the solution is valid
      bool solutionValid = true;
      if (_options.checkValidity && (srv.iterations ==
          _optimizer->options().maxIterations || srv.JFinal >= srv.JStart))
        solutionValid = false;

      // compute the information gain
      const double svLog2Sum = linearSolver->getSingularValuesLog2Sum();
      ret.informationGain = 0.5 * (svLog2Sum - _svLog2Sum);

      // batch is kept? information gain improvement or rank goes up or force
      bool keepBatch = false;
      if (((ret.informationGain > _options.infoGainDelta ||
          ret.rankTheta > _rankTheta) && solutionValid) || force) {
        // warning for rank going down
        if (ret.rankTheta < _rankTheta && _options.verbose)
          std::cerr << "IncrementalEstimator::addBatch(): "
            "WARNING: RANK GOING DOWN!" << std::endl;

        keepBatch = true;

        // update internal variables
        _informationGain = ret.informationGain;
        _svLog2Sum = svLog2Sum;
        _nobsBasis = ret.nobsBasis;
        _nobsBasisScaled = ret.nobsBasisScaled;
        _obsBasis = ret.obsBasis;
        _obsBasisScaled = ret.obsBasisScaled;
        _sigma2Theta = ret.sigma2Theta;
        _sigma2ThetaScaled = ret.sigma2ThetaScaled;
        _sigma2ThetaObs = ret.sigma2ThetaObs;
        _sigma2ThetaObsScaled = ret.sigma2ThetaObsScaled;
        _singularValues = ret.singularValues;
        _singularValuesScaled = ret.singularValuesScaled;
        _svdTolerance = ret.svdTolerance;
        _qrTolerance = ret.qrTolerance;
        _rankTheta = ret.rankTheta;
        _rankThetaDeficiency = ret.rankThetaDeficiency;
        _rankPsi = ret.rankPsi;
        _rankPsiDeficiency = ret.rankPsiDeficiency;
        _peakMemoryUsage = linearSolver->getPeakMemoryUsage();
        _memoryUsage = linearSolver->getMemoryUsage();
        _numFlops = linearSolver->getNumFlops();
        _initialCost = srv.JStart;
        _finalCost = srv.JFinal;
      }
      ret.batchAccepted = keepBatch;

      // remove batch if necessary
      if (!keepBatch) {
        // restore variables
        _problem->restoreDesignVariables();

        // kick out the problem from the container
        _problem->remove(problem);

        // restore the linear solver
        if (_problem->getNumOptimizationProblems() > 0)
          restoreLinearSolver();
      }

      // insert elapsed time
      ret.elapsedTime = Timestamp::now() - timeStart;

      // output informations
      return ret;
    }

    void IncrementalEstimator::removeBatch(size_t idx) {
      // remove the batch
      _problem->remove(idx);

      // reoptimize
      reoptimize();
    }

    void IncrementalEstimator::removeBatch(const BatchSP& batch) {
      auto it = _problem->getOptimizationProblem(batch);
      if (it != _problem->getOptimizationProblemEnd())
        removeBatch(std::distance(_problem->getOptimizationProblemBegin(), it));
    }

    size_t IncrementalEstimator::getNumBatches() const {
      return _problem->getNumOptimizationProblems();
    }

    void IncrementalEstimator::orderMarginalizedDesignVariables() {
      auto groupsOrdering = _problem->getGroupsOrdering();
      auto margGroupIt = std::find(groupsOrdering.begin(),
        groupsOrdering.end(), _margGroupId);
      if (margGroupIt == groupsOrdering.end())
        throw InvalidOperationException(
          "IncrementalEstimator::orderMarginalizedDesignVariables(): "
          "marginalized group ID should appear in the problem", __FILE__,
          __LINE__);
      else {
        if (*margGroupIt != groupsOrdering.back()) {
          std::swap(*margGroupIt, groupsOrdering.back());
          _problem->setGroupsOrdering(groupsOrdering);
        }
      }
    }

    void IncrementalEstimator::restoreLinearSolver() {
      // init the matrix structure
      std::vector<aslam::backend::DesignVariable*> dvs;
      const size_t numDVS = _problem->numDesignVariables();
      dvs.reserve(numDVS);
      size_t columnBase = 0;
      for (size_t i = 0; i < numDVS; ++i) {
        aslam::backend::DesignVariable* dv = _problem->designVariable(i);
        if (dv->isActive()) {
          dvs.push_back(dv);
          dv->setBlockIndex(dvs.size() - 1);
          dv->setColumnBase(columnBase);
          columnBase += dv->minimalDimensions();
        }
      }
      std::vector<aslam::backend::ErrorTerm*> ets;
      const size_t numETS = _problem->numErrorTerms();
      ets.reserve(numETS);
      size_t dim = 0;
      for (size_t i = 0; i < numETS; ++i) {
        aslam::backend::ErrorTerm* et = _problem->errorTerm(i);
        et->setRowBase(dim);
        dim += et->dimension();
        ets.push_back(et);
      }
      auto linearSolver = _optimizer->getSolver<LinearSolver>();
      linearSolver->initMatrixStructure(dvs, ets, false);

      // build the system
      linearSolver->buildSystem(_optimizer->options().nThreads, true);
    }

  }
}
