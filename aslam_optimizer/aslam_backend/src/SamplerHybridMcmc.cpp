/*
 * SamplerHmc.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: Ulrich Schwesinger
 */

#include <aslam/backend/SamplerHybridMcmc.hpp>
#include <aslam/backend/DesignVariable.hpp>

#include <cmath>

#include <sm/logging.hpp>
#include <sm/random.hpp>
#include <sm/PropertyTree.hpp>

using namespace std;

namespace aslam {
namespace backend {

SamplerHybridMcmcOptions::SamplerHybridMcmcOptions() {
  check();
}

SamplerHybridMcmcOptions::SamplerHybridMcmcOptions(const sm::PropertyTree& config) {

  initialLeapFrogStepSize = config.getDouble("initialLeapFrogStepSize", initialLeapFrogStepSize);
  minLeapFrogStepSize = config.getDouble("minLeapFrogStepSize", minLeapFrogStepSize);
  maxLeapFrogStepSize = config.getDouble("maxLeapFrogStepSize", maxLeapFrogStepSize);
  incFactorLeapFrogStepSize = config.getDouble("incFactorLeapFrogStepSize", incFactorLeapFrogStepSize);
  decFactorLeapFrogStepSize = config.getDouble("decFactorLeapFrogStepSize", decFactorLeapFrogStepSize);
  targetAcceptanceRate = config.getDouble("targetAcceptanceRate", targetAcceptanceRate);
  nLeapFrogSteps = config.getInt("nLeapFrogSteps", nLeapFrogSteps);
  nThreads = config.getDouble("nThreads", nThreads);

  check();

}

void SamplerHybridMcmcOptions::check() const {
  SM_ASSERT_GT(Exception, initialLeapFrogStepSize, 0.0, "");
  SM_ASSERT_GE(Exception, initialLeapFrogStepSize, minLeapFrogStepSize, "");
  SM_ASSERT_LE(Exception, initialLeapFrogStepSize, maxLeapFrogStepSize, "");
  SM_ASSERT_GT(Exception, minLeapFrogStepSize, 0.0, "");
  SM_ASSERT_GT(Exception, maxLeapFrogStepSize, 0.0, "");
  SM_ASSERT_GE(Exception, maxLeapFrogStepSize, minLeapFrogStepSize, "");
  SM_ASSERT_GE(Exception, incFactorLeapFrogStepSize, 1.0, "");
  SM_ASSERT_GT(Exception, decFactorLeapFrogStepSize, 0.0, "");
  SM_ASSERT_LE(Exception, decFactorLeapFrogStepSize, 1.0, "");
  SM_ASSERT_GE(Exception, targetAcceptanceRate, 0.0, "");
  SM_ASSERT_LE(Exception, targetAcceptanceRate, 1.0, "");
}

ostream& operator<<(ostream& out, const aslam::backend::SamplerHybridMcmcOptions& options) {
  out << "SamplerHmcOptions:\n";
  out << "\tinitialLeapFrogStepSize: " << options.initialLeapFrogStepSize << endl;
  out << "\tminLeapFrogStepSize: " << options.minLeapFrogStepSize << endl;
  out << "\tmaxLeapFrogStepSize: " << options.maxLeapFrogStepSize << endl;
  out << "\tincFactorLeapFrogStepSize: " << options.incFactorLeapFrogStepSize << endl;
  out << "\tdecFactorLeapFrogStepSize: " << options.decFactorLeapFrogStepSize << endl;
  out << "\ttargetAcceptanceRate: " << options.targetAcceptanceRate << endl;
  out << "\tnLeapFrogSteps: " << options.nLeapFrogSteps << endl;
  out << "\tnThreads: " << options.nThreads << endl;
  return out;
}




SamplerHybridMcmc::SamplerHybridMcmc() :
  _options(),
  _gradient(),
  _u(),
  _stepLength(_options.initialLeapFrogStepSize) {

}

SamplerHybridMcmc::SamplerHybridMcmc(const SamplerHybridMcmcOptions& options) :
  _options(options),
  _gradient(),
  _u(),
  _stepLength(_options.initialLeapFrogStepSize) {

}

void SamplerHybridMcmc::saveDesignVariables() {
  Timer t("SamplerHmc: Save design variables", false);
  for (size_t i = 0; i < getProblemManager().numDesignVariables(); i++) {
    const DesignVariable* dv = getProblemManager().designVariable(i);
    dv->getParameters(_dvState[i].second);
  }
}

void SamplerHybridMcmc::revertUpdateDesignVariables() {
  Timer t("SamplerHmc: Revert update design variables", false);
  for (auto dvParamPair : _dvState)
    dvParamPair.first->setParameters(dvParamPair.second);
}

void SamplerHybridMcmc::initialize() {
  SamplerBase::initialize();
  _dvState.resize(getProblemManager().numDesignVariables());
  for (size_t i = 0; i < _dvState.size(); i++)
    _dvState[i].first = getProblemManager().designVariable(i);
  _gradient.resize(getProblemManager().numOptParameters());
}

void SamplerHybridMcmc::setOptions(const SamplerHybridMcmcOptions& options) {
  _options = options;
  _stepLength = _options.initialLeapFrogStepSize;
}

void SamplerHybridMcmc::step(bool& accepted, double& acceptanceProbability) {

  // Note: The notation follows the implementation here:
  // https://theclevermachine.wordpress.com/2012/11/18/mcmc-hamiltonian-monte-carlo-a-k-a-hybrid-monte-carlo/

  using namespace Eigen;

  // Adapt step length
  if (isBurnIn()) {
    if (statistics().getWeightedMeanAcceptanceProbability() > _options.targetAcceptanceRate)
      _stepLength *= _options.incFactorLeapFrogStepSize;
    else
      _stepLength *= _options.decFactorLeapFrogStepSize;
    _stepLength = max(min(_stepLength, _options.maxLeapFrogStepSize), _options.minLeapFrogStepSize); // clip
  }

  SM_FINEST_STREAM_NAMED("sampling", "Current leap frog step size is " << _stepLength << ".");

  // pre-computations for speed-up of upcoming calculations
  const double deltaHalf = _stepLength/2.;

  ColumnVectorType dxStar;
  ColumnVectorType pStar;
  double u0, k0, kStar, eTotal0, eTotalStar;
  bool success = false;

  // save the state of the design variables to be able to revert them later to this stage
  saveDesignVariables();

  // ******** Simulate Hamiltonian dynamics via Leap-Frog method ********* //

  while (!success) { // repeat until a valid sample was produced (no divergence of trajectory simulation)

    bool diverged = false;
    const bool doRecompute = isRecomputationNegLogDensityNecessary();

    // sample random momentum
    auto normal_dist = [&] (int) { return sm::random::randn()*_options.standardDeviationMomentum; };
    pStar = ColumnVectorType::NullaryExpr(getProblemManager().numOptParameters(), normal_dist);

    // evaluate energies at start of trajectory
    if (doRecompute) {
      u0 = evaluateNegativeLogDensity(); // potential energy
    } else {
      u0 = _u;
      SM_ASSERT_EQ_DBG(Exception, evaluateNegativeLogDensity(), u0, ""); // check that caching works
    }
    k0 = 0.5*pStar.transpose()*pStar; // kinetic energy
    eTotal0 = u0 + k0;

    // first half step of momentum
    if (doRecompute) { // we can avoid recomputing the gradient if the last sample was accepted
      Timer timer("SamplerHybridMcmc: Compute---Gradient", false);
      getProblemManager().computeGradient(_gradient, _options.nThreads, false /*TODO: useMEstimator*/, false /*TODO: use scaling*/, true /*TODO: useDenseJacobianContainer */);
      timer.stop();
    }
#ifndef NDEBUG
    else {
      RowVectorType grad;
      getProblemManager().computeGradient(grad, _options.nThreads, false /*TODO: useMEstimator*/, false /*TODO: use scaling*/, true /*TODO: useDenseJacobianContainer */);
      SM_ASSERT_TRUE(Exception, _gradient.isApprox(grad), ""); // check that caching works
    }
#endif
    RowVectorType gradient0 = _gradient; // to be able to restore later


    pStar -= deltaHalf*_gradient;

    // first full step for position/sample
    dxStar = _stepLength*pStar;
    getProblemManager().applyStateUpdate(dxStar);

    SM_ALL_STREAM_NAMED("sampling", "Step 0 -- Momentum: " << pStar.transpose() << ", position update: " << dxStar.transpose());

    // L-1 full steps
    for(size_t l = 1; l < _options.nLeapFrogSteps - 1; ++l) {
      try {
        // momentum
        Timer timer("SamplerHybridMcmc: Compute---Gradient", false);
        getProblemManager().computeGradient(_gradient, _options.nThreads, false /*TODO: useMEstimator*/, false /*TODO: use scaling*/, true /*TODO: useDenseJacobianContainer */);
        timer.stop();

        pStar -= _stepLength*_gradient;

        // position/sample
        dxStar = _stepLength*pStar;
        if(!dxStar.allFinite()) { // we can abort the trajectory generation if it diverged
          diverged = true;
          break;
        }
        getProblemManager().applyStateUpdate(dxStar);

        SM_ALL_STREAM_NAMED("sampling", "Step " << l << " -- Momentum: " << pStar.transpose() << ", position update: " << dxStar.transpose());
      } catch (const std::exception& e) {
        SM_WARN_STREAM(e.what() << ": Compute gradient failed, terminating leap-frog simulation and rejecting sample");
        diverged = true;
        break;
      }
    }

    if (!diverged) {
      try {
        // last half step
        Timer timer("SamplerHybridMcmc: Compute---Gradient", false);
        getProblemManager().computeGradient(_gradient, _options.nThreads, false /*TODO: useMEstimator*/, false /*TODO: use scaling*/, true /*TODO: useDenseJacobianContainer */);
        timer.stop();
        pStar -= deltaHalf*_gradient;

        // ******************************************************************* //

        // evaluate energies at end of trajectory
        _u = evaluateNegativeLogDensity(); // potential energy
        kStar = 0.5*pStar.transpose()*pStar; // kinetic energy
        eTotalStar = _u + kStar;
      } catch (const std::exception& e) {
        SM_WARN_STREAM(e.what() << ": Compute gradient failed, terminating leap-frog simulation and rejecting sample");
        diverged = true;
      }
    }

    // Check the total energy is finite
    diverged = diverged || !isfinite(eTotalStar);

    // acceptance/rejection probability
    acceptanceProbability = 0.0; // this rejects the sample if the energy is not finite
    if (!diverged) {
      acceptanceProbability = min(1.0, exp(eTotal0 - eTotalStar)); // TODO: can we remove the thresholding?
      SM_FINEST_STREAM_NAMED("sampling", "Energy " << eTotal0 << " (potential: " << u0 << ", kinetic: " << k0 << ") ==> " <<
                             eTotalStar << " (potential: " << _u << ", kinetic: " << kStar << "), acceptanceProbability = " << acceptanceProbability);
      success = true;
    } else {
      _stepLength *= _options.decFactorLeapFrogStepSize;
      _stepLength = max(_stepLength, _options.minLeapFrogStepSize); // clip
      SM_WARN_STREAM("Leap-Frog method diverged, reducing step length to " << _stepLength << " and repeating sample...");
    }

    if (sm::random::randLU(0., 1.0) < acceptanceProbability) { // sample accepted, we keep the new design variables
      SM_FINEST_STREAM_NAMED("sampling", "Sample accepted");
      accepted = true;
    } else { // sample rejected, we revert the update
      revertUpdateDesignVariables();
      _u = u0;
      _gradient = gradient0;
      SM_FINEST_STREAM_NAMED("sampling", "Sample rejected");
      accepted = false;
    }

    if (diverged && _stepLength - _options.minLeapFrogStepSize < 1e-12) {
      SM_ERROR("Leap-Frog method diverged and current step length reached minimum. "
          "Cannot generate sample. Consider reducing minimum step length!");
      break;
    }
  }

}

} /* namespace aslam */
} /* namespace backend */
