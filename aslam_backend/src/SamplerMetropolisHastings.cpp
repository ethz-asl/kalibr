/*
 * SamplerMetropolisHastings.cpp
 *
 *  Created on: 24.07.2015
 *      Author: sculrich
 */

#include <aslam/backend/SamplerMetropolisHastings.hpp>

#include <cmath> // std::exp

#include <sm/logging.hpp>
#include <sm/random.hpp>

using namespace std;

namespace aslam {
namespace backend {

SamplerMetropolisHastingsOptions::SamplerMetropolisHastingsOptions() {

}

SamplerMetropolisHastingsOptions::SamplerMetropolisHastingsOptions(const sm::PropertyTree& config) {
    transitionKernelSigma = config.getDouble("transitionKernelSigma", transitionKernelSigma);
    nThreadsEvaluateLogDensity = config.getDouble("nThreadsEvaluateLogDensity", nThreadsEvaluateLogDensity);
}

void SamplerMetropolisHastingsOptions::check() const {
  SM_ASSERT_POSITIVE( Exception, transitionKernelSigma, "");
}

std::ostream& operator<<(std::ostream& out, const aslam::backend::SamplerMetropolisHastingsOptions& options) {
  out << "SamplerMetropolisHastingsOptions:" << std::endl;
  out << "\ttransitionKernelSigma: " << options.transitionKernelSigma << std::endl;
  out << "\tnThreadsEvaluateLogDensity: " << options.nThreadsEvaluateLogDensity << std::endl;
  return out;
}



SamplerMetropolisHastings::SamplerMetropolisHastings()
    : SamplerMetropolisHastings::SamplerMetropolisHastings(Options()) {

}

SamplerMetropolisHastings::SamplerMetropolisHastings(const SamplerMetropolisHastingsOptions& options) :
  _options(options),
  _negLogDensity(std::numeric_limits<double>::signaling_NaN()) {

}

void SamplerMetropolisHastings::step(bool& accepted, double& acceptanceProbability) {


  if (isRecomputationNegLogDensityNecessary())
    _negLogDensity = evaluateNegativeLogDensity(_options.nThreadsEvaluateLogDensity);
#ifndef NDEBUG
  else
    SM_ASSERT_EQ(Exception, evaluateNegativeLogDensity(_options.nThreadsEvaluateLogDensity), _negLogDensity, ""); // check that caching works
#endif

  auto normal_dist = [&] (int) { return _options.transitionKernelSigma*sm::random::randn(); };
  const ColumnVectorType dx = ColumnVectorType::NullaryExpr(getProblemManager().numOptParameters(), normal_dist);
  getProblemManager().applyStateUpdate(dx);

  const double negLogDensityNew = evaluateNegativeLogDensity(_options.nThreadsEvaluateLogDensity);

  acceptanceProbability = std::exp(std::min(0.0, -negLogDensityNew + _negLogDensity));
  SM_VERBOSE_STREAM_NAMED("sampling", "NegLogDensity: " << _negLogDensity << "->" << negLogDensityNew << ", acceptance probability: " << acceptanceProbability);

  if (sm::random::randLU(0.0, 1.0) < acceptanceProbability) { // sample accepted, we keep the new design variables
    _negLogDensity = negLogDensityNew;
    accepted = true;
    SM_VERBOSE_STREAM_NAMED("sampling", "Sample accepted");
  } else { // sample rejected, we revert the update
    getProblemManager().revertLastStateUpdate();
    accepted = false;
    SM_VERBOSE_STREAM_NAMED("sampling", "Sample rejected");
  }

}

void SamplerMetropolisHastings::resetImplementation() {
  _negLogDensity = std::numeric_limits<double>::signaling_NaN();
}

} /* namespace aslam */
} /* namespace backend */
