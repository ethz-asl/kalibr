/*
 * OptimizerBaseImplementation.hpp
 *
 *  Created on: 12.04.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERBASEIMPLEMENTATION_HPP_
#define INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERBASEIMPLEMENTATION_HPP_

#include <aslam/Exceptions.hpp>

#include <boost/serialization/nvp.hpp>

namespace aslam
{
namespace backend
{

template<class Archive>
inline void OptimizerOptionsBase::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & BOOST_SERIALIZATION_NVP(convergenceGradientNorm);
  ar & BOOST_SERIALIZATION_NVP(convergenceDeltaX);
  ar & BOOST_SERIALIZATION_NVP(convergenceDeltaObjective);
  ar & BOOST_SERIALIZATION_NVP(maxIterations);
  ar & BOOST_SERIALIZATION_NVP(numThreadsGradient);
  ar & BOOST_SERIALIZATION_NVP(numThreadsError);
}

template<class Archive>
inline void OptimizerStatus::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & BOOST_SERIALIZATION_NVP(convergence);
  ar & BOOST_SERIALIZATION_NVP(numIterations);
  ar & BOOST_SERIALIZATION_NVP(numDerivativeEvaluations);
  ar & BOOST_SERIALIZATION_NVP(numObjectiveEvaluations);
  ar & BOOST_SERIALIZATION_NVP(gradientNorm);
  ar & BOOST_SERIALIZATION_NVP(maxDeltaX);
  ar & BOOST_SERIALIZATION_NVP(error);
  ar & BOOST_SERIALIZATION_NVP(deltaError);
}

bool OptimizerBase::isConverged() const
{
  return this->getStatus().success();
}

bool OptimizerBase::isFailed() const
{
  return this->getStatus().failure();
}

bool OptimizerBase::isInProgress() const
{
  return !this->getStatus().failure() && !this->getStatus().success();
}

OptimizerStatus& OptimizerBase::status()
{
  return const_cast<OptimizerStatus&>(static_cast<const OptimizerBase&>(*this).getStatus());
}

} /* namespace aslam */
} /* namespace backend */


#endif /* INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERBASEIMPLEMENTATION_HPP_ */
