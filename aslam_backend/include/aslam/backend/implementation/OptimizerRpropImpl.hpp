/*
 * OptimizerRpropImpl.hpp
 *
 *  Created on: 10.06.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERRPROPIMPL_HPP_
#define INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERRPROPIMPL_HPP_

#include <boost/serialization/nvp.hpp>

namespace aslam {
namespace backend {

template<class Archive>
inline void OptimizerOptionsRprop::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(OptimizerOptionsBase);
  ar & BOOST_SERIALIZATION_NVP(etaMinus);
  ar & BOOST_SERIALIZATION_NVP(etaPlus);
  ar & BOOST_SERIALIZATION_NVP(initialDelta);
  ar & BOOST_SERIALIZATION_NVP(minDelta);
  ar & BOOST_SERIALIZATION_NVP(maxDelta);
  ar & BOOST_SERIALIZATION_NVP(useDenseJacobianContainer);
  ar & BOOST_SERIALIZATION_NVP(regularizer);
  ar & BOOST_SERIALIZATION_NVP(method);
}

} /* namespace aslam */
} /* namespace backend */

#endif /* INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERRPROPIMPL_HPP_ */
