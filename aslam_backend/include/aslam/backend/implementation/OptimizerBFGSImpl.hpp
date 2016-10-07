/*
 * OptimizerBFGSImpl.hpp
 *
 *  Created on: 10.06.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERBFGSIMPL_HPP_
#define INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERBFGSIMPL_HPP_

#include <boost/serialization/nvp.hpp>

namespace aslam {
namespace backend {

template<class Archive>
inline void OptimizerOptionsBFGS::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(OptimizerOptionsBase);
  ar & BOOST_SERIALIZATION_NVP(linesearch);
  ar & BOOST_SERIALIZATION_NVP(useDenseJacobianContainer);
  ar & BOOST_SERIALIZATION_NVP(regularizer);
}

} /* namespace aslam */
} /* namespace backend */

#endif /* INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_OPTIMIZERBFGSIMPL_HPP_ */
