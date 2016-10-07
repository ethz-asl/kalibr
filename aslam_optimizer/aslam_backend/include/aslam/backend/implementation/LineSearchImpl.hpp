/*
 * LineSearchImpl.hpp
 *
 *  Created on: 10.06.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_LINESEARCHIMPL_HPP_
#define INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_LINESEARCHIMPL_HPP_

#include <boost/serialization/nvp.hpp>

namespace aslam {
namespace backend {

template<class Archive>
inline void LineSearchOptions::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & BOOST_SERIALIZATION_NVP(c1WolfeCondition);
  ar & BOOST_SERIALIZATION_NVP(c2WolfeCondition);
  ar & BOOST_SERIALIZATION_NVP(maxStepLength);
  ar & BOOST_SERIALIZATION_NVP(minStepLength);
  ar & BOOST_SERIALIZATION_NVP(xtol);
  ar & BOOST_SERIALIZATION_NVP(initialStepLength);
  ar & BOOST_SERIALIZATION_NVP(nMaxIterWolfe1);
  ar & BOOST_SERIALIZATION_NVP(nMaxIterWolfe2);
  ar & BOOST_SERIALIZATION_NVP(nMaxIterZoom);
}

} /* namespace aslam */
} /* namespace backend */

#endif /* INCLUDE_ASLAM_BACKEND_IMPLEMENTATION_LINESEARCHIMPL_HPP_ */
