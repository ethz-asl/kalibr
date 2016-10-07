/*
 * utils.hpp
 *
 *  Created on: 24.09.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_UTIL_EXPRESSIONUTILS_HPP_
#define INCLUDE_ASLAM_BACKEND_UTIL_EXPRESSIONUTILS_HPP_

#include <aslam/backend/util/utils.hpp>

namespace aslam {
namespace backend {
namespace utils {

template <typename T>
inline bool isFinite(const JacobianContainer& jc, const T& expr) {
  DesignVariable::set_t dvs;
  expr.getDesignVariables(dvs);
  for (auto& dv : dvs) {
    if (dv->isActive() && !jc.isFinite(*dv))
      return false;
  }
  return true;
}

}
}
}

#endif /* INCLUDE_ASLAM_BACKEND_UTIL_EXPRESSIONUTILS_HPP_ */
