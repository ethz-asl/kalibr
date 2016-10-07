/*
 * NsecTimePolicy.hpp
 *
 *  Created on: Sep 16, 2013
 *      Author: hannes
 */

#ifndef NSECTIMEPOLICY_HPP_
#define NSECTIMEPOLICY_HPP_

#include <sm/timing/NsecTimeUtilities.hpp>
#include "SimpleTypeTimePolicy.hpp"

namespace bsplines {

struct NsecTimePolicy : public SimpleTypeTimePolicy<sm::timing::NsecTime> {
	constexpr inline static sm::timing::NsecTime getOne() {
		return sm::timing::NsecTime(1E9);
	}
};

}  // namespace bsplines

#endif /* NSECTIMEPOLICY_HPP_ */
