/*
 * SimpleTypeTimePolicy.hpp
 *
 *  Created on: May 10, 2012
 *      Author: hannes
 */

#ifndef SIMPLETYPETIMEPOLICY_HPP_
#define SIMPLETYPETIMEPOLICY_HPP_

#include <cmath>
#include <cassert>

namespace bsplines {
	template <typename SimpleType_>
	struct SimpleTypeTimePolicy {
		typedef SimpleType_ time_t;
		typedef SimpleType_ duration_t;

		inline static duration_t computeDuration(time_t from, time_t till){
			return till - from;
		}

		inline static duration_t addScaledDuration(time_t from, duration_t dist, int scale){
			return from + dist * scale;
		}

		inline static double divideDurations(duration_t a, duration_t b){
			return (double) a/b;
		}

		inline static time_t linearlyInterpolate(time_t from, time_t till, int segments, int pos)
		{
			if(pos == segments) return till;
			return from + (computeDuration(from, till) * pos) / segments;
		}

		inline static int getSegmentNumber(time_t from, time_t till, int segments, time_t t)
		{
			duration_t duration = computeDuration(from, till);
			assert(duration * segments / duration == segments);
			// The above assertion fails if the type time_t is not capable to carry out the following operation precise enough for the duration given by (from,till). I.e. to correctly invert the linearlyInterpolate method above.
			// This basically means the spline is too large (in duration or resolution) to be used with this time policy. So either shorten it or implement a better time policy.
			// For example with nanoseconds as signed 64 bit integers this holds if 
			// duration * segments = duration * duration / T <= ~9.22e9 seconds. // where T is the uniform knot distance.
			// I.e. for 1000s the maximum supported knot rate is around 9kHz.
			// TODO increase the maximal supported duration^2 * knotFreq.
			return ((t - from) * segments) / duration;
		}

		constexpr inline static duration_t getZero(){
			return duration_t(0.0);
		}

		constexpr inline static duration_t getOne(){
			static_assert(!std::is_integral<duration_t>::value, "Please shadow this (getOne) method for your simple type's time policy - preferably with an inline or even better constexpr function.");
			return duration_t(1.0);
		}
	};

} // namespace bsplines

#endif /* SIMPLETYPETIMEPOLICY_HPP_ */
