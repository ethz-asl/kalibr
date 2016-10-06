/*
 * numerical_comparisons.hpp
 *
 *  Created on: Aug 13, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef SM_NUMERICALCOMPARISON_HPP_
#define SM_NUMERICALCOMPARISON_HPP_

#include <algorithm> // std::max
#include <limits> // std::numeric_limits
#include <cmath> // std::abs
#include <stdexcept> // std::invalid_argument
#include "assert_macros.hpp"

namespace sm {

/*
 * Functions to compare floating point numbers with a relative error (epsilon).
 *
 * As the precision of floating point numbers are limited and depending on the
 * magnitude, we compare two numbers a and b by comparing their difference |a - b|
 * to the magnitude of the the bigger number max(|a|, |b|) multiplied with the
 * relative error epsilon. Therefore, a and b are approximately equal if it holds
 * |a - b| <= max(|a|, |b|) * epsilon. This can be applied analogously to greater
 * and less than comparisons.
 *
 * More information on compare floating point numbers is given in
 * - http://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 * - The art of computer programming, Volume 2 / Seminumerical Algorithms, Donald E. Knuth (page 218)
 */

namespace internal {

/*!
 * Takes max(|a|, |b|) and multiplies it with epsilon.
 * @param a the first number.
 * @param b the second number.
 * @param epsilon the precision (epsilon > 0).
 * @return the result of max(|a|, |b|) * epsilon.
 */
template<typename ValueType_>
static inline ValueType_ maxTimesEpsilon(const ValueType_ a, const ValueType_ b, const ValueType_ epsilon)
{
  SM_ASSERT_GT_DBG(std::invalid_argument, epsilon, 0.0, "This method is only valid for an epsilon greater than 0.");
  return std::max(std::abs(a), std::abs(b)) * epsilon;
}

} /* namespace internal */

/*!
 * Checks if two numbers a and b are equal within a relative error.
 * @param[in] a the first number to compare.
 * @param[in] b the second number to compare.
 * @param[in] epsilon the relative error (optional, if not declared the precision of the datatype).
 * @return true if a and b are approximately equal, false otherwise.
 */
template<typename ValueType_>
static bool approximatelyEqual(const ValueType_ a, const ValueType_ b, ValueType_ epsilon = std::numeric_limits<ValueType_>::epsilon())
{
  SM_ASSERT_GT(std::invalid_argument, epsilon, 0.0, "This method is only valid for an epsilon greater than 0.");
  return std::abs(a - b) <= internal::maxTimesEpsilon(a, b, epsilon);
}

/*!
 * Checks if a is greater than b (a > b) within a relative error.
 * @param a the first number to compare.
 * @param b the second number to compare.
 * @param epsilon the relative error (optional, if not declared the precision of the datatype).
 * @return true if a definitely greater than b, false otherwise.
 */
template<typename ValueType_>
static bool definitelyGreaterThan(const ValueType_ a, const ValueType_ b, ValueType_ epsilon = std::numeric_limits<ValueType_>::epsilon())
{
  SM_ASSERT_GT(std::invalid_argument, epsilon, 0.0, "This method is only valid for an epsilon greater than 0.");
  return (a - b) > internal::maxTimesEpsilon(a, b, epsilon);
}

/*!
 * Checks if a is less than b (a < b) within a relative error.
 * @param a the first number to compare.
 * @param b the second number to compare.
 * @param epsilon the relative error (optional, if not declared the precision of the datatype).
 * @return true if a definitely less than b, false otherwise.
 */
template<typename ValueType_>
static bool definitelyLessThan(const ValueType_ a, const ValueType_ b, ValueType_ epsilon = std::numeric_limits<ValueType_>::epsilon())
{
  SM_ASSERT_GT(std::invalid_argument, epsilon, 0.0, "This method is only valid for an epsilon greater than 0.");
  return (b - a) > internal::maxTimesEpsilon(a, b, epsilon);
}

} /* namespace sm */
#endif /* SM_NUMERICALCOMPARISON_HPP_ */
