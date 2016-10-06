#ifndef SM_MATHS_HPP
#define SM_MATHS_HPP

#include <cmath>

#include <utility> // std::pair, if you can believe that.
#include "assert_macros.hpp"
namespace sm {

  /** 
   * \brief Solve a quadratic equation of the form \f$ a x^2 + b x + c = 0 \f$
   * 
   * This uses the numerically stable algorithm described here:
   * http://en.wikipedia.org/wiki/Loss_of_significance#Instability_of_the_quadratic_equation
   *
   * The function does not support complex solutions. If \f$ b*b - 4*a*c < 0\f$, the function
   * will throw an exception
   *
   * @return a 2x1 column with the solutions
   */
  inline std::pair<double,double> solveQuadratic(double a, double b, double c)
  {
    double bb_m_4ac = b*b - 4*c*a;
    
    SM_ASSERT_GE(std::runtime_error, bb_m_4ac, 0.0, "b^2 - 4ac < 0. Imaginary solutions are not supported");

    double n;
    if(b < 0.f)
      {
	n = sqrt(bb_m_4ac) - b;
      } 
    else
      {
	n = -sqrt(bb_m_4ac) - b;
      }

    return std::make_pair(n/(2*a),(2*c)/n);

  }
  

  /** 
   * \brief An implementation of the signum function
   * 
   * @param val the variable to check
   * 
   * @return -1 if the variable is negative, +1 if the variable is positive, and 0 if the variable is 0
   */
  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

} // namespace sm


#endif /* SM_MATHS_HPP */
