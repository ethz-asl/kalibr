#include <gtest/gtest.h>
#include <sm/maths.hpp>
#include <algorithm>

TEST(SmCommonTestSuite,testSolveQuadratic)
{
  // ( x - 1) ( x + 2 ) = x^2 + x - 2
  // x = 1, x = -2
  std::pair<double,double> r = sm::solveQuadratic(1,1,-2);
  if(r.first < r.second)
    {
      std::swap(r.first,r.second);
    }

  ASSERT_NEAR(r.first,1,1e-10);
  ASSERT_NEAR(r.second,-2,1e-10);


  // (2x - 2)(x - 4) = 2x^2 - 10 x + 8
  r = sm::solveQuadratic(2, -10, 8);
  if(r.first < r.second)
    {
      std::swap(r.first,r.second);
    }

  ASSERT_NEAR(r.first,4,1e-10);
  ASSERT_NEAR(r.second,1,1e-10);

  
}

TEST(SmCommonTestSuite,testSolveQuadraticImaginary)
{

  // b*b - 4*a*c = 0
  ASSERT_NO_THROW(sm::solveQuadratic(1,2,1));

  // b*b - 4*a*c = -3
  ASSERT_THROW(sm::solveQuadratic(1,1,1),std::runtime_error);

}
