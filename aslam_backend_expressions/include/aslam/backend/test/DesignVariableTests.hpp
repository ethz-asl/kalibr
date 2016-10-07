/*
 * DesignVariableTests.hpp
 *
 *  Created on: Oct 7, 2013
 *      Author: hannes
 */

#ifndef DESIGNVARIABLETESTS_HPP_
#define DESIGNVARIABLETESTS_HPP_

#include <aslam/backend/DesignVariableMinimalDifferenceExpression.hpp>
#include <aslam/backend/FixedPointNumber.hpp>
#include "ExpressionTests.hpp"

namespace aslam {
namespace backend {
namespace test {

template <typename DesignVariable_>
struct DesignVariableUpdateTraits {
  typedef std::numeric_limits<double> Limits;

  static double defaultTolerance() {
    return sqrt(Limits::epsilon()) * 1E2;
  }
  static double defaulEps() {
    return sqrt(Limits::epsilon());
  }

  constexpr static bool is_iteger = false;
};


template <typename Scalar_>
struct DesignVariableUpdateTraits<GenericScalar<Scalar_> > {
  typedef std::numeric_limits<Scalar_> Limits;
  constexpr static bool is_iteger = Limits::is_integer;

  static double defaultTolerance() {
    return defaulEps() * 50;
  }

  static double defaulEps() {
    return is_iteger ? 1.0 : (is_fixed_point_number<Scalar_>::value ? Limits::epsilon() : sqrt(Limits::epsilon()));
  }

};

template <typename DesignVariable_, int MinimalDimensions = DesignVariable_::MinimalDimension>
void testDesignVariable(DesignVariable_ & dv, const double tolerance = DesignVariableUpdateTraits<DesignVariable_>::defaultTolerance(), const double eps = DesignVariableUpdateTraits<DesignVariable_>::defaulEps()) {
  ASSERT_EQ(MinimalDimensions, dv.minimalDimensions());
  Eigen::MatrixXd startPoint;
  dv.getParameters(startPoint);

  DesignVariableMinimalDifferenceExpression<MinimalDimensions> dvMDE(dv, startPoint);
  Eigen::VectorXd updateVector(MinimalDimensions);
  updateVector.setRandom();
  if(DesignVariableUpdateTraits<DesignVariable_>::is_iteger){
    updateVector = (updateVector * 10).cast<int>().cast<double>();
  }

  dv.update(&updateVector[0], updateVector.size());

  Eigen::VectorXd vector;
  dv.minimalDifference(startPoint, vector);
  sm::eigen::assertNear(vector, updateVector, tolerance, SM_SOURCE_FILE_POS);
  sm::eigen::assertEqual(dvMDE.evaluate(), vector, SM_SOURCE_FILE_POS);

  SCOPED_TRACE("");
  testExpression(dvMDE, 1, false, tolerance, eps);
}

}
}
}

#endif /* DESIGNVARIABLETESTS_HPP_ */
