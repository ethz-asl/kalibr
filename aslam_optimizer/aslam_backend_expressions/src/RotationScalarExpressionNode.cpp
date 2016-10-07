/*
 * RotationScalarExpressionNode.cpp
 *
 *  Created on: Oct 5, 2014
 *      Author: hannes
 */

#include <Eigen/Core>
#include <aslam/backend/RotationScalarExpressionNode.hpp>

namespace aslam {
namespace backend {

    Eigen::Matrix3d RotationScalarExpressionNode::toRotationMatrixImplementation() const
    {
      const double s = _s->toScalar(), cs = cos(s), ss = sin(s);
      Eigen::Matrix3d ret;
      ret.setZero();

      ret(_axis, _axis) = 1;
      ret((_axis + 1) % 3, (_axis + 1) % 3) = cs;
      ret((_axis + 2) % 3, (_axis + 2) % 3) = cs;
      ret((_axis + 1) % 3, (_axis + 2) % 3) = -ss;
      ret((_axis + 2) % 3, (_axis + 1) % 3) = ss;
      return  ret;
    }

    void RotationScalarExpressionNode::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      /*
       * Assume R to be the current rotation value of this exp.
       * J_v (Phi_R^{-1} R(s(v))) = J_v  ( log ( R(s(v) * R^{-1})) = e_axis
       */
      _s->evaluateJacobians(outJacobians, -Eigen::Vector3d::Unit(_axis));
    }

    void RotationScalarExpressionNode::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _s->getDesignVariables(designVariables);
    }
}
}
