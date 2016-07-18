/*
 * RotationExpressionTests.hpp
 *
 *  Created on: Jul 16, 2013
 *      Author: hannes
 */

#ifndef ROTATIONEXPRESSIONTESTS_HPP_
#define ROTATIONEXPRESSIONTESTS_HPP_

#include <aslam/backend/test/ExpressionTests.hpp>

namespace aslam {
namespace backend {
namespace test {

template<>
struct ExpressionValueTraits<RotationExpression> {
  typedef Eigen::Vector3d value_t;
};

template<>
class ExpressionJacobianTestTraits<RotationExpression> {
 private:
  struct RotationExpressionNodeFunctor {
    typedef Eigen::Vector3d value_t;
    typedef value_t::Scalar scalar_t;
    typedef Eigen::VectorXd input_t;
    typedef Eigen::MatrixXd jacobian_t;

    RotationExpressionNodeFunctor(RotationExpression dv, Eigen::Vector3d p)
        : _p(p),
          _dv(dv) {
    }

    input_t update(const input_t & x, int c, scalar_t delta) {
      input_t xnew = x;
      xnew[c] += delta;
      return xnew;
    }

    Eigen::VectorXd _p;
    RotationExpression _dv;

    Eigen::VectorXd operator()(const Eigen::VectorXd & dr) {

      Eigen::Matrix3d C = _dv.toRotationMatrix();
      JacobianContainer J(3);
      _dv.evaluateJacobians(J);

      int offset = 0;
      for (size_t i = 0; i < J.numDesignVariables(); i++) {
        DesignVariable * d = J.designVariable(i);
        d->update(&dr[offset], d->minimalDimensions());
        offset += d->minimalDimensions();
      }

      C = _dv.toRotationMatrix();

      for (size_t i = 0; i < J.numDesignVariables(); i++) {
        DesignVariable * d = J.designVariable(i);
        d->revertUpdate();
      }

      return C * _p;

    }
  };
 public:
  static void testJacobian(const ExpressionTester<RotationExpression> & expressionTester) {
    auto rotExp = expressionTester.getExp();
    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d p;
      p.setZero();
      p(i) = 1;
      RotationExpressionNodeFunctor functor(rotExp, p);

      sm::eigen::NumericalDiff<RotationExpressionNodeFunctor> numdiff(functor, expressionTester.getEps());

      /// Discern the size of the jacobian container
      Eigen::Matrix3d C = rotExp.toRotationMatrix();
      JacobianContainer Jc(3);
      rotExp.evaluateJacobians(Jc);
      Eigen::Matrix3d Cp_cross = sm::kinematics::crossMx(C * p);
      Jc.applyChainRule(Cp_cross);

      Eigen::VectorXd dp(Jc.cols());
      dp.setZero();
      Eigen::MatrixXd Jest = numdiff.estimateJacobian(dp);

      auto JcM = Jc.asSparseMatrix();
      sm::eigen::assertNear(
        Jc.asSparseMatrix(),
        Jest,
        expressionTester.getTolerance(),
        SM_SOURCE_FILE_POS,
        (std::stringstream("Testing the RotationExpression's Jacobian (column=") << i << ")").str()
      );
      if (expressionTester.getPrintResult()) {
        std::cout << "Jest=\n" << Jest << std::endl;
        std::cout << "Jc=\n" << JcM << std::endl;
      }
    }
  }
};

}
}
}

#endif /* ROTATIONEXPRESSIONTESTS_HPP_ */
