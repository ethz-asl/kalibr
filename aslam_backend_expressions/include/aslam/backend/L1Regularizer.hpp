/*
 * L1Regularizer.hpp
 *
 *  Created on: 22.09.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_L1REGULARIZER_HPP_
#define INCLUDE_ASLAM_BACKEND_L1REGULARIZER_HPP_

#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <aslam/backend/Scalar.hpp>

namespace aslam {
namespace backend {

class L1Regularizer : public aslam::backend::ScalarNonSquaredErrorTerm {

 public:
  L1Regularizer(const std::vector<Scalar*>& scalarDvs, const double beta);
  virtual ~L1Regularizer() { }

  void setBeta(const double beta) { setWeight(beta); }

 private:
  /// \brief evaluate the error term and return the scalar error \f$ e \f$
  virtual double evaluateErrorImplementation() override;

  /// \brief evaluate the Jacobians for \f$ e \f$
  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) override;

  const std::vector<Scalar*>& _dvs;

};

}
}

#endif /* INCLUDE_ASLAM_BACKEND_L1REGULARIZER_HPP_ */
