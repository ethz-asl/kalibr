#ifndef ASLAM_JACOBIAN_CONTAINER_PRESCALE_HPP
#define ASLAM_JACOBIAN_CONTAINER_PRESCALE_HPP

#include "JacobianContainer.hpp"

namespace aslam {
namespace backend {

/**
 * \class JacobianContainerFunctor
 * \brief Wrapper for any JacobianContainer in order to apply an operation to the Jacobian
 * of a design variable before it is added.
 *
 * \tparam Functor Custom functor type for the Jacobian operation
 */
template <typename Functor>
class JacobianContainerFunctor : public JacobianContainer {
 public:

  JacobianContainerFunctor(JacobianContainer& jc, int rows, Functor functor) : JacobianContainer(rows), _jc(jc), _functor(functor) { }
  virtual ~JacobianContainerFunctor() { }

  virtual void add(DesignVariable* designVariable, const Eigen::Ref<const Eigen::MatrixXd>& Jacobian) override {
    if (this->chainRuleEmpty())
      _jc.add(designVariable, _functor(Jacobian));
    else
      _jc.add(designVariable, _functor(this->chainRuleMatrix()*Jacobian));
  }

  virtual void add(DesignVariable* designVariable) override {
    if (this->chainRuleEmpty()) {
      _jc.add(designVariable, _functor(Eigen::MatrixXd::Identity(this->rows(), designVariable->minimalDimensions())));
    } else {
      auto CR = this->chainRuleMatrix();
      _jc.add(designVariable, _functor(CR*Eigen::MatrixXd::Identity(CR.cols(), designVariable->minimalDimensions())));
    }
  }

  virtual Eigen::MatrixXd asDenseMatrix() const override {
    return _jc.asDenseMatrix();
  }

  virtual bool isFinite(const DesignVariable& dv) const override {
    return _jc.isFinite(dv);
  }

 private:
  JacobianContainer& _jc;
  Functor _functor;
};

/**
 * \class JacobianContainerPrescaled
 * \brief Wraps any JacobianContainer in order to apply a scalar scale to the Jacobian
 * of a design variable before it is added.
 */
class JacobianContainerPrescaled : public JacobianContainerFunctor<JacobianContainerPrescaled&> {
 private:
  double _scalar;

 public:
  JacobianContainerPrescaled(JacobianContainer& jc, const double scalar)
 : JacobianContainerFunctor<JacobianContainerPrescaled&>(jc, jc.rows(), *this), _scalar(scalar)
   {

   }
  auto operator()(const Eigen::Ref<const Eigen::MatrixXd>& Jacobian) -> decltype(_scalar*Jacobian) {
    return _scalar*Jacobian;
  }
};

/// \brief Helper function to circumvent missing automatic compiler template argument deduction for class types
template <typename Functor>
JacobianContainerFunctor<Functor> getJacobianContainerFunctor(JacobianContainer& jc, int rows, Functor prescaleFcn) {
  return JacobianContainerFunctor<Functor>(jc, rows, prescaleFcn);
}

} // namespace backend
} // namespace aslam

#endif /* ASLAM_JACOBIAN_CONTAINER_PRESCALE_HPP */
