#ifndef OPTIMIZER_CALLBACK_HPP
#define OPTIMIZER_CALLBACK_HPP

#include <boost/make_shared.hpp>

namespace aslam {
namespace backend {
namespace callback {

/**
 * \brief The callback occasion specifies the callback injection point in the optimizer.
 */
enum Occasion {
  OPTIMIZATION_INITIALIZED,   /// \brief Right after the the initial cost has been computed.
  ITERATION_START,            /// \brief At the start of an iteration before any work has been done, same state as in PER_ITERATION_END in previous iteration
  ITERATION_END,              /// \brief At the end of an iteration after all work has been done, same state as in PER_ITERATION_START in next iteration
  LINEAR_SYSTEM_SOLVED,       /// \brief Right after the linear system was solved
  DESIGN_VARIABLES_UPDATED,   /// \brief Right after the design variables (X) have been updated.
  RESIDUALS_UPDATED,          /// \brief After the raw squared error for each error term has been updated but before the m-estimators are applied to compute the effective cost. This is the right occasion to update m-estimators based on residuals.
  COST_UPDATED,               /// \brief Right after the m-estimators are applied to compute the effective cost.
};

/**
 * \brief The argument class for the optimizer callbacks.
 */
class Argument {
public:
  Argument(Occasion occasion_,
           double currentCost_ = std::numeric_limits<double>::signaling_NaN(),
           double previousLowestCost_ = std::numeric_limits<double>::signaling_NaN())
      : occasion(occasion_), currentCost(currentCost_), previousLowestCost(previousLowestCost_) { }

  /**
   * \brief the callback occasion. (where in the optimizer was it issued).
   */
  Occasion occasion;
  /**
   * \brief the most recently evaluated effective cost (J = sum of squared errors after the m-estimators being applied).
   * Its relation to the optimization phase depends on the callback occasion:
   *   OPTIMIZATION_INITIALIZED:      the initial value (before any optimization)
   *   X_UPDATED, RESIDUALS_UPDATED:  undefined
   *   COST_UPDATED:                  the new cost after an update of the design variables
   */
  double currentCost;
  /**
   * \brief the most recently evaluated effective cost (J = sum of squared errors after the m-estimators being applied).
   * Its relation to the optimization phase depends on the callback occasion:
   *   OPTIMIZATION_INITIALIZED, X_UPDATED, RESIDUALS_UPDATED:  undefined
   *   COST_UPDATED:                  the lowest previous cost so far (or -1 if this is the initial update)
   */
  double previousLowestCost;

  //TODO (HannesSommer) specify more argument values
};


enum class ProceedInstruction {
  CONTINUE,
  SUCCEED,
  FAIL
};

class OptimizerCallbackInterface {
public:
  typedef boost::shared_ptr<OptimizerCallbackInterface> Ptr;
  virtual ~OptimizerCallbackInterface() {}
  virtual ProceedInstruction operator() (const Argument & arg) = 0;
};

template <typename Funct>
class CallbackFunctor : public OptimizerCallbackInterface {
public:
  CallbackFunctor(Funct f) : f(f) {}
  virtual ProceedInstruction operator() (const Argument & arg) override {
    return call(arg);
  }
private:
  template<typename F = Funct, std::is_same<ProceedInstruction, decltype((*static_cast<F*>(nullptr))(*static_cast<Argument*>(nullptr)))>* returnsProceedInstruction = nullptr>
  ProceedInstruction call(const Argument & arg) override {
    return withArg(arg, returnsProceedInstruction);
  }
  template<typename F = Funct, std::is_same<ProceedInstruction, decltype((*static_cast<F*>(nullptr))())>* returnsProceedInstruction = nullptr>
  ProceedInstruction call(const Argument &) override {
    return withoutArg(returnsProceedInstruction);
  }

  ProceedInstruction withoutArg(std::true_type*) {
    return f();
  }
  ProceedInstruction withoutArg(std::false_type*) {
    f();
    return ProceedInstruction::CONTINUE;
  }
  ProceedInstruction withArg(const Argument & arg, std::true_type*) {
    return f(arg);
  }
  ProceedInstruction withArg(const Argument & arg, std::false_type*) {
    f(arg);
    return ProceedInstruction::CONTINUE;
  }
  Funct f;
};

template <typename Funct>
OptimizerCallbackInterface::Ptr toOptimizerCallback(Funct f){
  return boost::make_shared<CallbackFunctor<Funct>>(f);
}

class OptimizerCallback {
public:
  OptimizerCallback(const OptimizerCallbackInterface::Ptr & impl) : impl_(impl) {}

  template <typename Funct>
  OptimizerCallback(Funct funct) : OptimizerCallback(toOptimizerCallback(funct)) {}

  bool operator == (const OptimizerCallback & other) const { return impl_ == other.impl_; };
  ProceedInstruction operator() (const Argument & arg){
    return (*impl_)(arg);
  }
private:
  OptimizerCallbackInterface::Ptr impl_;
};



} // namespace callback
} // namespace backend
}  // namespace aslam

#endif /* OPTIMIZER_CALLBACK_HPP */
