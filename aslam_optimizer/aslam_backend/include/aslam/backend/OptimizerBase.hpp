/*
 * OptimizerBase.hpp
 *
 *  Created on: 11.04.2016
 *      Author: sculrich
 */

#ifndef INCLUDE_ASLAM_BACKEND_OPTIMIZERBASE_HPP_
#define INCLUDE_ASLAM_BACKEND_OPTIMIZERBASE_HPP_

// standard
#include <iostream>
#include <limits> // signaling_NaN, max

// self
#include <aslam/backend/util/CommonDefinitions.hpp> // RowVectorType
#include <aslam/backend/OptimizationProblemBase.hpp>
#include <aslam/backend/OptimizerCallbackManager.hpp>

namespace sm
{
  class PropertyTree; // forward declaration
} /* namespace sm  */

namespace aslam
{
namespace backend
{

/**
 * \enum ConvergenceStatus
 */
enum ConvergenceStatus
{
  IN_PROGRESS = 0,//!< IN_PROGRESS
  FAILURE = 1,    //!< FAILURE
  GRADIENT_NORM,  //!< GRADIENT_NORM
  DX,             //!< DX
  DOBJECTIVE      //!< DOBJECTIVE
};

/// \brief Stream operator for ConvergenceStatus
std::ostream& operator<<(std::ostream& out, const ConvergenceStatus& convergence);


/**
 * \struct OptimizerOptionsBase
 * Options common to all optimizer classes
 *
 * TODO: Rename to OptimizerOptions after conflict with legacy OptimizerOptions is resolved
 */
struct OptimizerOptionsBase
{
  OptimizerOptionsBase();
  OptimizerOptionsBase(const sm::PropertyTree& config);
  virtual ~OptimizerOptionsBase() { }

  double convergenceGradientNorm = 1e-3; /// \brief Convergence criterion on gradient norm
  double convergenceDeltaX = 0.0; /// \brief Convergence criterion on maximum absolute state update coefficient
  double convergenceDeltaObjective = 0.0; /// \brief Convergence criterion on change of objective/error
  int maxIterations = 100; /// \brief Stop if we reach this number of iterations without hitting any of the above stopping criteria. -1 for unlimited.
  std::size_t numThreadsGradient = 4; /// \brief The number of threads to use for gradient computation
  std::size_t numThreadsError = 1; /// \brief The number of threads to use for error computation

  /// \brief Checks options for sanity. Throws if any options is not valid.
  virtual void check() const;

  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);
};

/// \brief Stream operator for OptimizerOptionsBase
std::ostream& operator<<(std::ostream& out, const OptimizerOptionsBase& options);

/**
 * \struct OptimizerStatus
 * Collects information about the current status of the optimizer
 */
struct OptimizerStatus
{
  friend class OptimizerBase;

  /// \brief Constructor
  OptimizerStatus() { }
  /// \brief Destructor
  virtual ~OptimizerStatus() { }

  /// \brief Converged?
  bool success() const;
  /// \brief Failed?
  bool failure() const;

  ConvergenceStatus convergence = IN_PROGRESS;
  std::size_t numIterations = 0; /// \brief Number of iterations run
  std::size_t numDerivativeEvaluations = 0; /// \brief Number of Jacobian/gradient evaluations performed
  std::size_t numObjectiveEvaluations = 0; /// \brief Number of objective/error evaluations performed
  double gradientNorm = std::numeric_limits<double>::signaling_NaN(); /// \brief Norm of the gradient
  double maxDeltaX = std::numeric_limits<double>::signaling_NaN(); /// \brief Maximum absolute value of change in design variables
  double error = std::numeric_limits<double>::max(); /// \brief Current error/objective value. numeric_limits<double>::max() if error is not evaluated.
  double deltaError = std::numeric_limits<double>::signaling_NaN(); /// \brief last change of the error. numeric_limits<double>::signaling_NaN() if error is not evaluated.

  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 protected:
  /// \brief Reset to initial values
  void reset();

 private:
  /// \brief Implement in derived class to reset derived-specific information
  virtual void resetImplementation() { }
};

/// \brief Stream operator for OptimizerStatus
std::ostream& operator<<(std::ostream& out, const OptimizerStatus& ret);


/**
 * \class OptimizerBase
 * Base class for all optimizers
 */
class OptimizerBase
{
 public:

  /// \brief Constructor
  OptimizerBase();
  /// \brief Destructor
  virtual ~OptimizerBase();

  /// \brief Set up to work on the optimization problem.
  virtual void setProblem(boost::shared_ptr<OptimizationProblemBase> problem) = 0;

  /// \brief Do a bunch of checks to see if the problem is well-defined.
  virtual void checkProblemSetup() = 0;

  /// \brief Is everything initialized?
  virtual bool isInitialized() = 0;

  /// \brief Run the optimization until convergence
  void optimize();

  /// \brief Get the optimizer's status
  virtual const OptimizerStatus& getStatus() const = 0;

  /// \brief Get the optimizer options.
  virtual const OptimizerOptionsBase& getOptions() const = 0;

  /// \brief Set the optimizer options.
  virtual void setOptions(const OptimizerOptionsBase& options) = 0;

  /// \brief Initialize the optimizer to run on an optimization problem.
  void initialize();

  /// \brief Reset internal states but don't re-initialize the whole problem
  void reset();

  /// \brief Get the design variables
  virtual const std::vector<DesignVariable*>& getDesignVariables() const = 0;

  /// \brief Has the optimizer converged?
  inline bool isConverged() const ;
  /// \brief Has the optimizer failed?
  inline bool isFailed() const;
  /// \brief Is the optimizer still running?
  inline bool isInProgress() const;

  /// \brief expose callback registry
  inline callback::Registry & callback() { return _callbackManager; }

 protected:
  /// \brief Helper function to update the convergence status. Set the error/derivative information
  ///        before you call this method.
  void updateConvergenceStatus();

  /// \brief A class that manages the optimizer callbacks
  callback::Manager _callbackManager;

 private:
  /// \brief Implement in derived class
  virtual void optimizeImplementation() = 0;
  /// \brief Implement in derived class to initialize any derived-specific information
  virtual void initializeImplementation() { }
  /// \brief Implement in derived class to reset any derived-specific reset
  virtual void resetImplementation() { }

  /// \brief Mutable getter for the options
  inline OptimizerStatus& status();

};

} /* namespace aslam */
} /* namespace backend */


#include "implementation/OptimizerBaseImplementation.hpp"

#endif /* INCLUDE_ASLAM_BACKEND_OPTIMIZERBASE_HPP_ */
