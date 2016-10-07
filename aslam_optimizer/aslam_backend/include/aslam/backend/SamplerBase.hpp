/*
 * SamplerBase.hpp
 *
 *  Created on: 10.08.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_SAMPLERBASE_HPP_
#define INCLUDE_ASLAM_BACKEND_SAMPLERBASE_HPP_

#include <limits>

#include <aslam/backend/util/ProblemManager.hpp>

namespace aslam {
namespace backend {

class SamplerBase {

 public:
  class Statistics {
   public:
    friend class SamplerBase;

    Statistics();
    ~Statistics() { }

    /// \brief Reset all information, i.e. counters and average acceptance probability to zero
    void reset();
    /// \brief Getter for the acceptance rate
    double getAcceptanceRate() const;
    /// \brief Getter for the number of iterations since the last initialize() or reset() call
    std::size_t getNumIterations() const;
    /// \brief Getter for the number of iterations since the last run(), initialize() or reset() call
    std::size_t getNumAcceptedSamples(bool total) const;

    double getWeightedMeanAcceptanceProbability() const { return weightedMeanAcceptanceProbability; }
    void updateWeightedMeanAcceptanceProbability(const double prob);

    double getWeightedMeanSmoothingFactor() const { return weightedMeanSmoothingFactor; }
    void setWeightedMeanSmoothingFactor(const double alpha) { weightedMeanSmoothingFactor = alpha; }

   public:

   private:
    std::size_t nIterations = 0; /// \brief How many iterations the sampler has run in the last initialize() call
    std::size_t nSamplesAcceptedTotal = 0; /// \brief How many samples were accepted since the last initialize() call
    std::size_t nSamplesAcceptedThisRun = 0; /// \brief How many samples were accepted since the last run() call
    double weightedMeanAcceptanceProbability = 0.0; /// \brief Exponential moving average of acceptance probabilities
    double weightedMeanSmoothingFactor = 0.1; /// \brief Smoothing factor (0.0, 1.0] for exponential moving average of acceptance probabilities
  };

 public:
  virtual ~SamplerBase() { }

  /// \brief Initialization method
  virtual void initialize();

  /// \brief Reset the sampler
  virtual void reset();

  /// \brief Run the sampler for \p nSteps
  void run(const std::size_t nSteps);

  /// \brief Set up to work on the log density. The log density may neglect the normalization constant.
  void setNegativeLogDensity(boost::shared_ptr<OptimizationProblemBase> negLogDensity);

  /// \brief Mutable getter for the log density formulation
  boost::shared_ptr<OptimizationProblemBase> getNegativeLogDensity();

  /// \brief Const getter for the log density formulation
  boost::shared_ptr<const OptimizationProblemBase> getNegativeLogDensity() const;

  /// \brief Signal the sampler that the negative log density formulation changed.
  void signalNegativeLogDensityChanged();

  /// \brief Tell the sampler not to use cached values from accepted samples in the last step
  void forceRecomputationNegLogDensity();

  /// \brief Whether the very last sample was accepted
  bool isLastSampledAccepted() const { return _isLastSampleAccepted; }

  /// \brief State of the force recomputation flag
  bool isRecomputationNegLogDensityNecessary() const { return _forceRecomputationNegLogDensity || !_isLastSampleAccepted; }

  /// \brief Do a bunch of checks to see if the problem is well-defined. This includes checking that every error term is
  ///        hooked up to design variables and running finite differences on error terms where this is possible.
  void checkNegativeLogDensitySetup() const;

  /// \brief Const getter for statistics
  const Statistics& statistics() const;

  /// \brief Set smoothing factor for exponential moving average of acceptance probabilities
  void setWeightedMeanSmoothingFactor(const double alpha);

  /// \brief Set the burn-in phase state of the sampler
  void setIsBurnIn(const bool isBurnIn) { _isBurnIn = isBurnIn; }
  /// \brief Whether or not the sampler is in burn-in phase
  bool isBurnIn() const { return _isBurnIn; }

 protected:
  /// \brief Evaluate the current negative log density
  double evaluateNegativeLogDensity(const size_t nThreads = 1) const;

  /// \brief Getter for problem manager
  ProblemManager& getProblemManager() { return _problemManager; }

 private:
  /// \brief Create one sample
  virtual void step(bool& accepted, double& acceptanceProbability) = 0;

  /// \brief Implement reset functionality for the derived class
  virtual void resetImplementation() { }

 private:
  Statistics _statistics; /// \brief statistics collected during runtime
  ProblemManager _problemManager;  /// \brief the manager for the attached problem

  bool _isLastSampleAccepted = false; /// \brief Was the sample in the last iteration accepted
  bool _forceRecomputationNegLogDensity = true; /// \brief Should density information definitely be recomputed in the next step

  bool _isBurnIn = false; /// \brief Whether or not the sampler is in burn-in phase

};

}
}

#endif /* INCLUDE_ASLAM_BACKEND_SAMPLERBASE_HPP_ */
