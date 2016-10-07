/*
 * OptimizerProblemManagerBase.hpp
 *
 *  Created on: 13.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_ASLAM_BACKEND_UTIL_OPTIMIZERPROBLEMMANAGERBASE_HPP_
#define INCLUDE_ASLAM_BACKEND_UTIL_OPTIMIZERPROBLEMMANAGERBASE_HPP_

#include <aslam/backend/OptimizerBase.hpp>
#include <aslam/backend/util/ProblemManager.hpp>

namespace aslam
{
namespace backend
{

/**
 * \class OptimizerProblemManagerBase
 * Helper class to facilitate deriving optimizers that use the ProblemManager under the hood
 */
class OptimizerProblemManagerBase : public OptimizerBase
{
 public:
  virtual ~OptimizerProblemManagerBase() { }
  void setProblem(boost::shared_ptr<OptimizationProblemBase> problem) override { _problemManager.setProblem(problem); }
  void checkProblemSetup() override { _problemManager.checkProblemSetup(); }
  bool isInitialized() override { return _problemManager.isInitialized(); }
  const std::vector<DesignVariable*>& getDesignVariables() const override { return _problemManager.designVariables(); }

 protected:
  const ProblemManager& problemManager() const { return _problemManager; }
  ProblemManager& problemManager() { return _problemManager; }
  virtual void initializeImplementation() override { _problemManager.initialize(); }

 private:
  ProblemManager _problemManager; /// \brief Problem manager

};

} // namespace backend
} // namespace aslam


#endif /* INCLUDE_ASLAM_BACKEND_UTIL_OPTIMIZERPROBLEMMANAGERBASE_HPP_ */
