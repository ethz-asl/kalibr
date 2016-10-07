#ifndef ASLAM_BACKEND_OPTIMIZER_BFGS_HPP
#define ASLAM_BACKEND_OPTIMIZER_BFGS_HPP

#include <aslam/backend/util/OptimizerProblemManagerBase.hpp>
#include <aslam/backend/LineSearch.hpp>

/*
 * The following BFGS implementation is based on https://github.com/scipy/scipy/blob/a81bc79ba38825139e97b14c91e158f4aabc0bed/scipy/optimize/optimize.py#L874.
 * See /LICENSE_SciPy.txt for the SciPy license.
 */

namespace sm {
  class PropertyTree;
}

namespace aslam {
  namespace backend {

    struct OptimizerOptionsBFGS : public OptimizerOptionsBase
    {
      OptimizerOptionsBFGS();
      OptimizerOptionsBFGS(const sm::PropertyTree& config);
      LineSearchOptions linesearch; /// \brief Linesearch options
      bool useDenseJacobianContainer = true; /// \brief Whether or not to use a dense Jacobian container
      boost::shared_ptr<ScalarNonSquaredErrorTerm> regularizer = NULL; /// \brief Regularizer

      void check() const override;

      template<class Archive>
      inline void serialize(Archive & ar, const unsigned int version);
    };

    std::ostream& operator<<(std::ostream& out, const aslam::backend::OptimizerOptionsBFGS& options);

    typedef OptimizerStatus OptimizerStatusBFGS;

    /**
     * \class OptimizerBFGS
     *
     * Broyden-Fletcher-Goldfarb-Shannon algorithm implementation for the ASLAM framework.
     */
    class OptimizerBFGS : public OptimizerProblemManagerBase
    {
     public:
      typedef boost::shared_ptr<OptimizerBFGS> Ptr;
      typedef boost::shared_ptr<const OptimizerBFGS> ConstPtr;
      typedef OptimizerOptionsBFGS Options;
      typedef OptimizerStatusBFGS Status;

     public:
      /// \brief Constructor with default options
      OptimizerBFGS();
      /// \brief Constructor with custom options
      OptimizerBFGS(const Options& options);
      /// \brief Constructor from property tree
      OptimizerBFGS(const sm::PropertyTree& config);
      /// \brief Destructor
      ~OptimizerBFGS();

      /// \brief Return the status
      const Status& getStatus() const override { return _status; }

      /// \brief Get the optimizer options.
      const Options& getOptions() const override { return _options; }

      /// \brief Set the optimizer options.
      void setOptions(const Options& options) { _options = options; _linesearch.options() = _options.linesearch; }

      /// \brief Set the optimizer options.
      void setOptions(const OptimizerOptionsBase& options) override { static_cast<OptimizerOptionsBase&>(_options) = options; }

      /// \brief Const getter for the linesearch object
      const LineSearch& getLineSearch() const { return _linesearch; }

    private:

      /// \brief Run the optimization
      void optimizeImplementation() override;

      /// \brief Reset information
      void resetImplementation() override;

      /// \brief Update the status
      void updateStatus(bool lineSearchSuccess);

    private:

      /// \brief Problem manager
      ProblemManager _problemManager;

      /// \brief the current set of options
      Options _options;

      /// \brief The current estimate of the inverse Hessian
      Eigen::MatrixXd _Bk;

      /// \brief Line-search class
      LineSearch _linesearch;

      /// \brief Status of the optimizer
      Status _status;

    };

  } // namespace backend
} // namespace aslam

#include "implementation/OptimizerBFGSImpl.hpp"

#endif /* ASLAM_BACKEND_OPTIMIZER_BFGS_HPP */
