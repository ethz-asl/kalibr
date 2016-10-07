#ifndef ASLAM_BACKEND_OPTIMIZER_RPROP_HPP
#define ASLAM_BACKEND_OPTIMIZER_RPROP_HPP

#include <aslam/backend/util/OptimizerProblemManagerBase.hpp>

namespace sm {
  class PropertyTree;
}

namespace aslam {
  namespace backend {

    struct OptimizerOptionsRprop : public OptimizerOptionsBase
    {
      enum Method { RPROP_PLUS, RPROP_MINUS, IRPROP_MINUS, IRPROP_PLUS };

      OptimizerOptionsRprop();
      OptimizerOptionsRprop(const sm::PropertyTree& config);
      double etaMinus = 0.5; /// \brief Decrease factor for step size if gradient direction changes
      double etaPlus = 1.2; /// \brief Increase factor for step size if gradient direction is same
      double initialDelta = 0.1; /// \brief Initial step size
      double minDelta = 1e-20; /// \brief Minimum step size
      double maxDelta = 1.0; /// \brief Maximum step size
      bool useDenseJacobianContainer = true; /// \brief Whether or not to use a dense Jacobian container
      boost::shared_ptr<ScalarNonSquaredErrorTerm> regularizer = NULL; /// \brief Regularizer
      Method method = RPROP_PLUS; /// \brief the RProp method used

      void check() const override;

      template<class Archive>
      inline void serialize(Archive & ar, const unsigned int version);
    };
    std::ostream& operator<<(std::ostream& out, const aslam::backend::OptimizerOptionsRprop::Method& method);
    std::ostream& operator<<(std::ostream& out, const aslam::backend::OptimizerOptionsRprop& options);

    typedef OptimizerStatus OptimizerStatusRprop;

    /**
     * \class OptimizerRprop
     *
     * RPROP implementation for the ASLAM framework.
     */
    class OptimizerRprop : public OptimizerProblemManagerBase
    {
     public:

      typedef boost::shared_ptr<OptimizerRprop> Ptr;
      typedef boost::shared_ptr<const OptimizerRprop> ConstPtr;
      typedef OptimizerOptionsRprop Options;
      typedef OptimizerStatusRprop Status;

     public:

      /// \brief Constructor with default options
      OptimizerRprop();
      /// \brief Constructor with custom options
      OptimizerRprop(const Options& options);
      /// \brief Constructor from property tree
      OptimizerRprop(const sm::PropertyTree& config);
      /// \brief Destructor
      ~OptimizerRprop();

      /// \brief Return the status
      const Status& getStatus() const override { return _status; }

      /// \brief Const getter for the optimizer options.
      const Options& getOptions() const override { return _options; }

      /// \brief Mutable getter for the optimizer options (we explicitly allow direct modification of options).
      Options& getOptions() { return _options; }

      /// \brief Set the optimizer options.
      void setOptions(const Options& options) { _options = options; }

      /// \brief Set the optimizer options.
      void setOptions(const OptimizerOptionsBase& options) override { static_cast<OptimizerOptionsBase&>(_options) = options; }

    private:

      /// \brief Run the optimization
      void optimizeImplementation() override;

      /// \brief Reset information
      void resetImplementation() override;

      /// \brief branchless signum method
      static inline int sign(const double& val) {
        return (0.0 < val) - (val < 0.0);
      }

    private:

      /// \brief The dense update vector.
      ColumnVectorType _dx;

      /// \brief current step-length to be performed into the negative direction of the gradient
      ColumnVectorType _delta;

      /// \brief gradient in the previous iteration
      RowVectorType _prev_gradient;

      /// \brief error in the previous iteration (only used for IRPROP_PLUS version)
      double _prev_error = std::numeric_limits<double>::max();

      /// \brief the current set of options
      Options _options;

      /// \brief Status of the optimizer
      Status _status;

    };

  } // namespace backend
} // namespace aslam

#include "implementation/OptimizerRpropImpl.hpp"

#endif /* ASLAM_BACKEND_OPTIMIZER_RPROP_HPP */
