#include <aslam/backend/OptimizerRprop.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <Eigen/Dense>
#include <sm/eigen/assert_macros.hpp>
#include <aslam/backend/sparse_matrix_functions.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/logging.hpp>

namespace aslam {
namespace backend {

OptimizerOptionsRprop::OptimizerOptionsRprop()
    : OptimizerOptionsBase()
{
}

OptimizerOptionsRprop::OptimizerOptionsRprop(const sm::PropertyTree& config)
    : OptimizerOptionsBase(config)
{
  etaMinus = config.getDouble("etaMinus", etaMinus);
  etaPlus = config.getDouble("etaPlus", etaPlus);
  initialDelta = config.getDouble("initialDelta", initialDelta);
  minDelta = config.getDouble("minDelta", minDelta);
  maxDelta = config.getDouble("maxDelta", maxDelta);
  check();
}

void OptimizerOptionsRprop::check() const
{
  SM_ASSERT_GT( Exception, etaMinus, 0.0, "");
  SM_ASSERT_GT( Exception, etaPlus, etaMinus, "");
  SM_ASSERT_GT( Exception, initialDelta, 0.0, "");
  SM_ASSERT_GT( Exception, minDelta, 0.0, "");
  SM_ASSERT_GT( Exception, maxDelta, minDelta, "");
  OptimizerOptionsBase::check();
}

std::ostream& operator<<(std::ostream& out, const aslam::backend::OptimizerOptionsRprop::Method& method)
{
  switch(method)
  {
    case OptimizerOptionsRprop::Method::IRPROP_MINUS:
      out << "IRPROP_MINUS";
      break;
    case OptimizerOptionsRprop::Method::IRPROP_PLUS:
      out << "IRPROP_PLUS";
      break;
    case OptimizerOptionsRprop::Method::RPROP_MINUS:
      out << "RPROP_MINUS";
      break;
    case OptimizerOptionsRprop::Method::RPROP_PLUS:
      out << "RPROP_PLUS";
      break;
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const aslam::backend::OptimizerOptionsRprop& options)
{
  out << static_cast<OptimizerOptionsBase>(options) << std::endl;
  out << "OptimizerOptionsRprop:" << std::endl;
  out << "\tetaMinus: " << options.etaMinus << std::endl;
  out << "\tetaPlus: " << options.etaPlus << std::endl;
  out << "\tinitialDelta: " << options.initialDelta << std::endl;
  out << "\tminDelta: " << options.minDelta << std::endl;
  out << "\tmaxDelta: " << options.maxDelta << std::endl;
  out << "\tmethod: " << options.method << std::endl;
  out << "\tuseDenseJacobianContainer: " << (options.useDenseJacobianContainer ? "TRUE" : "FALSE") << std::endl;
  out << "\thasRegularizer: " << ((options.regularizer != nullptr) ? "TRUE" : "FALSE");
  return out;
}


OptimizerRprop::OptimizerRprop(const OptimizerOptionsRprop& options)
    : _options(options)
{
  _options.check();
}

OptimizerRprop::OptimizerRprop()
    : OptimizerRprop::OptimizerRprop(OptimizerOptionsRprop())
{
}


OptimizerRprop::OptimizerRprop(const sm::PropertyTree& config)
    : OptimizerRprop::OptimizerRprop(OptimizerOptionsRprop(config))
{
}

OptimizerRprop::~OptimizerRprop()
{
}

void OptimizerRprop::resetImplementation() {
  _dx = ColumnVectorType::Constant(problemManager().numOptParameters(), 0.0);
  _prev_gradient = ColumnVectorType::Constant(problemManager().numOptParameters(), 0.0);
  _prev_error = std::numeric_limits<double>::max();
  _delta = ColumnVectorType::Constant(problemManager().numOptParameters(), _options.initialDelta);
}

void OptimizerRprop::optimizeImplementation()
{
  Timer timeGrad("OptimizerRprop: Compute---Gradient", true);
  Timer timeStep("OptimizerRprop: Compute---Step size", true);
  Timer timeUpdate("OptimizerRprop: Compute---State update", true);

  if (!isInitialized())
    initialize();

  using namespace Eigen;

  for ( ; _options.maxIterations == -1 || _status.numIterations < static_cast<size_t>(_options.maxIterations); ++_status.numIterations) {

    _callbackManager.issueCallback( {callback::Occasion::ITERATION_START} );

    _status.convergence = ConvergenceStatus::IN_PROGRESS;

    RowVectorType gradient;
    timeGrad.start();
    problemManager().computeGradient(gradient, _options.numThreadsGradient, false /*useMEstimator*/, false /*use scaling */, _options.useDenseJacobianContainer /*useDenseJacobianContainer*/);

    // optionally add regularizer
    if (_options.regularizer) {
      JacobianContainerSparse<1> jc(1);
      _options.regularizer->evaluateJacobians(jc);
      SM_FINER_STREAM_NAMED("optimization", "RPROP: Regularization term gradient: " << jc.asDenseMatrix());
      gradient += jc.asDenseMatrix();
    }
    _status.numDerivativeEvaluations++;
    timeGrad.stop();

    SM_ASSERT_TRUE_DBG(Exception, gradient.allFinite (), "Gradient " << gradient.format(IOFormat(2, DontAlignCols, ", ", ", ", "", "", "[", "]")) << " is not finite");

    timeStep.start();
    _status.gradientNorm = gradient.norm();

    if (_status.gradientNorm < _options.convergenceGradientNorm) {
      _status.convergence = ConvergenceStatus::GRADIENT_NORM;
      SM_DEBUG_STREAM_NAMED("optimization", "RPROP: Current gradient norm " << _status.gradientNorm <<
                            " is smaller than convergenceGradientNorm option -> terminating");
      break;
    }

    // Compute error for iPRop+
    bool errorIncreased = false;
    if (_options.method == OptimizerOptionsRprop::IRPROP_PLUS) {
      _status.error = problemManager().evaluateError(_options.numThreadsError);
      _status.numObjectiveEvaluations++;
      errorIncreased = (_status.error - _prev_error) > 0.0;
      _prev_error = _status.error;
    }

    // determine whether gradient direction switched
    Eigen::Matrix<double, 1, Eigen::Dynamic> gg = _prev_gradient.cwiseProduct(gradient);
    Eigen::Matrix<bool, 1, Eigen::Dynamic> switchNo = gg.array() > 0.0;
    Eigen::Matrix<bool, 1, Eigen::Dynamic> switchYes = gg.array() < 0.0;
    _prev_gradient = gradient;

    for (std::size_t d = 0; d < problemManager().numOptParameters(); ++d) {

      // Adapt delta
      if (switchNo(d))
        _delta(d) = std::min(_delta(d) * _options.etaPlus, _options.maxDelta);
      else if (switchYes(d))
        _delta(d) = std::max(_delta(d) * _options.etaMinus, _options.minDelta);

      // Note: see http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.17.1332
      // for a good description of the algorithms
      switch (_options.method) {

        // RPROP_PLUS
        // With backtracking. If gradient switched direction, revert this update.
        case OptimizerOptionsRprop::RPROP_PLUS:
        {
          // Compute design variable update vector
          if (switchYes(d)) {
            _dx(d) = -_dx(d); // revert update
            _prev_gradient(d) = 0.0; // this forces switchYes=false in the next step
          } else {
            _dx(d) = -sign(gradient(d))*_delta(d);
          }

          break;
        }

        // RPROP_MINUS
        // No backtracking. Reduce step-length if gradient switched direction,
        // Increase step-length if gradient in same direction.
        case OptimizerOptionsRprop::RPROP_MINUS:
        {
          // Compute design variable update vector
          _dx(d) = -sign(gradient(d))*_delta(d);

          break;
        }
        // IRPROP_MINUS
        // In case gradient direction switched, stay at this point for one iteration and
        // then move into the direction of the gradient with half the step-length.
        case OptimizerOptionsRprop::IRPROP_MINUS:
        {
          // Compute design variable update vector
          if (switchYes(d))
            _dx(d) = _prev_gradient(d) = 0.0;
          else
            _dx(d) = -sign(gradient(d))*_delta(d);

          break;
        }
        // IRPROP_PLUS
        // Revert only weight updates that have caused changes of the corresponding
        // partial derivatives in case of an error increase.
        case OptimizerOptionsRprop::IRPROP_PLUS:
        {

          // Compute design variable update vector
          if (switchYes(d)) {
            if (errorIncreased)
              _dx(d) = -_dx(d); // revert update if gradient direction switched and error increased
            else
              _dx(d) = 0.0;
            _prev_gradient(d) = 0.0; // this forces switchYes=false in the next step
          } else {
            _dx(d) = -sign(gradient(d))*_delta(d);
          }

          break;
        }
      }

    }

    timeStep.stop();

    timeUpdate.start();
    problemManager().applyStateUpdate(_dx);
    timeUpdate.stop();
    _callbackManager.issueCallback( {callback::Occasion::DESIGN_VARIABLES_UPDATED} );

    _status.maxDeltaX = _dx.cwiseAbs().maxCoeff();
    if (_status.maxDeltaX < _options.convergenceDeltaX) {
      _status.convergence = ConvergenceStatus::DX;
      SM_DEBUG_STREAM_NAMED("optimization", "RPROP: Maximum dx coefficient " << _status.maxDeltaX <<
                            " is smaller than convergenceDx option -> terminating");
      break;
    }

    if (_options.method == OptimizerOptionsRprop::IRPROP_PLUS) {
      _status.deltaError = problemManager().evaluateError(_options.numThreadsError) - _status.error;
      if (fabs(_status.deltaError) < _options.convergenceDeltaObjective) {
        _status.convergence = ConvergenceStatus::DOBJECTIVE;
        SM_DEBUG_STREAM_NAMED("optimization", "RPROP: Change in error " << _status.deltaError <<
                              " is smaller than convergenceDObjective option -> terminating");
        break;
      }
    }

    SM_FINE_STREAM_NAMED("optimization", _status << std::endl <<
                         "\tgradient: " << gradient << std::endl <<
                         "\tdx: " << _dx.transpose() << std::endl <<
                         "\tdelta: " << _delta.transpose());

    _callbackManager.issueCallback( {callback::Occasion::ITERATION_END} );

  }

  SM_DEBUG_STREAM_NAMED("optimization", _status);

}

} // namespace backend
} // namespace aslam
