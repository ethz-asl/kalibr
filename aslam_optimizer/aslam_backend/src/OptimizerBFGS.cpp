#include <iomanip>
#include <aslam/backend/OptimizerBFGS.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <Eigen/Dense>
#include <sm/eigen/assert_macros.hpp>
#include <aslam/backend/sparse_matrix_functions.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/logging.hpp>

/*
 * The following BFGS implementation is based on https://github.com/scipy/scipy/blob/a81bc79ba38825139e97b14c91e158f4aabc0bed/scipy/optimize/optimize.py#L874.
 * See /LICENSE_SciPy.txt for the SciPy license.
 */

namespace aslam {
namespace backend {

OptimizerOptionsBFGS::OptimizerOptionsBFGS()
    : OptimizerOptionsBase(), linesearch()
{
  // base options checked by OptimizerOptionsBase
  linesearch.check();
}

OptimizerOptionsBFGS::OptimizerOptionsBFGS(const sm::PropertyTree& config)
    : OptimizerOptionsBase(config), linesearch(sm::PropertyTree(config, "linesearch"))
{
  useDenseJacobianContainer = config.getDouble("useDenseJacobianContainer", useDenseJacobianContainer);
  // base options checked by OptimizerOptionsBase
  linesearch.check();
}

void OptimizerOptionsBFGS::check() const
{
  OptimizerOptionsBase::check();
  linesearch.check();
}

std::ostream& operator<<(std::ostream& out, const aslam::backend::OptimizerOptionsBFGS& options)
{
  out << static_cast<OptimizerOptionsBase>(options) << std::endl;
  out << options.linesearch << std::endl;
  out << "OptimizerOptionsBFGS:" << std::endl;
  out << "\tuseDenseJacobianContainer: " << (options.useDenseJacobianContainer ? "TRUE" : "FALSE") << std::endl;
  out << "\thasRegularizer: " << ((options.regularizer != nullptr) ? "TRUE" : "FALSE");
  return out;
}


OptimizerBFGS::OptimizerBFGS(const OptimizerOptionsBFGS& options)
    : _options(options),
      _linesearch(getCostFunction<false,true,false,true,true>(problemManager(), false, _options.useDenseJacobianContainer, false, _options.numThreadsGradient, _options.numThreadsError), _options.linesearch)
{
  _options.check();
  _linesearch.setEvaluateErrorCallback( [&]() { _status.numObjectiveEvaluations++; } );
  _linesearch.setEvaluateGradientCallback( [&]() { _status.numDerivativeEvaluations++; });
}

OptimizerBFGS::OptimizerBFGS()
    : OptimizerBFGS::OptimizerBFGS(OptimizerOptionsBFGS())
{
}


OptimizerBFGS::OptimizerBFGS(const sm::PropertyTree& config)
    : OptimizerBFGS::OptimizerBFGS(OptimizerOptionsBFGS(config))
{
}

OptimizerBFGS::~OptimizerBFGS()
{
}

void OptimizerBFGS::resetImplementation() {
  _Bk.setIdentity(problemManager().numOptParameters(), problemManager().numOptParameters());
  _linesearch.initialize();
}

void OptimizerBFGS::optimizeImplementation()
{
  Timer timeUpdateHessian("OptimizerBFGS: Update---Hessian", true);

  using namespace Eigen;

  const MatrixXd I = MatrixXd::Identity(problemManager().numOptParameters(), problemManager().numOptParameters());
  RowVectorType gfk, gfkp1;
  gfk = _linesearch.getGradient();
  _status.gradientNorm = gfk.norm();
  _status.error = _linesearch.getError();
  SM_FINE_STREAM_NAMED("optimization", std::setprecision(20) << "OptimizerBFGS: Start optimization at state " <<
                       problemManager().getFlattenedDesignVariableParameters().transpose().format(IOFormat(15, DontAlignCols, ", ", ", ", "", "", "[", "]")) <<
                        " with gradient " << gfk.transpose().format(IOFormat(15, DontAlignCols, ", ", ", ", "", "", "[", "]")) << " (norm: " <<
                        _status.gradientNorm << ") and error " << _status.error);
  this->updateStatus(true);

  if (!_status.success()) {

    std::size_t cnt = 0;
    for (cnt = 0; _options.maxIterations == -1 || cnt < static_cast<size_t>(_options.maxIterations); ++cnt, ++_status.numIterations) {

      _callbackManager.issueCallback( {callback::Occasion::ITERATION_START} );

      // compute search direction
      // Note: this could fail due to numerical issues making the inverse Hessian approximation negative definite
      // and resulting in an ascent direction where the eigenvalues become negative. We rely on the line search to detect
      // that here, and reset the inverse Hessian to the identity matrix. This will be done only once, if it fails the exception
      // is re-thrown. Instead of resetting to identity we could of course do something smarter.
      RowVectorType pk;
      for(std::size_t j=0; j<2; ++j) {
        try {
          pk = -_Bk*gfk.transpose();
          _linesearch.setSearchDirection(pk);
          break;
        } catch (const std::exception& e) {
          if (j == 0) {
            SM_WARN("Inverse Hessian approximation became negative, resetting to identity matrix. "
                "Check your problem setup anyways and potentially re-scale your parameters.");
            _Bk = I;
          } else {
            throw;
          }
        }
      }

      // store last design variables
      const Eigen::VectorXd dv = problemManager().getFlattenedDesignVariableParameters();

      // perform line search
      bool lsSuccess = _linesearch.lineSearchWolfe12();
      _callbackManager.issueCallback( {callback::Occasion::DESIGN_VARIABLES_UPDATED} );

      const double alpha_k = _linesearch.getCurrentStepLength();
      gfkp1 = _linesearch.getGradient();
      _status.gradientNorm = gfkp1.norm();
      _status.deltaError = _linesearch.getError() - _status.error;
      _status.error = _linesearch.getError();
      _status.maxDeltaX = (problemManager().getFlattenedDesignVariableParameters() - dv).cwiseAbs().maxCoeff();

      this->updateStatus(lsSuccess);
      if (_status.success() || _status.failure())
        break;

      SM_FINE_STREAM_NAMED("optimization", std::setprecision(20) << _status << std::endl <<
                           "\tsteplength: " << alpha_k);

      // Update Hessian
      timeUpdateHessian.start();

      RowVectorType sk = alpha_k * pk;
      RowVectorType yk = gfkp1 - gfk;
      gfk = gfkp1;

      double rhok = 1./(yk*sk.transpose());
      if (std::isinf(rhok)) {
        rhok = 1000.0;
        SM_WARN("Divide-by-zero encountered: rhok assumed large");
      }

      MatrixXd C = sk.transpose() * yk * rhok;
      MatrixXd A = I - C;
      _Bk = A * (_Bk * A.transpose()) + (rhok * sk.transpose() * sk); // Sherman-Morrison formula

      timeUpdateHessian.stop();

      _callbackManager.issueCallback( {callback::Occasion::ITERATION_END} );
    }
  }

  if (!_status.failure())
    SM_DEBUG_STREAM_NAMED("optimization", _status);
  else
    SM_ERROR_STREAM(_status);

}

void OptimizerBFGS::updateStatus(const bool lineSearchSuccess)
{

  // Test failure criteria
  if (!lineSearchSuccess) {
    _status.convergence = ConvergenceStatus::FAILURE;
    return;
  }

  if (!std::isfinite(_status.error)) {
    _status.convergence = ConvergenceStatus::FAILURE; // TODO: Is this really a failure?
    SM_WARN("OptimizerBFGS: We correctly found +-inf as optimal value, or something went wrong?");
    return;
  }

  // Test success criteria
  _status.convergence = ConvergenceStatus::IN_PROGRESS; // if none of the success criteria succeed, we are not converged yet
  this->updateConvergenceStatus();

}

} // namespace backend
} // namespace aslam
